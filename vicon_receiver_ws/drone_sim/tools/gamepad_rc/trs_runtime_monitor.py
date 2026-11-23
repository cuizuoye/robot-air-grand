#!/usr/bin/env python3
"""Persistent single-TRS/Vicon/HOME integration foundation, NOT a flight runner.

Never sends poses, targets, mode changes, arm/unlock, parameter writes or rate
changes. HOME can ONLY lock an identified, freshly DISARMED PX4 1.14.3. It
continues observing after HOME release, unlike the one-shot benches. This
entry does not claim airborne emergency-stop or independent-PC safety.
After a valid initial baseline, actuator query loss latches degraded readiness
but keeps HOME stop-only monitoring alive. Query retries require a drained
prompt and are bounded; stale/late data never proves current output lock.
"""
import argparse
from collections import Counter
from contextlib import ExitStack
import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import serial
from pymavlink.dialects.v20 import common as m
from estop_only_policy import (DurableStopLatch, FIRMWARE, LOCK_COMMAND,
    lock_message, shell_message, shell_message_allowed,
    stop_message_allowed)
from gamepad_estop_monitor import EVENT, HOME, find_pad
from mavlink_readonly_probe import Evidence, FC_USB, PassivePowerEvidence, readonly_message
from trs_battery_readiness import PARAMS, parameters_match
from trs_uart_readonly_check import UID2
from rigid_body_revision import inspect_current_calibration, digest
from actuator_query import ActuatorQuery
from preflight_session_status import describe as describe_session
from readiness_console import LatestStatusPrinter, StableReadiness, render as render_readiness
from live_attitude_check import LiveAttitudeCheck
from mavlink_crc_stream import CrcResyncParser
from trs_transport_config import add_baud_argument, require_baud
from mission_live_inputs import LiveMissionInputs
from flight_mission_core import MissionCore
import mission_audit_collector as audit_policy
from mission_telemetry_rates import TemporaryTelemetryRates


def allowed(msg, *, identity_fresh=False, home_latched=False, audit_shadow=False):
    """A live transport whitelist, not advisory mission intents."""
    kind = msg.get_type()
    if kind == 'PARAM_REQUEST_READ':
        name = msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id, bytes) else msg.param_id
        return bool(identity_fresh and name in PARAMS and
                    (msg.target_system, msg.target_component, msg.param_index) == (1, 1, -1))
    if kind == 'SERIAL_CONTROL':
        return bool(identity_fresh and (shell_message_allowed(msg)
                    or audit_shadow and audit_policy.allowed(msg)))
    if stop_message_allowed(msg, home_latched, True):
        return bool(identity_fresh)
    return readonly_message(msg)


def observation_age(now, last):
    return now-last if last is not None and now >= last else None


def observed_rate(times):
    return ((len(times)-1)/(times[-1]-times[0])
            if len(times) >= 2 and times[-1] > times[0] else None)


def gamepad_identity(path):
    """eventN and rdev can be reused after a USB reconnect; bind this generation."""
    stat=os.stat(path)
    sysfs=Path('/sys/class/input')/Path(path).name/'device'
    return {'rdev':stat.st_rdev,'inode':stat.st_ino,
            'sysfs_device':str(sysfs.resolve(strict=True))}


def require_gamepad_identity(fd,path,expected):
    opened=os.fstat(fd)
    if ((opened.st_rdev,opened.st_ino)!=(expected['rdev'],expected['inode'])
            or gamepad_identity(path)!=expected):
        raise RuntimeError('Gamepad device changed or re-enumerated; new session required')


def readiness_fault(previous, *, baseline_seen, joint_seen, identity_fresh, source_fresh):
    """Do not silently resume readiness after a transient; keep diagnosis alive."""
    if previous is not None:return previous
    if baseline_seen and not identity_fresh:return 'fc_heartbeat_stale_after_ready'
    if joint_seen and not source_fresh:return 'vicon_source_lost_after_ready'
    return None


def home_observation_fault(now, request_at, verified):
    # A failed diagnostic is not a reason to stop listening for HOME. This
    # fault is latched by the runner, never converted to permission to fly.
    if request_at is not None and not verified and now-request_at > 10:
        return 'home_lock_confirmation_timeout'
    return None


def query_readiness_fault(query, tolerate_isolated_frames=False):
    """Only a recoverable damaged reply may wait for an entirely new proof.

    No fault/latch is cleared. Unknown state remains unknown during recovery;
    missing boundaries, exhausted retries, timeouts and malformed/stale state
    still block. This is a DISARMED diagnostic policy, not a flight policy.
    """
    if isinstance(query,audit_policy.MissionAuditCollector):
        # Separate fixed-audit collector always discards/drains/retries at most
        # twice per command. A clean retry is not weakened/strengthened by the
        # actuator-only policy switch; missing boundary/budget still latches.
        return query.fault
    if not tolerate_isolated_frames or query.disabled:
        return query.fault
    return next((row['reason'] for row in query.failures
                 if row['reason'] != 'actuator_reply_damaged_packet'), None)


def check_physical(mode):
    if (mode == 'usb-only') != FC_USB.exists():
        raise RuntimeError('Physical mode differs from FC USB presence; no automatic reconnect/changeover')


def inspect_evidence(heading, board, proof_path):
    def retained_path(value):
        # Historical reports were produced from the drone workspace. Their
        # internal relative references are NOT relative to a launch terminal.
        path=Path(value)
        return path if path.is_absolute() else Path(__file__).resolve().parents[2]/path
    registry = inspect_current_calibration(heading, board)
    proof = json.loads(Path(proof_path).read_text())
    permit_path = retained_path(proof['vision_reference_permit_path'])
    permit = json.loads(permit_path.read_text())
    consumed = json.loads(Path(str(permit_path)+'.consumed.json').read_text())
    if (proof.get('rigid_body_revision_id') != registry['active_revision_id']
            or proof.get('rigid_revision_static_validation') is not True
            or proof.get('ekf_stationary_xyz_yaw_fusion_verified') is not True
            or proof.get('error') or proof.get('restore_errors')
            or proof.get('arming_mode_motion_or_unlock_commands_sent') is not False
            or proof.get('restored') != {'EKF2_EV_CTRL':True,'EKF2_EV_DELAY':True}
            or proof.get('actuator_after', {}).get('armed') is not False
            or proof.get('actuator_after', {}).get('lockdown') is not True
            or any(proof.get('identities',{}).get(route,{}).get('uid2') != UID2 for route in ('usb','trs'))
            or retained_path(proof['heading_candidate_path']).resolve() != Path(heading).resolve()
            or retained_path(proof['board_center_candidate_path']).resolve() != Path(board).resolve()
            or retained_path(permit['output_path']).resolve() != Path(proof_path).resolve()
            or permit.get('heading_sha256') != digest(heading)
            or permit.get('board_sha256') != digest(board)
            or consumed.get('permit_sha256') != digest(permit_path)):
        raise ValueError('Same-revision passing static proof and consumed candidate-bound permit required')
    return registry


def run(args):
    baud=require_baud(getattr(args,'baud',None))
    check_physical(args.physical_mode)
    if not args.props_installed_monitor_stop_only_confirmed:
        raise ValueError('Operator-confirmed stationary physical mode required; never a flight authorization')
    ground_age=getattr(args,'ground_status_max_age',.5)
    if (type(ground_age) not in (int,float) or ground_age not in (.5,1.)
            or ground_age>.5 and (args.physical_mode!='usb-only'
                                  or not getattr(args,'mission_audit_shadow',False))):
        raise ValueError('1s ground status tolerance requires USB-only fixed-audit monitoring')
    rates_enabled=getattr(args,'mission_telemetry_rates',False)
    recovery=getattr(args,'rates_recovery_journal',None)
    if rates_enabled and (args.physical_mode!='usb-only' or not getattr(args,'mission_audit_shadow',False)
                          or recovery is None or recovery.exists()
                          or Path(str(recovery)+'.pending').exists()):
        raise ValueError('Temporary rates require USB-only fixed-audit scope and NEW recovery journal')
    for path in (args.output, args.status_log, args.latch_file):
        if path.exists():
            raise ValueError('New paths required; retaining old reports and stop latches')
    registry = inspect_evidence(args.heading, args.board, args.static_proof)
    h, b = [json.loads(Path(p).read_text()) for p in (args.heading, args.board)]
    if subprocess.run(['fuser',str(Path(args.trs_port).resolve())],capture_output=True).returncode == 0:
        raise ValueError('TRS already occupied; no process killed')
    device = find_pad()
    dec=CrcResyncParser()  # Validate the pinned RX schema before opening any FC transport.
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros2_ws/src/uav_coord_bridge'))
    from uav_coord_bridge.mavlink_vision import PoseGate, transform_rigid_pose
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped

    class Source(Node):
        def __init__(self):
            super().__init__('single_trs_runtime_monitor')
            self.gate = PoseGate(max_age_ms=150)
            self.subscription_kept = self.create_subscription(PoseStamped,
                '/vicon/MircoAir/MircoAir', self.receive,
                QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE))

        def receive(self,msg):
            p,q = msg.pose.position,msg.pose.orientation
            self.gate.ingest(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                self.get_clock().now().nanoseconds,time.monotonic_ns(),msg.header.frame_id,
                [p.x,p.y,p.z],[q.x,q.y,q.z,q.w])

    rclpy.init(args=[])
    node = Source()
    ev,power = Evidence(250,202,expected_parameters=tuple(PARAMS)), PassivePowerEvidence()
    enc=m.MAVLink(None,srcSystem=250,srcComponent=202)
    latch=DurableStopLatch(args.latch_file,UID2)
    start=time.monotonic()
    tx,commands=Counter(),Counter()
    snapshots,acks,events,statustext,local_times=[],[],[],[],[]
    last_attitude=last_local=last_estimator=last_land=None
    audit_enabled = getattr(args,'mission_audit_shadow',False)
    query=(audit_policy.MissionAuditCollector(ground_status_max_age=ground_age)
           if audit_enabled else ActuatorQuery())
    tolerate_isolated_frames=getattr(args,'tolerate_isolated_frames',False)
    attitude_check=LiveAttitudeCheck();calibration_current=True
    calibration_errors=[];next_calibration_check=0.
    shell_fragments=[]
    next_hb=next_version=next_params=next_status=0;param_cursor=0
    next_retry=0;attempts=0;request_at=release_at=None
    ready=joint_ready=lock_after_home=False
    pad_open=False
    pad_identity=None
    error=None;failure_latched=None;first_fault_context=None;last_status=None
    max_loop_gap=0.;previous_loop=None;bad_packets=[]
    until_ready=getattr(args,'until_ready',False)
    console=None;completion=StableReadiness();completed=False;completion_snapshot=None
    next_completion_check=next_console=0.
    shadow = (LiveMissionInputs(registry['active_revision_id'])
              if getattr(args, 'mission_input_shadow', False) or audit_enabled else None)
    # Even a complete receive-side observation can only PREPARE this core.
    # There is no start/cycle/vision/flight transport call in the monitor.
    def forbidden_shadow_send(message):
        raise PermissionError('Input shadow may not dispatch mission messages')
    shadow_core = MissionCore(forbidden_shadow_send) if shadow else None

    def persist_rates(record):
        pending=Path(str(recovery)+'.pending')
        with pending.open('w') as stream:
            json.dump(record,stream,ensure_ascii=False,indent=2,allow_nan=False)
            stream.flush();os.fsync(stream.fileno())
        pending.replace(recovery)
    rates=TemporaryTelemetryRates(persist=persist_rates) if rates_enabled else None

    def feed_shadow(method, *values, **options):
        if shadow is None:
            return
        if method=='observe_actuator' and ground_age>.5:
            snapshot,now=values
            # A tolerated ground read must never poison or satisfy strict
            # mission inputs. Its original timestamp remains unchanged.
            age=now-snapshot['at']
            if not 0<=age<=.5 or snapshot['fc_age_at_receive_s']+age>.5:
                return
        try:
            getattr(shadow, method)(*values, **options)
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            # Diagnostic adapter failure never preempts existing HOME service.
            shadow.fail('input_shadow_invalid: '+str(exc))

    def strict_lockdown(now):
        return (query.live_lockdown(now,max_age=.5) if audit_enabled
                else query.live_lockdown(now))

    def service_shadow(t):
        if shadow is None:
            return
        if node.gate.fresh(node.get_clock().now().nanoseconds,time.monotonic_ns()):
            feed_shadow('observe_pose', node.gate.last, node.get_clock().now().nanoseconds,
                        time.monotonic_ns(), t)
        try:
            observation, missing = shadow.build(t)
            if observation is not None and shadow_core.phase == 'PREPARE':
                shadow_core.prepare(observation, shadow.selector)
        except (ValueError, TypeError, KeyError, RuntimeError) as exc:
            shadow.fail('input_shadow_preparation_invalid: '+str(exc))

    def elapsed():return time.monotonic()-start

    def identity_fresh():
        return bool(ev.identity and ev.identity['uid2']==UID2 and ev.identity['firmware']==FIRMWARE
            and not ev.fatal and not power.reboot_observed and ev.heartbeats
            and not ev.heartbeats[-1]['armed'] and 0<=elapsed()-ev.heartbeats[-1]['elapsed_s']<2)

    def send(port,msg):
        accepted=allowed(msg,identity_fresh=identity_fresh(),home_latched=latch.latched,
                         audit_shadow=audit_enabled)
        if rates is not None:
            accepted |= rates.permits(msg,fresh=identity_fresh(),locked=strict_lockdown(elapsed()) is True)
        if not accepted:
            raise RuntimeError('Monitor-stop-only transport gate rejected '+msg.get_type())
        packet=msg.pack(enc)
        if port.write(packet)!=len(packet):raise OSError('Short serial write')
        enc.seq=(enc.seq+1)%256;tx[msg.get_type()]+=1
        if msg.get_type()=='COMMAND_LONG':commands[str(msg.command)]+=1

    def record_fault(reason):
        nonlocal failure_latched,first_fault_context
        if reason is not None and failure_latched is None:
            failure_latched=reason
            if shadow is not None:
                shadow.fail('monitor_fault: '+reason)
            first_fault_context={'at':elapsed(),'reason':reason,
                'heartbeat_age_s':observation_age(elapsed(),ev.heartbeats[-1]['elapsed_s'] if ev.heartbeats else None),
                'recent_heartbeats':list(ev.heartbeats[-4:]),
                'local_position_age_s':observation_age(elapsed(),last_local['at'] if last_local else None),
                'received_bad_data_count':ev.counts['BAD_DATA'],
                'last_retained_bad_packet_example':bad_packets[-1] if bad_packets else None,
                'max_reactor_loop_gap_s':max_loop_gap,
                'scope':'Receive-side context, not proof of RF loss or FC failure'}
            latch.fail(reason)
            print('DEGRADED: '+reason+'; HOME stop-only monitoring continues, no flight readiness.',flush=True)

    def status():
        t=elapsed();fresh=node.gate.fresh(node.get_clock().now().nanoseconds,time.monotonic_ns())
        position=None
        if fresh:
            sample=node.gate.last
            position,_=transform_rigid_pose(sample.position,sample.quaternion,
                b['board_center_in_rigid_m_candidate'],h['body_orientation_in_rigid_xyzw_candidate'])
        local_age=observation_age(t,last_local['at'] if last_local else None)
        hb_times=[row['elapsed_s'] for row in ev.heartbeats]
        result={'elapsed_s':round(t,3),'identity_fresh_disarmed':identity_fresh(),
            'trs_port':str(args.trs_port),'trs_baud':baud,
            'heartbeat_age_s':observation_age(t,hb_times[-1] if hb_times else None),
            'heartbeat_observed_rate_hz':observed_rate(hb_times),
            'heartbeat_max_arrival_gap_s':max((b-a for a,b in zip(hb_times,hb_times[1:])),default=None),
            'vicon_fresh':fresh,'vicon_accepted':node.gate.accepted,'vicon_rejected':node.gate.rejected,
            'live_lockdown':query.live_lockdown(t) if identity_fresh() else None,
            'ground_status_max_age_s':ground_age,
            'ground_status_tolerance_scope':'USB-only disarmed monitor; NOT mission/flight freshness',
            'strict_500ms_lockdown':strict_lockdown(t) if identity_fresh() else None,
            'actuator_observation_age_s':observation_age(t,snapshots[-1]['at'] if snapshots else None),
            'actuator_query_pending':query.pending_at is not None,
            'actuator_query_draining':query.draining,'actuator_queries_disabled':query.disabled,
            'actuator_query_failures':len(query.failures),
            'tolerate_isolated_frames':tolerate_isolated_frames,
            'actuator_query_recovering':bool(tolerate_isolated_frames and query.failures
                and query.live_lockdown(t) is None and not query.disabled),
            'stop_latched':latch.latched,'home_press_observed':request_at is not None,
            'home_release_lock_verified':lock_after_home,
            'home_lock_currently_confirmed':bool(lock_after_home and identity_fresh()
                                                and query.live_lockdown(t) is True),
            'home_input_monitor_running':pad_open,
            'last_home_event_age_s':observation_age(t,events[-1]['at'] if events else None),
            'gamepad_fd_present_not_awake_proof':pad_open,'sleep_detection_present':False,
            'local_position_age_s':local_age,'local_position_rate_hz':observed_rate(local_times),
            'local_position_within_500ms':local_age is not None and local_age<=.5,
            'estimator_status_age_s':observation_age(t,last_estimator['at'] if last_estimator else None),
            'estimator_flags':last_estimator['flags'] if last_estimator else None,
            'landed_state':last_land['state'] if last_land else None,
            'parameters_match_maintenance_baseline':parameters_match(ev.parameters),
            'calibration_current':calibration_current,
            'attitude_consistency_observed':attitude_check.summary(t)['observed'],
            'attitude_consistency':attitude_check.summary(t),
            'preview_approximate_board_position_frd_m':position,
            'preview_is_not_a_control_mapping':True,'monitor_fault_latched':failure_latched,
            'first_fault_context':first_fault_context,
            'physical_mode':args.physical_mode,'fc_usb_opened_by_this_program':False,
            'pose_messages_sent':0,'motion_targets_sent':0,'parameter_writes_sent':0,
            'flight_readiness_approved':False}
        result['session']=describe_session({**result,
            'actuator_baseline_seen':ready,
            'live_lockdown_confirmed':result['live_lockdown'] is True},
            fault=failure_latched or error,running=pad_open)
        if shadow is not None:
            result['mission_input_shadow'] = shadow.describe(t)
            result['mission_input_shadow']['core_phase'] = shadow_core.phase
            result['mission_input_shadow']['mission_messages_sent'] = len(shadow_core.sent)
            result['mission_input_shadow']['receive_owner_running'] = pad_open
            result['mission_input_shadow']['full_audit_producer_attached'] = audit_enabled
            if audit_enabled:
                result['mission_input_shadow']['fixed_audit_reads'] = query.reads
                result['mission_input_shadow']['rejected_audit_rows'] = query.rejected_rows[-10:]
                result['mission_input_shadow']['unavailable_aid_slots'] = query.unavailable_aid_slots
                result['mission_input_shadow']['noncurrent_actuator_reads'] = query.noncurrent_actuator_reads[-10:]
        if rates is not None:result['temporary_telemetry_rates']=rates.summary()
        return result

    def read_home(fd,t):
        nonlocal pending_input,request_at,release_at,next_retry
        try:
            chunk=os.read(fd,EVENT.size*64)
            if not chunk:raise OSError('Gamepad disconnected')
            pending_input+=chunk
        except BlockingIOError:pass
        while len(pending_input)>=EVENT.size:
            _,_,kind,code,value=EVENT.unpack(pending_input[:EVENT.size]);pending_input=pending_input[EVENT.size:]
            if kind==0 and code==3:raise RuntimeError('Gamepad event queue overflow')
            if kind in (1,3):
                feed_shadow('observe_gamepad',pad_identity,True,t,event=True,
                            home=(value != 0) if kind==1 and code==HOME else None)
            if kind==1 and code==HOME:
                # Even an early press is latched. Transmission still waits
                # for the exact same UID/firmware and fresh DISARMED heartbeat.
                latch.event(kind,code,value);events.append({'at':t,'value':value})
                if value in (1,2) and request_at is None:
                    request_at=t;next_retry=t
                elif value==0 and request_at is not None:release_at=t

    def service_home(port,t):
        nonlocal attempts,next_retry
        if request_at is not None and not acks and t>=next_retry and attempts<3 and identity_fresh():
            send(port,lock_message(enc,ev.identity,UID2,latch,True,identity_fresh(),confirmation=attempts))
            attempts+=1;next_retry=t+.8

    def restore_rates(port,fd):
        """Runs BEFORE the ExitStack closes this same serial/input owner.

        No second descriptor or process. Keep HOME + strict CRC + fixed
        diagnostics alive; restore only under fresh same-FC locked proof.
        """
        nonlocal next_hb
        if rates is None:return
        rates.begin_restore(elapsed())
        deadline=elapsed()+max(6,len(rates.changed)*7+3)
        try:
            while rates.phase not in ('CLOSED','RESTORE_FAILED') and elapsed()<deadline:
                t=elapsed();check_physical(args.physical_mode)
                try:
                    require_gamepad_identity(fd,device['path'],pad_identity)
                    read_home(fd,t)
                except (OSError,RuntimeError) as exc:
                    record_fault('restore_input_unavailable: '+str(exc))
                service_home(port,t)
                for msg in dec.parse_buffer(port.read(4096)) or []:
                    ev.ingest(msg,t);power.ingest(msg,t);rates.ingest(msg,t)
                    if msg.get_type()=='BAD_DATA':query.damaged_packet(t)
                    if (msg.get_srcSystem(),msg.get_srcComponent())!=(1,1):continue
                    if msg.get_type()=='SERIAL_CONTROL':
                        sample=power.boot_samples[-1] if power.boot_samples else None
                        boot=(int(sample['time_boot_ms']+max(0,t-sample['elapsed_s'])*1000)
                              if sample else None)
                        snap=query.receive(bytes(msg.data[:msg.count]).decode('utf8','replace'),t,boot)
                        if snap:snapshots.append(snap)
                    elif msg.get_type()=='COMMAND_ACK' and msg.command==LOCK_COMMAND:
                        if (msg.target_system,msg.target_component)==(250,202):
                            acks.append({'at':t,'result':msg.result})
                if ev.fatal or power.reboot_observed:
                    raise RuntimeError(ev.fatal or 'FC reboot during rate restoration')
                service_home(port,t)
                if t>=next_hb:
                    send(port,enc.heartbeat_encode(m.MAV_TYPE_GCS,m.MAV_AUTOPILOT_INVALID,0,0,0));next_hb=t+1
                action=query.action(t,identity_fresh())
                if action is not None:send(port,audit_policy.message(enc,action))
                rate_now=elapsed()
                outgoing=rates.action(rate_now,fresh=identity_fresh(),
                                      locked=strict_lockdown(rate_now+.05) is True)
                if outgoing is not None:send(port,outgoing)
            if rates.phase not in ('CLOSED','RESTORE_FAILED'):
                rates.restoration_incomplete('bounded_restore_deadline_or_fresh_lock_unavailable')
        except (Exception,KeyboardInterrupt) as exc:
            rates.restoration_incomplete(str(exc) or 'restore_interrupted')

    try:
        with ExitStack() as stack:
            port=stack.enter_context(serial.Serial(args.trs_port,baud,timeout=.002,write_timeout=.3,exclusive=True))
            log=stack.enter_context(args.status_log.open('x'))
            fd=os.open(device['path'],os.O_RDONLY|os.O_NONBLOCK);stack.callback(os.close,fd)
            pad_identity=gamepad_identity(device['path'])
            require_gamepad_identity(fd,device['path'],pad_identity)
            pad_open=True
            feed_shadow('observe_gamepad',pad_identity,True,elapsed())
            bitmap=fcntl.ioctl(fd,0x80000000|(96<<16)|(ord('E')<<8)|0x18,bytes(96))
            if bitmap[HOME//8]&(1<<(HOME%8)):raise RuntimeError('Release HOME before monitor starts')
            pending_input=b''
            if rates is not None:stack.callback(restore_rates,port,fd)
            port.reset_input_buffer()
            if until_ready:console=LatestStatusPrinter()
            while elapsed()<args.seconds:
                t=elapsed();check_physical(args.physical_mode)
                if previous_loop is not None:max_loop_gap=max(max_loop_gap,t-previous_loop)
                previous_loop=t
                require_gamepad_identity(fd,device['path'],pad_identity)
                # HOME is serviced before callbacks/parsing that may fail.
                try:read_home(fd,t)
                finally:service_home(port,t)
                rclpy.spin_once(node,timeout_sec=0)
                for msg in dec.parse_buffer(port.read(4096)) or []:
                    ev.ingest(msg,t);power.ingest(msg,t)
                    if rates is not None:rates.ingest(msg,t)
                    feed_shadow('ingest',msg,t)
                    if msg.get_type()=='BAD_DATA' and len(bad_packets)<50:
                        bad_packets.append({'at':t,'reason':str(getattr(msg,'reason','')),
                            'data_hex':bytes(getattr(msg,'data',b''))[:96].hex()})
                    if msg.get_type()=='BAD_DATA':query.damaged_packet(t)
                    if (msg.get_srcSystem(),msg.get_srcComponent())!=(1,1):continue
                    kind=msg.get_type()
                    if kind=='ATTITUDE_QUATERNION':
                        last_attitude={'at':t,'boot_ms':msg.time_boot_ms}
                        if node.gate.fresh(node.get_clock().now().nanoseconds,time.monotonic_ns()):
                            sample=node.gate.last
                            try:
                                _,source_q=transform_rigid_pose(sample.position,sample.quaternion,
                                    b['board_center_in_rigid_m_candidate'],h['body_orientation_in_rigid_xyzw_candidate'])
                                attitude_check.observe(source_q,[msg.q1,msg.q2,msg.q3,msg.q4],t,int(msg.time_boot_ms))
                            except (ValueError,TypeError) as exc:
                                record_fault('attitude_comparison_invalid: '+str(exc))
                            record_fault(attitude_check.fault)
                    elif kind=='LOCAL_POSITION_NED':
                        values=[msg.x,msg.y,msg.z,msg.vx,msg.vy,msg.vz]
                        if not all(math.isfinite(x) for x in values):raise RuntimeError('Non-finite local telemetry')
                        last_local={'at':t,'boot_ms':msg.time_boot_ms,'position':values[:3],'velocity':values[3:]}
                        local_times.append(t)
                    elif kind=='ESTIMATOR_STATUS':last_estimator={'at':t,'flags':msg.flags}
                    elif kind=='EXTENDED_SYS_STATE':last_land={'at':t,'state':msg.landed_state}
                    elif kind=='STATUSTEXT':statustext.append({'at':t,'text':msg.text})
                    elif kind=='COMMAND_ACK' and msg.command==LOCK_COMMAND and request_at is not None:
                        if (msg.target_system,msg.target_component)==(250,202):
                            acks.append({'at':t,'result':msg.result})
                            if msg.result!=m.MAV_RESULT_ACCEPTED:record_fault('home_output_lock_rejected')
                    elif kind=='SERIAL_CONTROL':
                        text=bytes(msg.data[:msg.count]).decode('utf8','replace')
                        shell_fragments.append({'at':t,'packet_seq':msg.get_seq(),'text':text})
                        shell_fragments=shell_fragments[-100:]
                        boot=(int(last_attitude['boot_ms']+max(0,t-last_attitude['at'])*1000)
                              if last_attitude and audit_enabled else
                              last_attitude['boot_ms'] if last_attitude else None)
                        snapshot=query.receive(text,t,boot)
                        if snapshot:
                            snapshots.append(snapshot)
                            feed_shadow('observe_actuator',snapshot,t)
                            if not ready:
                                ready=True
                                print('READY: 单TRS常驻监控已就绪；HOME可短按，仅锁定未解锁输出；松开后继续监控。',flush=True)
                            if (any(a['result']==m.MAV_RESULT_ACCEPTED for a in acks)
                                    and release_at is not None and snapshot['requested_at']>=release_at+1.5
                                    and snapshot['lockdown'] and not lock_after_home):
                                lock_after_home=True
                                print('HOME PASS: TRS ACK接受，松开后内部lockdown=True；常驻监控继续运行。',flush=True)
                        if audit_enabled:
                            for audit in query.take_audits():
                                feed_shadow('observe_audit',audit['row'],audit['commander'],
                                            requested_at=audit['requested_at'],now=audit['now'])
                if ev.fatal or power.reboot_observed:raise RuntimeError(ev.fatal or 'FC reboot; session invalidated')
                if ev.identity and (ev.identity['uid2']!=UID2 or ev.identity['firmware']!=FIRMWARE):
                    raise RuntimeError('Unexpected FC identity')
                # A monitor may continue receiving after a stale heartbeat,
                # but the send gate still requires <2s heartbeat age. The
                # readiness fault stays latched after recovery; never auto-arm.
                source_fresh=node.gate.fresh(node.get_clock().now().nanoseconds,time.monotonic_ns())
                fault=readiness_fault(failure_latched,baseline_seen=ready,joint_seen=joint_ready,
                    identity_fresh=identity_fresh(),source_fresh=source_fresh)
                record_fault(fault)
                # Real HOME stop is ahead of diagnostics; never clear the lock.
                service_home(port,t)
                record_fault(home_observation_fault(t,request_at,lock_after_home))
                if calibration_current and t>=next_calibration_check:
                    try:
                        current=inspect_evidence(args.heading,args.board,args.static_proof)
                        if current['active_revision_id']!=registry['active_revision_id']:
                            raise ValueError('rigid_body_revision_changed')
                    except (OSError,ValueError,KeyError) as exc:
                        calibration_current=False
                        calibration_errors.append({'at':t,'error':str(exc)})
                        record_fault('calibration_changed_during_session')
                    next_calibration_check=t+2
                if t>=next_hb:
                    send(port,enc.heartbeat_encode(m.MAV_TYPE_GCS,m.MAV_AUTOPILOT_INVALID,0,0,0));next_hb=t+1
                if ev.heartbeats and ev.identity is None and t>=next_version:
                    ev.version_requested_at=t
                    feed_shadow('request_identity',t)
                    send(port,enc.command_long_encode(1,1,512,0,148,0,0,0,0,0,0));next_version=t+3
                if identity_fresh() and t>=next_params:
                    name=tuple(PARAMS)[param_cursor];param_cursor=(param_cursor+1)%len(PARAMS)
                    ev.requests[name]=t;send(port,enc.param_request_read_encode(1,1,name.encode(),-1))
                    next_params=t+(.1 if param_cursor else 30)
                if any(k in ev.parameters and ev.parameters[k]!=v for k,v in PARAMS.items()):
                    raise RuntimeError('Maintenance configuration changed; monitor will not correct it')
                action=query.action(t,identity_fresh())
                record_fault(query_readiness_fault(query,tolerate_isolated_frames))
                if action is not None:
                    send(port,audit_policy.message(enc,action) if audit_enabled
                         else shell_message(enc,poll=action=='poll'))
                if rates is not None:
                    rate_now=elapsed()
                    outgoing=rates.action(rate_now,fresh=identity_fresh(),
                                          locked=strict_lockdown(rate_now+.05) is True)
                    if outgoing is not None:send(port,outgoing)
                    if rates.error:record_fault('temporary_telemetry_rates: '+rates.error)
                service_shadow(t)
                fresh=node.gate.fresh(node.get_clock().now().nanoseconds,time.monotonic_ns())
                if ready and parameters_match(ev.parameters) and fresh:joint_ready=True
                if until_ready and t>=next_completion_check:
                    current=status();next_completion_check=t+.1
                    if t>=next_console:
                        console.publish(render_readiness(current));next_console=t+2
                    if completion.observe(current,t) and (rates is None or rates.phase=='ACTIVE'):
                        completed=True;completion_snapshot=current
                        log.write(json.dumps(current,ensure_ascii=False)+'\n');log.flush()
                        break
                if t>=next_status:
                    last_status=status();log.write(json.dumps(last_status,ensure_ascii=False)+'\n');log.flush()
                    # Detailed telemetry is file-backed, not a potentially blocking
                    # terminal stream in the serial owner/control event loop.
                    next_status=t+5
                if rates is not None and rates.active_at is not None and t-rates.active_at>=10:
                    break  # Only the new combined audit window; old standalone trial is not repeated.
                if not ready and t>15 and not until_ready:raise RuntimeError('Initial TRS/actuator readiness timeout')
    except (Exception,KeyboardInterrupt) as exc:
        error=str(exc) or 'Operator interrupted monitor'
        latch.fail('monitor_ended: '+error)
    finally:
        if console is not None:console.close()
        if (tolerate_isolated_frames and query.failures
                and query.live_lockdown(elapsed()) is None):
            record_fault('actuator_reply_recovery_incomplete_at_exit')
        pad_open=False
        last_status=status()
        node.destroy_node()
        # rclpy's SIGINT handler may already have shut the context down.
        # A second shutdown must not prevent the final evidence report.
        if rclpy.ok():rclpy.shutdown()
    result={'scope':__doc__,'captured_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'trs_port':str(args.trs_port),'trs_baud':baud,
        'physical_mode':args.physical_mode,'propeller_state':'installed','physical_conditions_source':'operator report',
        'rigid_body_revision_id':registry['active_revision_id'],'duration_s':elapsed(),
        'device':device,'gamepad_generation_identity':pad_identity,
        'identity':ev.identity,'last_status':last_status,
        'snapshots':snapshots,'home_events':events,'home_acks':acks,'statustext':statustext,
        'passive_power':power.result(),'parameters':ev.parameters,'tx_messages':dict(tx),
        'command_counts':dict(commands),'local_position_samples':len(local_times),
        'heartbeats':ev.heartbeats,'received_fc_message_counts':dict(ev.fc_counts),
        'max_reactor_loop_gap_s':max_loop_gap,'bad_packet_examples':bad_packets,
        'first_fault_context':first_fault_context,'rx_parser':dec.summary(),
        'actuator_query_faults':query.failures,'actuator_queries_disabled':query.disabled,
        'tolerate_isolated_frames':tolerate_isolated_frames,
        'attitude_consistency':attitude_check.summary(elapsed()),
        'calibration_current':calibration_current,'calibration_errors':calibration_errors,
        'incomplete_shell_text':query.buffer,'last_shell_fragments':shell_fragments,
        'local_position_rate_hz':observed_rate(local_times),'home_release_lock_verified':lock_after_home,
        'fc_usb_opened_by_this_program':False,'pose_messages_sent':0,'motion_targets_sent':0,
        'parameter_writes_sent':0,'flight_adapter_enabled':False,'airborne_emergency_stop_verified':False,
        'continuous_monitor_inputs_healthy':bool(joint_ready and not error and not failure_latched),
        'until_ready':until_ready,'readiness_completed':completed,
        'completion_snapshot':completion_snapshot,
        'error':error,'serial_port_released':True,'monitor_running_after_exit':False}
    if shadow is not None:
        result['mission_input_shadow'] = last_status['mission_input_shadow']
    if rates is not None:
        result['temporary_telemetry_rates']=rates.summary()
        result['rates_recovery_journal']=str(recovery)
    with args.output.open('x') as stream:json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False);stream.write('\n')
    print('MONITOR EXIT: '+json.dumps({'error':error,'home_release_lock_verified':lock_after_home,
                                     'serial_port_released':True,'flight_adapter_enabled':False}),flush=True)
    if until_ready:
        print('检查完成：台架连接条件已连续满足1秒。程序已退出，HOME监听已停止；这不是起飞许可。'
              if completed else '检查未通过：超时、故障或中断。程序已退出，HOME监听已停止；不要解锁起飞。',flush=True)
    rate_pass=rates is None or (rates.summary()['restoration_verified'] and rates.summary()['measured_profile_verified'])
    return 0 if not error and not failure_latched and (not until_ready or completed) and rate_pass else 2


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    add_baud_argument(ap)
    for name in ('trs-port','heading','board','static-proof','output','status-log','latch-file'):
        ap.add_argument('--'+name,type=str if name=='trs-port' else Path,required=True)
    ap.add_argument('--physical-mode',choices=('usb-only','battery-only'),required=True)
    ap.add_argument('--props-installed-monitor-stop-only-confirmed',action='store_true',required=True)
    ap.add_argument('--seconds',type=float,default=600)
    ap.add_argument('--until-ready',action='store_true',
                    help='Print missing conditions and exit after 1s of complete monitor observations; never a flight authorization')
    ap.add_argument('--mission-input-shadow',action='store_true',
                    help='Feed current RX/Vicon/input observations into the new mission input adapter; no start, pose or flight dispatch')
    ap.add_argument('--mission-audit-shadow',action='store_true',
                    help='Use ONE nonblocking fixed-read EKF/Commander collector instead of the actuator-only query; no poses, rate/parameter writes or flight')
    ap.add_argument('--ground-status-max-age',type=float,choices=(.5,1.),default=.5,
                    help='USB-only fixed-audit ground monitoring: tolerate actuator age up to 1s; mission, fusion, Vicon and rate-write gates remain unchanged')
    ap.add_argument('--mission-telemetry-rates',action='store_true',
                    help='USB-only: temporarily apply measured 10/5/5Hz profile during fixed-audit observation; always journal and restore before serial closes')
    ap.add_argument('--rates-recovery-journal',type=Path,
                    help='NEW journal required for temporary rate lifecycle; retained for recovery if restoration fails')
    ap.add_argument('--tolerate-isolated-frames',action='store_true',
                    help='Disarmed diagnostics only: discard damaged reply and allow existing bounded clean retry; no CRC bypass or freshness relaxation')
    args=ap.parse_args()
    if not 10<=args.seconds<=3600:ap.error('Use a bounded 10..3600 second monitor session')
    try:return run(args)
    except (ValueError,OSError,RuntimeError,KeyError) as exc:ap.exit(2,str(exc)+'\n')


if __name__=='__main__':raise SystemExit(main())
