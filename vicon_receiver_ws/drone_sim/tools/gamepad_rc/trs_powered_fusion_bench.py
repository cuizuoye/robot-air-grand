#!/usr/bin/env python3
"""One bounded BATTERY-ONLY, output-LOCKED static fusion check; NOT flight.

One serial owner handles HOME, FC state, poses, diagnostics and rollback.
Damaged diagnostic epochs are discarded and drained before one clean retry.
HOME remains serviced during rollback and the default 30s stop-only watch;
after process exit there is explicitly no resident HOME protection.
No arm/unlock/mode. Targets require a distinct frozen-FC ground-load scope.
Same-link restoration is best effort and verified
explicitly; failure never becomes successful rollback or automatic restart.
"""
import argparse
from collections import Counter, deque
from dataclasses import asdict
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
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.mavlink_vision import PoseGate, transform_rigid_pose, checked_quaternion
from estop_only_policy import DurableStopLatch, lock_message, parse_actuator_snapshot
from gamepad_estop_monitor import EVENT, HOME, find_pad
from mavlink_readonly_probe import Evidence, PassivePowerEvidence, FC_USB, decode_parameter
from diagnostic_transaction import DiagnosticTransaction
from parameter_snapshot import ParameterSnapshot
from trs_only_estop_bench import state_valid
from trs_static_fusion_bench import load_backup
from trs_uart_readonly_check import UID2
from mavlink_crc_stream import CrcResyncParser
from trs_transport_config import add_baud_argument, require_baud
from usb_vision_policy import parse_visual_snapshot
from usb_fusion_policy import aid_topic_for_estimator
from vicon_fc_attitude_check import attitude_comparison
from ground_estimator_settlement import GroundEstimatorSettlement, require_frozen_selector
from ekf_compare_readonly_policy import parse_selector
import compact_vision as compact
import usb_combined_fusion_policy as combined
import trs_vision_reference_policy as vision
import trs_powered_fusion_policy as policy
import trs_powered_fusion_permit as permit_policy
import one_home_preflight_policy as one_home_policy
from trs_runtime_monitor import gamepad_identity,require_gamepad_identity
from static_lock_gate import StaticLockGate
from short_status_transport import ShortStatusTransport, RateTransactionGroup
from fusion_binary_receiver import FusionBinaryReceiver
from preparation_authorization import RateRecoveryJournal, load_fixed_backup
import ekf_static_profile as fixed_profile
import low_bandwidth_vision as low_vision
import frozen_target_stream as frozen
from disarmed_offboard_policy import GroundObservation, restorable_mode

# Legacy offline harness switch. Production fixed entry uses its own reviewed
# permit consumer, never changes this global or accepts the old draft scope.
FIXED_GROUND_LIVE_ENABLED = False


def boot_reference_matches(boot_s, reference_boot_s, reference_utc, utc_now):
    return bool(all(math.isfinite(x) for x in (boot_s,reference_boot_s,reference_utc,utc_now))
                and utc_now>=reference_utc and boot_s>=reference_boot_s
                and abs((boot_s-reference_boot_s)-(utc_now-reference_utc))<=4)


class ReceiveDiagnosticComplete(Exception):
    """Internal bounded diagnostic completion, not fusion success."""


def _write_owner_packet(port, packet, *, recording=None, original=None,
                        observation=None, clock=None, wire_decoder=None,
                        source_stamp_ns=None, source_frame_id=None, noise_parameters=None):
    """Existing owner's ONE write boundary; never reads/opens/closes a port.

    Without recording this is exactly the old write/short-write check. The
    optional maintenance callbacks are exercised ONLY by injected offline
    tests at present: no CLI/normal run constructs a recording session here.
    An external reviewed host, current physical authorization and the existing
    sender guards remain necessary; this function is not a hardware permit.
    No retry, second reader, mode/parameter recovery or logger action is added.
    """
    if recording is None:
        if port.write(packet) != len(packet):
            raise OSError('Short TRS serial write')
        return len(packet)
    from ev_delay_maintenance import MaintenanceRecordingSession
    if type(recording) is not MaintenanceRecordingSession:
        raise TypeError('Exact maintenance recording session required')
    try:
        if not callable(clock) or not callable(observation) or type(wire_decoder) is not CrcResyncParser:
            raise ValueError('Owner clock, observation and existing strict TX decoder required')
        if type(packet) is not bytes or bytes(original.get_msgbuf()) != packet or wire_decoder.buffer:
            raise ValueError('Exact current packed frame and empty TX validation buffer required')
        decoded = wire_decoder.parse_buffer(packet)
        if (len(decoded) != 1 or decoded[0].get_type() == 'BAD_DATA' or wire_decoder.buffer
                or (decoded[0].get_srcSystem(), decoded[0].get_srcComponent()) != recording.gate.lock.clock.target
                or recording._wire_fields(decoded[0].to_dict()) != recording._wire_fields(original.to_dict())):
            raise ValueError('One exact CRC-valid current owner message required')
        if original.get_type() == 'VISION_POSITION_ESTIMATE' and (
                type(source_stamp_ns) is not int or source_stamp_ns <= 0
                or not isinstance(source_frame_id, str) or not source_frame_id):
            raise ValueError('Exact source header required before any pose write')
        before = clock()
        obs = observation(before)
        if (type(before) is not int or obs.now_ns != before
                or not recording.before_tx(original, obs, noise_parameters=noise_parameters)):
            raise PermissionError(recording.gate.fault or 'Maintenance TX proof failed')
        tx_start = clock()
        if type(tx_start) is not int or tx_start < before:
            raise ValueError('Actual TX start clock invalid')
        prewrite = observation(tx_start)
        if prewrite.now_ns != tx_start or not recording.gate.check(prewrite):
            raise PermissionError(recording.gate.fault or 'Maintenance proof expired before write')
        written = port.write(packet)  # Sole writer, no fallback or replay.
        if type(written) is not int or written != len(packet):
            raise OSError('Short TRS serial write; no automatic retry')
        tx_end = clock()
        callback = clock()
        after = observation(callback)
        if not recording.after_tx(decoded[0], after, tx_start_ns=tx_start,
                tx_end_ns=tx_end, callback_ns=callback,
                source_stamp_ns=source_stamp_ns, source_frame_id=source_frame_id):
            raise RuntimeError(recording.gate.fault or 'Maintenance post-write record failed')
        return written
    except BaseException:
        recording.abort('owner_write_or_observation_failed_delivery_not_retried')
        raise


def run(a):
    if getattr(a, 'maintenance_recording', None) is not None:
        raise PermissionError('Maintenance callback integration is offline-only; no live motion runner or legacy fallback')
    fixed_path=getattr(a,'fixed_ekf_profile',None)
    fixed_config=fixed_path is not None
    fixed_live=getattr(a,'fixed_ground_live',False) is True
    if fixed_live and not fixed_config:
        raise PermissionError('Fixed live entry requires a fixed profile; no legacy fallback')
    if fixed_config and not (fixed_live or FIXED_GROUND_LIVE_ENABLED):
        raise PermissionError('Plan B fixed ground candidate is offline-only; no serial opened, no legacy fallback')
    if fixed_live and not getattr(a,'permit',None):
        raise PermissionError('Dedicated fixed ground permit required before any device access')
    approved_profile=fixed_profile.load_json(fixed_path) if fixed_config else None
    unified=getattr(a,'one_home_preflight',False) is True
    receive_only=getattr(a,'receive_rate_only',False) is True
    compact_ground=getattr(a,'compact_ground_fusion',False) is True
    short_status=getattr(a,'short_status',False) is True
    binary_diagnostics=getattr(a,'binary_diagnostics',False) is True
    low_bandwidth_vision=getattr(a,'low_bandwidth_vision',False) is True
    frozen_target_stream=getattr(a,'frozen_target_stream',False) is True
    if frozen_target_stream and not low_bandwidth_vision:raise ValueError('Frozen target requires low bandwidth ground scope')
    if low_bandwidth_vision and not binary_diagnostics:raise ValueError('Low bandwidth requires binary ground scope')
    if binary_diagnostics and not short_status:raise ValueError('Binary diagnostics requires short-status ground scope')
    if short_status and not (unified and compact_ground):raise ValueError('Short status needs distinct compact one-HOME scope')
    if (receive_only or compact_ground) and not unified:raise ValueError('Ground diagnostic needs distinct one-HOME scope')
    if receive_only and compact_ground:raise ValueError('Distinct diagnostic scopes cannot overlap')
    if fixed_config and (not (unified and compact_ground and short_status and binary_diagnostics)
                         or receive_only or frozen_target_stream):
        raise ValueError('Fixed candidate requires HOME/short-clock/binary locked-ground scope; no motion targets')
    if fixed_live and not low_bandwidth_vision:
        raise ValueError('Fixed live scope requires the reviewed low-bandwidth encoding')
    read_params=fixed_profile.READ_PARAMS if fixed_config else policy.PARAMS
    home_wait_seconds=getattr(a,'home_wait_seconds',90.)
    if type(home_wait_seconds) not in (float,int) or not math.isfinite(home_wait_seconds) or not 10<=home_wait_seconds<=300:
        raise ValueError('HOME waiting budget must be 10..300 seconds')
    startup=one_home_policy.StartupHome() if unified else None
    watch_seconds=getattr(a,'home_watch_seconds',30.)
    if not math.isfinite(watch_seconds) or not 0<=watch_seconds<=120:
        raise ValueError('Bounded HOME watch must be 0..120 seconds')
    if not a.powered_locked_static_confirmed or FC_USB.exists() or a.output.exists():
        raise ValueError('Separate guarded battery-only confirmation, USB absent and new output required')
    if Path(str(a.output)+'.home.latch.json').exists():
        raise ValueError('Retained stop latch: new explicitly reviewed session/output required, no automatic resume')
    baud=require_baud(getattr(a,'baud',None))
    dec=CrcResyncParser()  # Validate pinned CRC definitions before consuming a permit or opening serial.
    occupied=subprocess.run(['fuser',str(Path(a.trs_port).resolve())],capture_output=True).returncode
    if occupied==0:
        raise ValueError('TRS occupied; no owner will be killed')
    if fixed_live and occupied!=1:
        raise ValueError('Cannot establish exclusive fixed-ground TRS ownership')
    expected_fixed_scope=one_home_policy.FIXED_SCOPE
    if fixed_live:
        import fixed_ground_entry
        expected_fixed_scope=fixed_ground_entry.SCOPE
        permit=fixed_ground_entry.consume(a.permit,backup=a.backup,output=a.output,
            heading=a.heading,board=a.board,baud=baud,profile=fixed_path,trs_port=a.trs_port)
    else:
        permit=(one_home_policy if unified else permit_policy).consume(a.permit,backup=a.backup,output=a.output,heading=a.heading,board=a.board,baud=baud)
    if fixed_config and (permit.get('scope')!=expected_fixed_scope
            or permit.get('profile_sha256')!=fixed_profile.fingerprint(approved_profile)):
        raise PermissionError('Fixed EKF mode cannot use a legacy or differently bound ground permit')
    if bool(permit.get('receive_rate_only',False))!=receive_only:
        raise ValueError('Receive diagnostic mode/permit mismatch')
    if bool(permit.get('compact_ground_fusion',False))!=compact_ground:
        raise ValueError('Compact ground mode/permit mismatch')
    if bool(permit.get('short_status',False))!=short_status:
        raise ValueError('Short-status mode/permit mismatch')
    if bool(permit.get('binary_diagnostics',False))!=binary_diagnostics:
        raise ValueError('Binary-diagnostics mode/permit mismatch')
    if bool(permit.get('low_bandwidth_vision',False))!=low_bandwidth_vision:
        raise ValueError('Low-bandwidth mode/permit mismatch')
    if (bool(permit.get('frozen_target_stream',False))!=frozen_target_stream
            or frozen_target_stream and permit.get('scope')!=one_home_policy.FROZEN_TARGET_SCOPE):
        raise ValueError('Frozen-target mode/permit mismatch')
    seal=permit.pop('_seal',None)
    fixed_deadline=(time.monotonic()+max(0.,permit['expires_unix']-time.time())) if fixed_live else None
    journal=one_home_policy.PoweredRecoveryJournal(a.output.parent/'recovery.json',
        scope=permit.get('scope',one_home_policy.SCOPE)) if unified else None
    baseline=(load_fixed_backup(a.backup,approved_profile,time.time()) if fixed_config
              else load_backup(a.backup,vision_reference_static=True))
    noise_equivalence=low_vision.noise_evidence(baseline) if low_bandwidth_vision else None
    h,b=[json.loads(p.read_text()) for p in (a.heading,a.board)]
    mount=h['body_orientation_in_rigid_xyzw_candidate'];offset=b['board_center_in_rigid_m_candidate']
    device=find_pad()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from geometry_msgs.msg import PoseStamped

    class Source(Node):
        def __init__(self):
            super().__init__('powered_locked_single_trs_static')
            self.gate=PoseGate(max_age_ms=150)
            self.sub=self.create_subscription(PoseStamped,'/vicon/MircoAir/MircoAir',self.receive,
                QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE))
        def receive(self,msg):
            p,q=msg.pose.position,msg.pose.orientation
            self.gate.ingest(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,
                self.get_clock().now().nanoseconds,time.monotonic_ns(),msg.header.frame_id,
                [p.x,p.y,p.z],[q.x,q.y,q.z,q.w])

    rclpy.init(args=[]);node=Source()
    ev=Evidence(250,213,expected_parameters=read_params);power=PassivePowerEvidence()
    enc=m.MAVLink(None,srcSystem=250,srcComponent=213)
    latch=DurableStopLatch(Path(str(a.output)+'.home.latch.json'),UID2)
    short_lock=StaticLockGate(target=(250,213)) if short_status else None
    rate_journal=RateRecoveryJournal(a.output.parent/'rates_recovery.json') if short_status else None
    rate_group=RateTransactionGroup(rate_journal) if binary_diagnostics else None
    short_transport=ShortStatusTransport(short_lock,enc,rate_group.member('lock') if rate_group else rate_journal) if short_status else None
    binary_transport=ShortStatusTransport(short_lock,enc,rate_group.member('fusion'),message_id=248,interval_us=500000) if binary_diagnostics else None
    local_transport=ShortStatusTransport(short_lock,enc,rate_group.member('local'),message_id=32,interval_us=50000) if frozen_target_stream else None
    binary_receiver=FusionBinaryReceiver() if binary_diagnostics else None
    gate=policy.ShortStatusStreamGate(short_lock) if short_status else policy.StreamGate()
    start=time.monotonic()
    active=restoring=False
    last_arm=last_attitude=origin_p=origin_q=alignment=None
    arm_at=-100.;next_hb=next_version=next_params=next_emit=0.;param_cursor=0
    reference_boot_verified=False;home_seen=False;pad_alive=True
    values={};parameter_times={};parameter_requests={};types={};possibly_changed=set();restored={}
    diagnostic=DiagnosticTransaction();receive_verified=False
    sends=[];queries=[];writes=[];audits=[];acks=[];events=[];bad=[];texts=[]
    tx=Counter();tx_wire_bytes=Counter();rx_wire_bytes=Counter();error=None;restore_errors=[];final_arm=None;receive_timing=None
    primary=None;frozen_selector=None;settlement=None
    home_attempts=0;home_next_attempt=0.;home_acknowledged=False
    home_send_errors=[];input_faults=[];watch_completed=False
    active_started=None
    pad_generation=None
    last_wait_notice=None;next_wait_notice=0.
    receive_diagnostic=None
    ground_receipts=None
    diagnostic_send_until=None;diagnostic_stream_stopped=None
    inputs_ready_at=None
    config_snapshot=ParameterSnapshot() if short_status and compact_ground else None
    stream_io_start=None;stream_io=None
    last_local=None;target_stream=None;target_active=False;target_complete=False
    target_deadline=next_target=0.;target_attempts=[];target_final_audit=None
    target_initial_observation=None
    rx_trace=deque(maxlen=128);rx_trace_at_failure=None
    last_pump_at=None;last_raw_rx=None;reactor_gaps=[];raw_receive_gaps=[]
    parameter_read_attempts=[]
    fixed_gate=policy.FixedGroundPreparation(fixed_profile.fingerprint(approved_profile)) if fixed_config else None
    fixed_configuration_report=None
    fixed_boot_session=None

    def now():return time.monotonic()-start
    def gate_call(method,obs):
        return getattr(gate,method)(obs,**({'now_ns':time.monotonic_ns()} if short_status else {}))
    def configuration_scope_ok():
        if fixed_config:
            rows={n:dict(name=n,type=types.get(n),value=values.get(n)) for n in values}
            return fixed_profile.compare(approved_profile,rows)['configuration_approved']
        return vision.scope_ok(values,baseline)
    def clock_lock_detail():
        return short_lock.assess(time.monotonic_ns(),identity_verified=identity_fresh())
    def current_boot_us():
        if not last_attitude or not 0<=now()-last_attitude['at']<=.5:
            raise RuntimeError('Fresh FC clock observation required')
        return last_attitude['boot_ms']*1000+int((now()-last_attitude['at'])*1e6)
    def identity_fresh():
        return bool(not ev.fatal and not power.reboot_observed and ev.identity
            and ev.identity['uid2']==UID2 and ev.identity['firmware']=='1.14.3'
            and ev.heartbeats and not ev.heartbeats[-1]['armed']
            and 0<=now()-ev.heartbeats[-1]['elapsed_s']<2)
    def source_pose():
        sample=node.gate.last
        if sample is None:raise RuntimeError('Vicon sample missing')
        return transform_rigid_pose(sample.position,sample.quaternion,offset,mount)
    def observation():
        t=now();sample=node.gate.last
        fresh=node.gate.fresh(node.get_clock().now().nanoseconds,time.monotonic_ns())
        age=(node.get_clock().now().nanoseconds-sample.stamp_ns)/1e9 if sample and fresh else math.inf
        tilt=math.inf;static=False
        if fresh and last_attitude is not None and origin_p is not None:
            p,q=source_pose()
            tilt=attitude_comparison(q,last_attitude['q'])['gravity_axis_difference_deg']
            static=vision.stationary(p,q,origin_p,origin_q)
        parameters_ok=bool(set(values)==set(read_params) and configuration_scope_ok()
            and all(0<=t-parameter_times.get(n,-100)<4 for n in read_params))
        if config_snapshot and config_snapshot.started is not None:
            parameters_ok=config_snapshot.valid(t,identity_ok=identity_fresh(),
                scope_ok=set(values)==set(read_params) and configuration_scope_ok())
        lock_age=t-arm_at
        if unified and last_arm and last_attitude:
            boot_us=last_attitude['boot_ms']*1000+int((t-last_attitude['at'])*1e6)
            lock_age=max(lock_age,(boot_us-last_arm['timestamp_us'])/1e6)
        locked=bool(last_arm and not last_arm['armed'] and last_arm['lockdown']
                    and not last_arm['manual_lockdown'] and not last_arm['force_failsafe'])
        if short_status and short_transport.enabled:
            detail=short_lock.assess(time.monotonic_ns(),identity_verified=identity_fresh())
            locked=detail['valid']
            lock_age=detail['source_age_bounds_ms'][1]/1000 if locked else math.inf
        return policy.Observation(t,identity_fresh(),not FC_USB.exists(),reference_boot_verified,
            locked,
            lock_age,age,t-last_attitude['at'] if last_attitude else math.inf,
            tilt,static,parameters_ok,pad_alive,home_seen)
    def send(msg):
        if fixed_config and msg.get_type() in ('PARAM_SET','PARAM_EXT_SET'):
            raise PermissionError('Fixed EKF mode forbids ALL parameter writes, including cleanup')
        if fixed_live and not restoring and (msg.get_type()=='VISION_POSITION_ESTIMATE'
                or msg.get_type()=='COMMAND_LONG' and msg.command in (510,511)):
            if time.monotonic()>=fixed_deadline or time.time()>=permit['expires_unix']:
                raise PermissionError('Fixed ground permit expired; stop poses, retain HOME and rate cleanup')
        if fixed_config and msg.get_type()=='VISION_POSITION_ESTIMATE' and not fixed_gate.check(
                observation(),clock_lock=clock_lock_detail(),identity=ev.identity,boot_session=fixed_boot_session):
            raise PermissionError('No current fixed-profile position authorization at actual send boundary')
        # HOME/rollback must not depend on source math or diagnostic health.
        if receive_only and msg.get_type()=='PARAM_SET':
            raise PermissionError('No parameter writes in receive diagnostic')
        is_target=msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED'
        needs_static=(msg.get_type() in ('VISION_POSITION_ESTIMATE','SET_POSITION_TARGET_LOCAL_NED')
            or (msg.get_type()=='PARAM_SET' and not restoring))
        if unified and needs_static and (not startup.verified or startup.aborted):
            raise RuntimeError('Current one-HOME post-release lock proof missing')
        if seal is not None and msg.get_type() in ('PARAM_SET','VISION_POSITION_ESTIMATE','SET_POSITION_TARGET_LOCAL_NED') and not seal.fresh():
            raise RuntimeError('One-HOME code/evidence seal lost')
        if short_status and msg.get_type() in ('TIMESYNC','COMMAND_LONG') and getattr(msg,'command',None) in (None,510,511) and not seal.fresh():
            raise RuntimeError('Short-status code/evidence seal lost')
        locked=observation().healthy() if needs_static and origin_p is not None else False
        extra=bool(short_transport and short_transport.allowed(msg,identity=identity_fresh(),
            locked=bool(startup and startup.verified and not startup.aborted and last_arm
                        and last_arm['lockdown'] and not last_arm['armed'] and 0<=now()-arm_at<2),
            restoring=restoring,bootstrap=bool(startup and startup.short_bootstrap_ready())))
        if binary_transport:
            binary_locked=bool(startup.verified and not startup.aborted and
                short_lock.assess(time.monotonic_ns(),identity_verified=identity_fresh())['valid'])
            extra |= binary_transport.allowed(msg,identity=identity_fresh(),locked=binary_locked,restoring=restoring)
            if local_transport:
                extra |= local_transport.allowed(msg,identity=identity_fresh(),locked=binary_locked,restoring=restoring)
        if is_target:
            # This exception exists ONLY after consuming the distinct sealed
            # frozen-target permit; old static scopes keep rejecting targets.
            if not (frozen_target_stream and target_active and not restoring
                    and active and locked and target_stream is not None
                    and target_stream.permits(msg,target_observation(),ev.heartbeats[-1]['custom_mode'])):
                raise PermissionError('Frozen current-FC target scope rejected')
            extra=True
        if binary_diagnostics and msg.get_type()=='SERIAL_CONTROL':
            raise RuntimeError('Binary diagnostic mode forbids NSH commands and empty polls')
        profile_context=({'low_bandwidth_vision':True,'vision_noise_parameters':values} if low_bandwidth_vision else {})
        if fixed_config:profile_context['fixed_config']=True
        if not extra and not (one_home_policy if unified else policy).allowed(msg,identity_fresh=identity_fresh(),authorized=True,
            locked_static=locked,receiving_verified=receive_verified,
            restore_safe=restoring and identity_fresh(),home_latched=latch.latched,**profile_context):
            raise RuntimeError('Powered static send whitelist rejected '+msg.get_type())
        packet=msg.pack(enc)
        if low_bandwidth_vision and msg.get_type()=='VISION_POSITION_ESTIMATE' and len(packet)!=low_vision.WIRE_BYTES:
            raise RuntimeError('Low-bandwidth packet differs from pinned 48-byte profile')
        if is_target:
            target_attempts.append(dict(at=now(),position=[msg.x,msg.y,msg.z],yaw=msg.yaw))
        _write_owner_packet(port, packet)
        enc.seq=(enc.seq+1)%256;tx[msg.get_type()]+=1
        tx_wire_bytes[msg.get_type()]+=len(packet)
        return len(packet)
    def target_observation():
        if not (active and observation().healthy() and local_transport.enabled and last_local
                and binary_receiver and not binary_receiver.fault):
            raise RuntimeError('Frozen target live position/static state unavailable')
        ns=time.monotonic_ns();lower,_=frozen.clock_bounds_us(short_lock.clock,ns)
        local_age=frozen.source_age(short_lock.clock,last_local['boot_ms'],last_local['received_ns'],ns,maximum=.15,label='LOCAL_POSITION_NED')
        attitude_age=frozen.source_age(short_lock.clock,last_attitude['boot_ms'],last_attitude['received_ns'],ns,maximum=.5,label='ATTITUDE_QUATERNION')
        row=binary_receiver.snapshot('fusion',now(),fc_now_us=lower,identity_verified=identity_fresh())
        row['yaw_comparison']=combined.compare_heading(sends[-1]['q'],last_attitude['q'],
            attitude_age_ms=attitude_age*1000,sent_age_ms=(now()-sends[-1]['elapsed_s'])*1000)
        if not combined.fusion_evidence_ok([audits[0],row],primary):
            raise RuntimeError('Frozen target live XYZ/yaw fusion no longer healthy')
        detail=short_lock.assess(ns,identity_verified=identity_fresh())
        if not detail['valid']:raise RuntimeError('Frozen target output lock unavailable')
        local=row['local']
        return GroundObservation(now(),UID2,'1.14.3',tuple(last_local['position']),
            compact.euler(last_attitude['q'])[2],
            tuple(local[key] for key in ('xy_reset_counter','z_reset_counter','heading_reset_counter')),
            primary,armed=False,lockdown=True,pose_age_s=local_age,
            status_age_s=detail['source_age_bounds_ms'][1]/1000,
            estimator_ready=True,gamepad_available=pad_alive,home_latched=home_seen)
    def emit_targets():
        nonlocal next_target,target_active
        if not target_active:return
        obs=target_observation()
        mode=ev.heartbeats[-1]['custom_mode'];target_stream.check(obs,mode)
        if now()>=target_deadline:
            target_active=False
            return
        if now()<next_target:return
        msg=target_stream.message(enc,last_local['boot_ms'],obs,mode)
        wire_bytes=send(msg)
        target_stream.record_send(now(),msg,wire_bytes)
        next_target=now()+1/frozen.RATE_HZ
    def emit():
        nonlocal next_emit,active,diagnostic_stream_stopped
        if not active:return
        if receive_only and diagnostic_send_until is not None and now()>=diagnostic_send_until:
            # One-way phase transition. The 2s uORB counter may keep running,
            # but no more poses are sent and the original lock gate is never
            # widened or silently restarted during its output wait.
            active=False;gate.stop()
            diagnostic_stream_stopped=dict(at=now(),reason='planned_receive_only_phase',
                last_send_at=gate.last_send,sent_packets=gate.sent)
            return
        obs=observation()
        if not gate_call('check',obs):
            gate.first_failure['parameter_details']=compact.parameter_freshness_details(
                read_params,parameter_times,parameter_requests,obs.now)
            gate.first_failure['parameter_scope_matches']=configuration_scope_ok()
            raise RuntimeError('Static stream guard: '+', '.join(gate.first_failure['reasons']))
        if fixed_gate and not fixed_gate.check(obs,clock_lock=clock_lock_detail(),
                identity=ev.identity,boot_session=fixed_boot_session):
            raise RuntimeError('Fixed ground authorization revoked: '+str(fixed_gate.failure))
        if now()<next_emit:return
        sample=node.gate.take_new(node.get_clock().now().nanoseconds,time.monotonic_ns())
        if sample is None:return
        p,q=transform_rigid_pose(sample.position,sample.quaternion,offset,mount)
        age_ns=node.get_clock().now().nanoseconds-sample.stamp_ns
        stamp=(time.monotonic_ns()-age_ns)//1000
        msg,p,q=(low_vision.message(enc,stamp,p,q,alignment,values) if low_bandwidth_vision
            else compact.message(enc,stamp,p,q,alignment))
        if short_status:gate_call('actual_send',observation())
        wire_bytes=send(msg)
        if not short_status:gate.actual_send(now())
        sends.append({'elapsed_s':now(),'source_stamp_ns':sample.stamp_ns,'position':p,'q':q,'wire_bytes':wire_bytes})
        next_emit+=1/policy.RATE_HZ
        if next_emit<now():next_emit=now()+1/policy.RATE_HZ
    def service_home():
        nonlocal active,home_attempts,home_next_attempt
        if not home_seen and not (startup and startup.needs_stop):return
        if home_seen:active=False;gate.stop()
        if home_acknowledged or home_attempts>=3 or now()<home_next_attempt or not identity_fresh():return
        attempt=home_attempts;home_attempts+=1;home_next_attempt=now()+.8
        try:send(lock_message(enc,ev.identity,UID2,latch,True,True,confirmation=attempt))
        except Exception as exc:
            home_send_errors.append({'at':now(),'error':str(exc),'attempt':attempt})
    def read_pad():
        nonlocal home_seen,pad_alive,error,home_acknowledged,home_attempts,home_next_attempt
        if not pad_alive:return
        try:
            if unified:require_gamepad_identity(pad_fd,device['path'],pad_generation)
            if os.stat(device['path']).st_rdev!=pad_device:raise OSError('Gamepad device changed')
            chunk=os.read(pad_fd,EVENT.size*64)
            if not chunk:raise OSError('Gamepad disconnected')
            input_buffer.extend(chunk)
        except BlockingIOError:pass
        except OSError as exc:
            pad_alive=False;input_faults.append({'at':now(),'error':str(exc)})
            error=error or 'Gamepad input lost; no automatic reconnect'
            if not restoring:raise
        while len(input_buffer)>=EVENT.size:
            _,_,kind,code,value=EVENT.unpack(input_buffer[:EVENT.size]);del input_buffer[:EVENT.size]
            if kind==0 and code==3:
                pad_alive=False;input_faults.append({'at':now(),'error':'Gamepad input overflow'})
                error=error or 'Gamepad input overflow'
                if not restoring:raise RuntimeError('Gamepad input overflow')
            if kind==1 and code==HOME:
                events.append({'at':now(),'value':value});latch.event(kind,code,value)
                if startup:
                    was_aborted=startup.aborted;startup.event(value,now())
                    if startup.aborted:
                        home_seen=True
                        if not was_aborted:home_acknowledged=False;home_attempts=0;home_next_attempt=now()
                elif value in (1,2):home_seen=True
    def pump():
        nonlocal next_hb,next_version,next_params,last_attitude,reference_boot_verified,param_cursor
        nonlocal home_acknowledged,last_wait_notice,next_wait_notice,last_local,last_pump_at,last_raw_rx
        t=now()
        if last_pump_at is not None and t-last_pump_at>.1:
            reactor_gaps.append(dict(start=last_pump_at,end=t,gap_s=t-last_pump_at,
                active=active,restoring=restoring,last_write=writes[-1] if writes else None))
        last_pump_at=t
        # Always process HOME before ROS callbacks, parsing, timers or diagnostics.
        try:read_pad()
        finally:service_home()
        timed_out=(t>45 if not unified else t>home_wait_seconds if startup.press is None else t-startup.press>45)
        if FC_USB.exists() or (not restoring and timed_out):raise RuntimeError('Physical scope changed or global deadline reached')
        rclpy.spin_once(node,timeout_sec=0)
        raw=port.read(min(port.in_waiting,8192))
        messages=dec.parse_buffer(raw) or []
        if raw:
            received_at=now()
            if last_raw_rx is not None and received_at-last_raw_rx>.1:
                raw_receive_gaps.append(dict(start=last_raw_rx,end=received_at,
                    gap_s=received_at-last_raw_rx,active=active,restoring=restoring,
                    last_write=writes[-1] if writes else None))
            last_raw_rx=received_at
            rx_trace.append(dict(at=received_at,bytes=len(raw),types=dict(Counter(msg.get_type() for msg in messages)),
                attitude_boot_ms=[msg.time_boot_ms for msg in messages if msg.get_type()=='ATTITUDE_QUATERNION'],
                clock_remote_ns=[msg.tc1 for msg in messages if msg.get_type()=='TIMESYNC' and msg.tc1>0]))
        for msg in messages:
            # Decoder/clock samples use the actual post-read monotonic time,
            # not the reactor's pre-read relative timestamp.
            if short_transport and msg.get_type()!='BAD_DATA':short_transport.ingest(msg,time.monotonic_ns())
            if binary_transport and msg.get_type()!='BAD_DATA':binary_transport.ingest(msg,time.monotonic_ns())
            if local_transport and msg.get_type()!='BAD_DATA':local_transport.ingest(msg,time.monotonic_ns())
            ev.ingest(msg,t);power.ingest(msg,t)
            if msg.get_type()=='BAD_DATA':
                if len(bad)<30:bad.append({'at':t,'reason':str(getattr(msg,'reason',''))})
                diagnostic.damage('damaged_packet_during_reply',t)
                continue
            if (msg.get_srcSystem(),msg.get_srcComponent())!=(1,1):continue
            kind=msg.get_type()
            rx_wire_bytes[kind]+=len(msg.get_msgbuf())
            if binary_transport and binary_transport.enabled and kind=='V2_EXTENSION' and not restoring:
                # Restoration deliberately disables fusion. Its inactive aid
                # records cannot refresh (or invalidate) completed acceptance.
                # Rate ACK/readback and live lock guards are still processed.
                binary_receiver.feed(bytes(msg.get_msgbuf()),now(),identity_verified=identity_fresh())
                if binary_receiver.fault and not restoring:
                    raise RuntimeError('Binary diagnostic fault: '+binary_receiver.fault)
            if kind=='ATTITUDE_QUATERNION':
                if last_attitude and msg.time_boot_ms<last_attitude['boot_ms']:raise RuntimeError('FC reboot')
                if not reference_boot_verified:
                    reference_boot_verified=(True if unified else boot_reference_matches(msg.time_boot_ms/1000,
                        permit['reference_boot_s'],permit['reference_utc_unix'],time.time()))
                    if not reference_boot_verified:raise RuntimeError('FC boot differs from current HOME-lock evidence')
                last_attitude={'at':t,'boot_ms':msg.time_boot_ms,'received_ns':time.monotonic_ns(),
                    'q':checked_quaternion([msg.q1,msg.q2,msg.q3,msg.q4]).tolist()}
            elif kind=='LOCAL_POSITION_NED' and local_transport and local_transport.enabled:
                position=[msg.x,msg.y,msg.z]
                if (not all(math.isfinite(v) for v in position)
                        or last_local and msg.time_boot_ms<last_local['boot_ms']):
                    raise RuntimeError('Frozen target FC position nonfinite or clock regression')
                # A repeated source timestamp cannot refresh the host receipt.
                if last_local is None or msg.time_boot_ms>last_local['boot_ms']:
                    last_local=dict(at=now(),boot_ms=msg.time_boot_ms,
                        received_ns=time.monotonic_ns(),position=position)
            elif kind=='PARAM_VALUE' and str(msg.param_id) in read_params:
                name=str(msg.param_id);values[name]=decode_parameter(msg);types[name]=msg.param_type;parameter_times[name]=t
                if not restoring and (not fixed_config or active):
                    expected=(baseline[name],11 if name=='EKF2_EV_CTRL' else 65) if name in possibly_changed else (baseline[name],)
                    if values[name] not in expected:raise RuntimeError('Unexpected live parameter: '+name)
                    if fixed_config and msg.param_type!=fixed_profile.PARAMETER_TYPES[name]:
                        raise RuntimeError('Unexpected live fixed parameter type: '+name)
            elif kind=='SERIAL_CONTROL':
                diagnostic.receive(bytes(msg.data[:msg.count]).decode('utf8','replace'),t)
            elif kind=='COMMAND_ACK' and msg.command==185:
                if home_attempts and (msg.target_system,msg.target_component)==(250,213):
                    acks.append({'at':t,'result':msg.result,'source':[1,1],
                                 'target':[msg.target_system,msg.target_component]})
                    if msg.result==m.MAV_RESULT_ACCEPTED:
                        home_acknowledged=True
                        if startup:
                            if last_attitude:
                                startup.ack(t,boot_us=last_attitude['boot_ms']*1000+int((t-last_attitude['at'])*1e6))
                            else:home_acknowledged=False
            elif kind=='STATUSTEXT':texts.append({'at':t,'text':msg.text})
        if ev.fatal or power.reboot_observed:raise RuntimeError(ev.fatal or 'FC reboot observed')
        if ev.identity and (ev.identity['uid2']!=UID2 or ev.identity['firmware']!='1.14.3'):
            raise RuntimeError('Wrong FC identity or firmware')
        service_home()
        if home_seen and not restoring:raise RuntimeError('HOME latched: stop poses and restore baseline, no restart')
        if short_transport:short_transport.probe(time.monotonic_ns(),send,identity=identity_fresh(),restoring=restoring)
        if t>=next_hb:
            send(enc.heartbeat_encode(m.MAV_TYPE_GCS,m.MAV_AUTOPILOT_INVALID,0,0,0));next_hb=t+1
        if ev.heartbeats and ev.identity is None and t>=next_version:
            ev.version_requested_at=t;send(enc.command_long_encode(1,1,512,0,148,0,0,0,0,0,0));next_version=t+2
        if (identity_fresh() and not restoring and t>=next_params
                and not (config_snapshot and config_snapshot.started is not None)):
            if compact_ground:
                # Human waiting is not an active acceptance window. Read the
                # baseline once, then refresh every field AFTER HOME below.
                waiting_home=bool(startup and startup.press is None and set(values)==set(read_params))
                name=None if waiting_home else compact.next_parameter_read(
                    read_params,parameter_times,parameter_requests,t,
                    minimum_reply_age=.6 if active else 0.)
            else:
                name=read_params[param_cursor];param_cursor=(param_cursor+1)%len(read_params)
            if name is not None:
                ev.requests[name]=t;parameter_requests[name]=t
                send(enc.param_request_read_encode(1,1,name.encode(),-1))
            # 31/10=3.1s left little slack; a lost round-robin reply waited
            # 6.2s. Prioritize oldest actual replies and retry without bursts.
            # 15Hz remains below startup's 20Hz; the <4s gate is unchanged.
            next_params=t+(1/15 if compact_ground and active else .05)
        diagnostic.tick(t)
        if startup and not active and not restoring and not startup.verified:
            missing=[]
            if not identity_fresh():missing.append('无线飞控')
            if not last_attitude or t-last_attitude['at']>.5:missing.append('姿态回传')
            if set(values)!=set(read_params):missing.append('关键参数读取')
            if not node.gate.fresh(node.get_clock().now().nanoseconds,time.monotonic_ns()):missing.append('实时Vicon')
            if startup.press is None:missing.append('一次HOME短按')
            elif startup.release is None:missing.append('HOME松开')
            elif startup.ack_at is None:missing.append('锁定接受回执')
            else:missing.append('松开后的锁定回读')
            summary=tuple(missing)
            if summary!=last_wait_notice or t>=next_wait_notice:
                print('等待：'+'、'.join(missing)+'；不解锁、不起飞。',flush=True)
                last_wait_notice=summary;next_wait_notice=t+15
        emit();emit_targets();time.sleep(.001)
    def binary_snapshot(kind):
        if not binary_transport.enabled:raise RuntimeError('Binary diagnostic stream not configured')
        requested=now();requested_boot=current_boot_us();deadline=now()+2
        while now()<deadline:
            pump()
            if binary_receiver.fault:raise RuntimeError(binary_receiver.fault)
            if binary_receiver.received_at.get(kind,-1)<requested:continue
            ref=short_lock.clock.reference;ns=time.monotonic_ns()
            if short_lock.clock.fault or not ref:continue
            elapsed=ns-ref['received_ns']
            if not 0<=elapsed<short_lock.clock.MAX_AGE_NS:continue
            margin=math.ceil((elapsed+ref['rtt_ns'])*short_lock.clock.DRIFT_PPM/1_000_000)
            lower=(ns+ref['lower_ns']-margin)//1000
            upper=(ns+ref['upper_ns']+margin+999)//1000
            latest=binary_receiver.latest[kind]
            stamps=([latest[key]['timestamp'] for key in ('selector','flags','local','aid','height_aid','yaw_aid')]
                    if kind=='fusion' else [latest['timestamp']])
            if any(stamp>lower for stamp in stamps):continue
            if any(upper-stamp>=2_000_000 for stamp in stamps):
                raise RuntimeError('Binary diagnostic source expired under bounded clock')
            result=binary_receiver.snapshot(kind,now(),fc_now_us=int(lower),identity_verified=identity_fresh())
            return result,requested,requested_boot
        raise RuntimeError('Fresh bounded binary diagnostic missing; no text fallback')

    def observe(topic):
        nonlocal last_arm,arm_at
        if short_status and topic=='actuator_armed':
            if not short_transport.enabled:raise RuntimeError('Short-status stream not configured; no text fallback')
            requested_ns=time.monotonic_ns();deadline=now()+2
            while now()<deadline:
                pump()
                sample=short_lock.status.sample
                detail=short_lock.clock.assess(sample,time.monotonic_ns(),
                    identity_verified=identity_fresh(),state_fault=short_lock.status.fault)
                if detail['valid'] and int(sample['received']*1e9)>=requested_ns:
                    bits=sample['bits']
                    result=dict(timestamp_us=sample['stamp']*1000,armed=bool(bits&2),
                        lockdown=bool(bits&4),manual_lockdown=bool(bits&8),force_failsafe=bool(bits&16),
                        in_esc_calibration_mode=bool(bits&32),ready_to_arm=bool(bits&64),prearmed=bool(bits&128))
                    last_arm=result;arm_at=now()
                    queries.append(dict(topic=topic,transport='OUTLOCK1',at=now(),parsed=result,
                        short_clock=detail,requested_at_ns=requested_ns))
                    return result
            raise RuntimeError('Fresh safe short output status missing; no text fallback')
        if binary_diagnostics:
            if topic not in ('vehicle_visual_odometry','selector','fusion_row'):
                raise RuntimeError('Unsupported binary diagnostic topic: '+topic)
            result,requested,requested_boot=binary_snapshot('visual' if topic=='vehicle_visual_odometry' else 'fusion')
            if topic=='selector':result=result['selector']
            queries.append(dict(topic=topic,transport='EVR1' if topic=='vehicle_visual_odometry' else 'EVF2',
                at=now(),requested_at=requested,requested_boot_us=requested_boot,
                completed_boot_us=current_boot_us(),parsed=result))
            return result
        if not diagnostic.idle:raise RuntimeError('Previous reply incomplete; no overlapping shell command')
        if topic!='actuator_armed' and last_arm is not None and not (short_transport and short_transport.enabled):
            # Reply arrival is not the source's publication time. A late
            # reply must not buy another full query interval of false youth.
            lock_age=observation().lock_age if unified else now()-arm_at
            # Reserve time for both the next reply and a subsequent lock
            # refresh. This schedules earlier; the 2s expiry is unchanged.
            if lock_age>(.35 if compact_ground else .65):observe('actuator_armed')
        parsers={'actuator_armed':parse_actuator_snapshot,'selector':parse_selector,
            'visual_series':compact.visual_series,'vehicle_visual_odometry':parse_visual_snapshot,
            'visual_rate':compact.publication_counter_rate,
            'local_position':combined.parse_local_position,'failsafe':combined.parse_failsafe}
        parse=(combined.parse_flags if topic.startswith('flags_') else
            combined.parse_height_aid if topic.startswith('height_aid_') else
            combined.parse_yaw_aid if topic.startswith('yaw_aid_') else
            combined.parse_aid if topic.startswith('aid_') else parsers[topic])
        payload=(compact.RATE_COMMAND if unified and topic=='visual_rate' else
            b'listener vehicle_visual_odometry -n 13\n'
            if unified and topic=='visual_series' else policy.COMMANDS[topic])
        for attempt in range(diagnostic.max_attempts):
            if attempt and compact_ground:
                # A terminal prompt ends the discarded epoch, not a burst of
                # radio corruption. Pace the clean retry while keeping HOME,
                # pose guards and the global deadline serviced by pump().
                retry_at=now()+.25
                while now()<retry_at:pump()
                if topic!='actuator_armed' and not (short_transport and short_transport.enabled):observe('actuator_armed')
            diagnostic_request_at=now()
            requested_boot_us=current_boot_us() if compact_ground and topic=='vehicle_visual_odometry' else None
            # Read lock-reply chunks at 10Hz instead of 4Hz. Only the current
            # transaction is polled: no overlapping shell commands or bursts.
            diagnostic.poll_interval=.1 if compact_ground and topic=='actuator_armed' else .25
            diagnostic.begin(payload,diagnostic_request_at)
            send(enc.serial_control_encode(10,2,0,0,len(payload),payload.ljust(70,b'\0')))
            while not diagnostic.idle:
                pump()
                text=diagnostic.ready_text()
                if text is not None:
                    result=(compact.visual_series(text,samples=13) if unified and topic=='visual_series' else parse(text))
                    if unified and topic=='visual_series' and result:
                        queries.append({'topic':'initial_visual_cache_excluded','at':now(),'parsed':result[0],'attempt':attempt+1})
                        result=result[1:]  # Only initial cached snapshot; no arbitrary sample filtering.
                    if result:
                        diagnostic.accept()
                        queries.append({'topic':topic,'at':now(),'parsed':result,'attempt':attempt+1,
                            'requested_at':diagnostic_request_at,'requested_boot_us':requested_boot_us,
                            'completed_boot_us':current_boot_us() if requested_boot_us is not None else None})
                        if topic=='actuator_armed':
                            if unified and last_attitude:
                                deadline=now()+.15
                                def source_age_us():
                                    return last_attitude['boot_ms']*1000+int((now()-last_attitude['at'])*1e6)-result['timestamp_us']
                                while source_age_us()<0 and now()<deadline:pump()
                                if not 0<=source_age_us()<=2_000_000:
                                    raise RuntimeError('Current nonfuture battery-static actuator sample required')
                            if (not state_valid(result,last_attitude['boot_ms'] if last_attitude else None)
                                    or not result['lockdown'] and not (startup and not startup.verified)):
                                raise RuntimeError('Current FC state is not fresh/disarmed/locked')
                            last_arm=result;arm_at=now()
                            if startup:
                                snapshot_boot_us=last_attitude['boot_ms']*1000+int((now()-last_attitude['at'])*1e6) if last_attitude else None
                                startup.observe(result,requested_at=diagnostic_request_at,now=now(),
                                    boot_ms=snapshot_boot_us//1000 if snapshot_boot_us is not None else None,
                                    boot_us=snapshot_boot_us)
                        return result
                    diagnostic.damage('complete_reply_fields_invalid',now())
                if not identity_fresh():raise RuntimeError('Wireless identity freshness lost during diagnostic')
                if diagnostic.phase=='blocked':raise RuntimeError('Diagnostic drain boundary missing: '+topic)
                if diagnostic.poll_due(now()):send(enc.serial_control_encode(10,2,0,0,0,b'\0'*70))
        raise RuntimeError('Clean fixed diagnostic reply not established after bounded retry: '+topic)
    def write_parameter(name,value):
        if fixed_config:
            raise PermissionError('Legacy parameter transaction is unreachable in fixed EKF mode')
        try:read_pad()
        finally:service_home()
        if home_seen and not restoring:
            raise RuntimeError('HOME latched before parameter write; no enable writes')
        # Mark before sending: a transport error may occur after FC reception.
        possibly_changed.add(name);requested=now();ev.requests[name]=requested
        value=int(value) if name=='EKF2_EV_CTRL' else float(value)
        if journal:journal(dict(name=name,value=value,at=requested,identity=ev.identity,baseline=baseline))
        send(vision.parameter_message(enc,name,value));writes.append({'at':now(),'name':name,'value':value})
        for repeat in range(2):
            request=now();ev.requests[name]=request
            send(enc.param_request_read_encode(1,1,name.encode(),-1))
            parameter_read_attempts.append(dict(at=now(),name=name,restoring=restoring,retry=False))
            deadline=now()+2;next_retry=request+.5
            def reply_ok():
                return (parameter_times.get(name,-1)>=request and values.get(name)==value
                    and types.get(name)==(6 if name=='EKF2_EV_CTRL' else 9))
            while not reply_ok():
                if now()>deadline:raise RuntimeError('Parameter readback incomplete: '+name)
                pump()
                if restoring and not reply_ok() and now()>=next_retry:
                    # Recovery ONLY: re-request an idempotent baseline read
                    # inside the SAME 2s deadline. Never repeat PARAM_SET,
                    # enable writes, widen active freshness or restart poses.
                    ev.requests[name]=now()
                    send(enc.param_request_read_encode(1,1,name.encode(),-1))
                    parameter_read_attempts.append(dict(at=now(),name=name,restoring=True,retry=True))
                    next_retry=now()+.5
            until=now()+.15
            while now()<until:pump()
        if restoring:restored[name]=True

    try:
        with ExitStack() as stack:
            if seal:seal.start();stack.callback(seal.close)
            port=stack.enter_context(serial.Serial(a.trs_port,baud,timeout=.002,write_timeout=.3,exclusive=True))
            port.reset_input_buffer()
            pad_fd=os.open(device['path'],os.O_RDONLY|os.O_NONBLOCK);stack.callback(os.close,pad_fd)
            pad_device=os.fstat(pad_fd).st_rdev;input_buffer=bytearray()
            if unified:
                pad_generation=gamepad_identity(device['path'])
                require_gamepad_identity(pad_fd,device['path'],pad_generation)
            bitmap=fcntl.ioctl(pad_fd,0x80000000|(96<<16)|(ord('E')<<8)|0x18,bytes(96))
            if bitmap[HOME//8]&(1<<(HOME%8)):raise RuntimeError('HOME held at startup')
            try:
                while not (identity_fresh() and last_attitude and set(values)==set(read_params)
                    and node.gate.fresh(node.get_clock().now().nanoseconds,time.monotonic_ns())):
                    if fixed_config and now()>10 and set(values)!=set(read_params):
                        fixed_configuration_report=fixed_profile.validate_current(approved_profile,values,types,
                            parameter_times,now(),identity=ev.identity)
                        raise RuntimeError('Fixed parameter read timeout; no positions or automatic repair')
                    if not unified and now()>10:raise RuntimeError('Fresh live startup readiness timeout')
                    pump()
                if fixed_config:
                    fixed_boot_session=(last_attitude['boot_ms'],start)
                    fixed_gate.connect(ev.identity,fixed_boot_session,now())
                    fixed_configuration_report=fixed_profile.validate_current(approved_profile,values,types,
                        parameter_times,now(),identity=ev.identity,requested_at=parameter_requests,
                        boot_session=fixed_boot_session)
                    if not fixed_gate.verify_configuration(fixed_configuration_report,now()):
                        raise RuntimeError('Fixed configuration validation failed; no correction or poses')
                    # The reused acceptance validates XYZ/yaw without velocity.
                    # This is a scope requirement, NOT approval of CTRL=11.
                    scoped={**values,'EKF2_EV_DELAY':0}
                    if values['EKF2_EV_CTRL']!=11 or not vision.scope_ok(scoped,scoped):
                        raise RuntimeError('Fixed profile outside existing XYZ/yaw ground safety envelope')
                elif not vision.scope_ok(values,baseline) or values['EKF2_EV_CTRL']!=2 or values['EKF2_EV_DELAY']!=0:
                    raise RuntimeError('Current baseline differs; no automatic correction')
                inputs_ready_at=now()
                if startup:
                    print('1/4 链路/参数/Vicon已就绪。只需短按一次HOME并松开；若此前已按，会自动继续。不会起飞。',flush=True)
                    while not startup.verified:
                        pump()
                        if short_status and startup.short_bootstrap_ready():
                            if not short_transport.changed:short_transport.configure(send,pump,time.monotonic_ns)
                            startup.observe_short(short_lock,now_ns=time.monotonic_ns(),
                                post_home_ns=int((start+max(startup.release,startup.ack_at))*1e9)+1,
                                identity_verified=identity_fresh())
                        elif not short_status and startup.query_due(now(),arm_at):observe('actuator_armed')
                    voltage=power.batteries[-1].get('reported_voltage_v') if power.batteries else None
                    if type(voltage) not in (int,float) or not math.isfinite(voltage) or voltage<=0:
                        raise RuntimeError('Battery-only voltage telemetry missing')
                    print('2/4 本次上电HOME锁定回读通过，自动进行'+('无改参接收诊断' if receive_only else '定位接收与静态融合')+'。再按HOME将中止。',flush=True)
                else:
                    observe('actuator_armed')
                if compact_ground and startup:
                    # Waiting may take minutes: cached startup config must NOT
                    # authorize poses. Require all actual replies after HOME,
                    # as well as the unchanged <4s check, before enabling stream.
                    while not (all(parameter_times.get(n,-1)>=startup.press
                                   and 0<=now()-parameter_times[n]<4 for n in read_params)):
                        pump()
                    if not configuration_scope_ok():
                        raise RuntimeError('Post-HOME baseline changed; no pose stream')
                    if fixed_config:
                        fixed_configuration_report=fixed_profile.validate_current(approved_profile,values,types,
                            parameter_times,now(),identity=ev.identity,requested_at=parameter_requests,
                            boot_session=fixed_boot_session)
                        if not fixed_configuration_report['current_configuration_verified']:
                            raise RuntimeError('Post-HOME fixed configuration freshness/type check failed')
                    observe('actuator_armed')
                origin_p,origin_q=source_pose()
                alignment=compact.yaw_alignment(origin_q,last_attitude['q'])
                if short_transport:
                    if not short_transport.changed:short_transport.configure(send,pump,time.monotonic_ns)
                    deadline=now()+3
                    while not short_lock.assess(time.monotonic_ns(),identity_verified=identity_fresh())['valid']:
                        if now()>deadline:raise RuntimeError('Fresh bounded short-status clock not established')
                        pump()
                if binary_transport:binary_transport.configure(send,pump,time.monotonic_ns)
                active_started=now()
                stream_io_start=dict(at=active_started,tx=dict(tx_wire_bytes),rx=dict(rx_wire_bytes))
                if config_snapshot:
                    config_snapshot.begin(now(),fresh=observation().parameters_fresh_match,
                        identity_ok=identity_fresh())
                if fixed_gate and not fixed_gate.authorize_positions(observation(),home=startup,
                        clock_lock=clock_lock_detail(),identity=ev.identity,boot_session=fixed_boot_session):
                    raise RuntimeError('Fixed preparation safety proof incomplete; no positions')
                gate_call('start',observation());active=True;next_emit=now()
                until=now()+.75
                while now()<until:pump()
                if receive_only:
                    observe('actuator_armed')
                    # PX4's one-shot uorb top sleeps 2s before printing.
                    # Account for already-aged lock evidence and scheduling
                    # reserve, then permanently stop poses before its expiry.
                    budget=min(1.25,2-observation().lock_age-.25)
                    if budget<.05:raise RuntimeError('Insufficient fresh lock budget for bounded diagnostic send window')
                    diagnostic_send_until=now()+budget
                    counter=observe('visual_rate')
                    if active:
                        active=False;gate.stop()
                        diagnostic_stream_stopped=dict(at=now(),reason='counter_returned_before_send_deadline',
                            last_send_at=gate.last_send,sent_packets=gate.sent)
                    observe('actuator_armed')
                    snapshot=observe('vehicle_visual_odometry')
                    receive_diagnostic=dict(counter=counter,snapshot=snapshot,
                        matches_sent_pose=bool(compact.match(snapshot,sends)),
                        receive_time_mode=vision.receive_time_mode(snapshot),
                        send_window_seconds=budget,stream_stopped=diagnostic_stream_stopped,
                        counter_window_includes_stopped_source=True,
                        counter_rate_is_not_continuous_running_rate=True,
                        maximum_gap_verified=False,source_clock_synchronization_verified=False,
                        fusion_acceptance=False,flight_authorized=False)
                    print('RECEIVE DIAGNOSTIC: '+json.dumps(receive_diagnostic,allow_nan=False),flush=True)
                    raise ReceiveDiagnosticComplete()
                if compact_ground:
                    receipts=[]
                    for index in range(3):
                        # observe(non-lock) owns the age-based refresh. Calling
                        # it here as well caused adjacent duplicate lock queries.
                        snapshot=observe('vehicle_visual_odometry')
                        query=queries[-1]
                        receipts.append(dict(snapshot=snapshot,at=query['at'],requested_at=query['requested_at'],
                            requested_boot_us=query['requested_boot_us'],completed_boot_us=query['completed_boot_us']))
                        if index<2:
                            until=now()+.1
                            while now()<until:pump()
                    ground_receipts=compact.ground_receipt_evidence(receipts,sends)
                    receive_verified=True
                    print('GROUND RECEIPTS: three fresh matching snapshots; host cadence checked, not full FC publication gaps.',flush=True)
                else:
                    group=observe('visual_series');receive_timing=compact.series_timing(group)
                    if unified:
                        # A USB-free RF clock may lag the newest publication.
                        # Wait briefly; never accept a future sample as current.
                        until=now()+.15
                        while group[-1]['timestamp']/1000>last_attitude['boot_ms'] and now()<until:pump()
                    receive_verified=bool(receive_timing['rate_hz']>=30
                        and receive_timing['maximum_publication_gap_ms']<=50
                        and all(vision.receive_time_mode(row) for row in group)
                        and compact.match(group[-1],sends)
                        and abs(group[-1]['timestamp']/1000-last_attitude['boot_ms'])<500)
                    if unified:receive_verified &= group[-1]['timestamp']/1000<=last_attitude['boot_ms']
                if not receive_verified:
                    raise RuntimeError('Fresh real FC reception/rate check failed; no enable writes')
                observe('actuator_armed')
                if fixed_config:
                    # Fusion may already have started on the FIRST accepted
                    # samples. Do not pretend reception preceded enabling it.
                    gate_call('begin_fusion',observation())
                else:
                    write_parameter('EKF2_EV_DELAY',65)
                    gate_call('begin_fusion',observation());write_parameter('EKF2_EV_CTRL',11)
                if unified:
                    until=now()+1
                    while now()<until:
                        pump()
                        if not short_status and observation().lock_age>.5:observe('actuator_armed')
                print('FUSION START: battery-only TRS, stationary, output locked; max XYZ/yaw window 12s.',flush=True)
                settlement=GroundEstimatorSettlement(now())
                while True:
                    selected=observe('selector')
                    if settlement.observe(selected,now(),targets_sent=0):
                        primary=selected['primary_instance'];frozen_selector=settlement.frozen_signature;break
                    until=now()+.1
                    while now()<until:pump()
                count=selected['instances_available'];slots={}
                if binary_receiver:binary_receiver.freeze(frozen_selector)
                else:
                    for prefix in ('aid','height_aid','yaw_aid'):
                        mapping={slot:observe(f'{prefix}_{slot}') for slot in range(count)}
                        slots[prefix]=aid_topic_for_estimator(mapping,primary)
                for audit in range(3 if unified else 2):
                    binary_row=observe('fusion_row') if binary_diagnostics else None
                    selected=binary_row['selector'] if binary_row else observe('selector')
                    require_frozen_selector(selected,frozen_selector)
                    if (not selected['healthy'][primary] or selected['gyro_fault_detected'] or selected['accel_fault_detected']):
                        raise RuntimeError('Selected estimator became unhealthy')
                    if binary_row:row=binary_row
                    else:
                        row={'selector':selected,'flags':observe(f'flags_{primary}')}
                        row.update({prefix:observe(f'{prefix}_{slots[prefix]}') for prefix in slots})
                        row['local']=observe('local_position')
                    row['yaw_comparison']=combined.compare_heading(sends[-1]['q'],last_attitude['q'],
                        attitude_age_ms=(now()-last_attitude['at'])*1000,sent_age_ms=(now()-sends[-1]['elapsed_s'])*1000)
                    if unified:
                        observe('actuator_armed')
                        audit_boot_us=last_attitude['boot_ms']*1000+int((now()-last_attitude['at'])*1e6)
                        if any(type(row[key].get('timestamp')) is not int
                                or not 0<=audit_boot_us-row[key]['timestamp']<=2_000_000
                                for key in ('selector','flags','aid','height_aid','yaw_aid','local')):
                            raise RuntimeError('Current same-boot full fusion audit required')
                    row['at']=now();audits.append(row)
                    if row['flags']['cs_in_air']:raise RuntimeError('In-air flag during locked static bench')
                    print('AUDIT '+str(audit+1)+': XYZ/yaw flags read; no motion commands.',flush=True)
                    until=now()+.25
                    while now()<until:pump()
                if not combined.fusion_evidence_ok(audits,primary):raise RuntimeError('Complete progressing XYZ/yaw fusion evidence not established')
                if fixed_gate and not fixed_gate.verify_fusion(audits,primary,reception_verified=receive_verified,
                        observation=observation(),clock_lock=clock_lock_detail(),identity=ev.identity,
                        boot_session=fixed_boot_session):
                    raise RuntimeError('Fixed preparation did not establish actual current fusion')
                if frozen_target_stream:
                    # Frozen targets never use Vicon XYZ as if it were EKF XYZ.
                    # Request and freeze genuine fresh FC LOCAL_POSITION_NED.
                    local_transport.configure(send,pump,time.monotonic_ns)
                    requested=now();deadline=requested+1
                    while last_local is None or last_local['at']<requested:
                        if now()>deadline:raise RuntimeError('Fresh post-rate FC local position missing')
                        pump()
                    mode=ev.heartbeats[-1]['custom_mode']
                    if not restorable_mode((mode>>16)&255,(mode>>24)&255):
                        raise RuntimeError('Frozen target requires stable non-Offboard ground mode')
                    initial_observation=target_observation()
                    target_initial_observation=asdict(initial_observation)
                    target_stream=frozen.FrozenTargetStream(initial_observation,mode,authorized=True)
                    target_active=True;next_target=now();target_deadline=now()+frozen.DURATION_S
                    print('FROZEN TARGET: actual FC XYZ/yaw held fixed, 16Hz/3s; NO mode/arm/unlock.',flush=True)
                    while target_active:pump()
                    target_stream.finish()
                    target_final_audit=observe('fusion_row')
                    target_final_audit['yaw_comparison']=combined.compare_heading(sends[-1]['q'],last_attitude['q'],
                        attitude_age_ms=(now()-last_attitude['at'])*1000,sent_age_ms=(now()-sends[-1]['elapsed_s'])*1000)
                    target_complete=bool(combined.fusion_evidence_ok([audits[-1],target_final_audit],primary)
                        and ev.heartbeats[-1]['custom_mode']==mode)
                    if not target_complete:raise RuntimeError('Post-target progressing fusion/mode evidence missing')
            except ReceiveDiagnosticComplete:
                pass
            except (Exception,KeyboardInterrupt) as exc:
                error=str(exc) or 'Interrupted'
                if fixed_gate:fixed_gate.fail(error,now())
                rx_trace_at_failure=list(rx_trace)
                latch.fail('powered_static_failed: '+error)
            finally:
                if stream_io_start is not None:
                    duration=max(0.,now()-stream_io_start['at'])
                    outgoing={k:v-stream_io_start['tx'].get(k,0) for k,v in tx_wire_bytes.items()}
                    incoming={k:v-stream_io_start['rx'].get(k,0) for k,v in rx_wire_bytes.items()}
                    stream_io=dict(duration_s=duration,tx_valid_wire_bytes=outgoing,rx_valid_wire_bytes=incoming,
                        tx_bytes_per_second=sum(outgoing.values())/duration if duration else None,
                        rx_bytes_per_second=sum(incoming.values())/duration if duration else None,
                        scope='Actual CRC-valid MAVLink bytes during pose stage; not radio airtime, capacity or delivery ratio')
                target_active=False;active=False;gate.stop();restoring=True
                if not binary_diagnostics:diagnostic.damage('session_stopped_discard_incomplete_reply',now())
                for name in (() if fixed_config else ('EKF2_EV_CTRL','EKF2_EV_DELAY')):
                    if name in possibly_changed:
                        try:write_parameter(name,baseline[name])
                        except Exception as exc:restore_errors.append(name+': '+str(exc))
                # Restoration reads do not depend on the broken shell. Only
                # empty polls may drain it; never overlap a new command.
                while diagnostic.phase=='draining' and identity_fresh():
                    pump()
                    if diagnostic.poll_due(now()):send(enc.serial_control_encode(10,2,0,0,0,b'\0'*70))
                # Before any HOME attempt, stream/rate setup or parameter
                # change, there is nothing to restore and no configured short
                # status to query. Keep final_arm unknown instead of reporting
                # a false restoration failure for an input-only startup abort.
                final_state_required=(not short_status or short_transport.changed
                    or bool(possibly_changed) or bool(home_attempts))
                if final_state_required and (short_status or diagnostic.idle) and identity_fresh():
                    try:final_arm=observe('actuator_armed')
                    except Exception as exc:restore_errors.append('Final state: '+str(exc))
                # Keep the same serial owner and HOME fd for a bounded
                # post-test stop-only watch. This is NOT a flight supervisor.
                if watch_seconds and identity_fresh() and pad_alive:
                    print(f'HOME WATCH: stop-only listener remains active for at most {watch_seconds:g}s; no poses or enable writes.',flush=True)
                    deadline=now()+watch_seconds
                    try:
                        while now()<deadline:
                            pump()
                            if not identity_fresh() or not pad_alive:
                                raise RuntimeError('HOME watch identity/input lost; no automatic reconnect')
                        watch_completed=True
                        if short_status or diagnostic.idle:final_arm=observe('actuator_armed')
                    except (Exception,KeyboardInterrupt) as exc:
                        error=error or str(exc) or 'HOME watch interrupted'
                        latch.fail('home_watch_failed: '+str(exc))
                if home_seen:error=error or 'HOME latched; no automatic restart'
                if local_transport:
                    try:local_transport.restore(send,pump,time.monotonic_ns)
                    except Exception as exc:restore_errors.append('FC local rate restore: '+str(exc))
                if binary_transport:
                    try:binary_transport.restore(send,pump,time.monotonic_ns)
                    except Exception as exc:restore_errors.append('Binary diagnostic rate restore: '+str(exc))
                if short_transport:
                    try:short_transport.restore(send,pump,time.monotonic_ns)
                    except Exception as exc:restore_errors.append('Short-status rate restore: '+str(exc))
                restoring=False
                if fixed_gate:fixed_gate.close(now())
    except (Exception,KeyboardInterrupt) as exc:
        error=error or str(exc) or 'Interrupted before stream'
        if fixed_gate:fixed_gate.fail(error,now())
        latch.fail('powered_static_failed: '+error)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    parameters_closed=(not writes and not possibly_changed and not restored and configuration_scope_ok()
        if fixed_config else restored=={'EKF2_EV_CTRL':True,'EKF2_EV_DELAY':True})
    passed=bool(not error and not restore_errors and final_arm and final_arm['lockdown']
        and final_arm['armed'] is False and parameters_closed
        and combined.fusion_evidence_ok(audits,primary))
    recovery_verified=(journal.finish_fixed(writes,final_arm) if fixed_config
        else journal.finish(possibly_changed,restored,final_arm)) if journal else None
    if unified:passed=bool(passed and recovery_verified and startup.verified and not startup.aborted and len(audits)==3)
    if frozen_target_stream:passed=bool(passed and target_complete)
    diagnostic_done=bool(receive_only and receive_diagnostic and not error and not restore_errors
        and startup.verified and not startup.aborted and final_arm and final_arm['lockdown']
        and final_arm['armed'] is False and not writes and recovery_verified)
    compact_passed=bool(compact_ground and ground_receipts and passed)
    strict_passed=bool(passed and not compact_ground)
    fixed_passed=bool(fixed_config and compact_passed and fixed_gate.ready_observed)
    if fixed_config:compact_passed=strict_passed=False  # Old permits cannot consume the new result.
    finished_at=now()
    press_at=startup.press if startup else None
    timing=dict(total_seconds=finished_at,inputs_ready_at_s=inputs_ready_at,
        home_press_at_s=press_at,initial_home_wait_limit_s=home_wait_seconds if unified else None,
        human_wait_after_inputs_s=max(0,press_at-inputs_ready_at) if press_at is not None and inputs_ready_at is not None else None,
        after_home_seconds=finished_at-press_at if press_at is not None else None,
        scope='Current process timing only; not aircraft battery uptime, flight approval or guaranteed completion time')
    result={'captured_at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'trs_port':str(a.trs_port),'trs_baud':baud,'rx_parser':dec.summary(),
        'scope':expected_fixed_scope if fixed_config else __doc__,
        'physical_scope':'props-off/battery-only/disarmed/locked' if fixed_live else 'offline-fixed-ground-candidate/no-hardware-proof' if fixed_config
            else 'battery-only/props-installed-protected/disarmed/locked',
        'fixed_ekf_mode':fixed_config,'fixed_configuration_report':fixed_configuration_report,
        'fixed_ground_preparation':fixed_gate.status() if fixed_gate else None,
        'fixed_ground_acceptance_verified':fixed_passed,
        'fixed_live_enabled':fixed_live if fixed_config else None,
        'fixed_hardware_verified':bool(fixed_live and fixed_passed) if fixed_config else None,
        'control_ready':False,'physical_authorization':False,
        'identity':ev.identity,'boot_reference_verified':reference_boot_verified,'permit_path':str(a.permit),
        'duration_s':now(),'fc_usb_present_at_end':FC_USB.exists(),'stream_guard_failure':gate.first_failure,
        'receive_rate_only':receive_only,'receive_diagnostic':receive_diagnostic,
        'compact_ground_fusion':compact_ground,'compact_ground_receipts':ground_receipts,
        'short_status':short_status,'short_status_transport':short_transport.summary() if short_transport else None,
        'binary_diagnostics':binary_diagnostics,
        'low_bandwidth_vision':low_bandwidth_vision,'vision_noise_equivalence':noise_equivalence,
        'frozen_target_stream':frozen_target_stream,'frozen_target_stream_verified':bool(frozen_target_stream and passed),
        'frozen_target_summary':target_stream.summary() if target_stream else None,
        'frozen_target_write_attempts':target_attempts,'target_final_fusion_audit':target_final_audit,
        'target_initial_observation':target_initial_observation,
        'parameter_read_attempts':parameter_read_attempts,
        'rx_trace_at_failure':rx_trace_at_failure,'rx_trace_at_end':list(rx_trace),
        'reactor_gaps':reactor_gaps,'raw_receive_gaps':raw_receive_gaps,
        'local_position_transport':local_transport.summary() if local_transport else None,
        'reused_wireless_static_proof':permit.get('wireless_proof'),
        'stream_io':stream_io,
        'binary_transport':binary_transport.summary() if binary_transport else None,
        'all_rate_transactions':rate_group.summary() if rate_group else short_transport.summary() if short_transport else None,
        'binary_receiver_fault':binary_receiver.fault if binary_receiver else None,
        'binary_not_ready_records':binary_receiver.not_ready_records if binary_receiver else 0,
        'binary_last_not_ready':binary_receiver.last_not_ready if binary_receiver else None,
        'short_clock_probe':short_lock.clock.summary() if short_lock else None,
        'configuration_snapshot':({'started_at':config_snapshot.started,'failed':config_snapshot.failed,
            'maximum_seconds':20,'scope':'bounded static bench only; no live parameter polling; explicit writes still read back'}
            if config_snapshot else None),
        'compact_ground_fusion_verified':compact_passed,
        'timing':timing,
        'receive_diagnostic_completed':diagnostic_done,
        'pose_stream_duration_s':sends[-1]['elapsed_s']-sends[0]['elapsed_s'] if len(sends)>1 else 0,
        'fusion_window_duration_s':(sends[-1]['elapsed_s']-gate.fusion_started)
            if sends and gate.fusion_started is not None and sends[-1]['elapsed_s']>=gate.fusion_started else 0,
        'heading_path':str(a.heading),'board_path':str(a.board),'source_accepted':node.gate.accepted,
        'source_rejected':node.gate.rejected,'serial_ports_released':True,'received_fc_messages':dict(ev.fc_counts),
        'tx_messages':dict(tx),'tx_wire_bytes':dict(tx_wire_bytes),'passive_power':power.result(),'heartbeats':ev.heartbeats,'device':device,
        'home_events':events,'home_acks':acks,'home_monitor_running_after_exit':False,
        'home_lock_attempts':home_attempts,'home_lock_acknowledged':home_acknowledged,
        'home_send_errors':home_send_errors,'input_faults':input_faults,
        'home_watch_seconds':watch_seconds,'home_watch_completed':watch_completed,
        'alignment_yaw_this_boot_rad':alignment,'pose_sends':sends,'receive_timing':receive_timing,
        'settlement':settlement.rows if settlement else [],'frozen_selector':frozen_selector,
        'internal_observations':queries,'fusion_audits':audits,'parameter_writes':writes,
        'restored':restored,'parameters_after':values,'actuator_after':final_arm,
        'error':error,'restore_errors':restore_errors,'shell_reply_incomplete_at_exit':not diagnostic.idle,
        'diagnostic_failures':diagnostic.failures,'diagnostic_phase_at_exit':diagnostic.phase,
        'bad_packet_observations':bad,'statustext':texts,'standalone_locked_static_fusion_verified':strict_passed,
        'arming_output_unlock_mode_or_motion_commands_sent':bool(target_attempts),
        'arming_output_unlock_mode_or_moving_targets_sent':False,
        'same_link_restore_guaranteed':False,'flight_authorized':False,'global_holds_modified':False}
    if unified:result.update(one_home_preflight=True,startup_home=startup.status(),
        parameter_restoration_verified=None if fixed_config else recovery_verified,
        fixed_configuration_unchanged_verified=recovery_verified if fixed_config else None,
        one_home_preflight_verified=strict_passed,
        telemetry_rate_writes_sent=(len(short_transport.writes) if short_transport else 0)
            +(len(binary_transport.writes) if binary_transport else 0)
            +(len(local_transport.writes) if local_transport else 0),all_checks_in_one_process=True)
    with a.output.open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False);stream.write('\n')
    summary_keys=(('fixed_ground_acceptance_verified','fixed_configuration_unchanged_verified',
        'fixed_hardware_verified','error','restore_errors','flight_authorized') if fixed_config else
        ('standalone_locked_static_fusion_verified','compact_ground_fusion_verified','restored','error','restore_errors','flight_authorized'))
    print(json.dumps({k:result[k] for k in summary_keys},indent=2),flush=True)
    print('HOME LISTENER EXITED: serial released; this process no longer monitors HOME.',flush=True)
    if restore_errors:print('RESTORE UNCONFIRMED: do not arm; operator power isolation and explicit recovery required.',flush=True)
    return 0 if (fixed_passed if fixed_config else diagnostic_done if receive_only else compact_passed if compact_ground else strict_passed) else 2


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    add_baud_argument(ap)
    for name in ('trs-port','permit','backup','heading','board','output'):
        ap.add_argument('--'+name,type=str if name=='trs-port' else Path,required=True)
    ap.add_argument('--powered-locked-static-confirmed',action='store_true',required=True)
    ap.add_argument('--legacy-dynamic-ekf',action='store_true',
                    help='Explicit old experimental write/restore mode; not plan B')
    ap.add_argument('--home-watch-seconds',type=float,default=30,
                    help='Keep the same HOME/TRS owner after rollback for 0..120s (default 30); exits explicitly, never arms')
    a=ap.parse_args()
    if not a.legacy_dynamic_ekf:
        ap.error('Standalone legacy runner requires --legacy-dynamic-ekf; fixed plan B is offline-only')
    try:return run(a)
    except (OSError,ValueError,RuntimeError,KeyError) as exc:ap.exit(2,str(exc)+'\n')


if __name__=='__main__':raise SystemExit(main())
