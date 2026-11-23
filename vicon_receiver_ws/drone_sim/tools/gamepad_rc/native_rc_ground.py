"""Pocket/WiFi ground observation, called ONLY by vicon_fixed_point_entry.

Reuses strict CRC, persistent NSH, parameter decoding and the Vicon receiver.
Explicit motor-isolated fusion mode adds Vicon injection, with no RC prerequisite.
No RC override, HOME/output commands, arm or parameter writes.
Explicit timesync check temporarily enables only message 111, then restores it.
Explicit fusion-clock check keeps that sync active during continuous poses and EKF reads.
Explicit target-load mode adds a bounded frozen ground-origin setpoint stream.
Explicit offboard-check adds one mode entry and one return to Position, disarmed.
Battery-static permits only a protected 60 s stationary fusion observation,
with USB absent, current landed/disarmed state and the existing mission rate lifecycle.
Target/mode/arming messages remain prohibited in that branch.
This is not a flight controller and never fabricates gamepad readiness.
"""
from contextlib import ExitStack
import json
import math
import struct
from pathlib import Path
import time

from pymavlink.dialects.v20 import common as m
from mavlink_crc_stream import CrcResyncParser
from mavlink_link import FixedPeerUDP
from mavlink_readonly_probe import decode_parameter, readonly_message, FC_USB
from persistent_shell import current_reply
from rc_watch import NativeRCMonitor, review_parameters, summarize_observation
from trs_uart_readonly_check import UID2
import compact_vision
from disarmed_offboard_policy import POSITION_YAW_MASK
from mission_audit_collector import COMMANDS as AUDIT_COMMANDS, parse as parse_audit
from usb_vision_policy import parse_visual_snapshot, parse_timesync_snapshot, ClockReadiness, check_clock_probe
from usb_fusion_policy import parse_topic

PARAMS = ('COM_RC_IN_MODE', 'COM_RC_OVERRIDE', 'COM_RC_STICK_OV',
          'RC_MAP_ROLL', 'RC_MAP_PITCH', 'RC_MAP_THROTTLE', 'RC_MAP_YAW',
          'RC_MAP_FLTMODE', 'RC_MAP_ARM_SW', 'RC_MAP_KILL_SW', 'RC_MAP_OFFB_SW',
          'COM_RC_LOSS_T', 'NAV_RCL_ACT', 'COM_RCL_EXCEPT',
          'COM_OF_LOSS_T', 'COM_OBL_RC_ACT', 'COM_DL_LOSS_T', 'NAV_DLL_ACT',
          'COM_FAIL_ACT_T', 'COM_KILL_DISARM', 'CBRK_USB_CHK', 'COM_ARM_WO_GPS') + tuple(f'COM_FLTMODE{i}' for i in range(1, 7))
COMMANDS = {topic: f'listener {topic} -n 1\n'.encode() for topic in NativeRCMonitor.TOPICS}
COMMANDS.update(vehicle_status=b'listener vehicle_status -n 1\n',
                health_report=b'listener health_report -n 1\n',
                commander=b'commander check\n')
FUSION_COMMANDS = {k: v for k, v in AUDIT_COMMANDS.items()
                   if k in ('selector', 'local_position', 'failsafe')
                   or k.startswith(('flags_', 'aid_', 'height_aid_', 'yaw_aid_'))}
FUSION_COMMANDS['visual'] = b'listener vehicle_visual_odometry -n 1\n'
FUSION_COMMANDS['offboard_control_mode'] = b'listener offboard_control_mode -n 1\n'
SYNC_COMMANDS = {f'timesync_{i}': f'listener timesync_status -i {i} -n 1\n'.encode() for i in range(4)}
FUSION_PARAMS = ('EKF2_EV_CTRL', 'EKF2_HGT_REF', 'EKF2_EV_DELAY',
                 'EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z',
                 'EKF2_OF_CTRL', 'EKF2_RNG_CTRL', 'EKF2_BARO_CTRL')


def allowed(msg, identified=False):
    kind = msg.get_type()
    if kind in ('HEARTBEAT', 'COMMAND_LONG'):
        return readonly_message(msg)  # COMMAND_LONG only REQUEST_MESSAGE/version.
    if kind == 'TIMESYNC':
        return identified and msg.tc1 == 0 and msg.ts1 > 0
    if kind == 'PARAM_REQUEST_READ':
        name = msg.param_id
        if isinstance(name, bytes): name = name.decode('ascii').rstrip('\0')
        return (identified and name in PARAMS + FUSION_PARAMS and msg.param_index == -1
                and (msg.target_system, msg.target_component) == (1, 1))
    if kind == 'SERIAL_CONTROL':
        return (identified and (msg.device, msg.flags, msg.timeout, msg.baudrate) == (10, 2, 0, 0)
                and any(msg.count == len(p) and bytes(msg.data) == p.ljust(70, b'\0')
                        for p in (b'', *COMMANDS.values(), *FUSION_COMMANDS.values())))
    return False


class GroundObserver:
    """One writer and one parser for the new link; device-free construction."""
    def __init__(self, port, record, *, clock=None, fusion_observe=False, ground_target_load=False, offboard_check=False,
                 timesync_check=False, ros_clock_ns=None, battery_static=False):
        self.port, self.record, self.clock = port, record, clock or time.monotonic
        self.encoder = m.MAVLink(None, srcSystem=250, srcComponent=220)
        self.parser = CrcResyncParser()
        self.rc = NativeRCMonitor()
        self.identity = None
        self.started = self.clock()
        self.next_hb = self.next_sync = self.started
        self.requested_version = False
        self.pending_sync = {}
        self.parameters = {}
        self.parameter_readbacks = {}
        self.requested_parameters = set()
        self.shell_buffer = ''
        self.tx_counts = {}
        self.status_texts = []
        self.fusion_observe = fusion_observe
        self.attitude = None
        self.alignment = None
        self.last_pose_stamp = 0
        self.last_pose_at = None
        self.next_pose = 0.
        self._pose_message = None
        self.pose_count = 0
        self.max_pose_gap = 0.
        self.ground_target_load = ground_target_load
        self.target_origin = self.target_local = self._target_message = None
        self.target_started = self.next_target = None
        self.target_rows = []
        self.offboard_check = offboard_check
        self.target_duration = 15 if offboard_check else 3
        self.mode_requests = []
        self.mode_acks = []
        self._mode_message = None
        self.timesync_check = timesync_check
        self.ros_clock_ns = ros_clock_ns
        self.sync_rate = None
        self.sync_restoring = False
        self._sync_reply = None
        self.sync_replies = []
        self.sync_last_token = 0
        self.last_pose_row = None
        self.battery_static = battery_static
        self.landed = None
        self.sync_lifecycle = False

    def battery_ground_current(self):
        return bool(self.identified() and self.landed and self.landed['state'] == 1
                    and 0 <= self.clock()-self.landed['at'] < 2 and not FC_USB.exists())

    def identified(self):
        hb = self.rc.heartbeat
        return bool(self.identity and hb and not hb['armed'] and 0 <= self.clock()-hb['received_at'] < 2)

    def send(self, msg):
        if self.battery_static:
            if FC_USB.exists(): raise RuntimeError('Battery-static route observed FC USB')
            kind = msg.get_type()
            if (kind not in ('HEARTBEAT','COMMAND_LONG','TIMESYNC','SERIAL_CONTROL','VISION_POSITION_ESTIMATE')
                    or kind == 'COMMAND_LONG' and msg.command not in (510,511,512)):
                raise PermissionError('Battery-static observation never sends control/parameter messages')
            if (kind == 'VISION_POSITION_ESTIMATE' or kind == 'COMMAND_LONG' and msg.command in (510,511)) and not self.battery_ground_current():
                raise PermissionError('Fresh landed/disarmed battery state required')
        pose_permitted = (self.fusion_observe and self.identified()
                          and msg is self._pose_message and compact_vision.pose_allowed(msg))
        target_permitted = (self.ground_target_load and msg is self._target_message
            and msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED' and self._target_state_valid())
        mode_permitted = (self.offboard_check and msg is self._mode_message and self.identified()
            and msg.get_type()=='COMMAND_LONG' and msg.command==176
            and (msg.target_system,msg.target_component,msg.confirmation)==(1,1,0)
            and msg.param1==1 and msg.param2 in (3,6)
            and all(getattr(msg,f'param{i}')==0 for i in range(3,8)))
        sync_permitted = (self.timesync_check and self.identified() and (
            msg is self._sync_reply or self.sync_rate is not None and (
                self.sync_rate.permits(msg,fresh=self.identified(),locked=self.battery_ground_current())
                if self.sync_lifecycle else self.sync_rate.allowed(
                    msg, identity=True, locked=True, restoring=self.sync_restoring))
            or msg.get_type() == 'SERIAL_CONTROL'
            and (msg.device,msg.flags,msg.timeout,msg.baudrate)==(10,2,0,0)
            and any(msg.count==len(p) and bytes(msg.data)==p.ljust(70,b'\0') for p in SYNC_COMMANDS.values())))
        if not (sync_permitted or pose_permitted or target_permitted or mode_permitted or allowed(msg, self.identified())):
            raise PermissionError('Ground observer refused '+msg.get_type())
        wire = msg.pack(self.encoder)
        self.record(dict(direction='tx_attempt', at=self.clock(), kind=msg.get_type(), wire_hex=wire.hex()))
        if self.port.write(wire) != len(wire): raise OSError('Short send; no retry')
        self.encoder.seq = (self.encoder.seq+1) % 256
        kind = msg.get_type()
        self.tx_counts[kind] = self.tx_counts.get(kind, 0)+1
        self.record(dict(direction='tx_complete', at=self.clock(), kind=kind, bytes=len(wire)))
        return wire

    def freeze_ground_target(self, local, status, actuator):
        """Freeze once for the explicit motor-isolated bounded target window."""
        now=self.clock();clock=self.rc.fc_clock
        if not self.ground_target_load or not self.fusion_observe or self.target_origin is not None:
            raise PermissionError('Ground target load not enabled or already consumed')
        if not clock or not 0<=now-clock[1]<2:raise RuntimeError('Current FC clock required')
        boot=clock[0]+(now-clock[1])*1e6
        for row in (local,status,actuator):
            value=row.get('parsed')
            if not value or not 0<=now-row['at']<2 or not 0<=boot-value['timestamp']<2e6:
                raise RuntimeError('Ground target snapshots missing/stale')
        p=local['parsed'];v=status['parsed'];a=actuator['parsed']
        if (v['nav_state']!=2 or not v['pre_flight_checks_pass'] or a['armed']
                or any(a[k] for k in ('lockdown','manual_lockdown','force_failsafe'))
                or not all(p[k] for k in ('xy_valid','z_valid','heading_good_for_control'))
                or not all(math.isfinite(p[k]) for k in ('x','y','z','heading'))):
            raise RuntimeError('Fresh disarmed Position ground origin required')
        self.target_origin=dict(p);self.target_local=local
        self.target_started=self.next_target=now
        if not self._target_state_valid():raise RuntimeError('Ground target starting state invalid')

    def _target_state_valid(self):
        now=self.clock();hb=self.rc.heartbeat;clock=self.rc.fc_clock;row=self.target_local
        modes=(3<<16,6<<16) if self.offboard_check and self.mode_requests else (3<<16,)
        if not (self.identified() and hb['custom_mode'] in modes and self.target_origin and row
                and clock and 0<=now-clock[1]<2 and 0<=now-self.target_started<self.target_duration
                and len(self.target_rows)<self.target_duration*20 and self.last_pose_at is not None
                and 0<=now-self.last_pose_at<=.15 and 0<=now-row['at']<=1):return False
        p=row['parsed'];origin=self.target_origin
        return bool(p and 0<=clock[0]+(now-clock[1])*1e6-p['timestamp']<=1e6
            and all(p[k] for k in ('xy_valid','z_valid','heading_good_for_control'))
            and all(p[k]==origin[k] for k in ('xy_reset_counter','z_reset_counter','heading_reset_counter'))
            and math.dist([p[k] for k in ('x','y','z')],[origin[k] for k in ('x','y','z')])<=.03
            and abs(math.remainder(p['heading']-origin['heading'],2*math.pi))<=.05)

    def ground_target_tick(self):
        if self.target_started is None:return
        now=self.clock()
        if now-self.target_started>=self.target_duration or len(self.target_rows)>=self.target_duration*20:return
        if not self._target_state_valid():raise RuntimeError('Ground target state changed/stale; no restart')
        if self.target_rows and now-self.target_rows[-1]['at']>.2:
            raise RuntimeError('Ground target send continuity lost; no restart')
        if now<self.next_target:return
        p=self.target_origin;clock=self.rc.fc_clock
        boot_ms=int((clock[0]+(now-clock[1])*1e6)/1000)%(2**32)
        msg=self.encoder.set_position_target_local_ned_encode(boot_ms,1,1,m.MAV_FRAME_LOCAL_NED,
            POSITION_YAW_MASK,p['x'],p['y'],p['z'],0,0,0,0,0,0,p['heading'],0)
        self._target_message=msg
        try:wire=self.send(msg)
        finally:self._target_message=None
        self.target_rows.append(dict(at=now,wire_hex=wire.hex()))
        self.next_target=now+.05

    def request_ground_mode(self, main):
        """One entry, one exit; never retry an uncertain write or permit arm."""
        if not self.offboard_check or not self.identified():
            raise PermissionError('Mode check requires identified disarmed aircraft')
        previous=[r['main'] for r in self.mode_requests]
        if main==6:
            if (previous or not self._target_state_valid() or len(self.target_rows)<30
                    or self.clock()-self.target_rows[0]['at']<2
                    or self.clock()-self.target_rows[-1]['at']>.2):
                raise PermissionError('One Offboard entry requires two seconds of fresh fixed targets')
        elif main==3:
            if previous!=[6] or self.rc.heartbeat['custom_mode'] not in (3<<16,6<<16):
                raise PermissionError('Position return requires one prior entry and no other pilot mode')
        else:raise PermissionError('Only Offboard and Position permitted')
        row=dict(main=main,at=self.clock(),confirmed=False)
        self.mode_requests.append(row)  # Mark before write: no retry even if delivery is uncertain.
        msg=self.encoder.command_long_encode(1,1,176,0,1,main,0,0,0,0,0)
        self._mode_message=msg
        try:row['wire_hex']=self.send(msg).hex()
        finally:self._mode_message=None
        return row

    def mode_confirmed(self, request):
        hb=self.rc.heartbeat
        acks=[a for a in self.mode_acks if a['at']>=request['at']]
        if any(a['result'] not in (0,5) for a in acks):
            raise RuntimeError(f"Mode {request['main']} rejected: {acks[-1]['result']}")
        confirmed=bool(self.identified() and hb['received_at']>=request['at']
            and hb['custom_mode']==request['main']<<16 and any(a['result']==0 for a in acks))
        if confirmed:
            request.update(confirmed=True,confirmed_at=self.clock(),heartbeat=dict(hb))
        return confirmed

    def restore_ground_mode(self):
        """Error cleanup: one Position return if still disarmed; no automatic re-entry."""
        if [r['main'] for r in self.mode_requests]!=[6]:return
        request=self.request_ground_mode(3)
        while self.clock()-request['at']<4:
            self.pump()
            if self.mode_confirmed(request):return
            time.sleep(.002)
        raise RuntimeError('Position return not confirmed; operator mode selection required')

    def ingest(self, data):
        now = self.clock()
        for msg in self.parser.parse_buffer(data):
            kind = msg.get_type()
            if kind == 'BAD_DATA':
                self.record(dict(direction='bad_frame', at=now, reason=msg.reason, wire_hex=bytes(msg.data).hex()))
                continue  # Isolated corrupt bytes do not refresh any evidence.
            if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1): continue
            if self.sync_rate: self.sync_rate.ingest(msg, now if self.sync_lifecycle else int(now*1e9))
            self.rc.observe_message(msg, now)
            if kind == 'HEARTBEAT' and msg.base_mode & 128:
                raise RuntimeError('Ground-only session observed ARMED; stopped without sending controls')
            if kind == 'EXTENDED_SYS_STATE':
                self.landed = dict(at=now,state=int(msg.landed_state))
                if self.battery_static and msg.landed_state != 1:
                    raise RuntimeError('Battery-static observation requires landed state')
            if kind == 'AUTOPILOT_VERSION' and self.requested_version:
                v = msg.flight_sw_version
                identity = dict(uid2=bytes(msg.uid2).hex(), firmware=f'{v >> 24}.{(v >> 16)&255}.{(v >> 8)&255}')
                if identity != dict(uid2=UID2, firmware='1.14.3'):
                    raise RuntimeError('Unexpected aircraft identity/firmware')
                self.identity = identity
            elif (kind == 'TIMESYNC' and msg.tc1 == 0 and self.timesync_check
                  and self.identified() and self.ros_clock_ns is not None
                  and msg.ts1 > self.sync_last_token
                  and getattr(msg,'target_system',0) in (0,250)
                  and getattr(msg,'target_component',0) in (0,220)):
                # Reply on the receiving WiFi owner, in the exact Vicon ROS clock domain.
                # Host-initiated probes alone never update the FC's filter.
                self.sync_last_token = msg.ts1
                stamp = self.ros_clock_ns()
                reply = self.encoder.timesync_encode(stamp,msg.ts1)
                self._sync_reply = reply
                try: self.send(reply)
                finally: self._sync_reply = None
                row = dict(at=now,fc_originate_ns=msg.ts1,pc_reply_ns=stamp)
                self.sync_replies.append(row)
                self.record(dict(direction='fc_timesync_reply',**row))
            elif kind == 'TIMESYNC' and msg.tc1 > 0:
                sent = self.pending_sync.pop(msg.ts1, None)
                if sent is not None:
                    rtt=now-sent; accepted=0<=rtt<=.1
                    self.record(dict(direction='clock_sample',at=now,rtt_s=rtt,accepted=accepted))
                    if accepted:self.rc.observe_clock(msg.tc1/1000+rtt*1e6, now,
                                                      remote_boot_us=msg.tc1/1000)
                    else:
                        # Reject the delayed sample, but do not wait another full
                        # second until the 2 s freshness deadline is already due.
                        self.next_sync=min(self.next_sync,now+.1)
            elif kind == 'PARAM_VALUE':
                name = msg.param_id
                if isinstance(name, bytes): name = name.decode('ascii')
                name = name.rstrip('\0')
                if name in self.requested_parameters:
                    self.parameters[name] = decode_parameter(msg)
                    self.parameter_readbacks[name] = dict(value=self.parameters[name],type=msg.param_type,
                        wire_value_hex=struct.pack('<f',msg.param_value).hex(),received_at=now)
            elif kind == 'ATTITUDE_QUATERNION':
                self.attitude = dict(at=now, boot_us=msg.time_boot_ms*1000,
                                     q=[msg.q1, msg.q2, msg.q3, msg.q4])
            elif kind == 'SERIAL_CONTROL' and msg.device == 10:
                self.shell_buffer += bytes(msg.data[:msg.count]).decode('utf8', 'replace')
                if len(self.shell_buffer) > 24000: raise RuntimeError('Diagnostic reply exceeded bound')
            elif kind == 'STATUSTEXT':
                row = dict(at=now, severity=msg.severity, text=str(msg.text).rstrip('\0'))
                self.status_texts.append(row)
                self.status_texts = self.status_texts[-256:]
                self.record(dict(direction='status_text', **row))
            elif kind == 'COMMAND_ACK' and msg.command==176 and self.mode_requests:
                if msg.target_system not in (0,250) or msg.target_component not in (0,220):continue
                row=dict(at=now,result=msg.result,wire_hex=bytes(msg.get_msgbuf()).hex())
                self.mode_acks.append(row)
                self.record(dict(direction='mode_ack',**row))

    def pump(self):
        if self.battery_static and FC_USB.exists():raise RuntimeError('Battery-static route observed FC USB')
        self.ingest(self.port.read(min(self.port.in_waiting, 65536)))
        now = self.clock()
        hb = self.rc.heartbeat
        if (hb and now-hb['received_at'] >= 2) or (not hb and now-self.started >= 8):
            raise RuntimeError('Fresh FC heartbeat lost; no automatic retry')
        if now >= self.next_hb:
            self.send(self.encoder.heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0))
            self.next_hb = now+1
        if hb and not self.requested_version:
            self.requested_version = True
            self.send(self.encoder.command_long_encode(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0))
        if self.identified() and now >= self.next_sync:
            stamp = int(now*1e9)
            self.pending_sync = {stamp: now}
            self.send(self.encoder.timesync_encode(0, stamp))
            self.next_sync = now+1
        if self.sync_lifecycle and self.sync_rate is not None:
            action=self.sync_rate.action(self.clock(),fresh=self.identified(),locked=self.battery_ground_current())
            if action is not None:
                try:self.send(action)
                except Exception as exc:
                    self.sync_rate.dispatch_failed(str(exc));raise

    def shell(self, payload=b''):
        self.send(self.encoder.serial_control_encode(10, 2, 0, 0, len(payload), payload.ljust(70, b'\0')))

    def pose(self, observation, now_ros_ns):
        """Only the isolated observation loop calls this with its transformed frame.

        No RC dependency, mode transition, parameter/rate change or clock rewrite.
        Reuse the mission's compact encoding and one fixed yaw alignment.
        """
        if not self.fusion_observe or not self.identified():
            raise PermissionError('Isolated fusion observation and current unarmed identity required')
        now = self.clock()
        clock = self.rc.fc_clock
        if not clock or not 0 <= now-clock[1] < 2:
            raise RuntimeError('Fresh FC clock missing; localization stopped')
        if not observation or not 0 <= now_ros_ns-observation['source_stamp_ns'] < 150_000_000:
            raise RuntimeError('Fresh Vicon frame missing; localization stopped')
        if self.alignment is None:
            a = self.attitude
            if not (clock and a and 0 <= now-clock[1] < 2 and 0 <= now-a['at'] < .5
                    and 0 <= clock[0]+(now-clock[1])*1e6-a['boot_us'] < 500_000):
                raise RuntimeError('Fresh FC attitude/clock missing; no localization sent')
            self.alignment = compact_vision.yaw_alignment(observation['quaternion_wxyz'], a['q'])
            self.record(dict(direction='ground_yaw_alignment', at=now, radians=self.alignment,
                             flight_authorized=False))
        stamp = observation['source_stamp_ns']//1000
        if stamp < self.last_pose_stamp:
            raise RuntimeError('Vicon timestamp regressed')
        if stamp == self.last_pose_stamp or now < self.next_pose:
            return
        msg, p, q = compact_vision.message(self.encoder, stamp, observation['position'],
                                         observation['quaternion_wxyz'], self.alignment)
        self._pose_message = msg
        tx_start_ns = time.monotonic_ns()
        try:
            wire = self.send(msg)
            tx_end_ns = time.monotonic_ns()
        finally:
            self._pose_message = None
        if self.last_pose_at is not None:
            self.max_pose_gap = max(self.max_pose_gap, now-self.last_pose_at)
        self.last_pose_at, self.last_pose_stamp, self.next_pose = now, stamp, now+.05
        self.pose_count += 1
        self.last_pose_row = dict(direction='vicon_sent', at=now,
                         source_stamp_ns=observation['source_stamp_ns'], source_frame_id='map',
                         wire_usec=stamp, wire_hex=wire.hex(), route='wifi',
                         tx_start_ns=tx_start_ns, tx_end_ns=tx_end_ns,
                         position=p, q=q, pose_fields_scope='pre-serialization; decode wire_hex for calibration')
        self.record(self.last_pose_row)


def run(*, output, peer, bind, motor_power_isolated=False, observe_seconds=15., layout_evidence=None,
        fusion_observe=False, start_ready=None, notice=print, target_load=False, offboard_check=False, timesync_check=False,
        fusion_clock_check=False, battery_static=False):
    """Explicit power-scoped ground diagnostics; no live flight route.

    Default: new-layout Vicon is received/transformed/logged only. The explicit
    fusion_observe option permits isolated ground injection, never flight.
    battery_static is a separate narrow stationary scope, never motor isolation.
    """
    sync_enabled = timesync_check or fusion_clock_check
    max_seconds = 180 if sync_enabled else 60
    if battery_static and (motor_power_isolated or not fusion_clock_check or not fusion_observe
                           or target_load or offboard_check or timesync_check or start_ready is not None
                           or observe_seconds != 60 or FC_USB.exists()):
        raise ValueError('Battery-static requires USB absent and only 60 s stationary fusion-clock observation')
    if (motor_power_isolated is not True and not battery_static) or not 0 <= observe_seconds <= max_seconds:
        raise ValueError(f'Motor power isolation and 0..{max_seconds} observation seconds required')
    if fusion_observe and (not layout_evidence or not 15 <= observe_seconds <= max_seconds):
        raise ValueError(f'Fusion observation requires current measured layout and 15..{max_seconds} seconds')
    if target_load and not fusion_observe:
        raise ValueError('Isolated target load requires the explicit fusion observation route')
    if offboard_check and (not target_load or observe_seconds<30):
        raise ValueError('Offboard check requires isolated target load and at least 30 seconds')
    if timesync_check and (not fusion_observe or target_load or offboard_check or observe_seconds<30):
        raise ValueError('Timesync check requires only isolated fusion and 30..180 seconds')
    if fusion_clock_check and (not fusion_observe or timesync_check or target_load or offboard_check or observe_seconds<60):
        raise ValueError('Fusion clock check requires only isolated fusion and 60..180 seconds')
    if start_ready is not None and (fusion_observe or not callable(start_ready) or observe_seconds<=0):
        raise ValueError('GO is only for a nonempty Pocket observation window')
    layout = None
    if layout_evidence:
        from rigid_body_revision import measured_capture_inputs
        layout = measured_capture_inputs(layout_evidence)  # Retains unapproved scope.
    output = Path(output)
    output.mkdir(mode=0o700, parents=True, exist_ok=False)
    result = dict(scope='wifi-vicon-isolated-fusion' if fusion_observe else 'pocket-wifi-motor-isolated-observation', flight_authorized=False,
                  parameter_writes=0, control_messages=0, localization_messages=0,
                  layout=None if not layout else {k: str(v) if isinstance(v, Path) else v for k, v in layout.items()},
                  observations=[], rc_observations=[], fusion_rounds=[], observation_window_complete=False, error=None)
    result.update(battery_static=battery_static,motor_power_isolated=motor_power_isolated,
                  formal_flight_runtime_validated=False)
    observer = source = None
    try:
        with ExitStack() as stack:
            log = stack.enter_context((output/'traffic.jsonl').open('x'))
            def record(row):
                log.write(json.dumps(row, ensure_ascii=False)+'\n'); log.flush()
            if layout:
                from ev_delay_vicon_source import ViconSource
                from vicon_receiver_provenance import verify_current
                from uav_coord_bridge.mavlink_vision import transform_rigid_pose
                source = ViconSource(output, verify_current(), latest_only=fusion_observe)
                def close_source():
                    try: source.close()
                    except Exception as exc:
                        # Never replace the original probe/connection failure during unwind.
                        result['source_cleanup_error'] = type(exc).__name__+': '+str(exc)
                        if result['error'] is None:
                            result['error'] = result['source_cleanup_error']
                stack.callback(close_source)
                frames = stack.enter_context((output/'vicon_frames.jsonl').open('x'))
                def frame(row, at):
                    frames.write(json.dumps(dict(at=at, frame=row))+'\n'); frames.flush()
                    age = source.now_ros_ns()-row['ros_stamp_ns']
                    result['last_source_valid'] = bool(row['sdk_ok'] and not row['occluded']
                        and row['pose_published'] and not row['queue_dropped'] and 0 <= age < 150_000_000
                        and row['subject'] == row['segment'] == 'MircoAir' and row['frame_id'] == 'map')
                    if result['last_source_valid']:
                        p, q = transform_rigid_pose(row['world_pose'][:3], row['world_pose'][3:],
                                                    layout['origin'], layout['orientation'])
                        result['last_pose_observation'] = dict(at=at, position=p, quaternion_wxyz=q,
                            source_age_ns=age, source_stamp_ns=row['ros_stamp_ns'], sent_to_fc=False)
                    else:
                        result['last_pose_observation'] = None
                source.on_frame = frame
            port = FixedPeerUDP(peer, bind, on_datagram=lambda data, addr, accepted:
                record(dict(direction='rx_datagram', at=time.monotonic(), peer=addr,
                            accepted=accepted, wire_hex=data.hex())))
            stack.callback(port.close)
            observer = GroundObserver(port, record, fusion_observe=fusion_observe,
                ground_target_load=target_load,offboard_check=offboard_check,
                timesync_check=sync_enabled,ros_clock_ns=source.now_ros_ns if source else None,
                battery_static=battery_static)
            def restore():
                try:observer.restore_ground_mode()
                except Exception as exc:
                    result['mode_restore_error']=str(exc)
                    record(dict(direction='mode_restore_error',error=str(exc)))
            if offboard_check:stack.callback(restore)
            send_poses = False
            window_deadline = None
            def pump():
                observer.pump()
                if source: source.poll()
                if send_poses:
                    if window_deadline is not None and time.monotonic() >= window_deadline:
                        raise RuntimeError('Fusion window ended during incomplete diagnostics; evidence saved')
                    observer.pose(result.get('last_pose_observation'), source.now_ros_ns())
                    if target_load:observer.ground_target_tick()
                time.sleep(.002)
            deadline = time.monotonic()+8
            while not observer.identified():
                pump()
                if time.monotonic() > deadline: raise RuntimeError('No current aircraft identity')
            if battery_static:
                while not observer.battery_ground_current():
                    pump()
                    if time.monotonic()>deadline:raise RuntimeError('No current landed battery state')
            # Read once, never rewrite or request parameter lists/stream rates.
            requested = () if target_load or sync_enabled else FUSION_PARAMS if fusion_observe else PARAMS
            for name in requested:
                observer.requested_parameters.add(name)
                observer.send(observer.encoder.param_request_read_encode(1, 1, name.encode(), -1))
                pump()
            def query(topic):
                payload = (SYNC_COMMANDS if topic in SYNC_COMMANDS else FUSION_COMMANDS if topic in FUSION_COMMANDS else COMMANDS)[topic]
                observer.shell_buffer = ''; observer.shell(payload)
                deadline = time.monotonic()+6; next_poll = time.monotonic()+.25
                while True:
                    pump(); now = time.monotonic()
                    reply = current_reply(observer.shell_buffer, payload)
                    if reply is not None:
                        row = dict(topic=topic, at=now, text=reply)
                        if topic in observer.rc.TOPICS:
                            row['parsed'] = observer.rc.observe_topic(topic, reply, now)
                        if topic in FUSION_COMMANDS:
                            row['parsed'] = (parse_visual_snapshot(reply) if topic == 'visual' else
                                parse_topic(reply,topic,integer_fields=('timestamp',),bool_fields=('position','velocity','acceleration','attitude','body_rate'))
                                if topic=='offboard_control_mode' else parse_audit(topic, reply))
                        if topic == 'vehicle_status':
                            row['parsed'] = parse_audit('commander', reply)
                        if topic in SYNC_COMMANDS:
                            row['parsed'] = parse_timesync_snapshot(reply)
                        parsed = row.get('parsed')
                        fc_clock = observer.rc.fc_clock
                        row['source_age_s'] = None
                        if parsed and 'timestamp' in parsed and fc_clock and 0 <= now-fc_clock[1] < 2:
                            row['source_age_s'] = (fc_clock[0]+(now-fc_clock[1])*1e6-parsed['timestamp'])/1e6
                        result['observations'].append(row)
                        record(dict(direction='snapshot', **row))
                        if target_load and topic=='local_position':observer.target_local=row
                        return row
                    if now > deadline: raise RuntimeError('Incomplete reply: '+topic)
                    if now >= next_poll: observer.shell(); next_poll = now+.25
            if sync_enabled:
                from types import SimpleNamespace
                from short_status_transport import ShortStatusTransport
                from preparation_authorization import RateRecoveryJournal
                query('actuator_armed')
                if not observer.rc.topics['actuator_armed']['value'] or observer.rc.topics['actuator_armed']['value']['armed']:
                    raise RuntimeError('Current disarmed actuator snapshot required')
                # Fail before rate changes if the source needed by the final probe is absent.
                source_deadline = time.monotonic()+5
                while not result.get('last_pose_observation'):
                    pump()
                    if time.monotonic() >= source_deadline:
                        raise RuntimeError('No fresh Vicon frame; timesync rate not changed')
                journal=RateRecoveryJournal(output/'rates_recovery.json',scope=(
                    'wifi-battery-static-timesync-no-flight' if battery_static else 'wifi-motor-isolated-timesync-no-flight'))
                endpoint=SimpleNamespace(status=SimpleNamespace(source=(1,1)),clock=SimpleNamespace(target=(250,220)))
                if battery_static:
                    from mission_telemetry_rates import TemporaryTelemetryRates
                    observer.sync_lifecycle=True
                    rate=observer.sync_rate=TemporaryTelemetryRates(component=220,persist=journal,profile='timesync')
                else:
                    rate=observer.sync_rate=ShortStatusTransport(endpoint,observer.encoder,journal,message_id=111,interval_us=100000)
                def restore_sync():
                    observer.sync_restoring=True
                    def restore_pump():
                        # A failed Vicon source must not block restoring a live FC link.
                        observer.pump()
                        time.sleep(.002)
                    try:
                        if battery_static:
                            rate.begin_restore(time.monotonic())
                            deadline=time.monotonic()+10
                            while rate.phase not in ('CLOSED','RESTORE_FAILED'):
                                restore_pump()
                                if time.monotonic()>deadline:raise RuntimeError('Battery-static rate restoration timeout')
                            if rate.phase!='CLOSED':raise RuntimeError('Battery-static rate restoration failed')
                        else:rate.restore(observer.send,restore_pump,time.monotonic_ns)
                    except Exception as exc: result['timesync_restore_error']=str(exc)
                    finally: result['timesync_rate']=rate.summary()
                stack.callback(restore_sync)
                if battery_static:
                    deadline=time.monotonic()+10
                    while rate.phase!='ACTIVE':
                        pump()
                        if rate.error:raise RuntimeError(rate.error)
                        if time.monotonic()>deadline:raise RuntimeError('Battery-static rate setup timeout')
                else:rate.configure(observer.send,pump,time.monotonic_ns)
            if timesync_check:
                started=time.monotonic(); end=started+observe_seconds
                result.update(scope='wifi-same-channel-timesync-check',window_started_at=started,
                    timesync_status=[],clock_probe=None,clock_probes=[],timestamp_mapping_verified=False,
                    fc_accepted_sample_count=None,sync_timeout_s=observe_seconds,
                    sync_stop_reason='timeout',clock_stability_candidate_pass=False)
                notice(f'连续 WiFi 时间同步开始，最多 {observe_seconds:g} 秒，稳定后提前结束；保持机身静止。')
                slot=None; readiness=ClockReadiness(); ready=False
                next_snapshot=started+2
                next_probe=started
                def probe_once():
                    # Real source timestamp, deliberately held; never invent a camera time.
                    pump(); held=result.get('last_pose_observation')
                    if held is None:
                        raise RuntimeError('Fresh Vicon frame missing before timestamp probe')
                    until=time.monotonic()+.05
                    while time.monotonic()<until:pump()
                    observer.pose(held,source.now_ros_ns())
                    snapshot=query('visual').get('parsed')
                    status=query(slot).get('parsed')
                    sent=dict(observer.last_pose_row,wire_time_usec=observer.last_pose_row['wire_usec'])
                    own=bool(status and status['remote_timestamp'] in {r['pc_reply_ns']//1000 for r in observer.sync_replies})
                    probe=check_clock_probe(status if own else None,snapshot,sent,source.now_ros_ns()//1000,
                        expected_pose_frame=1)
                    result['clock_probe']=dict(probe,sent=sent,snapshot=snapshot,status=status)
                    result['clock_probes'].append(result['clock_probe'])
                    result['timestamp_mapping_verified']=probe['verified']
                    return probe
                while time.monotonic()<end:
                    pump()
                    if time.monotonic()<next_snapshot: continue
                    for name in ([slot] if slot else SYNC_COMMANDS):
                        row=query(name); status=row.get('parsed')
                        # Exact remote timestamp must belong to our WiFi replies,
                        # not another MAVLink channel's uORB instance.
                        own=bool(status and status['remote_timestamp'] in {r['pc_reply_ns']//1000 for r in observer.sync_replies})
                        if own:
                            slot=name; result['timesync_status'].append(row)
                            ready=readiness.update(status,source.now_ros_ns()//1000)
                            break
                    next_snapshot=time.monotonic()+5
                    if ready and time.monotonic() >= next_probe:
                        probe=probe_once()
                        next_probe=time.monotonic()+10
                        if probe.get('clock_accuracy_candidate_pass'):
                            result.update(sync_stop_reason='stable_mapping',clock_stability_candidate_pass=True)
                            break
                        # A bad probe invalidates the preceding stable-series candidate.
                        readiness=ClockReadiness(); ready=False
                result['timestamp_probe_prerequisites_met']=ready
                # Reply count is NOT the FC's accepted-sample count. A fresh internal
                # sample mapping, plus stable residuals, supplies the evidence instead.
                if slot is not None and result['sync_stop_reason']=='timeout' and time.monotonic()>=next_probe:
                    probe_once()
                result.update(window_completed_at=time.monotonic(),observation_window_complete=True,
                    timesync_response_count=len(observer.sync_replies))
                return result
            if fusion_observe:
                # No RC scans, HOME, arming, log/rate changes or flight readiness here.
                query('actuator_armed')
                actuator = observer.rc.topics['actuator_armed']['value']
                if not actuator or actuator['armed']:
                    raise RuntimeError('Current disarmed actuator snapshot required')
                deadline = time.monotonic()+5
                while not (result.get('last_pose_observation') and observer.attitude and observer.rc.fc_clock):
                    pump()
                    if time.monotonic() > deadline: raise RuntimeError('Vicon/FC attitude/clock not available')
                send_poses = True
                pump()
                started = time.monotonic(); end = started+observe_seconds
                window_deadline = end+.01
                result['window_started_at'] = started
                if target_load:
                    # Reuse proven fusion, do not run its full aid-source scan.
                    # Give the restarted estimator input time to settle before
                    # freezing a current FC-local ground origin once.
                    while time.monotonic()<started+5:pump()
                    status=query('vehicle_status');actuator=query('actuator_armed')
                    local=query('local_position')
                    observer.freeze_ground_target(local,status,actuator)
                    if offboard_check:
                        def refresh_local():
                            row=query('local_position')
                            # Diagnostic shell traffic must leave bandwidth for
                            # pose, target, heartbeat and clock traffic.
                            until=time.monotonic()+.3
                            while time.monotonic()<until:pump()
                            return row
                        def wait_mode(request):
                            while time.monotonic()-request['at']<4:
                                refresh_local()
                                if observer.mode_confirmed(request):return
                                pump()
                            raise RuntimeError(f"Mode {request['main']} ACK/state not confirmed within 4 s")
                        notice('Ground mode check: fixed targets, one Offboard request, then one Position return; no arm.')
                        while time.monotonic()-observer.target_started<2.1:refresh_local()
                        wait_mode(observer.request_ground_mode(6))
                        result['offboard_status']=query('vehicle_status')
                        held_until=time.monotonic()+1
                        while time.monotonic()<held_until:refresh_local()
                        wait_mode(observer.request_ground_mode(3))
                        for topic in ('vehicle_status','commander','actuator_armed'):query(topic)
                        result['window_completed_at']=time.monotonic()
                        result['observation_window_complete']=True
                        return result
                    notice('Fixed ground-origin load started: 3 s at up to 20 Hz; remain disarmed in Position.')
                    receipt=None
                    while time.monotonic()-observer.target_started<3:
                        query('local_position')
                        if receipt is None and time.monotonic()-observer.target_started>=1.3:
                            receipt=query('offboard_control_mode')
                            result['during_load_failsafe']=query('failsafe')
                        until=min(observer.target_started+3,time.monotonic()+.1)
                        while time.monotonic()<until:pump()
                    result['target_receipt']=receipt
                    for topic in ('vehicle_status','commander','actuator_armed'):query(topic)
                    result['window_completed_at']=time.monotonic()
                    result['observation_window_complete']=bool(receipt and len(observer.target_rows)>=40)
                    return result  # finally saves evidence and closes the same owner.
                slots = {}
                previous_primary = None
                sync_slot = None
                if fusion_clock_check:
                    result.update(scope='wifi-battery-static-sync-fusion' if battery_static else 'wifi-continuous-sync-fusion-effect',timesync_status=[],
                                  stationary_only=True,flight_control_effect_verified=False)
                    notice('持续定位与同步融合观察开始；保持静止，不解锁、不切模式、不发运动目标。')
                    while time.monotonic()<started+5:pump()
                # Two complete, separate snapshots; keep sending fresh poses between/after them.
                while len(result['fusion_rounds']) < (3 if fusion_clock_check else 2):
                    first = query('selector')
                    selector = first.get('parsed')
                    if not selector or not 0 <= selector['primary_instance'] < 9:
                        raise RuntimeError('Active estimator selector unavailable')
                    primary = selector['primary_instance']
                    if primary != previous_primary: slots.clear()
                    previous_primary = primary
                    round_ = dict(selector_before=first, primary=primary, aids={})
                    round_['flags'] = query(f'flags_{primary}')
                    for prefix in ('aid', 'height_aid', 'yaw_aid'):
                        candidates = [slots[prefix]] if prefix in slots else range(9)
                        for slot in candidates:
                            row = query(f'{prefix}_{slot}')
                            parsed = row.get('parsed')
                            if parsed and parsed['estimator_instance'] == primary:
                                slots[prefix] = slot; round_['aids'][prefix] = row; break
                        if prefix not in round_['aids']: slots.pop(prefix, None)
                    for topic in ('visual', 'local_position', 'failsafe', 'vehicle_status', 'health_report', 'commander'):
                        round_[topic] = query(topic)
                        if fusion_clock_check and topic=='visual':
                            for name in ([sync_slot] if sync_slot else SYNC_COMMANDS):
                                row=query(name);status=row.get('parsed')
                                own=bool(status and status['remote_timestamp'] in {r['pc_reply_ns']//1000 for r in observer.sync_replies})
                                if own:
                                    sync_slot=name;round_['timesync']=row
                                    result['timesync_status'].append(row);break
                    round_['selector_after'] = query('selector')
                    after = round_['selector_after'].get('parsed')
                    round_['selector_stable'] = bool(after and all(after[k] == selector[k]
                        for k in ('primary_instance', 'instance_changed_count')))
                    round_['completed_at'] = time.monotonic()
                    result['fusion_rounds'].append(round_)
                    record(dict(direction='fusion_round', **round_))
                    if fusion_clock_check:
                        offsets=(observe_seconds/2,observe_seconds-15,observe_seconds)
                        wait_until=started+offsets[len(result['fusion_rounds'])-1]
                    else:
                        wait_until = max(started+observe_seconds/2, round_['completed_at']+1)
                    while time.monotonic() < min(end, wait_until): pump()
                while time.monotonic() < end: pump()
                result['window_completed_at'] = time.monotonic()
                result['observation_window_complete'] = True
            else:
                for topic in COMMANDS: query(topic)
                if start_ready is not None:
                    result['waiting_for_go_at']=time.monotonic()
                    notice('Pocket 只读观察已准备好。先不拨杆；准备好后在本终端输入 GO 回车，STOP 可取消。')
                    deadline=time.monotonic()+120
                    while not start_ready():
                        pump()
                        if time.monotonic()>=deadline:raise RuntimeError('GO wait timed out; observation window not started')
                    result['go_at']=time.monotonic()
                    # Refresh topics through the normal window, no second precheck.
                result['window_started_at']=time.monotonic()
                notice(f'开始 Pocket 只读观察：{observe_seconds:g} 秒。只做本次约定的动作；不解锁，不移动机身。')
            # Only NEW Pocket evidence; never rerun the prior gamepad tests.
            end = time.monotonic()+(0 if fusion_observe else observe_seconds)
            while time.monotonic() < end:
                cycle_end = min(end, time.monotonic()+1.)
                for topic in observer.rc.TOPICS:
                    if time.monotonic() >= end: break
                    query(topic)
                row=dict(at=time.monotonic(),status=observer.rc.status(time.monotonic()))
                record(dict(direction='native_rc_status', **row))
                # A last shell reply may finish after the window; preserve its
                # raw evidence but do not count it as an in-window response.
                if row['at']<=end:result['rc_observations'].append(row)
                while time.monotonic() < cycle_end: pump()
            if not fusion_observe:
                result['window_deadline_at']=end
                result['window_completed_at']=time.monotonic()
                result['observation_window_complete']=True
                notice('Pocket 观察结束。保存本次原始数据和差异报告；不会自动重试。')
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = type(exc).__name__+': '+str(exc)
    finally:
        if observer:
            result.update(identity=observer.identity, native_rc=observer.rc.status(time.monotonic()),
                          parameters=observer.parameters,
                          parameter_readbacks=observer.parameter_readbacks,
                          missing_parameters=sorted(set(FUSION_PARAMS if fusion_observe else PARAMS)-set(observer.parameters)),
                          status_texts=observer.status_texts,
                          tx_counts=observer.tx_counts, crc=observer.parser.summary(),
                          localization_messages=observer.pose_count, max_pose_send_gap_s=observer.max_pose_gap,
                          calibration_approved=False, fusion_assessment='raw snapshots; not flight approval')
            if target_load:
                result.update(scope='wifi-vicon-isolated-fixed-ground-target-load',
                    target_origin=observer.target_origin,target_rows=observer.target_rows,
                    target_messages=len(observer.target_rows),mode_or_arm_commands_sent=0,
                    control_messages=len(observer.target_rows),
                    control_scope='Frozen ground XYZ/yaw setpoints only; no mode/arm/altitude increment',
                    missing_parameters=[])
                if offboard_check:
                    result.update(scope='wifi-vicon-isolated-offboard-entry-exit',
                        mode_requests=observer.mode_requests,mode_acks=observer.mode_acks,
                        mode_or_arm_commands_sent=sum('wire_hex' in r for r in observer.mode_requests),
                        control_messages=len(observer.target_rows)+sum('wire_hex' in r for r in observer.mode_requests),
                        control_scope='Frozen ground XYZ/yaw plus one Offboard entry and one Position return; never arm')
            result['diagnostic_gaps'] = []
            if fusion_clock_check:
                result['missing_parameters']=[]
                result['timesync_response_count']=len(observer.sync_replies)
                if result.get('timesync_restore_error'):
                    result['diagnostic_gaps'].append('timesync_interval_restore_unverified')
                if any('timesync' not in row for row in result['fusion_rounds']):
                    result['diagnostic_gaps'].append('fusion_round_clock_snapshot_missing')
            if timesync_check:
                result['missing_parameters']=[]
                result['timesync_response_count']=len(observer.sync_replies)
                result['clock_accuracy_candidate_pass']=bool(
                    (result.get('clock_probe') or {}).get('clock_accuracy_candidate_pass'))
                if not result.get('timestamp_mapping_verified'):
                    result['diagnostic_gaps'].append('wifi_fc_sample_time_mapping_not_verified')
                if not result['clock_accuracy_candidate_pass']:
                    result['diagnostic_gaps'].append('wifi_clock_accuracy_not_verified')
                if not result.get('clock_stability_candidate_pass'):
                    result['diagnostic_gaps'].append('wifi_clock_stability_not_verified')
                if result.get('timesync_restore_error'):
                    result['diagnostic_gaps'].append('timesync_interval_restore_unverified')
            if not fusion_observe:
                result['parameter_review']=review_parameters(observer.parameter_readbacks)
                result['observation_summary']=summarize_observation(result['rc_observations'])
                if not result['observation_window_complete']:result['diagnostic_gaps'].append('incomplete_window')
                if not result['rc_observations']:result['diagnostic_gaps'].append('no_in_window_rc_snapshots')
                elif not result['observation_summary']['native_input_samples']:
                    result['diagnostic_gaps'].append('no_native_rc_in_window')
                if result['missing_parameters']:result['diagnostic_gaps'].append('missing_parameter_readback')
            if fusion_observe:
                if result['missing_parameters']: result['diagnostic_gaps'].append('missing_parameter_readback')
                if not result['observation_window_complete']: result['diagnostic_gaps'].append('incomplete_window')
                for i, row in enumerate(result['fusion_rounds']):
                    if not row['selector_stable']: result['diagnostic_gaps'].append(f'round_{i}:selector_changed')
                    for name, item in [('flags', row['flags']), ('visual', row['visual']),
                                       ('local_position', row['local_position']),
                                       ('vehicle_status', row['vehicle_status'])]:
                        if not item.get('parsed') or item.get('source_age_s') is None or not 0 <= item['source_age_s'] <= 2:
                            result['diagnostic_gaps'].append(f'round_{i}:{name}_missing_or_stale')
                    if len(row['aids']) != 3: result['diagnostic_gaps'].append(f'round_{i}:missing_selected_aid')
        if source: result['vicon_receiver'] = source.report
        if result.get('source_cleanup_error'):
            result.setdefault('diagnostic_gaps', []).append('vicon_source_cleanup_failed')
        (output/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    return result
