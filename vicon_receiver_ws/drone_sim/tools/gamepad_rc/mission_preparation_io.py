"""Single nonblocking port adapter for preparation; NO device opener or CLI.

An owning host supplies an already exclusively opened, nonblocking port plus
nonblocking gamepad/latest-pose readers. Default output is ground read/stop
only. Physical pose/configuration writes require a separate reviewed per-frame
authorizer; PARAM_SET additionally requires a successful write-ahead journal.
No such authorization is installed by this module or the offline demo.
All flight, unlock, mode and reboot commands are ALWAYS rejected here. The
optional, separately authorized rate lifecycle only accepts the exact pending
GET/SET for the existing fixed three telemetry IDs and durable recovery log.
"""
import math
from collections import Counter

from pymavlink.dialects.v20 import common as m

from mavlink_crc_stream import CrcResyncParser
from mission_fusion_service import context_from_inputs
from mission_audit_collector import allowed as audit_allowed
from mavlink_readonly_probe import decode_parameter
import trs_vision_reference_policy as vision


class PreparationIO:
    def __init__(self, runtime, port, *, poll_gamepad, poll_pose, usb_present,
                 authorize=None, journal=None, rates_journal=None, observer_port=None):
        if getattr(runtime,'fixed_configuration',False):
            raise PermissionError('Plan B mission preparation is OFFLINE ONLY; no physical adapter enabled')
        if port.timeout!=0 or port.write_timeout!=0:
            raise ValueError('One nonblocking port required: timeout=write_timeout=0')
        if not all(callable(f) for f in (poll_gamepad,poll_pose,usb_present)):
            raise ValueError('Nonblocking readers and current USB presence callback required')
        self.runtime,self.port=runtime,port
        if observer_port is not None and (not runtime.observer_required or observer_port.timeout!=0
                or observer_port.write_timeout!=0 or observer_port is port):
            raise ValueError('Explicit distinct nonblocking USB observer required')
        if runtime.observer_required and observer_port is None:raise ValueError('USB-ground observer port missing')
        self.observer_port=observer_port
        self.poll_gamepad,self.poll_pose,self.usb_present=poll_gamepad,poll_pose,usb_present
        self.authorize,self.journal=authorize,journal
        self.rates_journal=rates_journal
        self.decoder=CrcResyncParser()
        # Attaching an adapter must NEVER replay earlier offline captures.
        self.seen=sum(runtime.sent_counts.values());self.failure=None
        self.physical_tx_bytes=0;self.physical_messages=0
        self.possibly_changed=set()
        self.route_messages=Counter();self.route_bytes=Counter()

    @staticmethod
    def _stop(msg):
        return (msg.get_type()=='COMMAND_LONG' and msg.command==185
            and (msg.target_system,msg.target_component)==(1,1)
            and msg.confirmation in (0,1,2)
            and tuple(getattr(msg,f'param{i}') for i in range(1,8))==(2,0,0,0,0,0,0))

    def _dispatch(self, record):
        wire=bytes.fromhex(record['wire_hex'])
        decoded=self.decoder.parse_buffer(wire)
        if len(decoded)!=1 or decoded[0].get_type()=='BAD_DATA':
            raise ValueError('Exactly one CRC-valid captured frame required')
        msg=decoded[0];kind=msg.get_type()
        if (msg.get_srcSystem(),msg.get_srcComponent())!=(250,220) or kind!=record['type']:
            raise ValueError('Captured producer identity changed')
        r=self.runtime;c=r.context()
        route=record.get('route','trs')
        if route not in ('trs','usb') or route=='usb' and self.observer_port is None:
            raise PermissionError('Unbound ground diagnostic route')
        if r.observer_required:
            expected='usb' if kind in ('SERIAL_CONTROL','PARAM_REQUEST_READ','PARAM_SET') else 'trs'
            startup=kind=='HEARTBEAT' or kind=='COMMAND_LONG' and msg.command==512
            if route!=expected and not (route=='usb' and startup):
                raise PermissionError('Wrong ground diagnostic/pose route')
            if not startup and not r.observer_ok():raise PermissionError('Independent USB identity not current')
        is_stop=self._stop(msg)
        if self.failure and not is_stop:
            return  # Never replay skipped/partially written frames on recovery.
        if not self.usb_present():
            raise PermissionError('This preparation adapter requires FC USB; no battery-only/flight scope')
        accepted=False
        if kind=='HEARTBEAT':
            accepted=(msg.type,msg.autopilot,msg.base_mode,msg.custom_mode)==(m.MAV_TYPE_GCS,m.MAV_AUTOPILOT_INVALID,0,0)
        elif kind=='COMMAND_LONG':
            params=tuple(getattr(msg,f'param{i}') for i in range(1,8))
            accepted=((msg.target_system,msg.target_component)==(1,1)
                and (is_stop and r.home_check_requested and r._identity_ok()
                     or msg.command==512 and msg.confirmation==0 and params==(148,0,0,0,0,0,0)))
            if msg.command in (510,511):
                if (self.authorize is None or self.authorize(r,record,msg) is not True
                        or not r.rates or not r.rates.permits(msg,fresh=r._identity_ok(),
                            locked=r.coordinator.audit.live_lockdown(r.now) is True)):
                    raise PermissionError('No exact separately authorized temporary-rate transaction')
                if msg.command==511 and (self.rates_journal is None
                        or not self.rates_journal.covers(r.rates,msg)):
                    raise PermissionError('Durable exact-rate recovery journal required')
                accepted=True
        elif kind=='SERIAL_CONTROL':accepted=r._identity_ok() and audit_allowed(msg)
        elif kind=='PARAM_REQUEST_READ':
            name=msg.param_id.rstrip('\0')
            accepted=r._identity_ok() and name in vision.READ_PARAMS and (
                msg.target_system,msg.target_component,msg.param_index)==(1,1,-1)
        elif kind in ('PARAM_SET','VISION_POSITION_ESTIMATE'):
            if self.authorize is None or self.authorize(r,record,msg) is not True:
                raise PermissionError('No separate physical preparation send authorization installed')
            s=r.coordinator.fusion
            restoring=kind=='PARAM_SET' and s.phase in ('WAIT_RESTORE_CTRL','WAIT_RESTORE_DELAY')
            if not (c.restore_ground() if restoring else c.locked_ground()):
                raise PermissionError('Fresh known locked ground state required for preparation writes')
            if not s._bound(c):raise PermissionError('Preparation epoch changed')
            if kind=='PARAM_SET':
                name=msg.param_id.rstrip('\0');value=decode_parameter(msg)
                if (not s.pending or name!=s.pending['name'] or value!=s.pending['value']
                        or name not in ('EKF2_EV_CTRL','EKF2_EV_DELAY')
                        or (msg.target_system,msg.target_component)!=(1,1)
                        or msg.param_type!=(m.MAV_PARAM_TYPE_INT32 if name=='EKF2_EV_CTRL' else m.MAV_PARAM_TYPE_REAL32)):
                    raise PermissionError('Not the exact pending fusion transaction')
                entry=dict(name=name,value=value,at=r.now,boot_session=r.inputs.boot_session,
                    rigid_revision=r.inputs.rigid_revision,identity=r.inputs.identity,
                    baseline=s.baseline,wire_hex=record['wire_hex'])
                if self.journal is None or self.journal(entry) is not True:
                    raise PermissionError('Durable write-ahead journal confirmation required')
                self.possibly_changed.add(name)  # BEFORE possibly partial write.
            elif (msg.usec!=r.inputs.pose_stamp//1000 or msg.reset_counter!=0
                  or not all(math.isfinite(v) for v in (msg.x,msg.y,msg.z,msg.roll,msg.pitch,msg.yaw,*msg.covariance))):
                raise PermissionError('Not a finite current-source pose')
            accepted=True
        if not accepted:raise PermissionError('Outside ground preparation transport scope: '+kind)
        # Durability callbacks may take time. Recheck current physical-send
        # authorization AFTER the journal, immediately before actual bytes.
        privileged=(kind in ('PARAM_SET','VISION_POSITION_ESTIMATE')
            or kind=='COMMAND_LONG' and msg.command in (510,511))
        if privileged and self.authorize(r,record,msg) is not True:
            raise PermissionError('Preparation authorization expired before write')
        try:
            written=(self.observer_port if route=='usb' else self.port).write(wire)
        except BaseException:
            self.failure=self.failure or 'write_interrupted_or_failed_delivery_uncertain'
            self.runtime._fail(self.failure)
            raise
        if type(written) is int and written>0:self.physical_tx_bytes+=written
        if written!=len(wire):
            self.failure=self.failure or 'short_write_delivery_uncertain'
            self.runtime._fail(self.failure)
            raise OSError('Short nonblocking write; delivery uncertain, never automatically retry')
        self.physical_messages+=1
        self.route_messages[route]+=1;self.route_bytes[route]+=written

    def step(self, now, now_ros_ns):
        """One turn; never opens/closes ports or sleeps waiting for replies.

        poll_gamepad returns current generation/presence/event/HOME kwargs;
        poll_pose returns only the newest raw pose or None. Reads must not wait.
        A read failure still feeds any collected HOME event and services stop.
        """
        pad={};rx=b'';observer_rx=b'';pose=None;problem=None
        try:pad=self.poll_gamepad()
        except (OSError,ValueError,RuntimeError) as exc:problem=exc
        try:
            rx=self.port.read(self.runtime.max_rx_bytes)
            if self.observer_port is not None:observer_rx=self.observer_port.read(self.runtime.max_rx_bytes)
            pose=self.poll_pose()
            if callable(now_ros_ns):now_ros_ns=now_ros_ns()
        except (OSError,ValueError,RuntimeError) as exc:problem=exc
        if callable(now_ros_ns):now_ros_ns=0  # Clock failure is a latched input fault, never a fresh pose.
        if problem:
            self.failure=self.failure or 'input_read_failed: '+str(problem)
            self.runtime._fail(self.failure)
        self.runtime.turn(now,rx=rx,observer_rx=observer_rx,now_ros_ns=now_ros_ns,pose=pose,**pad)
        total=sum(self.runtime.sent_counts.values());count=total-self.seen
        frames=list(self.runtime.frames)
        if count>len(frames):
            self.failure=self.failure or 'captured_frame_overrun'
            self.runtime._fail(self.failure)
            self.seen=total
            return self.status()
        new=frames[-count:] if count else []
        self.seen=total  # A failed frame is never replayed after reconnect.
        for record in new:
            try:self._dispatch(record)
            except (ValueError,PermissionError,OSError,RuntimeError,TypeError) as exc:
                self.failure=self.failure or type(exc).__name__+': '+str(exc)
                self.runtime._fail(self.failure)
                # Continue scanning only so a later stop frame is not swallowed.
        return self.status()

    def status(self):
        return dict(scope='USB-present ground preparation I/O; no flight or default fusion authorization',
            failure=self.failure,physical_tx_bytes=self.physical_tx_bytes,physical_messages=self.physical_messages,
            physical_messages_by_route=dict(self.route_messages),physical_bytes_by_route=dict(self.route_bytes),
            separately_authorized_preparation=self.authorize is not None,
            parameter_recovery_required=bool(self.possibly_changed and (self.failure or not self.runtime.closed)),
            possibly_changed=sorted(self.possibly_changed),flight_authorized=False,
            port_ownership='external caller retains exclusive port and close responsibility')
