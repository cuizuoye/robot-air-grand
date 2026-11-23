"""Full one-owner transport API, default DENY for every possible flight effect.

This module opens no devices and creates no authorizer. A reviewed future
host must inject resources, a current per-frame authorizer, and a durable
journal. Native observe/query CLI deliberately cannot construct this adapter.
Tests use only fake ports/inputs. No blanket execute flag or hold bypass here.
"""
from collections import Counter
from threading import get_ident
import math

from pymavlink.dialects.v20 import common as m

from continuous_mission_runtime import ContinuousMissionRuntime
from mavlink_crc_stream import CrcResyncParser
from mission_audit_collector import allowed as audit_allowed
from flight_execution_journal import FlightExecutionJournal
import trs_vision_reference_policy as vision
import compact_vision


class ContinuousExecutionIO:
    def __init__(self,runtime,port,*,poll_gamepad,poll_pose,now_ros_ns,
                 check_port_identity,usb_present,authorizer=None,journal=None,fixed_profile=None):
        if getattr(runtime,'fixed_configuration',False):
            # Binding enables this adapter, never an output authorization.
            # Check before touching any supplied resource.
            if fixed_profile is None:
                raise PermissionError('Plan B is OFFLINE ONLY without an explicit fixed-profile binding')
            from ekf_static_profile import require_mission_profile, fingerprint
            require_mission_profile(fixed_profile,pilot_source=runtime.pilot_source,
                height=runtime.core.preview.height,hover_seconds=runtime.core.preview.hover_seconds)
            if (fingerprint(fixed_profile)!=runtime.coordinator.fusion.profile_sha256
                    or type(journal) is not FlightExecutionJournal):
                raise PermissionError('Matching fixed profile and durable journal required')
        elif fixed_profile is not None:
            raise ValueError('Fixed profile cannot be attached to a legacy runtime')
        if type(runtime) is not ContinuousMissionRuntime:
            raise ValueError('Exact continuous runtime required, never upgrade a receive-shadow session')
        if port.timeout!=0 or port.write_timeout!=0:
            raise ValueError('One exclusive nonblocking port required')
        if getattr(runtime,'device_adapter_attached',False):raise ValueError('Second transport owner forbidden')
        if not all(callable(f) for f in (poll_pose,now_ros_ns,check_port_identity,usb_present)):
            raise ValueError('Nonblocking bindings and current physical USB checker required')
        if (runtime.pilot_source=='native_rc' and poll_gamepad is not None
                or runtime.pilot_source!='native_rc' and not callable(poll_gamepad)):
            raise ValueError('Native RC has no gamepad device binding')
        if authorizer is not None and not callable(authorizer):raise ValueError('Per-frame authorizer must be callable')
        if journal is not None and type(journal) is not FlightExecutionJournal:
            raise ValueError('Durable execution journal required, not a boolean callback')
        runtime.device_adapter_attached=True
        self.runtime,self.port=runtime,port
        self.poll_gamepad,self.poll_pose=poll_gamepad,poll_pose
        self.now_ros_ns,self.check_port_identity=now_ros_ns,check_port_identity
        self.usb_present,self.authorizer,self.journal=usb_present,authorizer,journal
        self.capture=runtime.writer
        self.writer=self.send
        self.owner=get_ident();self.closed=False
        self.failure=None;self.delivery_uncertain=False
        self.actual_tx_bytes=self.complete_messages=self.rx_bytes=0
        self.types=Counter();self.home_down=False;self.home_presses=0
        self.decoder=CrcResyncParser()
        self.latest=None;self.last_pad_generation=None
        runtime.writer=self.writer
        runtime.coordinator.writer=self.writer
        runtime.coordinator.fusion.sink=self.writer
        runtime.core.sink=self.writer

    def _owner(self):
        if get_ident()!=self.owner:raise RuntimeError('One immutable execution I/O thread required')
        if self.closed:raise RuntimeError('Execution I/O closed; no automatic reconnection')

    @staticmethod
    def _readonly(msg,runtime):
        kind=msg.get_type()
        if kind=='HEARTBEAT':
            return (msg.type,msg.autopilot,msg.base_mode,msg.custom_mode)==(m.MAV_TYPE_GCS,m.MAV_AUTOPILOT_INVALID,0,0)
        if kind=='COMMAND_LONG':
            if msg.command==510 and runtime.sync_rate_allowed(msg):return True
            return ((msg.target_system,msg.target_component,msg.command,msg.confirmation)==(1,1,512,0)
                and tuple(getattr(msg,f'param{i}') for i in range(1,8))==(148,0,0,0,0,0,0))
        if not runtime._identity_ok():return False
        if kind=='TIMESYNC' and runtime.fixed_configuration:
            return runtime.fixed_timesync_allowed(msg)
        if kind=='SERIAL_CONTROL':return audit_allowed(msg)
        if kind=='PARAM_REQUEST_READ':
            return msg.param_id.rstrip('\0') in runtime.coordinator.read_params and (
                msg.target_system,msg.target_component,msg.param_index)==(1,1,-1)
        return False

    def _authorized(self,record,msg):
        r=self.runtime
        if (msg.get_type()=='COMMAND_LONG' and msg.command==511 and r.sync_rates
                and r.sync_rates.restoring and r.sync_rate_allowed(msg)):
            # Exact original interval only, after a known-safe ground state;
            # restoration must survive Vicon loss or review expiry.
            from preparation_authorization import RateRecoveryJournal
            return bool(type(r.sync_rates.persist) is RateRecoveryJournal
                and r.sync_rates.persist.covers(r.sync_rates,msg)
                and not self.journal.failed and not self.journal.closed
                and record['at']==r.now)
        return bool(self.authorizer is not None and self.journal is not None
            and not self.journal.failed and not self.journal.closed
            and not self.usb_present() and record['at']==r.now
            and self.authorizer(r,record,msg) is True)

    def send(self,msg,**kwargs):
        self._owner()
        if kwargs.get('route') not in (None,'trs'):raise PermissionError('No USB/fallback flight route')
        kind=msg.get_type()
        # After uncertain delivery, no targets, arm/enable retries or params.
        # Fresh localization and separately authorized LAND/HOME can still
        # support safety handling. Unknown identity/boot remains blocked.
        safety=kind=='VISION_POSITION_ESTIMATE' or kind=='COMMAND_LONG' and (
            msg.command==21 or msg.command==185 and msg.param1==2
            or msg.command==511 and self.runtime.sync_rates and self.runtime.sync_rates.restoring
            and self.runtime.sync_rate_allowed(msg))
        if self.delivery_uncertain and not safety and not self._readonly(msg,self.runtime):
            raise OSError('Uncertain delivery latched; no normal writes or automatic replay')
        before=sum(self.runtime.sent_counts.values())
        self.capture(msg,**kwargs)  # Existing exact producer/envelope checks.
        if sum(self.runtime.sent_counts.values())!=before+1:raise ValueError('One current candidate required')
        record=self.runtime.frames[-1]
        wire=bytes.fromhex(record['wire_hex'])
        values=self.decoder.parse_buffer(wire)
        if len(values)!=1 or values[0].get_type()=='BAD_DATA':raise ValueError('Exactly one CRC-valid frame required')
        actual=values[0]
        if (actual.get_srcSystem(),actual.get_srcComponent())!=(250,220):raise PermissionError('Producer changed')
        if kind=='VISION_POSITION_ESTIMATE':
            source=self.runtime.coordinator.source_pose
            alignment=self.runtime.coordinator.fusion.alignment
            if not source or source[0]!=self.runtime.inputs.pose_stamp or alignment is None:
                raise PermissionError('No exact current transformed Vicon sample')
            stamp,position,quaternion=source
            expected,_,_=compact_vision.message(self.runtime.encoder,stamp//1000,position,quaternion,alignment)
            # Compare the original message before float32 wire quantization;
            # decoded values are independently checked by the CRC envelope.
            if msg.to_dict()!=expected.to_dict():raise PermissionError('Pose must match the exact current source, not a fabricated/replayed sample')
        readonly=self._readonly(actual,self.runtime)
        if kind=='COMMAND_LONG' and actual.command==511 and self.runtime.sync_rates:
            from preparation_authorization import RateRecoveryJournal
            rates=self.runtime.sync_rates
            if type(rates.persist) is not RateRecoveryJournal or not rates.persist.covers(rates,actual):
                raise PermissionError('Durable original sync interval required before SET')
        if not readonly:
            if not self._authorized(record,actual):raise PermissionError('No current reviewed per-frame execution authorization')
            if self.journal.before(self.runtime,record,actual) is not True:
                raise PermissionError('Write-ahead durability not established')
            # Disk sync can take time; the external reviewer must recheck
            # actual elapsed time, epochs, holds and all freshness gates.
            if not self._authorized(record,actual):raise PermissionError('Authorization changed/expired after journal sync')
        try:
            self.check_port_identity()
            written=self.port.write(wire)
            if type(written) is int and written>0:self.actual_tx_bytes+=written
            if type(written) is not int or written!=len(wire):raise OSError('Short write; do not retry arm/enable')
        except BaseException:
            self.delivery_uncertain=True
            self.failure=self.failure or 'delivery_uncertain'
            raise
        self.complete_messages+=1;self.types[kind]+=1
        return True

    def step(self,now):
        self._owner()
        if type(now) not in (int,float) or not math.isfinite(now) or now<0:
            raise ValueError('Finite monotonic owner time required')
        pad={};rx=b'';pose=None;ros_ns=0;problems=[]
        try:
            pad={} if self.poll_gamepad is None else self.poll_gamepad()
            self.last_pad_generation=pad.get('pad_generation')
            for down in pad.get('home_events',()):
                if type(down) is not bool:raise ValueError('Boolean HOME input required')
                if down and not self.home_down:self.home_presses+=1
                self.home_down=down
        except (ValueError,RuntimeError,OSError) as exc:
            problems.append('gamepad: '+str(exc))
            if self.last_pad_generation is not None:pad=dict(pad_generation=self.last_pad_generation,pad_present=False)
        try:
            self.check_port_identity()
            queued=self.port.in_waiting
            limit=65536 if getattr(self.port,'bounded_udp',False) else self.runtime.max_rx_bytes
            if type(queued) is not int or not 0<=queued<=limit:
                raise OSError('Serial backlog exceeds current-turn budget')
            rx=self.port.read(min(queued,self.runtime.max_rx_bytes)) if queued else b''
            if type(rx) is not bytes or len(rx)>self.runtime.max_rx_bytes:raise OSError('Invalid bounded RX')
            self.rx_bytes+=len(rx)
        except (ValueError,RuntimeError,OSError) as exc:problems.append('serial: '+str(exc))
        try:pose=self.poll_pose();ros_ns=self.now_ros_ns()
        except (ValueError,RuntimeError,OSError) as exc:problems.append('ROS: '+str(exc))
        if problems:
            self.failure=self.failure or '; '.join(problems)
            self.runtime._fail(self.failure)
        try:self.latest=self.runtime.turn(now,rx=rx,pose=pose,now_ros_ns=ros_ns,**pad)
        except (ValueError,RuntimeError,OSError,TypeError) as exc:
            self.failure=self.failure or str(exc);self.runtime._fail(self.failure)
        return self.status()

    def status(self):
        return dict(scope='Guarded execution transport API, no device opener or installed physical permit',
            failure=self.failure,delivery_uncertain=self.delivery_uncertain,
            actual_tx_bytes=self.actual_tx_bytes,complete_tx_messages=self.complete_messages,
            tx_types=dict(self.types),rx_bytes=self.rx_bytes,home_presses=self.home_presses,
            authorizer_supplied=self.authorizer is not None,flight_authorized=False,
            recovery_pending=bool(self.journal and self.journal.record['needs_restore']),
            runtime=self.latest,closed=self.closed)

    def close(self):
        self._owner()
        if self.runtime.core.arm_attempted and not self.runtime.context().restore_ground():
            raise PermissionError('Unknown/airborne state: require explicit operator handoff; do not silently close localization')
        rates=self.runtime.sync_rates
        if rates and rates.changed and rates.phase not in ('CLOSED','RESTORE_FAILED'):
            raise PermissionError('Sync-rate restoration pending; keep servicing the existing owner')
        self.closed=True  # The external owner still owns the actual resources.
