"""Real-device receive/query adapter; never a flight or HOME-stop sender.

No devices opened here. One host supplies exclusive nonblocking resources.
The same continuous reactor is used in a sealed receive-shadow mode. Captures
are candidate frames, not delivery proof. Only this adapter's counters denote
actual complete writes. Privileged traffic is ALWAYS denied, including HOME.
"""
from collections import Counter
import math
from threading import get_ident

from pymavlink.dialects.v20 import common as m

from continuous_mission_runtime import ContinuousMissionRuntime
from mavlink_crc_stream import CrcResyncParser
from mission_audit_collector import allowed as audit_allowed
import trs_vision_reference_policy as vision


class ReceiveShadowRuntime(ContinuousMissionRuntime):
    """Real inputs may be inspected, never fed to simulated flight readiness."""
    def request_prepare(self, **kwargs):
        raise PermissionError('Device shadow cannot enable fusion or prepare a flight')

    def request_start(self, **kwargs):
        raise PermissionError('Device shadow has no flight start or output-enable operation')

    def _service_home(self):
        # Only records physical input. No motor-stop command is produced.
        return

    def _advance_owner(self, now):
        self.coordinator.step(now,enable_fusion_preparation=False,service_diagnostics=False)

    def _ingest_owner_message(self,msg,now):
        if msg.get_type()=='COMMAND_ACK' and msg.command==185:
            return  # No lock was sent here; an unrelated ACK cannot prove HOME.
        return super()._ingest_owner_message(msg,now)

    def status(self):
        result=super().status()
        result.update(scope='Real-device receive shadow, NOT flight preparation or emergency protection',
            input_adapter_mode='receive_shadow',home_stop_sender_active=False,
            hardware_input_adapter_attached=bool(getattr(self,'device_adapter_attached',False)),
            hardware_adapter_installed=bool(getattr(self,'device_adapter_attached',False)),
            physical_tx_counter_owner='ContinuousDeviceIO, not the runtime capture log',
            auto_prepare_present=False,hardware_execution_available=False,flight_authorized=False)
        return result


class ContinuousDeviceIO:
    """Synchronous validated write boundary: no replay, retry, or reconnect.

    Default queries=False means ZERO TX, including identity/heartbeat probes.
    queries=True allows exact readonly probes/reads only. All commands for
    output lock/unlock, arm/disarm, mode, landing, targets, params and rates
    remain forbidden. This adapter must not be used to supervise a flight.
    """
    def __init__(self,runtime,port,*,poll_gamepad,poll_pose,now_ros_ns,
                 check_port_identity,queries=False):
        if type(runtime) is not ReceiveShadowRuntime:
            raise ValueError('Only the sealed receive-shadow runtime can attach to real devices')
        if port.timeout!=0 or port.write_timeout!=0:
            raise ValueError('Exclusive nonblocking serial timeout=write_timeout=0 required')
        if type(queries) is not bool or not all(callable(x) for x in
                (poll_gamepad,poll_pose,now_ros_ns,check_port_identity)):
            raise ValueError('Typed query scope and nonblocking readers required')
        if getattr(runtime,'device_adapter_attached',False):
            raise ValueError('A second device reader/writer is forbidden')
        runtime.device_adapter_attached=True
        self.runtime,self.port=runtime,port
        self.poll_gamepad,self.poll_pose=poll_gamepad,poll_pose
        self.now_ros_ns,self.check_port_identity=now_ros_ns,check_port_identity
        self.queries=queries
        self.owner=get_ident()
        self.capture=runtime.writer
        self.decoder=CrcResyncParser()
        self.failure=None
        self.closed=False
        self.actual_tx_bytes=self.complete_messages=self.rx_bytes=0
        self.tx_types=Counter();self.suppressed=Counter()
        self.identity_queries_sent=0
        self.home_events_observed=0
        self.home_down=False
        self.last_pad_generation=None
        self.latest_status=None
        self.writer=self.send  # Retain one callable identity for every producer.
        runtime.writer=self.writer
        runtime.coordinator.writer=self.writer
        runtime.coordinator.core.sink=self.writer
        runtime.coordinator.fusion.sink=self.writer

    def _owner(self):
        if get_ident()!=self.owner:
            raise RuntimeError('Only one immutable device I/O thread is allowed')
        if self.closed:
            raise RuntimeError('Adapter closed; create an explicitly reviewed new session, never reconnect')

    def allowed(self,msg):
        kind=msg.get_type()
        if (msg.get_srcSystem(),msg.get_srcComponent())!=(250,220):return False
        if kind=='HEARTBEAT':
            return (msg.type,msg.autopilot,msg.base_mode,msg.custom_mode)==(m.MAV_TYPE_GCS,m.MAV_AUTOPILOT_INVALID,0,0)
        if kind=='COMMAND_LONG':
            return bool((msg.target_system,msg.target_component,msg.command,msg.confirmation)==(1,1,512,0)
                and tuple(getattr(msg,f'param{i}') for i in range(1,8))==(148,0,0,0,0,0,0))
        if not self.runtime._identity_ok():return False
        if kind=='PARAM_REQUEST_READ':
            name=msg.param_id.rstrip('\0')
            return bool(name in vision.READ_PARAMS
                and (msg.target_system,msg.target_component,msg.param_index)==(1,1,-1))
        if kind=='SERIAL_CONTROL':return audit_allowed(msg)
        return False

    def send(self,msg,**kwargs):
        self._owner()
        if self.failure:raise OSError('Device I/O failure latched; no subsequent writes/replay')
        if kwargs.get('route') not in (None,'trs'):
            raise PermissionError('There is exactly one TRS transport; USB fallback is forbidden')
        # The original capture validates producer/state-specific exact shapes.
        # A failed physical write cannot return True to the owning service.
        before=sum(self.runtime.sent_counts.values())
        self.capture(msg,**kwargs)
        if sum(self.runtime.sent_counts.values())!=before+1:
            raise RuntimeError('Exactly one validated candidate frame required')
        record=self.runtime.frames[-1]
        wire=bytes.fromhex(record['wire_hex'])
        decoded=self.decoder.parse_buffer(wire)
        if len(decoded)!=1 or decoded[0].get_type()=='BAD_DATA' or not self.allowed(decoded[0]):
            raise PermissionError('Outside sealed receive/query scope; no privileged frame may reach the port')
        if not self.queries:
            self.suppressed[msg.get_type()]+=1
            if msg.get_type()=='COMMAND_LONG' and msg.command==512:
                # A memory capture is NOT a sent identity request.
                self.runtime.inputs.version_request=None
            return True  # Memory capture only, never physical-send evidence.
        try:
            self.check_port_identity()
            written=self.port.write(wire)
            if type(written) is int and written>0:self.actual_tx_bytes+=written
            if type(written) is not int or written!=len(wire):
                raise OSError('Short nonblocking write; delivery uncertain, no retry')
        except BaseException:
            self.failure=self.failure or 'write_failed_or_uncertain_delivery'
            raise
        self.complete_messages+=1
        self.tx_types[msg.get_type()]+=1
        if msg.get_type()=='COMMAND_LONG' and msg.command==512:self.identity_queries_sent+=1
        return True

    def step(self,now):
        self._owner()
        if self.failure:return self.status()
        if type(now) not in (int,float) or not math.isfinite(now) or now<0:
            self.failure='invalid_owner_clock';return self.status()
        pad={};rx=b'';pose=None;ros_ns=0;problems=[]
        # HOME input is collected before serial, ROS or diagnostics. A later
        # read error must not erase a physical key event already observed.
        try:
            pad=self.poll_gamepad()
            self.last_pad_generation=pad.get('pad_generation')
            for down in pad.get('home_events',()):
                if type(down) is not bool:raise ValueError('Boolean HOME input required')
                if down and not self.home_down:self.home_events_observed+=1
                self.home_down=down
        except (OSError,ValueError,RuntimeError) as exc:
            problems.append('gamepad: '+str(exc))
            if self.last_pad_generation is not None:
                pad=dict(pad_generation=self.last_pad_generation,pad_present=False)
        try:
            self.check_port_identity()
            queued=self.port.in_waiting
            if type(queued) is not int or not 0<=queued<=self.runtime.max_rx_bytes:
                raise OSError('Serial RX backlog exceeds turn budget; never relabel buffered data as fresh')
            rx=self.port.read(min(queued,self.runtime.max_rx_bytes)) if queued else b''
            if type(rx) is not bytes or len(rx)>self.runtime.max_rx_bytes:
                raise OSError('Invalid bounded serial read')
            self.rx_bytes+=len(rx)
        except (OSError,ValueError,RuntimeError) as exc:problems.append('serial: '+str(exc))
        try:
            pose=self.poll_pose()
            ros_ns=self.now_ros_ns()  # Read ROS time AFTER servicing callbacks.
        except (OSError,ValueError,RuntimeError) as exc:problems.append('ROS: '+str(exc))
        if problems:
            self.failure='input_failure: '+'; '.join(problems)
            # Latch before the runtime can produce any query on this turn.
            self.runtime._fail(self.failure)
        try:
            self.latest_status=self.runtime.turn(now,rx=rx,pose=pose,now_ros_ns=ros_ns,**pad)
        except (ValueError,OSError,RuntimeError,TypeError) as exc:
            self.failure=self.failure or 'runtime: '+str(exc)
        if self.runtime.failure:self.failure=self.failure or self.runtime.failure
        hb=self.runtime.inputs.hb
        if hb and hb['armed']:
            self.failure=self.failure or 'armed_aircraft_outside_observation_scope'
        return self.status()

    def status(self):
        return dict(scope='Real TRS/Vicon/gamepad receive/query adapter; NOT flight or an active emergency monitor',
            failure=self.failure,queries_enabled=self.queries,rx_bytes=self.rx_bytes,
            actual_tx_bytes=self.actual_tx_bytes,complete_tx_messages=self.complete_messages,
            tx_types=dict(self.tx_types),identity_queries_sent=self.identity_queries_sent,
            suppressed_capture_types=dict(self.suppressed),home_presses_observed=self.home_events_observed,
            home_stop_sender_active=False,flight_authorized=False,hardware_flight_dispatch_available=False,
            restoration_required=False,closed=self.closed,runtime=self.latest_status)

    def close(self):
        if get_ident()!=self.owner:raise RuntimeError('Device I/O close must run on its owner')
        self.closed=True  # Resource descriptors remain owned by the host ExitStack.
