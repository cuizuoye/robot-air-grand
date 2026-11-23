"""Bounded single-owner preparation reactor; OFFLINE transport only.

Owns the RX parser, PoseGate, HOME checks, parameter discovery, preparation
coordinator and ONE sequence-numbered wire capture. No serial/socket/USB/input
API, supplied port/writer, arm, unlock, mode or flight-start entry exists.
Real device adapters and their physical authorization remain a separate task.

Each turn receives at most 2048 freshly collected RX bytes, eight current
HOME events, and the newest Vicon pose. Never pass archived reports as inputs.
HOME is serviced before RX/navigation parsing, then fresh poses, and finally
at most one diagnostic. NSH waits are polled over later turns, never slept on.
"""
from collections import Counter, deque
from dataclasses import asdict
from copy import deepcopy
import math

from pymavlink.dialects.v20 import common as m

from mission_live_inputs import LiveMissionInputs
from mission_preparation_coordinator import PreparationCoordinator
from mission_fusion_service import context_from_inputs
from mission_audit_collector import allowed as audit_allowed, COMMANDS
from mavlink_crc_stream import CrcResyncParser
import compact_vision
import trs_vision_reference_policy as vision
from uav_coord_bridge.mavlink_vision import PoseGate, transform_rigid_pose
from mission_telemetry_rates import TemporaryTelemetryRates


class PreparationRuntime:
    max_rx_bytes = 2048
    max_home_events = 8
    max_turn_gap = .15
    max_pose_continuity_gap = .15

    def __init__(self, rigid_revision, *, body_origin_in_rigid, body_orientation_in_rigid,
                 enable_offline_fusion=False, enable_temporary_rates=False, rates_persist=None,
                 ground_fusion_only=False, usb_observer=False,
                 fixed_configuration=False, fixed_profile=None):
        # Explicit extrinsics only: unknown offsets are not silently zeroed.
        transform_rigid_pose([0,0,0],[0,0,0,1],body_origin_in_rigid,body_orientation_in_rigid)
        self.origin = deepcopy(body_origin_in_rigid)
        self.orientation = deepcopy(body_orientation_in_rigid)
        self.inputs = LiveMissionInputs(rigid_revision,component=220)
        self.parser = CrcResyncParser()
        self.pose_gate = PoseGate(max_age_ms=150)
        self.now = None
        self.failure = None
        self.failure_context = None
        if ground_fusion_only and not enable_offline_fusion:
            raise ValueError('Ground fusion mode requires explicit fusion scope')
        self.ground_fusion_only=ground_fusion_only is True
        if usb_observer and not self.ground_fusion_only:
            raise ValueError('USB diagnostic route is ground-fusion-only, not a flight fallback')
        self.observer_required=usb_observer is True
        self.observer_inputs=LiveMissionInputs(rigid_revision,component=220) if self.observer_required else None
        self.observer_parser=CrcResyncParser() if self.observer_required else None
        self.observer_rx_bytes=0
        self.next_observer_identity=self.next_observer_heartbeat=0.
        if enable_temporary_rates and (not enable_offline_fusion or not callable(rates_persist)):
            raise ValueError('Temporary preparation rates need fusion scope and a durable journal')
        self.rates = TemporaryTelemetryRates(component=220,persist=rates_persist) if enable_temporary_rates else None
        self.offline_fusion = enable_offline_fusion is True
        self.fixed_configuration = fixed_configuration is True
        self.prepare_requested = False
        self.stop_latched = False
        self.closing = False
        self.closed = False
        self.home_check_requested = False
        self.home_down = False
        self.home_acknowledged = False
        self.home_request_at = None
        self.home_attempts = 0
        self.next_home = 0.
        self.next_identity = self.next_heartbeat = self.next_parameter = 0.
        self.parameter_cursor = 0
        self.encoder = m.MAVLink(None,srcSystem=250,srcComponent=220)
        self.frames = deque(maxlen=4096)
        self.trace = deque(maxlen=256)
        self.sent_counts = Counter()
        self.rx_bytes = 0
        self.max_observed_turn_gap = 0.
        # Keep the exact SAME callable object for service -> core handoff.
        self.writer = self._capture
        self.coordinator = PreparationCoordinator(self.inputs,self.writer,
            fusion_authorizer=self._offline_authorize if self.offline_fusion else None,
            ground_fusion_only=self.ground_fusion_only,
            fixed_configuration=self.fixed_configuration,fixed_profile=fixed_profile)

    def context(self,now=None):
        return self.coordinator.context(self.now if now is None else now)

    def observer_ok(self):
        i=self.observer_inputs
        return bool(not self.observer_required or i.identity and not i.fault and i.hb
            and not i.hb['armed'] and 0<=self.now-i.hb['at']<2)

    def _offline_authorize(self, service, msg, context):
        return bool(self.offline_fusion and service is self.coordinator.fusion
                    and self.observer_ok()
                    and (not self.failure and not self.stop_latched
                         or self.closing and context.restore_ground()))

    def _identity_ok(self):
        c=self.context()
        return bool(self.inputs.identity and c.epoch_valid and c.heartbeat_age < 2
                    and self.inputs.hb and self.inputs.hb['armed'] is False)

    def _ingest_owner_message(self, msg, now):
        self.coordinator.ingest(msg, now)

    def _owner_can_advance(self):
        return not (self.failure or self.stop_latched)

    def _advance_owner(self, now):
        self.coordinator.step(now, enable_fusion_preparation=self.prepare_requested,
                              service_diagnostics=False)

    def _owner_diagnostics(self, now, full):
        self.coordinator.service_diagnostics(now, full_audit=full)

    def _accept_owner_pose(self):
        return not (self.failure or self.stop_latched or self.closing)

    def _capture(self, msg, *, route=None):
        """ONLY memory bytes: strict producer shapes; no external write hook."""
        kind=msg.get_type();c=self.context()
        if self.fixed_configuration and kind in ('PARAM_SET','PARAM_EXT_SET'):
            raise PermissionError('Fixed configuration forbids every parameter write, including cleanup')
        if route is None:
            route='usb' if self.observer_required and kind in ('SERIAL_CONTROL','PARAM_REQUEST_READ','PARAM_SET') else 'trs'
        if route not in ('trs','usb') or route=='usb' and not self.observer_required:
            raise PermissionError('Unknown or unapproved diagnostic route')
        if self.observer_required:
            expected='usb' if kind in ('SERIAL_CONTROL','PARAM_REQUEST_READ','PARAM_SET') else 'trs'
            startup=kind=='HEARTBEAT' or kind=='COMMAND_LONG' and msg.command==512
            if route!=expected and not (route=='usb' and startup):
                raise PermissionError('Wrong ground diagnostic/pose route')
        s=self.coordinator.fusion
        accepted=False
        if kind=='HEARTBEAT':
            accepted=(msg.type,msg.autopilot,msg.base_mode,msg.custom_mode)==(m.MAV_TYPE_GCS,m.MAV_AUTOPILOT_INVALID,0,0)
        elif kind=='COMMAND_LONG':
            params=tuple(getattr(msg,f'param{i}') for i in range(1,8))
            target=(msg.target_system,msg.target_component)==(1,1)
            accepted=target and ((msg.command==512 and params==(148,0,0,0,0,0,0))
                or (msg.command==185 and params==(2,0,0,0,0,0,0)
                    and msg.confirmation in (0,1,2) and self.home_check_requested and self._identity_ok()))
            if self.rates and msg.command in (510,511):
                accepted=self.inputs.home_proven and self.rates.permits(msg,
                    fresh=self._identity_ok(),locked=c.armed is False and c.lockdown is True
                    and c.actuator_age<=.4 and self.coordinator.audit.live_lockdown(self.now) is True)
        elif kind=='PARAM_REQUEST_READ':
            name=msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id,bytes) else msg.param_id.rstrip('\0')
            accepted=self._identity_ok() and name in self.coordinator.read_params and (
                msg.target_system,msg.target_component,msg.param_index)==(1,1,-1)
        elif kind=='TIMESYNC' and self.fixed_configuration:
            gate=getattr(self,'fixed_timesync_allowed',None)
            accepted=bool(callable(gate) and gate(msg))
        elif kind=='SERIAL_CONTROL':
            query=self.coordinator.audit
            payload=bytes(msg.data[:msg.count])
            accepted=self._identity_ok() and audit_allowed(msg) and (
                payload==b'' and not query.transaction.idle
                or query.key in COMMANDS and payload==COMMANDS[query.key])
        elif kind=='PARAM_SET' and self.offline_fusion and s.pending:
            restoring=s.phase in ('WAIT_RESTORE_CTRL','WAIT_RESTORE_DELAY')
            safe=c.restore_ground() if restoring and self.closing else c.preparation_ground()
            expected=vision.parameter_message(self.encoder,s.pending['name'],s.pending['value'])
            accepted=bool(safe and s._bound(c) and msg.to_dict()==expected.to_dict()
                          and (not self.failure and not self.stop_latched or restoring and self.closing))
        elif kind=='VISION_POSITION_ESTIMATE' and self.offline_fusion and not (
                self.failure or self.stop_latched or self.closing or self.closed):
            source=self.coordinator.source_pose
            if source and s.alignment is not None and c.preparation_ground() and s._bound(c):
                stamp,p,q=source
                expected,_,_=compact_vision.message(self.encoder,stamp//1000,p,q,s.alignment)
                accepted=compact_vision.pose_allowed(msg) and msg.to_dict()==expected.to_dict()
        if not accepted:
            raise PermissionError('Offline preparation boundary rejected '+kind)
        wire=msg.pack(self.encoder)
        self.encoder.seq=(self.encoder.seq+1)%256
        self.frames.append(dict(at=self.now,type=kind,route=route,wire_hex=wire.hex()))
        self.sent_counts[kind]+=1
        self.trace.append(dict(at=self.now,event='capture',type=kind))
        return True

    def _fail(self, reason):
        if self.failure is None and self.now is not None:
            c=self.context()
            self.failure_context={k:(None if isinstance(v,float) and not math.isfinite(v) else v)
                                  for k,v in asdict(c).items()}
            self.failure_context.update(audit_key=self.coordinator.audit.key,
                audit_requested_at=self.coordinator.audit.requested_at,
                landed_message_age=(self.now-self.inputs.land['at'] if self.inputs.land else None),
                fusion_phase=self.coordinator.fusion.phase)
        self.failure=self.failure or reason
        if self.coordinator.core.phase not in ('STOP_LATCHED','COMPLETE'):
            self.coordinator.core.phase='BLOCKED'
            self.coordinator.core.fault=self.coordinator.core.fault or reason

    def _service_home(self):
        # Stop dispatch depends on current identity/link, NOT Vicon/audit health.
        if (not self.home_check_requested or self.home_acknowledged
                or self.home_attempts>=3 or self.now<self.next_home or not self._identity_ok()):
            return
        self.writer(self.encoder.command_long_encode(1,1,185,self.home_attempts,2,0,0,0,0,0,0))
        self.home_attempts+=1
        self.next_home=self.now+.8

    def request_prepare(self, *, operator_confirmed=False):
        if (operator_confirmed is not True or not self.offline_fusion or self.now is None
                or self.failure or self.stop_latched or self.closed or self.closing
                or self.prepare_requested or not self.inputs.preparation_verified(self.now)
                or self.rates is not None and not self.rates.summary()['measured_profile_verified']
                or not self.context().preparation_ground()):
            raise PermissionError('One explicit OFFLINE preparation after current HOME/locked-ground proof required')
        self.prepare_requested=True  # Does NOT clear physical lock or start core.

    def request_close(self):
        if self.now is None or self.closed or self.closing:
            raise ValueError('One initialized close operation required')
        s=self.coordinator.fusion
        if self.fixed_configuration:
            if s.phase=='NEW' and not self.prepare_requested:
                self.closed=True
                return
            # No restore request, baseline writes, rate changes or synthetic
            # safe-state proof. Unknown landing/lock leaves cleanup incomplete.
            s.close(self.context())
            self.closing=self.closed=True
            return
        self.coordinator.request_restore_reads()
        if s.phase=='NEW' or not s.changed and s.transferred_core is None:
            if self.rates is None:self.closed=True
            else:
                self.closing=True;s.phase='CLOSED';self.rates.begin_restore(self.now)
                if not self.rates.changed:
                    self.rates.action(self.now,fresh=True,locked=False)
                    self.closed=self.rates.phase=='CLOSED'
            return
        self.closing=True
        try:
            if self.coordinator.core.phase!='STOP_LATCHED':
                self.coordinator.core.phase='BLOCKED'  # No mission was ever started.
            s.close(self.context())
        except Exception:
            self.closing=False
            raise

    def turn(self, now, *, rx=b'', observer_rx=b'', now_ros_ns=None, pose=None, pad_generation=None,
             pad_present=None, pad_event=False, home_events=()):
        """One bounded turn; raw pose=(stamp_ns, frame, xyz, xyzw).

        Callers poll devices nonblocking outside this OFFLINE-only object.
        rx is freshly captured for this turn, not a stored delayed replay.
        No sleeps, shell waits, terminal printing or device calls occur here.
        """
        # HOME press suppresses poses even when another field/clock is invalid.
        for value in home_events[:self.max_home_events]:
            if value is True and not self.home_down:
                self.home_check_requested=True
                self.home_acknowledged=False
                self.home_request_at=None
                self.home_attempts=0
                self.next_home=0.
                if self.prepare_requested or self.closing:
                    self.stop_latched=True
                    self.coordinator.core.stop_latched=True
                    self.coordinator.core.phase='STOP_LATCHED'
            if type(value) is bool:self.home_down=value
        if (type(now) not in (int,float) or not math.isfinite(now) or now<0
                or self.now is not None and now<self.now):
            self._fail('owner_clock_invalid')
            raise ValueError('Finite monotonic owner clock required')
        gap=now-self.now if self.now is not None else 0.
        self.now=now
        if self.home_check_requested and self.home_request_at is None:
            self.home_request_at=now
            self.next_home=now
        self.max_observed_turn_gap=max(self.max_observed_turn_gap,gap)
        try:
            if len(home_events)>self.max_home_events or any(type(v) is not bool for v in home_events):
                raise ValueError('Bounded boolean HOME events required')
            if pad_generation is not None:
                self.inputs.observe_gamepad(pad_generation,pad_present,now,event=pad_event)
                for down in home_events:
                    self.inputs.observe_gamepad(pad_generation,pad_present,now,event=True,home=down)
            elif home_events:
                raise ValueError('Current opened gamepad generation required')
            self._service_home()
            if type(rx) is not bytes or len(rx)>self.max_rx_bytes:
                raise ValueError('Bounded current RX bytes required')
            self.rx_bytes+=len(rx)
            for msg in self.parser.parse_buffer(rx) or ():
                if self.rates:self.rates.ingest(msg,now)
                if self.observer_required and msg.get_type() in ('PARAM_VALUE','SERIAL_CONTROL','BAD_DATA'):
                    if msg.get_type()=='BAD_DATA':self.inputs.ingest(msg,now)
                    continue  # RF corruption never invalidates a USB diagnostic transaction.
                self._ingest_owner_message(msg,now)
                if (msg.get_type()=='COMMAND_ACK' and msg.command==185 and self.home_attempts
                        and (msg.get_srcSystem(),msg.get_srcComponent())==(1,1)
                        and (msg.target_system,msg.target_component)==(250,220)
                        and self.home_request_at<=now<=self.next_home+2.2):
                    if msg.result==m.MAV_RESULT_ACCEPTED:self.home_acknowledged=True
                    else:self._fail('home_stop_rejected')
            if type(observer_rx) is not bytes or len(observer_rx)>self.max_rx_bytes:
                raise ValueError('Bounded independent USB RX bytes required')
            if observer_rx and not self.observer_required:
                raise ValueError('USB RX supplied without explicit observer scope')
            if self.observer_required:
                self.observer_rx_bytes+=len(observer_rx)
                for msg in self.observer_parser.parse_buffer(observer_rx) or ():
                    self.observer_inputs.ingest(msg,now)
                    if self.observer_ok() and msg.get_type() in ('PARAM_VALUE','SERIAL_CONTROL','BAD_DATA'):
                        self.coordinator.ingest(msg,now)
                if self.observer_inputs.fault:self._fail('usb_observer: '+self.observer_inputs.fault)
                if self.prepare_requested and not self.observer_ok():self._fail('usb_observer_identity_or_heartbeat_lost')
            self._service_home()
            if self.inputs.fault:self._fail(self.inputs.fault)
            if gap>self.max_turn_gap and self.coordinator.fusion.phase not in ('NEW','CLOSED'):
                self._fail('owner_turn_gap')
            if self.closed:return self.status()
            # Check continuity BEFORE a returning sample can refresh pose_at.
            # Recovery may support landing, never erase a mission failure.
            if (self.prepare_requested and not self.closing and self.inputs.pose_at is not None
                    and now-self.inputs.pose_at > self.max_pose_continuity_gap):
                self._fail('vicon_source_gap')
            if pose is not None and self._accept_owner_pose():
                stamp,frame,p,q=pose
                if type(now_ros_ns) is not int:raise ValueError('Current ROS clock required for PoseGate')
                previous=self.pose_gate.last
                if (self.prepare_requested and previous is not None and stamp < previous.stamp_ns):
                    self._fail('vicon_source_clock_regression')
                elif self.pose_gate.ingest(stamp,now_ros_ns,int(now*1e9),frame,p,q):
                    sample=self.pose_gate.take_new(now_ros_ns,int(now*1e9))
                    self.inputs.observe_pose(sample,now_ros_ns,int(now*1e9),now)
                    position,quat=transform_rigid_pose(sample.position,sample.quaternion,self.origin,self.orientation)
                    self.coordinator.observe_source_pose(sample.stamp_ns,position,quat)
                elif self.prepare_requested and self.pose_gate.reason in ('frame_mismatch','implausible_position_jump'):
                    self._fail('vicon_'+self.pose_gate.reason)
            if self.closing:
                self.coordinator.fusion.tick(self.context(now))
                if self.coordinator.fusion.phase=='CLOSED':
                    if self.rates:
                        self.rates.begin_restore(now)
                        self.closed=self.rates.phase=='CLOSED'
                    else:self.closed=True
            elif self._owner_can_advance():
                self._advance_owner(now)
                if self.coordinator.failed:self._fail(self.coordinator.failed)
                if self.coordinator.fusion.phase=='BLOCKED':
                    self._fail(self.coordinator.fusion.fault or 'fusion_blocked')
            # Maintenance/diagnostics follow HOME and fresh localization.
            if now>=self.next_heartbeat:
                self.writer(self.encoder.heartbeat_encode(m.MAV_TYPE_GCS,m.MAV_AUTOPILOT_INVALID,0,0,0))
                self.next_heartbeat=now+1
            if self.inputs.identity is None and now>=self.next_identity:
                self.inputs.request_identity(now)
                self.writer(self.encoder.command_long_encode(1,1,512,0,148,0,0,0,0,0,0))
                self.next_identity=now+3
            if self.observer_required:
                if now>=self.next_observer_heartbeat:
                    self._capture(self.encoder.heartbeat_encode(m.MAV_TYPE_GCS,m.MAV_AUTOPILOT_INVALID,0,0,0),route='usb')
                    self.next_observer_heartbeat=now+1
                if self.observer_inputs.identity is None and now>=self.next_observer_identity:
                    self.observer_inputs.request_identity(now)
                    self._capture(self.encoder.command_long_encode(1,1,512,0,148,0,0,0,0,0,0),route='usb')
                    self.next_observer_identity=now+3
            # Human wait is read-only receive + identity heartbeat. Do not
            # burn shell retry budgets or flood parameter requests before
            # the current session's physical HOME input has even arrived.
            if self._identity_ok() and self.observer_ok() and (self.inputs.native_rc is not None or self.home_check_requested or self.prepare_requested or self.closing):
                if self.coordinator.fusion.phase=='NEW' and now>=self.next_parameter:
                    names=self.coordinator.read_params
                    name=names[self.parameter_cursor]
                    self.parameter_cursor=(self.parameter_cursor+1)%len(names)
                    self.coordinator.note_parameter_request(name,now)
                    self.writer(self.encoder.param_request_read_encode(1,1,name.encode(),-1))
                    self.next_parameter=now+.05
                full=(self.coordinator.fusion.phase in ('FUSING','READY','HANDED_OFF')
                      and not (self.closing or self.failure or self.stop_latched))
                self._owner_diagnostics(now,full)
                if self.coordinator.audit.disabled:self._fail('diagnostic_boundary_or_retry_budget_lost')
            if self.rates and self.inputs.home_proven:
                c=self.context(now)
                outgoing=self.rates.action(now,fresh=self._identity_ok() and self.observer_ok(),
                    locked=c.armed is False and c.lockdown is True and c.actuator_age<=.4
                    and self.coordinator.audit.live_lockdown(now) is True)
                if outgoing is not None:self.writer(outgoing)
                if self.rates.error:self._fail('temporary_telemetry_rates: '+self.rates.error)
        except (ValueError,TypeError,KeyError,RuntimeError,OSError,PermissionError) as exc:
            self._fail(type(exc).__name__+': '+str(exc))
        return self.status()

    def status(self):
        return dict(scope='OFFLINE wire-capture preparation runtime; not hardware flight',
            now=self.now,failure=self.failure,stop_latched=self.stop_latched,closed=self.closed,
            ground_fusion_only=self.ground_fusion_only,
            usb_diagnostic_observer_required=self.observer_required,
            usb_observer_identity_verified=self.observer_ok() if self.observer_required else None,
            usb_observer_rx_bytes=self.observer_rx_bytes,
            usb_observer_parser=self.observer_parser.summary() if self.observer_parser else None,
            failure_context=self.failure_context,
            temporary_telemetry_rates=self.rates.summary() if self.rates else None,
            diagnostic_failures=self.coordinator.audit.failures[-8:],
            noncurrent_actuator_reads=self.coordinator.audit.noncurrent_actuator_reads[-8:],
            prepare_requested=self.prepare_requested,home_response_verified=self.inputs.home_proven,
            home_attempts=self.home_attempts,coordinator=self.coordinator.status(),
            mission_inputs=self.inputs.describe(self.now) if self.now is not None else None,
            rx_bytes=self.rx_bytes,parser=self.parser.summary(),captured_counts=dict(self.sent_counts),
            maximum_observed_turn_gap_s=self.max_observed_turn_gap,
            retained_frame_count=len(self.frames),devices_opened=0,physical_tx_bytes=0,
            fusion_configuration_may_need_restore=bool(self.coordinator.fusion.changed and not self.closed),
            flight_authorized=False,auto_start_present=False,hardware_adapter_installed=False)
