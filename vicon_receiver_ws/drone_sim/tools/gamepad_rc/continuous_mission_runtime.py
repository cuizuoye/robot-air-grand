"""Continuous prepare -> explicit start -> flight -> ground restore, OFFLINE ONLY.

One parser/writer/encoder/PoseGate, inherited from PreparationRuntime. There
is deliberately no serial object, network socket, physical authorizer or real
execution flag. Inputs are injected by a synthetic peer in the offline host.
The output-enable transaction below is a MODEL, never a hardware permission.
Existing one-HOME battery checks/holds/permits are neither consumed nor changed.
"""
from copy import deepcopy
from dataclasses import replace
import math

from pymavlink.dialects.v20 import common as m

from flight_mission_core import MissionCore, Observation
from mission_live_inputs import age
from mission_fusion_service import context_from_inputs
from mission_preparation_runtime import PreparationRuntime
from mission_wire_transport import MissionWireTransport
from mission_audit_collector import MissionAuditCollector
from ground_estimator_settlement import require_frozen_selector
from px4_modes import COMMAND_ACK_TIMEOUT, confirmation_deadline


class ContinuousMissionRuntime(PreparationRuntime):
    start_timeout = COMMAND_ACK_TIMEOUT

    def __init__(self, rigid_revision, *, body_origin_in_rigid,
                 body_orientation_in_rigid, height=.3, hover_seconds=3.,
                 fixed_configuration=False, fixed_profile=None, pilot_source='gamepad'):
        if pilot_source not in ('gamepad','native_rc') or pilot_source=='native_rc' and not fixed_configuration:
            raise ValueError('Native RC requires explicit fixed configuration')
        if pilot_source=='native_rc' and (not isinstance(fixed_profile,dict)
                or fixed_profile.get('parameters',{}).get('COM_RC_IN_MODE',{}).get('value')!=0):
            raise ValueError('Native RC requires reviewed COM_RC_IN_MODE=0; never writes it')
        if isinstance(fixed_profile,dict) and 'initial_trial' in fixed_profile:
            from ekf_static_profile import require_mission_profile
            require_mission_profile(fixed_profile,pilot_source=pilot_source,height=height,hover_seconds=hover_seconds)
            trial=fixed_profile['initial_trial']
            if (rigid_revision!=trial['revision'] or list(body_origin_in_rigid)!=trial['origin']
                    or list(body_orientation_in_rigid)!=trial['orientation']):
                raise ValueError('Initial-trial runtime layout mismatch')
        super().__init__(rigid_revision, body_origin_in_rigid=body_origin_in_rigid,
                         body_orientation_in_rigid=body_orientation_in_rigid,
                         enable_offline_fusion=True,fixed_configuration=fixed_configuration,
                         fixed_profile=fixed_profile)
        from lock_clock_probe import LockClockProbe
        from rc_watch import NATIVE_CLOCK_MAX_AGE_S, NATIVE_DISPATCH_MAX_GAP_S
        self.fixed_clock = LockClockProbe(target=(250,220),
            max_reference_age_s=NATIVE_CLOCK_MAX_AGE_S if pilot_source=='native_rc' else .5) if self.fixed_configuration else None
        if pilot_source=='native_rc':
            self.max_turn_gap = self.max_pose_continuity_gap = NATIVE_DISPATCH_MAX_GAP_S
            self.coordinator.fusion.maximum_send_gap = NATIVE_DISPATCH_MAX_GAP_S
        self.next_fixed_timesync = 0.
        self.fixed_ros_ns = None
        self.cooperative_ros_ns = None
        self.fixed_sync_reply = None
        self.fixed_sync_last_token = 0
        self.fixed_sync_reply_count = 0
        from mission_telemetry_rates import TemporaryTelemetryRates
        self.sync_rates=(TemporaryTelemetryRates(component=220,profile='mission' if pilot_source=='native_rc' else 'timesync')
            if fixed_profile and fixed_profile.get('timestamp_mode')=='local_ros_timesync' else None)
        self.sync_rates_enabled=False  # Explicit bootstrap review enables setup.
        self.coordinator.core = MissionCore(self.writer, height=height,
            hover_seconds=hover_seconds,pilot_source=pilot_source,
            horizontal_motion=fixed_profile.get('horizontal_motion') if fixed_profile else None)
        self.pilot_source=pilot_source
        if pilot_source=='native_rc':
            from rc_watch import NativeRCMonitor
            self.inputs.native_rc=NativeRCMonitor()
            self.coordinator.read_params+=tuple(NativeRCMonitor.PARAMETER_TYPES)
        self.native_configuration=None
        self.native_mode_sample=None
        self.native_start_boot_us=None
        self.envelope = MissionWireTransport()  # Validation only; not a second sender.
        self.envelope.attach(self.coordinator.core)
        # Fixed-read parser only: does not grant any transmit permission.
        self.coordinator.audit = MissionAuditCollector(
            heading_provider=self.coordinator._heading,offline_observe_armed=True,native_rc=self.inputs.native_rc)
        self.coordinator.context_provider = self._continuous_context
        self.start_requested_at = self.enable_ack_at = None
        self.start_binding = None
        self.enable_sent = False
        self.wait_started_at = None
        self.cancel_requested = False
        self.cleanup_lock_at = self.cleanup_ack_at = None
        self.events = []
        self.owner_thread = None
        self.pilot_link=self.pilot_mapper=None
        self.pilot_state=None;self.pilot_sample_at=None
        self.pilot_channels=None;self.pilot_centered=False;self.pilot_rc_feedback=None
        self.next_pilot_rc=0.;self._pilot_candidate=None;self.pilot_fault=None

    def attach_cooperative(self, execution):
        """Install before the first turn; retain this runtime's sole writer."""
        from air_ground_execution import AirGroundExecution
        if (type(execution) is not AirGroundExecution or self.now is not None
                or self.core.cooperative is not None or self.core.motion is not None
                or self.pilot_source!='native_rc'):
            raise ValueError('Attach one cooperative task to an unused Pocket fixed-point runtime')
        execution.bind(self.core)
        if execution.car_task is not None:
            # A loaded approach is part of the existing bounded car task, not
            # a 90 s stationary fusion experiment. Keep streaming while the
            # core waits to capture its origin at the stopped takeoff point.
            self.coordinator.fusion.session_limit=execution.car_task.plan.config['maximum_task_seconds']

    def enable_pilot(self, calibration):
        if self.pilot_source=='native_rc':raise PermissionError('Native RC cannot attach a gamepad writer')
        """Reuse the existing mapper/mode tracker; open no second connection."""
        if self.now is not None or self.pilot_link is not None:
            raise ValueError('Pilot wiring is immutable before the first owner turn')
        from gamepad_rc import Mapper,Link
        self.pilot_mapper=Mapper(deepcopy(calibration))
        self.pilot_link=Link(None,writer=self._send_pilot,encoder=self.encoder,mission_core=self.core)

    def _send_pilot(self,msg):
        self._pilot_candidate=msg
        try:return self.writer(msg)
        finally:self._pilot_candidate=None

    def pilot_candidate_allowed(self,msg):
        if self._pilot_candidate is None or msg.to_dict()!=self._pilot_candidate.to_dict():return False
        c=self.context()
        if (self.pilot_link is None or self.pilot_sample_at!=self.now
                or self.stop_latched or self.core.stop_latched or self.inputs.home_down
                or self.closing or self.closed or self.inputs.fault or not c.epoch_valid
                or not c.gamepad_current or not self._identity_ok()
                or not 0<=c.actuator_age<=.5 or not 0<=c.attitude_age<=.5):return False
        if msg.get_type()=='RC_CHANNELS_OVERRIDE':
            values=[getattr(msg,'chan%d_raw'%i) for i in range(1,9)]
            return bool(self.inputs.home_proven and self.pilot_channels is not None
                and (msg.target_system,msg.target_component)==(1,1)
                and values==list(self.pilot_channels[:4])+[65535]*4
                and all(getattr(msg,'chan%d_raw'%i,0)==0 for i in range(9,19))
                and (self.pilot_centered or self.core.pilot_control))
        if msg.get_type()=='COMMAND_LONG':
            from px4_modes import PARAM_TO_MODE
            request=self.pilot_link.mode_request
            return bool(request and request['state']=='WAITING' and request['sent_at']==self.now
                and ((request.get('takeover',True) and self.core.pilot_control
                      and self.start_requested_at is not None)
                     or (request.get('takeover') is False and request['parameter']==2
                         and self.ground_position_allowed()))
                and (msg.target_system,msg.target_component,msg.command,msg.confirmation)==(1,1,176,0)
                and tuple(getattr(msg,'param%d'%i) for i in range(1,8))==
                    (1,*PARAM_TO_MODE[request['parameter']],0,0,0,0))
        return False

    def ground_position_allowed(self):
        """Explicit A only: leave Land while retaining the ground output lock."""
        return bool(self.start_requested_at is None and not self.enable_sent
            and not self.core.arm_attempted and not self.core.pilot_control
            and not self.failure and not self.cancel_requested
            and not self.stop_latched and not self.core.stop_latched
            and not self.closing and not self.closed and self.inputs.home_proven
            and self.pilot_sample_at==self.now and self.pilot_centered
            and self.context().locked_ground() and self._identity_ok()
            and self.inputs.hb and self.inputs.hb['armed'] is False)

    def ground_mode_confirmed(self):
        if self.pilot_source=='native_rc':
            h=self.inputs.hb
            return bool(h and self.now is not None and 0<=self.now-h['at']<2
                        and h['custom_mode']==3<<16 and not h['armed'])
        request=self.pilot_link.mode_request if self.pilot_link else None
        return not request or request.get('takeover',True) or request['state']=='CONFIRMED'

    def pilot_rc_received(self):
        row=self.pilot_rc_feedback
        boot=self.inputs._boot_us(self.now)
        return bool(row and boot is not None and row['boot_session']==self.inputs.boot_session
            and 0<=self.now-row['at']<=.5 and 0<=boot-row['boot_ms']*1000<=500000)

    def _service_pilot(self,now):
        if self.pilot_link is None:return
        self.pilot_link.check_mode_timeout(now)
        if self.stop_latched or self.core.stop_latched or self.inputs.home_down or self.closing or self.pilot_fault:return
        from types import SimpleNamespace
        from gamepad_rc import CODE_OF,KEY_NAME
        state=self.pilot_state
        if (not isinstance(state,dict) or not isinstance(state.get('axes'),dict)
                or any(type(state['axes'].get(code)) is not int for code in CODE_OF.values())
                or not isinstance(state.get('buttons'),(tuple,list))
                or not isinstance(state.get('edges'),(tuple,list))
                or len(state['edges'])>64
                or any(name not in KEY_NAME.values() for name in (*state['buttons'],*state['edges']))):
            self.pilot_fault='fresh_pilot_snapshot_missing_or_invalid'
            self.pilot_centered=False
            self._fail(self.pilot_fault)
            return  # Keep the existing LAND/localization/HOME owner running.
        pad=SimpleNamespace(axes=state['axes'],key_edges=state['edges'],held=lambda name:name in state['buttons'])
        channels=self.pilot_mapper.channels(pad)
        self.pilot_channels=tuple(channels)
        self.pilot_centered=all(v==1500 for v in channels[:4])
        self.pilot_sample_at=now
        c=self.context(now)
        if (not self.inputs.home_proven or self.inputs.fault or not c.gamepad_current
                or not c.epoch_valid or not self._identity_ok() or not 0<=c.actuator_age<=.5
                or not 0<=c.attitude_age<=.5):return
        mode=self.pilot_mapper.mode_request
        try:
            if self.start_requested_at is not None:
                # Yield BEFORE this turn's first non-neutral RC frame or mode command.
                # A stick movement requests Position once; later stick frames must
                # not repeatedly override an explicit Altitude/Land choice.
                if not self.pilot_centered and not self.core.pilot_control and mode is None:mode=2
                if mode is not None:self.pilot_link.request_mode(mode,now=now)
            elif mode==2 and self.ground_position_allowed():
                self.pilot_link.request_mode(mode,now=now,takeover=False)
            if not self.pilot_centered and not self.core.pilot_control:return  # Recenter before start.
            if now>=self.next_pilot_rc:
                self.pilot_link.send_rc(channels)
                self.next_pilot_rc=now+.05
        except (ValueError,RuntimeError,OSError,TypeError) as exc:
            self.pilot_fault='pilot_dispatch_failed: '+str(exc)
            self._fail(self.pilot_fault)
            # No replay of mode/RC. Fresh localization and HOME remain serviced.

    @property
    def core(self):
        return self.coordinator.core

    def _handed_off(self):
        return self.coordinator.fusion.transferred_core is self.core

    def _fc_epoch_usable(self):
        # A dead/replaced gamepad permanently aborts the mission, but does
        # not make the still-current FC boot clock/identity disappear. Keep
        # LAND, fresh localization and safe ground cleanup available. NEVER
        # clear the fault or use this path to start/resume a mission.
        return self.inputs.fault in (None,'gamepad_disconnected','gamepad_generation_changed')

    def _continuous_context(self, now):
        c = context_from_inputs(self.inputs,now)
        if self._handed_off() and self._fc_epoch_usable():
            return replace(c,epoch_valid=True)
        return c

    def _event(self, name):
        self.events.append(dict(at=self.now, event=name, phase=self.core.phase))

    def _identity_ok(self):
        if not self._handed_off():
            return super()._identity_ok()
        c = self.context()
        return bool(self.inputs.identity and c.epoch_valid and c.heartbeat_age < 2)

    def fixed_timesync_allowed(self, msg):
        if not self.fixed_configuration or not self._identity_ok():return False
        if msg.tc1 == 0:
            return bool(self.fixed_clock is not None and msg.ts1 == self.fixed_clock.pending
                and msg.ts1 == int(self.now*1e9))
        return bool(self.fixed_sync_reply is not None
            and (msg.tc1,msg.ts1) == self.fixed_sync_reply)

    def sync_rate_allowed(self,msg):
        return bool(self.sync_rates and self.sync_rates.permits(msg,
            fresh=self._identity_ok(),locked=self.context().restore_ground()))

    def _service_sync_rates(self,now):
        rates=self.sync_rates
        if rates is None:return
        if not rates.restoring and (self.failure or self.stop_latched or self.cancel_requested):return
        if not rates.restoring and not self.sync_rates_enabled:return
        outgoing=rates.action(now,fresh=self._identity_ok() or rates.restoring and not rates.changed,
            locked=(self.context().restore_ground() if rates.restoring else self.context().preparation_ground()))
        if outgoing is not None:
            try:self.writer(outgoing)
            except Exception as exc:
                rates.dispatch_failed(str(exc));raise
        if rates.error:self._fail(rates.error)
        if rates.restoring and rates.phase=='RESTORE_FAILED':self._fail('timesync_rate_restore_failed')

    def _capture(self, msg, *, route=None):
        if self.pilot_source=='native_rc' and (msg.get_type() in ('RC_CHANNELS_OVERRIDE','MANUAL_CONTROL')
                or msg.get_type()=='COMMAND_LONG' and msg.command==185):
            raise PermissionError('Native RC forbids computer stick/lock/unlock commands')
        kind = msg.get_type()
        if (kind=='COMMAND_LONG' and msg.command in (510,511) and self.sync_rates is not None):
            if route not in (None,'trs') or not self.sync_rate_allowed(msg):
                raise PermissionError('Only current same-owner ground sync-rate transaction allowed')
            wire=msg.pack(self.encoder);self.encoder.seq=(self.encoder.seq+1)%256
            self.frames.append(dict(at=self.now,type=kind,route='trs',wire_hex=wire.hex()))
            self.sent_counts[kind]+=1
            return True
        if self._pilot_candidate is not None:
            if route not in (None,'trs') or not self.pilot_candidate_allowed(msg):
                raise PermissionError('Not a current shared-owner pilot candidate')
            wire=msg.pack(self.encoder);self.encoder.seq=(self.encoder.seq+1)%256
            self.frames.append(dict(at=self.now,type=kind,route='trs',wire_hex=wire.hex()))
            self.sent_counts[kind]+=1
            return True
        if self.fixed_configuration:
            if kind in ('PARAM_SET','PARAM_EXT_SET'):
                raise PermissionError('Plan B mission never writes or restores EKF parameters')
            stop=kind=='COMMAND_LONG' and msg.command==185 and msg.param1==2
            land=kind=='COMMAND_LONG' and msg.command==21
            landing_pose=kind=='VISION_POSITION_ESTIMATE' and self.core.phase in ('LAND','LAND_TIMEOUT','PILOT_CONTROL')
            effect=kind in ('VISION_POSITION_ESTIMATE','SET_POSITION_TARGET_LOCAL_NED') or (
                kind=='COMMAND_LONG' and msg.command!=512)
            if effect and not (stop or land or landing_pose):
                if not self._fixed_clock_ok() or not self.coordinator.fusion.configuration_bound(self.context()):
                    raise PermissionError('Current clock and same-session fixed configuration required')
        if self._handed_off() and kind in ('VISION_POSITION_ESTIMATE',
                                         'SET_POSITION_TARGET_LOCAL_NED', 'COMMAND_LONG'):
            params = tuple(getattr(msg, f'param{i}') for i in range(1, 8)) if kind == 'COMMAND_LONG' else ()
            unlocking = bool(kind == 'COMMAND_LONG' and msg.command == 185 and params == (0,)*7)
            cleanup_lock = bool(kind == 'COMMAND_LONG' and msg.command == 185 and params == (2,0,0,0,0,0,0)
                and self.cleanup_lock_at == self.now and self.core.phase in ('COMPLETE', 'BLOCKED')
                and self._degraded_observation(self.now).link_ok()
                and self._degraded_observation(self.now).landed
                and not self._degraded_observation(self.now).armed)
            if unlocking:
                o, missing = self.inputs.build(self.now)
                allowed = bool(self.start_requested_at == self.now and not self.enable_sent
                    and not self.failure and not self.stop_latched and not self.cancel_requested
                    and self.core.phase == 'WAIT_EXPLICIT_START' and o is not None and not missing
                    and o.lockdown and o.landed and not o.armed and not o.home_pressed
                    and self.core._binding(o) == self.start_binding
                    and (msg.target_system, msg.target_component, msg.confirmation) == (1, 1, 0))
                if not allowed:
                    raise PermissionError('Output enable is an explicit one-shot OFFLINE ground transaction')
            elif not cleanup_lock and not self.envelope.candidate_allowed(msg):
                return super()._capture(msg, route=route)
            if route not in (None, 'trs'):
                raise PermissionError('Continuous mission has one TRS capture route only')
            wire = msg.pack(self.encoder)
            self.encoder.seq = (self.encoder.seq+1) % 256
            self.frames.append(dict(at=self.now, type=kind, route='trs', wire_hex=wire.hex()))
            self.sent_counts[kind] += 1
            self.trace.append(dict(at=self.now, event='capture', type=kind))
            return True
        return super()._capture(msg, route=route)

    def _ingest_owner_message(self, msg, now):
        super()._ingest_owner_message(msg, now)
        if self.sync_rates is not None and self._identity_ok():self.sync_rates.ingest(msg,now)
        if self.native_configuration is not None and msg.get_type()=='PARAM_VALUE' and (msg.get_srcSystem(),msg.get_srcComponent())==(1,1):
            from mavlink_readonly_probe import decode_parameter
            name=msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id,bytes) else msg.param_id.rstrip('\0')
            if name in self.native_configuration and (decode_parameter(msg)!=self.native_configuration[name]
                    or msg.param_type!=self.inputs.native_rc.PARAMETER_TYPES[name]):
                self._fail('native_rc_configuration_changed:'+name)
        if self.pilot_link is not None and self._identity_ok() and not self.inputs.fault:
            self.pilot_link.observe_mode(msg,now=now)
            if msg.get_type()=='RC_CHANNELS' and (msg.get_srcSystem(),msg.get_srcComponent())==(1,1):
                boot=self.inputs._boot_us(now)
                values=tuple(getattr(msg,'chan%d_raw'%i) for i in range(1,5))
                if (boot is not None and 0<=boot-msg.time_boot_ms*1000<=500000 and msg.chancount>=4
                        and (self.pilot_rc_feedback is None or msg.time_boot_ms>self.pilot_rc_feedback['boot_ms'])
                        and any(values==row[:4] for row in self.pilot_link._sent_history)):
                    self.pilot_rc_feedback=dict(at=now,boot_ms=msg.time_boot_ms,boot_session=self.inputs.boot_session)
        if self.fixed_configuration and self._identity_ok():
            profile=self.coordinator.fusion.profile
            if (isinstance(profile,dict) and profile.get('timestamp_mode')=='local_ros_timesync'
                    and msg.get_type()=='TIMESYNC' and msg.tc1==0
                    and (msg.get_srcSystem(),msg.get_srcComponent())==(1,1)
                    and getattr(msg,'target_system',0) in (0,250)
                    and getattr(msg,'target_component',0) in (0,220)
                    and msg.ts1>self.fixed_sync_last_token
                    and type(self.fixed_ros_ns) is int and self.fixed_ros_ns>0):
                # Same owner/socket and the SAME local ROS clock as Vicon.
                # Host-originated probes below do not update the FC filter.
                self.fixed_sync_last_token=msg.ts1
                self.fixed_sync_reply=(self.fixed_ros_ns,msg.ts1)
                try:
                    if self.writer(self.encoder.timesync_encode(*self.fixed_sync_reply)) is True:
                        self.fixed_sync_reply_count+=1
                        self.coordinator.fusion.timesync_reply_count=self.fixed_sync_reply_count
                finally:self.fixed_sync_reply=None
            self.fixed_clock.ingest(msg,int(now*1e9))
            if self.inputs.native_rc and self.fixed_clock.reference:
                ref=self.fixed_clock.reference
                if ref['received_ns']==int(now*1e9):
                    self.inputs.native_rc.observe_clock((int(now*1e9)+ref['upper_ns'])/1000,now,
                                                        remote_boot_us=ref['remote_ns']/1000)
        if (self.cleanup_lock_at is not None and msg.get_type() == 'COMMAND_ACK'
                and msg.command == 185 and msg.result == m.MAV_RESULT_ACCEPTED
                and (msg.get_srcSystem(), msg.get_srcComponent()) == (1,1)
                and (msg.target_system, msg.target_component) == (250,220)
                and self.cleanup_lock_at <= now <= self.cleanup_lock_at+self.start_timeout):
            self.cleanup_ack_at = now
        if (self.enable_sent and self.enable_ack_at is None and self.start_requested_at is not None
                and msg.get_type() == 'COMMAND_ACK' and msg.command == 185
                and (msg.get_srcSystem(), msg.get_srcComponent()) == (1, 1)
                and (msg.target_system, msg.target_component) == (250, 220)
                and self.start_requested_at <= now <= self.start_requested_at+self.start_timeout):
            if msg.result == m.MAV_RESULT_IN_PROGRESS:
                pass  # Wait for final acceptance inside the original ACK window.
            elif msg.result != m.MAV_RESULT_ACCEPTED:
                self._fail('output_enable_rejected')
            else:
                self.enable_ack_at = now

    def _degraded_observation(self, now):
        """Absent evidence remains invalid; preserve armed state for LAND/HOME.

        This observation is NEVER used for prepare/start. It lets the core
        handle missing inputs without stopping the entire owner/reactor.
        """
        i = self.inputs
        arm = i.actuator or {}
        local = i.local or {}
        attitude = i.attitude or {}
        identity = i.identity or ('', '')
        status_age = math.inf
        if arm and attitude and self._fc_epoch_usable():
            from rc_watch import state_feedback_age
            status_age = state_feedback_age(now,arm['at'],(i._boot_us(now)-arm['timestamp_us'])/1e6,
                native=self.pilot_source=='native_rc')
        counters = tuple(i.audit_rows[-1]['local'].get(k) for k in
            ('xy_reset_counter','z_reset_counter','heading_reset_counter')) if i.audit_rows else ()
        return Observation(now, i.boot_session, i.rigid_revision, counters,
            local.get('position', (math.nan,)*3), local.get('velocity', (math.nan,)*3),
            attitude.get('yaw', math.nan), uid2=identity[0], firmware=identity[1],
            status_age_s=status_age, local_age_s=age(now, local.get('at')),
            pose_age_s=age(now, i.pose_at), armed=arm.get('armed', False),
            landed=bool(i.land and age(now, i.land['at']) <= .5 and i.land['landed']),
            lockdown=arm.get('lockdown', True), offboard=bool(i.hb and i.hb['offboard']),
            manual_control_active=i.manual_mode_confirmed(now),
            fusion_ready=False, preflight_ready=False,
            gamepad_available=False, home_response_verified=i.home_proven,
            home_pressed=i.home_down)

    def _fail(self, reason):
        active = self._handed_off()
        phase = self.core.phase
        super()._fail(reason)
        if active and phase == 'COMPLETE':
            self.core.phase = phase
        elif active and self.core.pilot_control and not self.core.stop_latched:
            self.core.phase='PILOT_CONTROL'  # Base failure must not suppress fresh localization.
        elif active and not self.core.stop_latched:
            self.core._abort(self.failure, self._degraded_observation(self.now))

    def _service_home(self):
        if self.pilot_source=='native_rc':return
        if not self._handed_off():
            return super()._service_home()
        if self.stop_latched or self.inputs.home_down:
            self.core.home(self._degraded_observation(self.now))

    def request_start(self, *, operator_confirmed=False):
        """Explicit SYNTHETIC start; cannot obtain hardware authorization.

        A single modeled enable request, then accepted ACK AND a new actual
        unlocked actuator observation are required. Never retries enable/arm.
        """
        self._assert_owner()
        if not self.ground_mode_confirmed():
            raise PermissionError('Ground Position needs ACK and actual mode confirmation')
        if self.now is None:
            raise PermissionError('Continuous owner has not received any current inputs')
        if self.pilot_link is not None and (self.pilot_sample_at!=self.now or not self.pilot_centered or not self.pilot_rc_received()):
            raise PermissionError('Fresh centered sticks and RC_CHANNELS receipt required before start')
        if self.fixed_configuration and (not self._fixed_clock_ok()
                or not self.coordinator.fusion.configuration_bound(self.context())):
            self._fail('fixed_configuration_or_timesync_lost_before_start')
            raise PermissionError('No start with stale clock, changed profile/configuration or boot')
        try:require_frozen_selector(self.inputs.selector,self.core.frozen_selector)
        except (ValueError,RuntimeError,KeyError,TypeError):
            raise PermissionError('Estimator changed before output enable; no start or rebase')
        o, missing = self.inputs.build(self.now)
        if (operator_confirmed is not True or not self._handed_off()
                or self.core.phase != 'WAIT_EXPLICIT_START' or o is None or missing
                or self.failure or self.stop_latched or self.cancel_requested
                or self.start_requested_at is not None or self.closing or self.closed
                or not o.navigation_ok() or not o.preflight_ready or not o.preparation_ground_ok()
                or not o.start_pilot_ok()
                or o.armed or not o.landed
                or self.core._binding(o) != self.core.binding
                or math.dist(o.position_ned, self.core.origin.position_ned) > .03):
            raise PermissionError('Fresh unchanged locked readiness and one explicit OFFLINE start required')
        self.start_requested_at = self.now
        self.start_binding = self.core._binding(o)
        if self.pilot_source=='native_rc':
            # Only timestamps already received from the FC prove a publication
            # predates start. Do not extrapolate a host clock or add a grace period:
            # a genuinely new switch change must still yield immediately.
            self.native_start_boot_us=max(self.inputs.attitude['boot_ms']*1000,
                self.inputs.native_rc.fc_raw_boot_us or 0)
            self.core.start(o,self.inputs.selector,operator_confirmed=True)
            self._event('native_rc_explicit_start_without_output_enable')
            return
        self.writer(self.encoder.command_long_encode(1, 1, 185, 0, 0, 0, 0, 0, 0, 0, 0))
        self.enable_sent = True
        self._event('synthetic_output_enable_requested')

    def request_cancel(self, *, reason='operator_cancel'):
        self._assert_owner()
        self.cancel_requested = True
        if self.now is not None:
            self._fail(reason)

    def request_close(self):
        self._assert_owner()
        if self._handed_off() and self.core.phase not in ('COMPLETE', 'BLOCKED', 'STOP_LATCHED'):
            raise PermissionError('Do not restore fusion while a mission owns it; cancel/land first')
        phase = self.core.phase
        result = super().request_close()
        if self.sync_rates is not None:
            self.sync_rates.begin_restore(self.now)
            if self.coordinator.fusion.phase=='NEW':self.coordinator.fusion.phase='CLOSED'
            self.closing=True;self.closed=False
        if phase == 'COMPLETE':
            self.core.phase = phase
        return result

    def _owner_can_advance(self):
        return self._handed_off() or super()._owner_can_advance()

    def _accept_owner_pose(self):
        if self._handed_off():
            return not (self.stop_latched or self.closing)
        return super()._accept_owner_pose()

    def _advance_owner(self, now):
        self._service_native_rc(now)
        self._service_pilot(now)
        if self.core.cooperative is not None:
            self.core.cooperative.service(self,now)
            if (self.core.cooperative.car_task is not None and self.core.cooperative.fault
                    and not self.core.arm_attempted):
                self._fail(self.core.cooperative.fault)
        if self.sync_rates is not None and not self._handed_off() and self.sync_rates.phase!='ACTIVE':
            self.coordinator.waiting_reason='timesync_rate_setup_pending'
            return
        if self.fixed_configuration:
            configured=self.coordinator.fusion.configuration is not None
            if configured and (not self._fixed_clock_ok()
                    or not self.coordinator.fusion.configuration_bound(self.context(now))):
                self._fail(self.coordinator.fusion.fault or 'fixed_configuration_or_timesync_lost')
                if not self._handed_off():return
            elif not self._fixed_clock_ok():
                self.coordinator.waiting_reason='current_TIMESYNC_reference_missing'
                return
        if not self._handed_off():
            if (self.inputs.preparation_verified(now) and not self.prepare_requested
                    and (self.pilot_source!='native_rc' or self.native_configuration is not None)
                    and not self.cancel_requested and self.context(now).preparation_ground()):
                self.request_prepare(operator_confirmed=True)
                self._event('prepare_after_native_rc' if self.pilot_source=='native_rc' else 'prepare_after_one_home')
            super()._advance_owner(now)
            if self._handed_off():
                self.inputs.offline_flight_observation = True
                self.wait_started_at = now
                self._event('continuous_same_writer_handoff')
            return
        o, missing = self.inputs.build(now)
        fallback = self._degraded_observation(now)
        if self.core.phase == 'WAIT_EXPLICIT_START':
            if self.failure or missing or o is None:
                self._fail('inputs_lost_before_explicit_start')
            elif self.start_requested_at is not None:
                if now > confirmation_deadline(self.start_requested_at, self.enable_ack_at, self.start_timeout):
                    self._fail('output_enable_state_confirmation_timeout' if self.enable_ack_at is not None
                               else 'output_enable_ack_timeout')
                elif self.core._binding(o) != self.start_binding:
                    self._fail('start_epoch_changed')
                elif self.enable_ack_at is not None and not o.lockdown:
                    actor = self.inputs.actuator
                    boot_at_ack = self.inputs._boot_us(now)-(now-self.enable_ack_at)*1e6
                    if (actor['at'] > self.enable_ack_at
                            and actor['timestamp_us'] > boot_at_ack):
                        try:
                            self.core.start(o, self.inputs.selector, operator_confirmed=True)
                            self._event('synthetic_mission_started')
                        except (ValueError, RuntimeError, KeyError):
                            self._fail('start_observation_changed')
            elif self.pilot_source!='native_rc' and not o.lockdown:
                self._fail('unexpected_output_enable')
        # After a parser/input failure LAND/HOME must continue; never return
        # early merely because build() cannot produce a flight-ready object.
        obs = o if o is not None and not self.failure else fallback
        if self.failure and not self.core.stop_latched:
            self.core._abort(self.failure, obs)
        source = self.coordinator.source_pose
        vision = None
        if source and source[0] == self.inputs.pose_stamp and obs.pose_age_s <= .15:
            stamp, pos, quat = source
            vision = (stamp//1000, pos, quat, self.coordinator.fusion.alignment)
        boot = self.inputs._boot_us(now)
        # Arm state and Commander are independent replies. Use the pending
        # command's existing confirmation deadline, not a second 0.5s cutoff.
        # Hold the ground target; agreement is still required before ascent.
        if (missing == ['commander_and_actuator_armed_state_disagrees']
                and self.core.phase == 'WAIT_ARM' and self.core.pending
                and not self.failure):
            pending=self.core.pending
            if pending.get('rejected'):
                self.core._abort('command_rejected',obs)
            elif now>confirmation_deadline(pending['sent_at'],pending.get('ack_at'),self.core.command_timeout):
                self.core._abort('command_state_confirmation_timeout' if pending['accepted']
                                 else 'command_ack_timeout',obs)
            else:
                if vision is not None:
                    self.core.vision_message(obs, *vision)
                if self.core.last_setpoint is None or now-self.core.last_setpoint >= self.core.setpoint_interval:
                    self.core._setpoint(obs, int(boot//1000))
                return
        self.core.cycle(obs, self.inputs.selector or {},
                        boot_ms=int(boot//1000) if boot is not None else 0, vision=vision)
        if (self.core.phase=='PILOT_CONTROL'
                and fallback.link_ok() and fallback.landed and not fallback.armed):
            self.core.phase='COMPLETE' if self.core.arm_attempted else 'BLOCKED'
            self.core.ground_confirmed_at=now
            self._event('pilot_landing_observed' if self.core.arm_attempted else 'prearm_takeover_ground_close')
        # Restoration is possible only AFTER an observed landed AND disarmed
        # state, then a fresh lock ACK/state. Never turn fusion off in the air.
        if (self.core.phase in ('COMPLETE','BLOCKED','STOP_LATCHED')
                and fallback.link_ok() and fallback.landed and not fallback.armed):
            actor = self.inputs.actuator
            cleanup_proven = bool(self.cleanup_lock_at is None or self.cleanup_ack_at is not None
                and actor['at'] > self.cleanup_ack_at
                and actor['timestamp_us'] > self.inputs._boot_us(now)-(now-self.cleanup_ack_at)*1e6)
            if (fallback.lockdown or self.pilot_source=='native_rc') and cleanup_proven:
                self.request_close()
                self._event('ground_close_without_parameter_writes' if self.fixed_configuration else 'ground_restore_requested')
            elif self.cleanup_lock_at is None and self.core.phase != 'STOP_LATCHED':
                self.cleanup_lock_at = now
                self.writer(self.encoder.command_long_encode(1,1,185,0,2,0,0,0,0,0,0))
                self._event('landed_ground_lock_requested')

    def _owner_diagnostics(self, now, full):
        if self.fixed_configuration and self._identity_ok() and now>=self.next_fixed_timesync:
            token=self.fixed_clock.request(int(now*1e9))
            if token is not None:self.writer(self.encoder.timesync_encode(0,token))
            self.next_fixed_timesync=now+.1
        # Once ground cleanup has its evidence, drain the existing reply but
        # do not open another query merely to keep the shell busy at shutdown.
        if (self.closing and self.sync_rates is not None and self.sync_rates.phase=='CLOSED'
                and self.context().restore_ground() and self.coordinator.audit.transaction.idle):
            return
        self.coordinator.service_diagnostics(now, full_audit=full,
            offline_active_mission=self._handed_off(),
            restoration_only=bool(self.closing or
                (self.failure or self.stop_latched) and not self.core.arm_attempted))

    def _fixed_clock_ok(self):
        if not self.fixed_configuration:return True
        ref=self.fixed_clock.reference
        return bool(self.now is not None and self.fixed_clock.fault is None and ref
            and 0<=int(self.now*1e9)-ref['received_ns']<self.fixed_clock.MAX_AGE_NS)

    def turn(self, now, *, pilot_state=None, **kwargs):
        # Reject cross-thread ownership instead of silently opening a second
        # reader/writer. No threads or devices are created by this object.
        self._assert_owner()
        # Set before RX dispatch; never substitute monotonic session time for
        # the ROS timestamp domain carried by VISION_POSITION_ESTIMATE.
        ros_ns=kwargs.get('now_ros_ns')
        # Car feedback/source headers use ROS time even when the approved
        # UAV VPE profile uses FC receive time. Do not conflate these clocks.
        self.cooperative_ros_ns=ros_ns if type(ros_ns) is int and ros_ns>0 else None
        if not (self.fixed_configuration and isinstance(self.coordinator.fusion.profile,dict)
                and self.coordinator.fusion.profile.get('timestamp_mode')=='local_ros_timesync'):
            ros_ns=None
        if type(ros_ns) is int and ros_ns>0:
            if self.fixed_ros_ns is not None and ros_ns<self.fixed_ros_ns:
                self.fixed_ros_ns=None
                self._fail('vicon_source_clock_regression')
            else:self.fixed_ros_ns=ros_ns
        else:self.fixed_ros_ns=None
        if self.pilot_source=='native_rc' and (pilot_state is not None or kwargs.get('pad_generation') is not None
                or kwargs.get('home_events')):
            raise PermissionError('Native RC session does not accept gamepad/HOME input')
        self.pilot_state=deepcopy(pilot_state);self.pilot_sample_at=None
        if self.pilot_link is not None and any(kwargs.get('home_events',())) and self.prepare_requested:
            self.pilot_link.stop_requested=True
            if self.pilot_link.mode_request and self.pilot_link.mode_request['state']=='WAITING':
                self.pilot_link.mode_request['state']='CANCELLED_BY_HOME'
        # The base closing branch skips _advance_owner while restoring rates.
        # Keep the existing car owner alive with STOP only during that gap.
        if self.closing and not self.closed and self.core.cooperative is not None:
            self.core.cooperative.service_stopped(now,self.cooperative_ros_ns)
        super().turn(now, **kwargs)
        self._service_sync_rates(now)
        if self.closing and self.sync_rates is not None:
            self.closed=self.sync_rates.phase in ('CLOSED','RESTORE_FAILED')
            if (self.sync_rates.phase=='CLOSED' and
                    (self.sync_rates.changed or self.prepare_requested)):
                # Rate readback may finish between ground-state samples.
                # Keep servicing diagnostics until the journal can use fresh
                # ground proof, rather than stopping with a stale snapshot.
                self.closed=(self.context().restore_ground() and
                    (self.coordinator.audit.transaction.idle or self.coordinator.audit.disabled))
        return self.status()

    def _service_native_rc(self, now):
        if self.inputs.native_rc is None:return
        s=self.inputs.pilot_status(now)
        switches=s['switches']
        mode_changed=False
        if s['native_input_observed'] and switches and 1 <= switches['mode_slot'] <= 6:
            sample=(switches['timestamp'],switches['mode_slot'])
            previous=self.native_mode_sample
            if previous is None or sample[0] > previous[0]:
                mode_changed=previous is not None and sample[1]!=previous[1]
                self.native_mode_sample=sample
                if (self.native_start_boot_us is not None
                        and sample[0] <= self.native_start_boot_us):
                    mode_changed=False  # Delayed pre-start publication, baseline only.
        if self.inputs.preparation_verified(now) and self.inputs.native_ready_at is None:
            self.inputs.native_ready_at=now
        if self.native_configuration is None and self.inputs.native_ready_at is not None:
            c=self.coordinator; names=self.inputs.native_rc.PARAMETER_TYPES
            fresh=all(self.inputs.native_ready_at<=c.parameter_requests.get(n,-1)<=c.parameter_times.get(n,-1)<=now
                      and now-c.parameter_times.get(n,-1)<4 for n in names)
            if fresh and self.inputs.native_rc.configuration_valid(c.parameters,c.parameter_types):
                self.native_configuration={n:c.parameters[n] for n in names}
            else:self.coordinator.waiting_reason='native_rc_override_kill_or_mode_configuration_missing'
        if not self.prepare_requested:return
        if s['kill_requested'] is True or s['stop_active'] is True:
            self.core.home(self._degraded_observation(now))
            self.stop_latched=True
            return
        # Physical manual-mode acceptance always wins over loss/abort handling.
        self.core.observe_takeover(self._degraded_observation(now))
        if self.core.pilot_control:return
        # A fresh physical mode-switch change yields immediately, before the
        # next mode/arm/target dispatch. FC acceptance remains a separate fact.
        # Mission LAND does not change the RC slot and cannot trigger this.
        if mode_changed and self.core.phase in ('WARMUP','WAIT_OFFBOARD','WAIT_ARM','ASCEND','HOVER','DESCEND','LAND','LAND_TIMEOUT'):
            self.core.request_takeover('native_rc_mode_switch_intent')
        elif s['sticks_moving'] and self.core.phase in ('WARMUP','WAIT_OFFBOARD','WAIT_ARM','ASCEND','HOVER','DESCEND','LAND','LAND_TIMEOUT'):
            self.core.request_takeover('native_rc_stick_intent')
        elif not s['native_input_observed'] or s['kill_requested'] is not False:
            self._fail('native_rc_loss_reported' if s['loss_reported'] else 'native_rc_diagnostics_expired')

    def native_takeover_confirmed(self, now):
        if self.inputs.manual_mode_confirmed(now):return True
        hb=self.inputs.hb
        return bool(self.core.takeover_reason=='native_rc_mode_switch_intent'
            and hb and 0 <= now-hb['at'] < 2 and hb['armed']
            and hb['custom_mode']==((6<<24)|(4<<16)))  # PX4 AUTO/LAND

    def _assert_owner(self):
        from threading import get_ident
        if self.owner_thread is None:
            self.owner_thread = get_ident()
        if get_ident() != self.owner_thread:
            raise RuntimeError('Continuous reactor has one immutable thread owner')

    def command_confirmation(self):
        """Small current-state view for the existing console, no new polling."""
        if self.pilot_link and self.pilot_link.mode_request:
            request=self.pilot_link.mode_request
            if request['state']=='WAITING':
                return dict(command=176,waiting_for='state' if request['ack'] else 'ack',
                    ack_deadline=request['sent_at']+COMMAND_ACK_TIMEOUT,
                    state_deadline=confirmation_deadline(request['sent_at'],request['ack_at']))
        if self.start_requested_at is not None and self.core.phase=='WAIT_EXPLICIT_START':
            return dict(command=185,waiting_for='state' if self.enable_ack_at is not None else 'ack',
                ack_deadline=self.start_requested_at+self.start_timeout,
                state_deadline=confirmation_deadline(self.start_requested_at,self.enable_ack_at,self.start_timeout))
        return self.core.status()['command_confirmation']

    def status(self):
        result = super().status()
        result.update(scope=('Continuous one-owner mission with execution adapter; physical evidence is external'
            if getattr(self,'device_adapter_attached',False) else
            'OFFLINE continuous one-owner mission integration; NOT hardware or SITL'),
            core=self.core.status(), start_requested=self.start_requested_at is not None,
            output_enable_ack_received=self.enable_ack_at is not None,
            command_confirmation=self.command_confirmation(),
            events=deepcopy(self.events), single_home_preparation=self.pilot_source=='gamepad',
            pilot_source=self.pilot_source,native_rc=self.inputs.pilot_status(self.now) if self.now is not None else None,
            native_rc_configuration=deepcopy(self.native_configuration),
            continuous_fusion_owner='core' if self._handed_off() else 'preparation',
            configuration_mode='fixed' if self.fixed_configuration else 'legacy_dynamic_experiment',
            fixed_clock=self.fixed_clock.summary() if self.fixed_clock else None,
            fc_timesync_replies=self.fixed_sync_reply_count,
            sync_rates=self.sync_rates.summary() if self.sync_rates else None,
            pilot=dict(connected=self.pilot_link is not None,sticks_centered=self.pilot_centered,fault=self.pilot_fault,
                mode_request=deepcopy(self.pilot_link.mode_request) if self.pilot_link else None,
                rc_received_fresh=self.pilot_rc_received() if self.now is not None else False,
                rc_rate_hz=20,aux_channels='ignored'),
            execution_adapter_attached=getattr(self,'device_adapter_attached',False),
            hardware_execution_available=False, flight_authorized=False)
        return result
