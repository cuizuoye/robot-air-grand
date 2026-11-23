"""PX4 1.14.3 TRS mission scheduler; hardware dispatch is NOT enabled here.

Reuses the existing EKF settlement, ENU/NED math and MissionPreview. Produces
real MAVLink messages and consumes correlated ACKs + fresh observed state.
No serial/network API, parameter writes, output-unlock or automatic start.
An injected sink is for offline integration only; implementing/authorizing a
real single-owner flight transport remains a separate reviewed task.
"""
from dataclasses import dataclass, replace
import math
from pathlib import Path
import sys

from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.frames import ned_to_enu_vector
from uav_coord_bridge.mission_preview import MissionPreview, Observation as PreviewObservation, State
from ground_estimator_settlement import GroundEstimatorSettlement, require_frozen_selector
from disarmed_offboard_policy import POSITION_YAW_MASK, POSITION_VELOCITY_YAW_MASK
from trs_uart_readonly_check import UID2
import compact_vision
from px4_modes import COMMAND_ACK_TIMEOUT, confirmation_deadline
from follow_target_generator import (NORMAL_DESCENT_SPEED_MPS,
    LAND_HANDOFF_TOLERANCE_M, LAND_HANDOFF_MAX_VERTICAL_SPEED_MPS)


@dataclass(frozen=True)
class Observation:
    t: float
    boot_session: str
    rigid_revision: str
    reset_counters: tuple
    position_ned: tuple
    velocity_ned: tuple
    yaw_ned: float
    uid2: str = ''
    firmware: str = ''
    status_age_s: float = math.inf
    local_age_s: float = math.inf
    pose_age_s: float = math.inf
    armed: bool = False
    landed: bool = False
    lockdown: bool = True
    offboard: bool = False
    fusion_ready: bool = False
    preflight_ready: bool = False
    gamepad_available: bool = False
    home_response_verified: bool = False
    home_pressed: bool = False
    manual_control_active: bool = False  # Fresh FC mode evidence, not merely !offboard.
    pilot_source: str = 'gamepad'
    native_rc_ready: bool = False
    native_sticks_ready: bool = False
    native_stop: bool = False

    def pilot_available(self):
        return self.native_rc_ready if self.pilot_source == 'native_rc' else self.gamepad_available

    def preparation_ground_ok(self):
        if self.pilot_source == 'native_rc':
            return bool(not self.armed and self.landed and not self.lockdown
                        and self.native_rc_ready and not self.native_stop)
        return bool(not self.armed and self.landed and self.lockdown
                    and self.home_response_verified and not self.home_pressed)

    def start_pilot_ok(self):
        return (self.native_rc_ready and self.native_sticks_ready and not self.native_stop
                if self.pilot_source == 'native_rc' else self.home_response_verified and not self.home_pressed)

    def link_ok(self):
        from rc_watch import NATIVE_COMMANDER_MAX_AGE_S
        return bool(self.uid2 == UID2 and self.firmware == '1.14.3'
                    and math.isfinite(self.status_age_s) and 0 <= self.status_age_s <=
                    (NATIVE_COMMANDER_MAX_AGE_S if self.pilot_source=='native_rc' else .5))

    def navigation_ok(self):
        numbers = (self.t, *self.position_ned, *self.velocity_ned, self.yaw_ned,
                   self.local_age_s, self.pose_age_s)
        flags = (self.armed,self.landed,self.lockdown,self.offboard,self.fusion_ready,
                 self.preflight_ready,self.gamepad_available,self.home_response_verified,self.home_pressed,
                 self.manual_control_active,self.native_rc_ready,self.native_sticks_ready,self.native_stop)
        return bool(self.link_ok() and all(math.isfinite(v) for v in numbers)
                    and all(type(flag) is bool for flag in flags)
                    and len(self.position_ned) == len(self.velocity_ned) == 3
                    and self.fusion_ready and 0 <= self.local_age_s <= .5
                    and self.pilot_source in ('gamepad','native_rc')
                    and 0 <= self.pose_age_s <= .15 and self.pilot_available())


class MissionCore:
    """One-shot scheduler. A clean start never clears an existing HOME latch.

    sink(message) must return True only after its offline transport accepted
    the entire message. Real transport/flight approval is deliberately absent.
    In particular, a command ACK never substitutes for mode/armed/landed state.
    """
    component = 220
    warmup_seconds = 2.0
    command_timeout = COMMAND_ACK_TIMEOUT
    setpoint_interval = .1
    maximum_setpoint_gap = .2
    vision_interval = 1 / 32
    maximum_vision_gap = .15
    maximum_climb_speed = .15

    def __init__(self, sink, *, height=.5, hover_seconds=5., pilot_source='gamepad', horizontal_motion=None):
        if not callable(sink):
            raise ValueError('Explicit offline message sink required')
        if not .2 <= height <= .5 or not 1 <= hover_seconds <= 10:
            raise ValueError('First mission limited to 0.2..0.5m and 1..10s hover')
        self.sink = sink
        if pilot_source not in ('gamepad','native_rc'):raise ValueError('Unknown pilot source')
        self.pilot_source = pilot_source
        if pilot_source=='native_rc':
            from rc_watch import NATIVE_DISPATCH_MAX_GAP_S
            self.maximum_vision_gap = self.maximum_setpoint_gap = NATIVE_DISPATCH_MAX_GAP_S
        self.encoder = m.MAVLink(None, srcSystem=250, srcComponent=self.component)
        from rc_watch import NATIVE_COMMANDER_MAX_AGE_S
        from follow_target_generator import HorizontalMotion, HorizontalMotionSpec
        self.motion = HorizontalMotion(horizontal_motion, hover_seconds=hover_seconds) if horizontal_motion is not None else None
        self.cooperative = None  # Optional existing-owner target source, never a second MAVLink writer.
        if self.motion is not None and pilot_source != 'native_rc':
            raise ValueError('Horizontal demonstration uses the existing Pocket route')
        arrival = dict(tolerance=HorizontalMotion.vertical_tolerance_m,
            horizontal_tolerance=self.motion.spec.arrival_radius_m if self.motion else HorizontalMotionSpec.arrival_radius_m,
            settling_speed_mps=HorizontalMotion.settling_speed_mps) if pilot_source=='native_rc' else {}
        # Use the same diagnostic status budget as Observation.link_ok().
        # Pose/local/output freshness is independently checked before preview.
        self.preview = MissionPreview(height=height, hover_seconds=hover_seconds,
            status_max_age_s=NATIVE_COMMANDER_MAX_AGE_S if pilot_source=='native_rc' else .5, **arrival)
        self.phase = 'PREPARE'
        self.settlement = None
        self.frozen_selector = None
        self.binding = None
        self.origin = None
        self.last_t = self.started_at = self.ascend_at = None
        self.last_setpoint = self.first_setpoint = self.last_vision = None
        self.next_vision = None
        self.last_source_stamp = None
        self.setpoints = 0
        self.pending = None
        self.arm_attempted = self.offboard_attempted = False
        self.stop_latched = False
        self.home_attempts = 0
        self.next_home_attempt = 0.
        self.home_acknowledged = False
        self.landing_at = self.last_land_request = None
        self.land_attempts = 0
        self.land_acknowledged = False
        self.landing_timed_out_at = self.ground_confirmed_at = None
        self.descent_at = self.descent_start_z = None
        self.descent_target = None
        self.descent_endpoint_sent = False
        self.fault = None
        self.sent = []
        self.acks = []
        self.pilot_control = False
        self.takeover_reason = None

    def request_takeover(self, reason='pilot_request'):
        """Irreversibly yield this mission BEFORE a caller sends manual/Land mode.

        This records relinquished authority, not confirmed FC mode. Localization
        may continue for existing Vicon users; no autonomous motion/arm/Land may.
        """
        self.pilot_control = True
        self.takeover_reason = self.takeover_reason or reason
        self.pending = None
        if not self.stop_latched:
            self.phase = 'PILOT_CONTROL'

    def observe_takeover(self, o):
        if (self.phase in ('WAIT_ARM', 'ASCEND', 'HOVER', 'DESCEND', 'LAND', 'LAND_TIMEOUT')
                and o.link_ok() and o.armed and o.manual_control_active is True
                and not o.offboard):
            self.request_takeover('observed_manual_mode')
        return self.pilot_control

    def _clock(self, t):
        if not math.isfinite(t) or t < 0 or (self.last_t is not None and t < self.last_t):
            self.fault = self.fault or 'invalid_session_clock'
            if not self.pilot_control:
                self.phase = 'LAND' if self.arm_attempted else 'BLOCKED'
            raise ValueError('Finite monotonic session time required')
        self.last_t = t

    @staticmethod
    def _binding(o):
        if (not o.boot_session or not o.rigid_revision or len(o.reset_counters) != 3
                or any(type(x) is not int or not 0 <= x <= 255 for x in o.reset_counters)):
            raise ValueError('Current boot, rigid revision and three reset counters required')
        return o.boot_session, o.rigid_revision, o.reset_counters

    def prepare(self, o, selector):
        self._clock(o.t)
        if (self.phase != 'PREPARE' or not o.navigation_ok() or o.armed
                or not o.preflight_ready or o.pilot_source != self.pilot_source
                or not o.preparation_ground_ok()):
            raise ValueError('Fresh fused disarmed/landed/locked preflight and HOME proof required')
        if self.cooperative is not None and not self.cooperative.preparation_allowed():
            return False
        binding = self._binding(o)
        if self.binding is not None and binding != self.binding:
            self.fault = 'preparation_epoch_changed'
            self.phase = 'BLOCKED'
            raise ValueError(self.fault)
        self.binding = binding
        if self.settlement is None:
            self.settlement = GroundEstimatorSettlement(o.t)
        try:
            settled = self.settlement.observe(selector, o.t, targets_sent=0)
        except (ValueError, KeyError, TypeError):
            self.phase = 'BLOCKED'
            self.fault = 'estimator_initialization_not_healthy_and_bounded'
            raise
        if settled:
            self.frozen_selector = self.settlement.frozen_signature
            self.origin = o
            self.phase = 'WAIT_EXPLICIT_START'
            return True
        return False

    def start(self, o, selector, *, operator_confirmed=False):
        self._clock(o.t)
        if (self.phase != 'WAIT_EXPLICIT_START' or not operator_confirmed
                or self.stop_latched or self.pilot_control):
            raise ValueError('One explicit start after completed preparation required')
        require_frozen_selector(selector, self.frozen_selector)
        if (not o.navigation_ok() or not o.preflight_ready or o.armed or not o.landed
                or o.lockdown or o.pilot_source != self.pilot_source or not o.start_pilot_ok()
                or self._binding(o) != self.binding
                or math.dist(o.position_ned, self.origin.position_ned) > .03):
            raise ValueError('Fresh stationary unlocked state required; NEVER clear HOME/output lock automatically')
        if self.cooperative is not None:
            self.cooperative.start(o)
        self.origin = o
        self.started_at = o.t
        self.phase = 'WARMUP'

    def _emit(self, msg, t):
        if self.pilot_source == 'native_rc' and (msg.get_type() in ('RC_CHANNELS_OVERRIDE','MANUAL_CONTROL')
                or msg.get_type() == 'COMMAND_LONG' and msg.command == 185):
            raise PermissionError('Native RC owns motor stop and sticks; no PC override/output command')
        if self.pilot_control and not (msg.get_type() == 'VISION_POSITION_ESTIMATE'
                or msg.get_type() == 'COMMAND_LONG' and msg.command == 185
                and msg.param1 == 2 and self.stop_latched):
            raise PermissionError('Pilot owns control; autonomous command suppressed')
        # There is no hardware writer in this module. False/short/failed sink
        # acceptance is not recorded as sent and cannot advance warmup/commands.
        try:
            if self.sink(msg) is not True:
                raise OSError('Message sink did not confirm complete acceptance')
        except Exception:
            self.fault = self.fault or 'dispatch_error_uncertain_delivery'
            self.phase = ('STOP_LATCHED' if self.stop_latched else 'PILOT_CONTROL' if self.pilot_control
                          else 'LAND' if self.arm_attempted else 'BLOCKED')
            raise
        self.sent.append({'t': t, 'type': msg.get_type(), 'message': msg.to_dict()})

    def _command(self, command, t, *params, confirmation=0):
        params = tuple(params) + (0.,) * (7-len(params))
        return self.encoder.command_long_encode(1, 1, command, confirmation, *params)

    def ack(self, msg, t):
        if self.pilot_control and not self.stop_latched:
            return False  # A delayed OFFBOARD/ARM/LAND ACK cannot revive this mission.
        self._clock(t)
        if (msg.get_type() != 'COMMAND_ACK'
                or (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1)
                or (msg.target_system, msg.target_component) != (250, self.component)):
            return False
        if self.stop_latched and self.home_attempts and msg.command == 185:
            self.home_acknowledged = msg.result == m.MAV_RESULT_ACCEPTED
            self.acks.append({'t': t, 'command': 185, 'result': msg.result})
            return True
        if (self.phase == 'LAND' and self.land_attempts and msg.command == 21
                and self.last_land_request <= t <= self.last_land_request+self.command_timeout):
            self.land_acknowledged = msg.result == m.MAV_RESULT_ACCEPTED
            self.acks.append({'t': t, 'command': 21, 'result': msg.result})
            return True
        if not self.pending or msg.command != self.pending['command']:
            return False
        if not self.pending['sent_at'] <= t <= self.pending['sent_at']+self.command_timeout:
            return False
        self.acks.append({'t': t, 'command': msg.command, 'result': msg.result})
        if msg.result == m.MAV_RESULT_IN_PROGRESS:
            return True  # Neither acceptance nor a deadline extension.
        if msg.result == m.MAV_RESULT_ACCEPTED:
            self.pending['accepted'] = True
            if self.pending.get('ack_at') is None:
                self.pending['ack_at'] = t
        else:
            self.pending['rejected'] = True
        return True

    def home(self, o):
        # This path intentionally accepts ARMED observations. No dependency on
        # Vicon, EKF health, local position or mission parsing to latch a stop.
        self.stop_latched = True
        self.phase = 'STOP_LATCHED'
        self.pending = None
        if self.pilot_source == 'native_rc':return  # Physical Kill is never replaced by MAV_CMD 185.
        if (o.link_ok() and not self.home_acknowledged and self.home_attempts < 3
                and o.t >= self.next_home_attempt):
            self._emit(self._command(185, o.t, 2., confirmation=self.home_attempts), o.t)
            self.home_attempts += 1
            self.next_home_attempt = o.t+.8

    def _abort(self, reason, o):
        if self.observe_takeover(o):
            return
        if reason != 'hover_complete':
            self.fault = self.fault or reason
        self.pending = None
        if o.armed or self.arm_attempted:
            self.phase = 'LAND'
            if self.landing_at is None:
                self.landing_at = o.t
        else:
            self.phase = 'BLOCKED'

    def vision_message(self, o, stamp_us, position_frd, q_wxyz, alignment):
        """Prepare VPE using the existing compact encoder; no repair/replay.

        Continue fresh localization through LAND; never turn fusion off in
        flight. No parameter restore exists here. Caller must stop stale poses.
        """
        if o.home_pressed:
            self.home(o)
            return False
        self.observe_takeover(o)
        if self.stop_latched or self.phase in ('COMPLETE', 'BLOCKED', 'STOP_LATCHED'):
            return False
        if not o.link_ok() or not 0 <= o.pose_age_s <= .15:
            return False
        if self.binding is not None and self._binding(o) != self.binding:
            return False
        if (self.phase in ('WARMUP','WAIT_OFFBOARD','WAIT_ARM','ASCEND','HOVER','DESCEND')
                and self.last_vision is not None and o.t-self.last_vision > self.maximum_vision_gap):
            # A just-arrived sample cannot erase an already exceeded send
            # gap. Abort first; if airborne, still deliver fresh localization
            # for LAND rather than turning fusion off as a cleanup action.
            self._abort('fresh_vision_dispatch_gap',o)
            if self.phase == 'BLOCKED':
                return False
        if self.last_source_stamp is not None and stamp_us <= self.last_source_stamp:
            return False
        if self.next_vision is not None and o.t < self.next_vision:
            return False
        msg, _, _ = compact_vision.message(self.encoder, stamp_us, position_frd, q_wxyz, alignment)
        self._emit(msg, o.t)
        self.last_source_stamp, self.last_vision = stamp_us, o.t
        if self.next_vision is None:
            self.next_vision = o.t+self.vision_interval
        else:
            slots = max(1,math.floor((o.t-self.next_vision)/self.vision_interval)+1)
            self.next_vision += slots*self.vision_interval
        return True

    def cycle(self, o, selector, *, boot_ms, vision=None):
        """Single-owner call order: HOME, epoch guard, pose stream, mission.

        vision=(fresh source timestamp, position FRD, body quaternion WXYZ,
        current-session alignment yaw). It is not a target or a stored replay.
        """
        if o.home_pressed or self.stop_latched:
            return self.tick(o, selector, boot_ms=boot_ms)
        self.observe_takeover(o)
        self._clock(o.t)
        if self.phase not in ('PREPARE','WAIT_EXPLICIT_START','COMPLETE','BLOCKED','STOP_LATCHED','LAND_TIMEOUT','PILOT_CONTROL'):
            try:
                require_frozen_selector(selector, self.frozen_selector)
                if self._binding(o) != self.binding:
                    raise ValueError('Session epoch changed')
            except (RuntimeError, ValueError, KeyError, TypeError):
                self._abort('boot_calibration_reset_or_estimator_changed',o)
        if vision is not None:
            try:
                self.vision_message(o,*vision)
            except (ValueError, TypeError):
                self._abort('invalid_vision_encoding',o)
        return self.tick(o,selector,boot_ms=boot_ms)

    def _setpoint(self, o, boot_ms, *, ascending=False):
        if type(boot_ms) is not int or not 0 <= boot_ms < 2**32:
            raise ValueError('Current FC boot milliseconds required')
        target = list(self.origin.position_ned)
        if ascending:
            target[2] -= min(self.preview.height, max(0, o.t-self.ascend_at)*self.maximum_climb_speed)
        if self.motion is not None and self.phase == 'HOVER':
            target = list(self.motion.target_ned)
        if self.phase == 'DESCEND':
            target = list(self.descent_target)
        yaw = self.origin.yaw_ned
        velocity=(0.,0.,0.)
        mask=POSITION_YAW_MASK
        if self.cooperative is not None and self.phase in ('ASCEND','HOVER'):
            target,yaw = self.cooperative.target(o)
            feedforward=self.cooperative.velocity_feedforward(o)
            if feedforward is not None:
                velocity=feedforward
                mask=POSITION_VELOCITY_YAW_MASK
        msg = self.encoder.set_position_target_local_ned_encode(boot_ms, 1, 1,
            m.MAV_FRAME_LOCAL_NED, mask, *target, *velocity, 0, 0, 0,
            yaw, 0)
        self._emit(msg, o.t)
        if self.phase == 'DESCEND' and target[2] >= self.origin.position_ned[2]-1e-9:
            self.descent_endpoint_sent = True
        if self.first_setpoint is None:
            self.first_setpoint = o.t
        self.last_setpoint = o.t
        self.setpoints += 1

    def expected_motion_target(self):
        """The exact current reference for the existing single-writer boundary."""
        target = list(self.origin.position_ned)
        if self.phase == 'ASCEND':
            target[2] -= min(self.preview.height, max(0, self.last_t-self.ascend_at)*self.maximum_climb_speed)
        elif self.phase == 'DESCEND':
            target = list(self.descent_target)
        elif self.phase == 'HOVER' and self.motion is not None:
            target = list(self.motion.target_ned)
        return tuple(target)

    @property
    def normal_descent_budget_s(self):
        # Includes time for tracking lag/settling, not just reference travel.
        return self.preview.height/NORMAL_DESCENT_SPEED_MPS + 15.

    def _begin_descent(self, o):
        # Keep the final horizontal target and yaw; do not jump toward a
        # drifting observation. Z returns to the captured resting-body height,
        # not world zero, platform-surface height or an inferred contact point.
        self.descent_at = o.t
        self.descent_start_z = self.origin.position_ned[2]-self.preview.height
        self.descent_target = (*self.origin.position_ned[:2], self.descent_start_z)
        self.phase = 'DESCEND'

    def _descent_tick(self, o):
        limit = self.motion.maximum_excursion_m if self.motion else self.preview.max_horizontal_excursion
        if math.dist(o.position_ned[:2], self.origin.position_ned[:2]) > limit:
            self._abort('Horizontal excursion exceeds preview boundary', o)
            return
        ground_z = self.origin.position_ned[2]
        z = min(ground_z, self.descent_start_z+
                max(0., o.t-self.descent_at)*NORMAL_DESCENT_SPEED_MPS)
        self.descent_target = (*self.descent_target[:2], z)
        if (self.descent_endpoint_sent and abs(o.position_ned[2]-ground_z) <= LAND_HANDOFF_TOLERANCE_M
                and abs(o.velocity_ned[2]) <= LAND_HANDOFF_MAX_VERTICAL_SPEED_MPS):
            # Near-resting pose is only permission to request PX4 Land.
            # It is NEVER touchdown evidence or permission to stop motors.
            self._abort('hover_complete', o)
        elif o.t-self.descent_at > self.normal_descent_budget_s:
            self._abort('normal_descent_tracking_timeout', o)

    def _motion_tick(self, o):
        motion = self.motion
        if math.dist(o.position_ned[:2], self.origin.position_ned[:2]) > motion.maximum_excursion_m:
            return dict(state=State.ABORT_LAND.value, reason='Horizontal motion excursion exceeds planned area')
        if self.phase == 'ASCEND':
            if o.t-self.ascend_at > self.preview.climb_timeout:
                return dict(state=State.ABORT_LAND.value, reason='Climb timeout')
            center = (*self.origin.position_ned[:2], self.origin.position_ned[2]-self.preview.height)
            # Fixed hover and horizontal missions share the same arrival test.
            if (not o.landed and self.preview.arrived(self._preview_observation(o))
                    and o.t-self.ascend_at >= self.preview.height/self.maximum_climb_speed):
                motion.start(o.t, center_ned=center, yaw_ned=self.origin.yaw_ned)
                self.preview.state = State.HOVER
                return dict(state='HOVER')
            return dict(state='ASCEND')
        motion.step(o.t, position_ned=o.position_ned, velocity_ned=o.velocity_ned)
        return dict(state=State.LAND.value if motion.complete else 'HOVER')

    def _preview_observation(self, o):
        return PreviewObservation(o.t, tuple(ned_to_enu_vector(o.position_ned)),
            tuple(ned_to_enu_vector(o.velocity_ned)), -o.yaw_ned,
            pose_age_s=o.pose_age_s, status_age_s=max(o.status_age_s, o.local_age_s),
            position_valid=o.fusion_ready, velocity_valid=o.fusion_ready,
            health_ok=o.navigation_ok(), emergency_available=o.pilot_available(),
            mapping_valid=self._binding(o) == self.binding, armed=o.armed, landed=o.landed)

    def tick(self, o, selector, *, boot_ms):
        # HOME precedes navigation checks and even a bad/backwards clock.
        if o.home_pressed or self.stop_latched:
            self.home(o)
            return self.status()
        if self.observe_takeover(o):
            return self.status()
        self._clock(o.t)
        if self.phase == 'LAND_TIMEOUT':
            # Timeout ends autonomous landing attempts, not ground observation.
            # A later actual landing may close resources without clearing fault.
            if self._landing_ground_confirmed(o):
                self.ground_confirmed_at = o.t
                self.phase = 'COMPLETE'
            return self.status()
        if self.phase in ('PREPARE', 'WAIT_EXPLICIT_START', 'BLOCKED', 'COMPLETE', 'LAND_TIMEOUT'):
            return self.status()
        if self.phase == 'LAND':
            self._land(o)
            return self.status()
        try:
            require_frozen_selector(selector, self.frozen_selector)
            same_epoch = self._binding(o) == self.binding
        except (ValueError, RuntimeError):
            same_epoch = False
        if not same_epoch:
            self._abort('boot_calibration_reset_or_estimator_changed', o)
        elif self.cooperative is not None and self.cooperative.fault:
            self._abort(self.cooperative.fault, o)
        elif not o.navigation_ok() or o.lockdown:
            self._abort('fresh_navigation_input_or_output_enable_lost', o)
        elif self.phase in ('WARMUP','WAIT_OFFBOARD') and o.armed:
            self._abort('unexpected_arming_before_requested_arm', o)
        elif self.last_vision is None or o.t-self.last_vision > self.maximum_vision_gap:
            self._abort('fresh_vision_dispatch_lost', o)
        elif (self.last_setpoint is not None
              and o.t-self.last_setpoint > self.maximum_setpoint_gap):
            self._abort('setpoint_dispatch_gap', o)
        elif self.pending and self.pending.get('rejected'):
            self._abort('command_rejected', o)
        elif self.pending and o.t > confirmation_deadline(
                self.pending['sent_at'], self.pending.get('ack_at'), self.command_timeout):
            reason = ('command_state_confirmation_timeout' if self.pending['accepted']
                      else 'command_ack_timeout')
            self._abort(reason, o)
        if self.phase == 'LAND':
            self._land(o)
            return self.status()
        if self.phase == 'BLOCKED':
            return self.status()
        if self.phase in ('WARMUP', 'WAIT_OFFBOARD', 'WAIT_ARM'):
            if self.last_setpoint is None or o.t-self.last_setpoint >= self.setpoint_interval-1e-9:
                self._setpoint(o, boot_ms)
            if self.phase == 'WARMUP':
                if (self.setpoints >= 20 and o.t-self.first_setpoint >= self.warmup_seconds
                        and not self.offboard_attempted):
                    self.offboard_attempted = True
                    self.pending = {'command': 176, 'sent_at': o.t, 'accepted': False}
                    self.phase = 'WAIT_OFFBOARD'
                    self._emit(self._command(176, o.t, 1., 6.), o.t)
            elif self.phase == 'WAIT_OFFBOARD' and self.pending['accepted'] and o.offboard:
                if not o.preflight_ready or o.armed or not o.landed:
                    self._abort('prearm_state_changed', o)
                else:
                    self.arm_attempted = True
                    self.pending = {'command': 400, 'sent_at': o.t, 'accepted': False}
                    self.phase = 'WAIT_ARM'
                    self._emit(self._command(400, o.t, 1.), o.t)
            elif self.phase == 'WAIT_ARM' and self.pending['accepted'] and o.armed and o.offboard:
                # Existing preview requires a disarmed start. Freeze its origin
                # before feeding the newly confirmed ARMED observation.
                initial = replace(self._preview_observation(o), armed=False, landed=True,
                    position=tuple(ned_to_enu_vector(self.origin.position_ned)))
                if not self.preview.start(initial, confirmed=True):
                    self._abort('mission_start_refused', o)
                else:
                    self.pending = None
                    self.ascend_at = o.t
                    self.phase = 'ASCEND'
        else:
            if not o.armed or not o.offboard:
                self._abort('armed_or_offboard_state_lost', o)
            elif self.phase == 'DESCEND':
                self._descent_tick(o)
                if self.phase == 'DESCEND' and (self.last_setpoint is None or
                        o.t-self.last_setpoint >= self.setpoint_interval-1e-9):
                    self._setpoint(o, boot_ms)
            else:
                result = (self.cooperative.flight_tick(o) if self.cooperative is not None else
                          self._motion_tick(o) if self.motion is not None else self.preview.tick(self._preview_observation(o)))
                if result['state'] in (State.LAND.value, State.ABORT_LAND.value):
                    if (result['state'] == State.LAND.value and self.pilot_source == 'native_rc'
                            and self.cooperative is None):
                        self._begin_descent(o)
                    else:
                        self._abort('hover_complete' if result['state'] == State.LAND.value else result['reason'], o)
                else:
                    self.phase = result['state']
                    if self.last_setpoint is None or o.t-self.last_setpoint >= self.setpoint_interval-1e-9:
                        self._setpoint(o, boot_ms, ascending=True)
        if self.phase == 'LAND':
            self._land(o)
        return self.status()

    def _landing_ground_confirmed(self, o):
        return bool(o.link_ok() and o.landed is True and o.armed is False
                    and self.binding is not None and o.boot_session == self.binding[0])

    def _land(self, o):
        if self.pilot_control:
            return
        if self._landing_ground_confirmed(o):
            self.ground_confirmed_at = o.t
            self.phase = 'COMPLETE'
            return
        if self.landing_at is None:
            self.landing_at = o.t
        if o.t-self.landing_at > self.preview.land_timeout:
            self.phase = 'LAND_TIMEOUT'
            if self.landing_timed_out_at is None: self.landing_timed_out_at = o.t
            self.fault = self.fault or 'landing_not_confirmed'
            return  # Never force-disarm or claim landing from elapsed time.
        if (o.link_ok() and not self.land_acknowledged and self.land_attempts < 3
                and (self.last_land_request is None or o.t-self.last_land_request >= 1)):
            self._emit(self._command(21, o.t), o.t)
            self.last_land_request = o.t
            self.land_attempts += 1

    def status(self):
        transport = self.sink.status() if callable(getattr(self.sink,'status',None)) else None
        return {'phase': self.phase, 'fault': self.fault, 'stop_latched': self.stop_latched,
                'cooperative':self.cooperative.describe() if self.cooperative is not None else None,
                'horizontal_motion': self.motion.describe() if self.motion is not None else None,
                'normal_descent': (dict(reference_speed_mps=NORMAL_DESCENT_SPEED_MPS,
                    target_ned=self.descent_target, resting_z_ned=self.origin.position_ned[2],
                    handoff_tolerance_m=LAND_HANDOFF_TOLERANCE_M,
                    handoff_max_vertical_speed_mps=LAND_HANDOFF_MAX_VERTICAL_SPEED_MPS,
                    started_at=self.descent_at) if self.descent_at is not None else None),
                'command_confirmation': (dict(command=self.pending['command'],
                    waiting_for='state' if self.pending['accepted'] else 'ack',
                    ack_deadline=self.pending['sent_at']+self.command_timeout,
                    state_deadline=confirmation_deadline(self.pending['sent_at'],
                        self.pending.get('ack_at'),self.command_timeout)) if self.pending else None),
                'pilot_control': self.pilot_control, 'takeover_reason': self.takeover_reason,
                'landing_confirmation': (dict(attempts=self.land_attempts,
                    ack_accepted=self.land_acknowledged,
                    waiting_for=None if self.ground_confirmed_at is not None else 'fresh_landed_and_disarmed',
                    timed_out_at=self.landing_timed_out_at, ground_confirmed_at=self.ground_confirmed_at)
                    if self.landing_at is not None else None),
                'selector_frozen': self.frozen_selector, 'commands': len(self.sent),
                'mission_sequence_passed_in_offline_sink':self.phase == 'COMPLETE' and self.fault is None,
                'hardware_transport_present': bool(transport and transport['mode']=='guarded_port'),
                'actual_sent_to_fc': transport['physical_dispatch_messages'] if transport else 0,
                'transport':transport,'flight_authorized': False,
                'scope': 'Offline scheduler/real MAVLink encoding, not a hardware flight runner'}


def readonly_dispatch_allowed(message):
    """Default real-device boundary: NO mission/control/positioning writes."""
    from mavlink_readonly_probe import readonly_message
    return readonly_message(message)
