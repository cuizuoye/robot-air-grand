"""Separate battery-only LOCKED static scope; never a flight/send-mode policy.

One radio owns all telemetry, diagnostics and poses. No USB-route aliases and
no weakening of the old unpowered profiles. Link loss stops poses; restoration
is attempted but cannot be guaranteed over the same lost link.
"""
from dataclasses import dataclass,asdict,replace
from copy import deepcopy
import math

from pymavlink.dialects.v20 import common as m
import compact_vision as compact
import trs_vision_reference_policy as vision
from estop_only_policy import shell_message_allowed, stop_message_allowed
from mavlink_readonly_probe import readonly_message, decode_parameter

RATE_HZ = 32
STREAM_SECONDS = 20
MAX_PACKETS = 640
FUSION_SECONDS = 12
PARAMS = vision.READ_PARAMS
COMMANDS = vision.COMMANDS


@dataclass(frozen=True)
class Observation:
    now: float
    identity_fresh: bool = False
    usb_absent: bool = False
    boot_unchanged: bool = False
    disarmed_locked: bool = False
    lock_age: float = math.inf
    source_age: float = math.inf
    attitude_age: float = math.inf
    tilt_difference_deg: float = math.inf
    stationary: bool = False
    parameters_fresh_match: bool = False
    gamepad_same_device: bool = False
    home_pressed: bool = False

    def healthy(self):
        numbers=(self.now,self.lock_age,self.source_age,self.attitude_age,self.tilt_difference_deg)
        return bool(all(math.isfinite(v) and v>=0 for v in numbers)
            and self.identity_fresh and self.usb_absent and self.boot_unchanged
            and self.disarmed_locked and self.lock_age<=2
            and self.source_age<=.15 and self.attitude_age<=.5
            and self.tilt_difference_deg<=5 and self.stationary
            and self.parameters_fresh_match and self.gamepad_same_device and not self.home_pressed)

    def failure_reasons(self):
        reasons=[]
        for name in ('now','lock_age','source_age','attitude_age','tilt_difference_deg'):
            value=getattr(self,name)
            if not math.isfinite(value) or value<0:reasons.append(name+'_invalid')
        for name in ('identity_fresh','usb_absent','boot_unchanged','disarmed_locked',
                     'stationary','parameters_fresh_match','gamepad_same_device'):
            if not getattr(self,name):reasons.append(name)
        for name,limit in (('lock_age',2),('source_age',.15),('attitude_age',.5),('tilt_difference_deg',5)):
            if getattr(self,name)>limit:reasons.append(name+'_exceeded')
        if self.home_pressed:reasons.append('home_pressed')
        return reasons


class StreamGate:
    """One shot. Recovery never clears a latched failure or restarts poses."""
    def __init__(self):
        self.started = self.fusion_started = None
        self.last_now = None
        self.last_send = None
        self.sent = 0
        self.blocked = False
        self.first_failure = None

    def start(self, observation):
        if self.started is not None or self.blocked or not observation.healthy():
            raise ValueError('Fresh locked static state required before one stream')
        self.started = self.last_now = observation.now

    def check(self, observation):
        if self.blocked:return False
        valid=bool(self.started is not None and observation.healthy()
            and self.last_now<=observation.now
            and observation.now-self.started<STREAM_SECONDS and self.sent<MAX_PACKETS
            and (self.last_send is None or observation.now-self.last_send<=.15)
            and (self.fusion_started is None or observation.now-self.fusion_started<FUSION_SECONDS))
        self.last_now=observation.now
        if not valid and self.first_failure is None:
            reasons=observation.failure_reasons()
            if self.started is None:reasons.append('not_started')
            if self.last_send is not None and observation.now-self.last_send>.15:reasons.append('send_gap_exceeded')
            if self.started is not None and observation.now-self.started>=STREAM_SECONDS:reasons.append('stream_deadline')
            if self.sent>=MAX_PACKETS:reasons.append('packet_budget')
            if self.fusion_started is not None and observation.now-self.fusion_started>=FUSION_SECONDS:reasons.append('fusion_deadline')
            self.first_failure=dict(reasons=reasons or ['clock_or_gate_state_invalid'],
                observation={k:(None if type(v) is float and not math.isfinite(v) else v)
                    for k,v in asdict(observation).items()},last_send_at=self.last_send,sent_packets=self.sent)
        self.blocked=not valid
        return valid

    def begin_fusion(self, observation):
        if self.fusion_started is not None or not self.check(observation):
            raise ValueError('No repeated fusion start or unhealthy initialization')
        self.fusion_started=observation.now

    def actual_send(self, now):
        if (self.blocked or self.started is None or self.sent>=MAX_PACKETS
                or not math.isfinite(now) or now<self.last_now
                or (self.last_send is not None and not 0<now-self.last_send<=.15)):
            self.blocked=True
            raise ValueError('Stream stopped or packet budget exhausted')
        self.sent+=1;self.last_send=now

    def stop(self):self.blocked=True


class ShortStatusStreamGate(StreamGate):
    """Opt-in static candidate; no CLI or serial authorization is added.

    Reuse every existing static-stream predicate, replacing only the textual
    actuator snapshot with current short-status/clock evidence. The caller's
    clock domain is explicit: Observation.now is seconds since runner start,
    whereas now_ns is the absolute monotonic clock used by the decoder.
    HOME session proof, permits, physical scope and restoration remain outer
    requirements. This class is not a flight gate.
    """
    def __init__(self, lock_gate):
        super().__init__()
        self.lock_gate = lock_gate
        self.lock_detail = None

    def _current(self, observation, now_ns):
        if type(now_ns) is not int or now_ns <= 0:
            self.stop()
            raise ValueError('Explicit monotonic nanoseconds required')
        self.lock_detail = self.lock_gate.assess(
            now_ns, identity_verified=observation.identity_fresh)
        bounds = self.lock_detail['source_age_bounds_ms']
        valid = self.lock_detail['valid']
        return replace(observation, disarmed_locked=valid,
                       lock_age=bounds[1]/1000 if valid else math.inf)

    def start(self, observation, *, now_ns):
        current = self._current(observation, now_ns)
        # Do not consume the lock stream start while another prerequisite is
        # still missing. Both gates become one-shot together.
        if self.started is not None or self.blocked or not current.healthy():
            raise ValueError('Fresh short status and all static guards required')
        if not self.lock_gate.begin(now_ns, identity_verified=observation.identity_fresh):
            self.stop()
            raise ValueError('Short lock stream already consumed or invalid')
        super().start(current)

    def check(self, observation, *, now_ns):
        valid = super().check(self._current(observation, now_ns))
        if not valid:
            self.lock_gate.stop()
            if self.first_failure is not None:
                self.first_failure['short_lock'] = self.lock_detail
        return valid

    def begin_fusion(self, observation, *, now_ns):
        if self.fusion_started is not None or not self.check(observation, now_ns=now_ns):
            raise ValueError('No repeated fusion start or unhealthy initialization')
        self.fusion_started = observation.now

    def actual_send(self, observation, *, now_ns):
        # Recheck immediately at the send boundary, not only at a previous
        # loop tick. The caller must perform this before writing a pose.
        if not self.check(observation, now_ns=now_ns):
            raise ValueError('Static short-state send boundary closed')
        try:
            super().actual_send(observation.now)
        except ValueError:
            self.lock_gate.stop()
            raise

    def stop(self):
        super().stop()
        self.lock_gate.stop()


class FixedGroundPreparation:
    """Ground-only plan B state machine. No sender, permit, or flight handoff.

    Callers must still obtain real HOME/clock/lock/pose and EKF observations.
    Offline tests can exercise these predicates but cannot issue a physical
    grant. Every terminal state is irreversible; reboot requires a NEW object.
    """
    def __init__(self, profile_sha256):
        self.profile_sha256 = profile_sha256
        self.phase = 'INITIAL'
        self.binding = None
        self.history = []
        self.failure = None
        self.configuration = None
        self.ready_observed = False
        self.last_now = None

    def _move(self, phase, now, reason=None):
        if type(now) not in (int,float) or not math.isfinite(now) or now < 0:
            raise ValueError('Actual finite monotonic observation time required')
        if self.last_now is not None and now < self.last_now:
            raise ValueError('Ground preparation time regression')
        self.last_now = now
        self.phase = phase
        self.history.append(dict(phase=phase, at=now, reason=reason))

    def fail(self, reason, now):
        if self.phase in ('FAILED', 'AUTH_REVOKED'):
            return False
        self.failure = self.failure or reason
        was_authorized = self.phase in ('POSITION_AUTHORIZED','EKF_FUSION_VERIFIED','READY')
        phase='AUTH_REVOKED' if was_authorized else 'FAILED'
        if (type(now) not in (int,float) or not math.isfinite(now) or now < 0
                or self.last_now is not None and now < self.last_now):
            # Reject the observation, do NOT mint a replacement timestamp.
            self.phase=phase
            self.history.append(dict(phase=phase,at=None,reason=self.failure,
                rejected_time=now if type(now) in (int,float) and math.isfinite(now) else None))
        else:
            self._move(phase, now, self.failure)
        return False

    def connect(self, identity, boot_session, now):
        if self.phase != 'INITIAL' or not boot_session or not identity:
            return self.fail('new_identity_and_boot_session_required', now)
        self.binding = (identity.get('uid2'), identity.get('firmware'), boot_session)
        self._move('CONNECTED', now)
        return True

    def verify_configuration(self, result, now):
        if self.phase != 'CONNECTED':
            return self.fail('configuration_order', now)
        self._move('CONFIG_VERIFYING', now)
        self.configuration = deepcopy(result)
        if (result.get('profile_sha256') != self.profile_sha256
                or result.get('current_configuration_verified') is not True
                or not self._bound(result.get('identity_binding'),result.get('boot_session'))):
            return self.fail('fixed_configuration_not_verified', now)
        self._move('CONFIG_VERIFIED', now)
        return True

    def _bound(self, identity, boot_session):
        return bool(identity and (identity.get('uid2'),identity.get('firmware'),boot_session) == self.binding)

    @staticmethod
    def _clock_lock_ok(detail):
        # StaticLockGate.assess uses the UNCHANGED 500ms clock/arrival bounds.
        return isinstance(detail,dict) and detail.get('valid') is True

    def authorize_positions(self, observation, *, home, clock_lock, identity, boot_session):
        now = observation.now
        if self.phase != 'CONFIG_VERIFIED' or not self._bound(identity,boot_session):
            return self.fail('identity_boot_or_configuration_changed', now)
        if not home.verified or home.aborted or not observation.disarmed_locked or not self._clock_lock_ok(clock_lock):
            return self.fail('current_home_disarmed_lock_or_clock_missing', now)
        self._move('HOME_VERIFIED', now)
        if not observation.healthy():
            return self.fail('vicon_or_other_ground_guard_failed', now)
        self._move('VICON_VERIFIED', now)
        self._move('POSITION_AUTHORIZED', now)
        return True

    def check(self, observation, *, clock_lock, identity, boot_session):
        if self.phase not in ('POSITION_AUTHORIZED','EKF_FUSION_VERIFIED','READY'):
            return False
        if not self._bound(identity,boot_session):
            return self.fail('identity_or_boot_changed', observation.now)
        if not observation.healthy() or not self._clock_lock_ok(clock_lock):
            return self.fail('current_ground_evidence_lost', observation.now)
        if self.last_now is not None and observation.now < self.last_now:
            return self.fail('clock_regressed', observation.now)
        self.last_now = observation.now
        return True

    def verify_fusion(self, rows, primary, *, reception_verified, observation,
                      clock_lock, identity, boot_session):
        import usb_combined_fusion_policy as actual_fusion
        if not self.check(observation,clock_lock=clock_lock,identity=identity,boot_session=boot_session):
            return False
        # Same existing real three-audit predicate; parameter equality/ACKs
        # or a caller's ready=True cannot substitute for the estimator rows.
        try:
            fused=bool(len(rows)==3 and actual_fusion.fusion_evidence_ok(rows,primary))
        except (KeyError,TypeError,ValueError):
            fused=False
        if not reception_verified or not fused:
            return self.fail('actual_current_xyz_yaw_fusion_missing', observation.now)
        self._move('EKF_FUSION_VERIFIED', observation.now)
        self._move('READY', observation.now)
        self.ready_observed = True
        return True

    def close(self, now, reason='ground_session_closed'):
        if self.phase not in ('FAILED','AUTH_REVOKED'):
            self._move('AUTH_REVOKED', now, reason)

    def status(self):
        return dict(phase=self.phase, failure=self.failure, history=deepcopy(self.history),
            ready_observed=self.ready_observed, position_authorized=self.phase in (
                'POSITION_AUTHORIZED','EKF_FUSION_VERIFIED','READY'),
            physical_authorization=False, control_ready=False, flight_authorized=False,
            note='Ground preparation observation only; no flight grant or mission handoff')


def allowed(msg, *, identity_fresh=False, authorized=False, locked_static=False,
            receiving_verified=False, restore_safe=False, home_latched=False, fixed_config=False):
    kind=msg.get_type()
    if fixed_config:
        # Applies during faults/finally too; neither restore_safe nor an old
        # permit can re-open a persistent write in the fixed profile.
        if kind in ('PARAM_SET','PARAM_EXT_SET'):
            return False
        if kind == 'PARAM_REQUEST_READ':
            from ekf_static_profile import READ_PARAMS
            name=msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id,bytes) else msg.param_id.rstrip('\0')
            return bool(identity_fresh and name in READ_PARAMS
                and (msg.target_system,msg.target_component,msg.param_index)==(1,1,-1))
    if kind=='PARAM_SET':
        name=msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id,bytes) else msg.param_id
        if (not authorized or (msg.target_system,msg.target_component)!=(1,1)
                or name not in ('EKF2_EV_CTRL','EKF2_EV_DELAY')):return False
        expected_type=m.MAV_PARAM_TYPE_INT32 if name=='EKF2_EV_CTRL' else m.MAV_PARAM_TYPE_REAL32
        if msg.param_type!=expected_type:return False
        value=decode_parameter(msg)
        baseline=2 if name=='EKF2_EV_CTRL' else 0
        candidate=11 if name=='EKF2_EV_CTRL' else 65
        return bool((value==baseline and restore_safe)
            or (value==candidate and identity_fresh and locked_static and receiving_verified))
    if kind=='VISION_POSITION_ESTIMATE':
        return bool(authorized and identity_fresh and locked_static and compact.pose_allowed(msg))
    if kind=='PARAM_REQUEST_READ':
        name=msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id,bytes) else msg.param_id
        return bool(identity_fresh and name in PARAMS
                    and (msg.target_system,msg.target_component,msg.param_index)==(1,1,-1))
    if kind=='SERIAL_CONTROL':
        payloads=[b'',*(p for name,p in COMMANDS.items() if name!='ground_visual_series')]
        return bool(identity_fresh and msg.flags==2 and (msg.device,msg.timeout,msg.baudrate)==(10,0,0)
            and any(msg.count==len(p) and bytes(msg.data)==p.ljust(70,b'\0') for p in payloads))
    if stop_message_allowed(msg,home_latched,authorized):return bool(identity_fresh)
    return readonly_message(msg) if kind in ('HEARTBEAT','COMMAND_LONG','PING') else False
