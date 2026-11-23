"""Ground-only frozen FC target load test. No mode, arm or flight permission."""
import json
import math
from pathlib import Path

from disarmed_offboard_policy import DisarmedHoldGate, restorable_mode
from rigid_body_revision import digest
from trs_uart_readonly_check import UID2

RATE_HZ = 16
DURATION_S = 3.
STATIC_LOCK_SOURCE_MAX_S = 2.


class _StaticNoModeHold(DisarmedHoldGate):
    """Ground load only, matching StaticLockGate's actual source-time limit.

    Commander publishes actuator_armed every 500ms or on change. The older
    mode-entry gate's 500ms SOURCE limit is not a feasible continuous static
    telemetry condition once transport time is included. Do not rewrite the
    timestamp or widen that older gate. This private adapter never enters a
    mode; the caller must still prove <500ms actual ARRIVAL/clock freshness
    using StaticLockGate, which also checks the newest unsafe bits.
    """
    def valid(self, o):
        if self.blocked:return False
        numbers=(o.t,*o.position,o.yaw,o.pose_age_s,o.status_age_s)
        valid=bool(self.operator_authorized and all(math.isfinite(v) for v in numbers)
            and len(o.position)==3 and len(o.reset_counters)==3
            and o.uid2==UID2 and o.firmware=='1.14.3' and not o.armed and o.lockdown
            and o.estimator_ready and o.gamepad_available and not o.home_latched
            and 0<=o.pose_age_s<=.15 and 0<=o.status_age_s<STATIC_LOCK_SOURCE_MAX_S
            and o.t>=self.last_time and o.reset_counters==self.origin.reset_counters
            and o.primary_estimator==self.origin.primary_estimator
            and math.dist(o.position,self.origin.position)<=.03)
        self.last_time=o.t;self.blocked=not valid
        return valid

    def mode_message(self, *args, **kwargs):
        raise ValueError('Static frozen-target adapter never permits a mode request')


def prior_paths(report):
    report = Path(report).resolve()
    return dict(wireless_static_report=report,
        wireless_static_permit=report.parent/'permit.json',
        wireless_static_consumed=report.parent/'permit.json.consumed.json',
        wireless_static_recovery=report.parent/'recovery.json',
        wireless_static_rates=report.parent/'rates_recovery.json')


def require_prior(report, heading, board):
    """Reuse actual same-calibration wireless success, not a summary boolean."""
    paths = prior_paths(report)
    r, p, c, recovery, rates = [json.loads(path.read_text()) for path in paths.values()]
    arm = r.get('actuator_after') or {}
    if not (r.get('compact_ground_fusion_verified') is True
            and r.get('low_bandwidth_vision') is True and r.get('binary_diagnostics') is True
            and r.get('fc_usb_present_at_end') is False
            and r.get('identity', {}).get('uid2') == UID2
            and r.get('error') is None and r.get('restore_errors') == []
            and len(r.get('fusion_audits', [])) == 3
            and r.get('parameter_restoration_verified') is True
            and r.get('all_rate_transactions', {}).get('restoration_verified') is True
            and arm.get('armed') is False and arm.get('lockdown') is True
            and c.get('permit_sha256') == digest(paths['wireless_static_permit'])
            and c.get('scope') == p.get('scope')
            and p.get('hashes', {}).get('heading') == digest(heading)
            and p.get('hashes', {}).get('board') == digest(board)
            and all(x.get('verified_closed') is True and x.get('needs_restore') is False
                    for x in (recovery, rates))
            and r.get('flight_authorized') is False):
        raise ValueError('Completed same-calibration low-bandwidth wireless static proof required')
    return paths


def clock_bounds_us(clock, now_ns):
    ref = clock.reference
    if clock.fault or ref is None:
        raise RuntimeError('Frozen target requires current bounded FC clock')
    elapsed = now_ns-ref['received_ns']
    if not 0 <= elapsed < clock.MAX_AGE_NS:
        raise RuntimeError('Frozen target FC clock expired')
    margin = math.ceil((elapsed+ref['rtt_ns'])*clock.DRIFT_PPM/1_000_000)
    return ((now_ns+ref['lower_ns']-margin)//1000,
            (now_ns+ref['upper_ns']+margin+999)//1000)


def source_age(clock, boot_ms, received_ns, now_ns, *, maximum, label='FC sample'):
    lower, upper = clock_bounds_us(clock, now_ns)
    if type(boot_ms) is not int or not 0 <= boot_ms < 2**32:
        raise RuntimeError('Frozen target FC source timestamp invalid')
    age = (upper-boot_ms*1000)/1e6
    # Account for millisecond timestamp truncation; receipt alone is not youth.
    if not (0 <= now_ns-received_ns <= maximum*1e9
            and boot_ms*1000+1000 <= lower and 0 <= age <= maximum):
        raise RuntimeError(f'Frozen target {label} source stale/uncertain: upper_age={age:.6f}s, '
            f'arrival_age={(now_ns-received_ns)/1e9:.6f}s, lower_age={(lower-boot_ms*1000-1000)/1e6:.6f}s')
    return age


class FrozenTargetStream:
    """Only exact frozen XYZ/yaw messages; never exposes the mode entry API."""
    def __init__(self, observation, custom_mode, *, authorized=False):
        if (type(custom_mode) is not int or not 0 <= custom_mode < 2**32
                or not restorable_mode((custom_mode>>16)&255,(custom_mode>>24)&255)):
            raise ValueError('Frozen target requires non-Offboard stable ground mode')
        self.hold = _StaticNoModeHold(observation, operator_authorized=authorized)
        self.custom_mode = custom_mode
        self.rows = []
        self.closed = False

    def check(self, observation, custom_mode):
        if (self.closed or custom_mode != self.custom_mode or not self.hold.valid(observation)
                or self.hold.last_sent is not None and observation.t-self.hold.last_sent > .2):
            self.hold.blocked = True
            raise RuntimeError('Frozen target state/mode changed; no restart')

    def message(self, encoder, boot_ms, observation, custom_mode):
        self.check(observation, custom_mode)
        if observation.t-self.hold.origin.t > DURATION_S+.1 or len(self.rows) >= 50:
            self.hold.blocked = True
            raise RuntimeError('Frozen target duration/packet budget exhausted')
        return self.hold.hold_message(encoder, boot_ms, observation)

    def permits(self, message, observation, custom_mode):
        self.check(observation, custom_mode)
        return bool(message.get_type() == 'SET_POSITION_TARGET_LOCAL_NED'
            and observation.t-self.hold.origin.t <= DURATION_S+.1 and len(self.rows) < 50
            and all(getattr(message, key) == 0 for key in
                    ('vx','vy','vz','afx','afy','afz','yaw_rate'))
            and type(message.time_boot_ms) is int and 0 <= message.time_boot_ms < 2**32
            and self.hold.permits(message, observation))

    def record_send(self, now, message, wire_bytes):
        if self.closed:
            raise RuntimeError('Frozen target stream already stopped')
        self.hold.record_actual_hold_send(now)
        self.rows.append(dict(at=now, boot_ms=message.time_boot_ms,
            position=[message.x,message.y,message.z], yaw=message.yaw,
            wire_bytes=wire_bytes))

    def finish(self):
        self.closed = True
        if (self.hold.blocked or len(self.rows) < 32
                or self.rows[-1]['at']-self.rows[0]['at'] < 2.):
            raise RuntimeError('Continuous actual frozen target sends not established')

    def summary(self):
        times = [r['at'] for r in self.rows]
        return dict(rate_hz=RATE_HZ, requested_duration_s=DURATION_S,
            lock_source_age_limit_s=STATIC_LOCK_SOURCE_MAX_S,
            lock_arrival_and_clock_limit_s=.5,older_mode_entry_gate_unchanged=True,
            actual_send_count=len(times), actual_span_s=times[-1]-times[0] if times else 0,
            maximum_send_gap_s=max((b-a for a,b in zip(times,times[1:])), default=None),
            frozen_position=list(self.hold.origin.position), frozen_yaw=self.hold.origin.yaw,
            custom_mode=self.custom_mode, rows=self.rows, closed=self.closed,
            mode_commands_sent=False, moving_targets_sent=False, flight_authorized=False,
            scope='Actual host writes only; not proof of target receipt, Offboard mode or flight')
