"""One-shot continuous fusion lifecycle for a future single-owner runner.

NO device access or live entry point. Even an injected sink is denied without
an explicit per-message authorizer. Existing bench permits do not authorize
this service. Reuses pose encoding, receive-time/rate validation and parameter
encoding; never unlocks, arms, changes mode or emits motion targets.
"""
from dataclasses import dataclass
from copy import deepcopy
import math

from pymavlink.dialects.v20 import common as m

import compact_vision as compact
import trs_vision_reference_policy as vision
from mavlink_readonly_probe import decode_parameter
from trs_uart_readonly_check import UID2
from usb_combined_fusion_policy import compare_heading
from rc_watch import NATIVE_AUX_STATUS_MAX_AGE_S, NATIVE_DISPATCH_MAX_GAP_S


def context_from_inputs(inputs, now, *, ground_status_max_age=.5):
    """Map current receive-side observations, never externally supplied ready=True."""
    from mission_live_inputs import age
    from rc_watch import state_feedback_age
    if type(ground_status_max_age) not in (int,float) or ground_status_max_age not in (.5,1.):
        raise ValueError('Explicit strict or USB-ground-only status profile required')
    inputs._clock(now)
    identity = inputs.identity or ('','')
    arm = inputs.actuator
    attitude = inputs.attitude
    native = inputs.pilot_status(now)
    arm_age = state_feedback_age(now,arm['at'],
        (inputs._boot_us(now)-arm['timestamp_us'])/1e6,native=native is not None) if arm and attitude else math.inf
    return Context(now,inputs.boot_session,inputs.rigid_revision,
        int(inputs._boot_us(now)/1000) if attitude else -1,
        uid2=identity[0],firmware=identity[1],heartbeat_age=age(now,inputs.hb['at']) if inputs.hb else math.inf,
        actuator_age=arm_age,source_age=age(now,inputs.pose_at),
        attitude_age=age(now,attitude['at']) if attitude else math.inf,
        armed=arm['armed'] if arm else None,lockdown=arm['lockdown'] if arm else None,
        landed=inputs.land['landed'] if inputs.land and age(now,inputs.land['at']) <= ground_status_max_age else None,
        gamepad_current=inputs.pad_present and age(now,inputs.pad_event_at) <= 30,
        home_pressed=inputs.home_down,epoch_valid=inputs.fault is None,
        ground_status_max_age=ground_status_max_age,
        pilot_source='native_rc' if native is not None else 'gamepad',
        native_rc_current=bool(native and native['native_input_observed']),
        native_stop_clear=bool(native and native['kill_requested'] is False and native['stop_active'] is False))


@dataclass(frozen=True)
class Context:
    now: float
    boot_session: str
    rigid_revision: str
    boot_ms: int
    uid2: str = ''
    firmware: str = ''
    heartbeat_age: float = math.inf
    actuator_age: float = math.inf
    source_age: float = math.inf
    attitude_age: float = math.inf
    armed: object = None
    landed: object = None
    lockdown: object = None
    gamepad_current: bool = False
    home_pressed: bool = False
    epoch_valid: bool = False
    ground_status_max_age: float = .5
    pilot_source: str = 'gamepad'
    native_rc_current: bool = False
    native_stop_clear: bool = False

    def pilot_current(self):
        return (self.native_rc_current and self.native_stop_clear
                if self.pilot_source == 'native_rc' else self.gamepad_current and not self.home_pressed)

    def preparation_ground(self):
        return (self.link_source_ok() and not self.armed and self.landed and not self.lockdown
                if self.pilot_source == 'native_rc' else self.locked_ground())

    def actuator_max_age(self):
        return NATIVE_AUX_STATUS_MAX_AGE_S if self.pilot_source=='native_rc' else self.ground_status_max_age

    def link_source_ok(self):
        return bool(self.uid2 == UID2 and self.firmware == '1.14.3'
            and self.boot_session and self.rigid_revision
            and type(self.boot_ms) is int and self.boot_ms >= 0
            and all(type(v) is bool for v in (self.armed,self.landed,self.lockdown,
                                               self.gamepad_current,self.home_pressed))
            and all(math.isfinite(v) and v >= 0 for v in
                (self.now,self.heartbeat_age,self.actuator_age,self.source_age,self.attitude_age))
            and type(self.ground_status_max_age) in (int,float) and self.ground_status_max_age in (.5,1.)
            and self.heartbeat_age < 2 and self.actuator_age <= self.actuator_max_age()
            and self.source_age <= .15 and self.attitude_age <= .5
            and self.epoch_valid is True and self.pilot_current())

    def locked_ground(self):
        return self.link_source_ok() and not self.armed and self.landed and self.lockdown

    def restore_ground(self):
        # Loss of Vicon/input must not prohibit SAFE ground-only restoration.
        # Unknown link/actuator/landed state or armed state still prohibits it.
        return bool(self.uid2 == UID2 and self.firmware == '1.14.3' and self.epoch_valid is True
            and all(type(v) is bool for v in (self.armed,self.landed,self.lockdown))
            and not self.armed and self.landed and (self.lockdown or self.pilot_source == 'native_rc')
            and math.isfinite(self.heartbeat_age) and 0 <= self.heartbeat_age < 2
            and type(self.ground_status_max_age) in (int,float) and self.ground_status_max_age in (.5,1.)
            and math.isfinite(self.actuator_age) and 0 <= self.actuator_age <= self.actuator_max_age())


class ContinuousFusionService:
    rate_hz = 32
    maximum_send_gap = .15
    session_limit = 90.
    echo_timeout = 3.

    def __init__(self, sink, *, authorizer=None):
        if not callable(sink):
            raise ValueError('Explicit sink required; no default device writer')
        self.sink,self.authorizer = sink,authorizer
        self.encoder = m.MAVLink(None,srcSystem=250,srcComponent=220)
        self.phase = 'NEW'
        self.binding = None
        self.start_at = self.last_now = self.last_send = self.last_stamp = None
        self.start_boot_ms = None
        self.next_send = None
        self.alignment = None
        self.baseline = None
        self.pending = None
        self.receive_verified = False
        self.receipt_checks = None
        self.sent = []
        self.changed = set()
        self.fault = None
        self.land_required = False
        self.transferred_core = None

    def _clock(self, now):
        if not math.isfinite(now) or now < 0 or self.last_now is not None and now < self.last_now:
            self.fault = self.fault or 'fusion_clock_invalid'
            self.phase = 'BLOCKED'
            raise ValueError(self.fault)
        self.last_now = now

    def _bound(self, c):
        return (c.boot_session,c.rigid_revision) == self.binding

    def _dispatch(self, msg, c):
        # This is NOT a substitute for a reviewed physical authorization.
        # No authorizer is installed by the existing monitor or this module.
        if self.authorizer is None or self.authorizer(self,msg,c) is not True:
            raise PermissionError('Continuous fusion has no separate dispatch authorization')
        if msg.get_type() == 'PARAM_SET':
            restoring = self.phase in ('WAIT_RESTORE_CTRL','WAIT_RESTORE_DELAY')
            safe = c.restore_ground() if restoring else c.preparation_ground()
            if not safe or not self._bound(c) or not self.pending:
                raise PermissionError('Never configure/restore fusion in flight or unknown state')
            expected = vision.parameter_message(self.encoder,self.pending['name'],self.pending['value'])
            if msg.to_dict() != expected.to_dict():
                raise PermissionError('Parameter outside exact pending fusion transaction')
        if msg.get_type() not in ('PARAM_SET','VISION_POSITION_ESTIMATE'):
            raise PermissionError('No control/arming/mode/targets in fusion service')
        if msg.get_type() == 'VISION_POSITION_ESTIMATE' and not compact.pose_allowed(msg):
            raise ValueError('Invalid compact pose')
        try:
            if self.sink(msg) is not True:
                raise OSError('Full write acceptance not confirmed')
        except Exception:
            self.fault = self.fault or 'fusion_dispatch_uncertain'
            self.phase = 'BLOCKED'
            raise

    def begin(self, c, *, parameters, parameter_times, source_q, fc_q):
        self._clock(c.now)
        if self.phase != 'NEW' or not c.preparation_ground():
            raise ValueError('One fresh locked-ground initialization required')
        if (set(vision.READ_PARAMS)-set(parameters)
                or parameters.get('EKF2_EV_CTRL') != 2 or parameters.get('EKF2_EV_DELAY') != 0
                or not vision.scope_ok(parameters,parameters)
                or any(not 0 <= c.now-parameter_times.get(k,-math.inf) <= 4 for k in vision.READ_PARAMS)):
            raise ValueError('Complete current maintenance baseline required, not archived backup readiness')
        alignment = compact.yaw_alignment(source_q,fc_q)
        self.binding = c.boot_session,c.rigid_revision
        self.baseline = deepcopy(parameters)
        self.alignment = alignment
        self.start_at = c.now
        self.start_boot_ms = c.boot_ms
        self.phase = 'RECEIVE_WARMUP'

    def pose(self, c, stamp_us, position, quaternion):
        self._clock(c.now)
        if self.phase not in ('RECEIVE_WARMUP','CONFIGURE_DELAY','WAIT_DELAY_ECHO',
                              'CONFIGURE_CTRL','WAIT_CTRL_ECHO','FUSING','READY'):
            return False
        if not self._bound(c) or not c.preparation_ground():
            self.fault = self.fault or 'fusion_preparation_context_lost'
            self.land_required |= c.armed is True
            self.phase = 'BLOCKED'
            return False
        if c.now-self.start_at > self.session_limit:
            self.fault = self.fault or 'fusion_preparation_deadline'
            self.phase = 'BLOCKED'
            return False
        if self.last_send is not None and c.now-self.last_send > self.maximum_send_gap:
            self.fault = self.fault or 'fusion_stream_gap'
            self.phase = 'BLOCKED'
            return False
        if (type(stamp_us) is not int or stamp_us <= 0
                or self.last_stamp is not None and stamp_us <= self.last_stamp):
            return False
        if self.next_send is not None and c.now < self.next_send:
            return False
        msg,p,q = compact.message(self.encoder,stamp_us,position,quaternion,self.alignment)
        self._dispatch(msg,c)
        self.sent.append(dict(elapsed_s=c.now,source_stamp_ns=stamp_us*1000,position=p,q=q))
        self.sent = self.sent[-128:]
        self.last_send,self.last_stamp = c.now,stamp_us
        period=1/self.rate_hz
        if self.next_send is None:self.next_send=c.now+period
        else:
            # Advance the original cadence, not arrival+period. Otherwise
            # 330Hz Vicon quantizes every 31.25ms wait to ~33.3ms (~30Hz),
            # and 100Hz Vicon to 40ms (25Hz). Never replay missed slots.
            slots=max(1,math.floor((c.now-self.next_send)/period)+1)
            self.next_send+=slots*period
        return True

    def reception_checks(self,c,rows,*,requested_at):
        timing=compact.series_timing(rows)
        request_boot_us=self.start_boot_ms*1000+(requested_at-self.start_at)*1e6
        latest_age_s=c.boot_ms/1000-rows[-1]['timestamp']/1e6
        checks=dict(sample_count=len(rows)>=12,rate_at_least_30hz=timing['rate_hz']>=30,
            gaps_at_most_50ms=timing['maximum_publication_gap_ms']<=50,
            first_sample_current=rows[0]['timestamp']>=request_boot_us-50_000,
            receive_time_mode=all(vision.receive_time_mode(row) for row in rows),
            matches_sent_pose=bool(compact.match(rows[-1],self.sent)),
            latest_sample_current=0<=latest_age_s<=.5)
        self.receipt_checks=dict(at=c.now,requested_at=requested_at,timing=timing,
            latest_age_s=latest_age_s,first_timestamp_us=rows[0]['timestamp'],
            latest_timestamp_us=rows[-1]['timestamp'],request_boot_us=request_boot_us,
            checks=checks,failed=[key for key,ok in checks.items() if not ok])
        return self.receipt_checks

    def confirm_reception(self, c, rows, *, requested_at):
        self._clock(c.now)
        if (self.phase != 'RECEIVE_WARMUP' or not c.preparation_ground() or not self._bound(c)
                or not self.start_at <= requested_at <= c.now or c.now-requested_at > 3
                or self.last_send is None or c.now-self.last_send > self.maximum_send_gap):
            raise ValueError('Current same-session receive transaction required')
        result=self.reception_checks(c,rows,requested_at=requested_at)
        if result['failed']:
            raise ValueError('Fresh matching real FC receipt at original >=30Hz/<=50ms gate required')
        self.receive_verified = True
        self.phase = 'CONFIGURE_DELAY'
        return result['timing']

    def _parameter(self, c, name, value, phase):
        msg = vision.parameter_message(self.encoder,name,value)
        # Mark possible change BEFORE handing bytes to a potentially failing writer.
        self.changed.add(name)
        self.pending = dict(name=name,value=value,at=c.now)
        self.phase = phase
        self._dispatch(msg,c)

    def tick(self, c):
        self._clock(c.now)
        if self.phase in ('NEW','BLOCKED','CLOSED','HANDED_OFF'):
            return
        restoring = self.phase.startswith('RESTORE_') or self.phase.startswith('WAIT_RESTORE_')
        if not self._bound(c) or not (c.restore_ground() if restoring else c.preparation_ground()):
            self.fault = self.fault or 'fusion_preparation_context_lost'
            self.land_required |= c.armed is True
            self.phase = 'BLOCKED'
            return
        if not restoring and (c.now-self.start_at > self.session_limit
                or self.last_send is not None and c.now-self.last_send > self.maximum_send_gap):
            self.fault = self.fault or 'fusion_preparation_deadline_or_stream_gap'
            self.phase = 'BLOCKED'
            return
        if self.pending and c.now-self.pending['at'] > self.echo_timeout:
            self.fault = self.fault or 'fusion_parameter_echo_timeout'
            self.phase = 'BLOCKED'
            return
        if self.phase == 'CONFIGURE_DELAY':
            self._parameter(c,'EKF2_EV_DELAY',vision.DELAY_MS,'WAIT_DELAY_ECHO')
        elif self.phase == 'CONFIGURE_CTRL':
            self._parameter(c,'EKF2_EV_CTRL',11,'WAIT_CTRL_ECHO')
        elif self.phase == 'RESTORE_CTRL':
            self._parameter(c,'EKF2_EV_CTRL',self.baseline['EKF2_EV_CTRL'],'WAIT_RESTORE_CTRL')
        elif self.phase == 'RESTORE_DELAY':
            self._parameter(c,'EKF2_EV_DELAY',self.baseline['EKF2_EV_DELAY'],'WAIT_RESTORE_DELAY')

    def ingest(self, msg, c):
        self._clock(c.now)
        waiting = {'WAIT_DELAY_ECHO':'CONFIGURE_CTRL','WAIT_CTRL_ECHO':'FUSING',
                   'WAIT_RESTORE_CTRL':'RESTORE_DELAY','WAIT_RESTORE_DELAY':'CLOSED'}
        restoring = self.phase in ('WAIT_RESTORE_CTRL','WAIT_RESTORE_DELAY')
        if (self.phase not in waiting or not self.pending or not self._bound(c)
                or not (c.restore_ground() if restoring else c.preparation_ground())
                or msg.get_type() != 'PARAM_VALUE'
                or (msg.get_srcSystem(),msg.get_srcComponent()) != (1,1)
                or not 0 <= c.now-self.pending['at'] <= self.echo_timeout):
            return False
        name = msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id,bytes) else msg.param_id.rstrip('\0')
        if name != self.pending['name']:
            return False
        expected_type = m.MAV_PARAM_TYPE_INT32 if name == 'EKF2_EV_CTRL' else m.MAV_PARAM_TYPE_REAL32
        if msg.param_type != expected_type or decode_parameter(msg) != self.pending['value']:
            self.fault = self.fault or 'fusion_parameter_echo_mismatch'
            self.phase = 'BLOCKED'
            return False
        self.phase = waiting[self.phase]
        self.pending = None
        return True

    def mark_ready(self, c, inputs):
        self._clock(c.now)
        if c.ground_status_max_age!=.5:
            raise PermissionError('USB-ground fusion profile cannot produce mission readiness')
        observation,missing = inputs.build(c.now)
        if (self.phase != 'FUSING' or not c.preparation_ground() or not self._bound(c)
                or observation is None or missing
                or (observation.boot_session,observation.rigid_revision) != self.binding):
            raise ValueError('Actual fresh three-audit input proof required before READY')
        self.phase = 'READY'

    @staticmethod
    def heading_age_limits(c):
        return (dict(attitude_max_age_ms=500, sent_max_age_ms=NATIVE_DISPATCH_MAX_GAP_S*1000)
                if c.pilot_source=='native_rc' else {})

    def heading_comparison(self, c, source_position, source_q, fc_q):
        self._clock(c.now)
        if self.alignment is None or not self._bound(c) or not c.link_source_ok() or self.last_send is None:
            return None
        _,aligned_q = compact.aligned_pose(source_position,source_q,self.alignment)
        return compare_heading(aligned_q,fc_q,attitude_age_ms=c.attitude_age*1000,
                               sent_age_ms=(c.now-self.last_send)*1000, **self.heading_age_limits(c))

    def handoff(self, c, core):
        self._clock(c.now)
        if c.ground_status_max_age!=.5:
            raise PermissionError('USB-ground fusion profile cannot hand off to mission core')
        if (self.phase != 'READY' or not c.preparation_ground() or not self._bound(c)
                or core.phase != 'WAIT_EXPLICIT_START' or core.binding is None
                or core.binding[:2] != self.binding or core.sink is not self.sink
                or self.last_send is None or not 0 <= c.now-self.last_send <= self.maximum_send_gap):
            raise ValueError('Prepared same-epoch core and continuous pose stream required')
        core.last_vision,core.last_source_stamp = self.last_send,self.last_stamp
        core.next_vision = self.next_send
        self.transferred_core = core
        self.phase = 'HANDED_OFF'  # The core is now the sole pose sender.
        # No core.start(), unlock, arm or mode change here.

    def close(self, c):
        self._clock(c.now)
        if not c.restore_ground() or not self._bound(c):
            self.land_required |= c.armed is True
            raise PermissionError('No parameter rollback while armed or state unknown')
        if self.transferred_core is not None and self.transferred_core.phase not in (
                'COMPLETE','BLOCKED','STOP_LATCHED'):
            raise PermissionError('Mission owns fusion until actual end; never restore during LAND')
        self.pending = None
        self.phase = 'RESTORE_CTRL' if self.changed else 'CLOSED'

    def status(self):
        return dict(phase=self.phase,fault=self.fault,land_required=self.land_required,
                    receive_verified=self.receive_verified,pose_owner='core' if self.transferred_core else 'service',
                    receipt_checks=deepcopy(self.receipt_checks),
                    physical_flight_authorized=False,live_runner_installed=False)
