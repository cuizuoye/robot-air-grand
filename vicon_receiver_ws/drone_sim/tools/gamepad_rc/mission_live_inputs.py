"""Receive-side mission adapter. No device opens, sends, parameter writes or start.

The existing single-owner monitor supplies CRC-validated MAVLink and PoseGate
samples. Full EKF audits are supplied by the existing audit producer, not by
loading old JSON reports. Missing inputs stay missing, with explicit reasons.
This module is deliberately NOT a live fusion configuration/flight runner.
"""
from collections import deque
from copy import deepcopy
import math
import uuid
from rc_watch import (state_feedback_age, NATIVE_AUDIT_MAX_AGE_S, NATIVE_COMMANDER_MAX_AGE_S,
    NATIVE_RC_MAX_AGE_S, NATIVE_AUX_STATUS_MAX_AGE_S, NATIVE_YAW_TOLERANCE_DEG, NATIVE_REORDER_WINDOW_MS)

from pymavlink.dialects.v20 import common as m

from flight_mission_core import Observation
from ground_estimator_settlement import GroundEstimatorSettlement
from trs_uart_readonly_check import UID2
import control_readonly_policy as control
import usb_combined_fusion_policy as fusion


def age(now, then):
    return now-then if then is not None and 0 <= then <= now else math.inf


class LiveMissionInputs:
    """One boot/revision/input generation, fail-closed receive-side reducer.

    Times are the owner's monotonic session clock. Callers must never replay
    files into these methods as live observations. This adapter does not grant
    flight permission, clear HOME holds, or manufacture an awake gamepad.
    """
    def __init__(self, rigid_revision, *, component=202):
        if not isinstance(rigid_revision, str) or not rigid_revision:
            raise ValueError('Current inspected rigid revision required')
        self.rigid_revision = rigid_revision
        self.component = component
        self.boot_session = uuid.uuid4().hex
        self.now = None
        self.fault = None
        self.version_request = None
        self.identity = None
        self.hb = self.local = self.attitude = self.land = self.actuator = None
        self.pose_at = self.pose_stamp = None
        self.pad_generation = None
        self.pad_present = False
        self.pad_event_at = None
        self.home_down = False
        self.home_press = self.home_release = self.home_ack = None
        self.home_proven = False
        self.audit_rows = deque(maxlen=3)
        self.audit_at = self.audit_request = None
        self.selector = None
        self.last_boot_by_kind = {}
        self.reordered_frames = deque(maxlen=64)
        self.reorder_since = {}
        self.bad_frames = 0
        self.offline_flight_observation = False
        # Legacy benches require barometer aiding. The fixed-profile owner
        # selects the EV-height contract; actual EV fusion is still required.
        self.require_baro_height = True
        self.native_rc = None
        self.native_ready_at = None
        self.critical_status = {}

    def observe_status(self, values, *, requested_at, now):
        """Current native-path reset/Commander reads, independent of full audits.

        Source timestamps are preserved, including unchanged 2Hz Commander
        publications. Re-reading a frozen topic cannot extend its source age.
        """
        self._clock(now)
        boot=self._boot_us(now)
        transaction_age=NATIVE_AUX_STATUS_MAX_AGE_S if self.native_rc else .5
        if boot is None or not 0<=requested_at<=now or now-requested_at>transaction_age:
            raise ValueError('Bounded current status transaction required')
        checked={}
        for name,value in values.items():
            if name not in ('local','commander','selector'):
                raise ValueError('Unknown current status topic')
            stamp=value.get('timestamp')
            limit=(NATIVE_COMMANDER_MAX_AGE_S+.5)*1_000_000 if name=='commander' else transaction_age*1_000_000 if name=='local' else 2_000_000
            if type(stamp) is not int or not 0<=boot-stamp<=limit:
                raise ValueError('Current status source stale/future')
            previous=self.critical_status.get(name)
            if previous and (stamp<previous['value']['timestamp'] or
                    stamp==previous['value']['timestamp'] and value!=previous['value']):
                raise ValueError('Status source regressed or changed at same timestamp')
            if name=='local':
                if any(type(value.get(k)) is not int or not 0<=value[k]<=255 for k in
                       ('xy_reset_counter','z_reset_counter','heading_reset_counter')):
                    raise ValueError('Actual current reset counters required')
                if any(type(value.get(k)) is not bool for k in ('xy_valid','z_valid','v_xy_valid','v_z_valid')):
                    raise ValueError('Explicit local validity required')
            elif name=='commander':
                if (type(value.get('pre_flight_checks_pass')) is not bool or
                    type(value.get('arming_state')) is not int or not 0<=value['arming_state']<=7 or
                    type(value.get('nav_state')) is not int or not 0<=value['nav_state']<=30):
                    raise ValueError('Explicit current Commander state required')
            else:
                GroundEstimatorSettlement(now).observe(value,now)
                if self.selector and any(value[k]!=self.selector[k] for k in ('primary_instance','instance_changed_count')):
                    if self.offline_flight_observation:self.fail('current_ekf_selector_changed')
                    else:self.audit_rows.clear()
            checked[name]=dict(value=deepcopy(value),at=now)
        self.critical_status.update(checked)

    def pilot_status(self, now):
        return self.native_rc.status(now, max_age=NATIVE_RC_MAX_AGE_S) if self.native_rc else None

    def preparation_verified(self, now):
        if self.native_rc:
            s = self.pilot_status(now)
            return bool(s['native_input_observed'] and s['kill_requested'] is False
                        and s['stop_active'] is False)
        return self.home_proven

    def preparation_barrier(self):
        return self.native_ready_at if self.native_rc else max(self.home_press or 0,self.home_release or 0,self.home_ack or 0)

    def _clock(self, now):
        if (not isinstance(now, (float, int)) or isinstance(now, bool)
                or not math.isfinite(now) or now < 0
                or (self.now is not None and now < self.now)):
            self.fail('session_clock_invalid')
            raise ValueError(self.fault)
        self.now = now

    def fail(self, reason):
        self.fault = self.fault or reason  # Never silently recover a changed epoch.

    def request_identity(self, now):
        self._clock(now)
        self.version_request = now

    def _boot_us(self, now):
        if self.attitude is None:
            return None
        return self.attitude['boot_ms']*1000 + int(age(now, self.attitude['at'])*1e6)

    def ingest(self, msg, now):
        self._clock(now)
        kind = msg.get_type()
        if kind == 'BAD_DATA':
            self.bad_frames += 1
            return  # CRC failure never updates a state timestamp.
        if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
            return
        if hasattr(msg, 'time_boot_ms'):
            stamp = int(msg.time_boot_ms)
            previous = self.last_boot_by_kind.get(kind)
            if previous is not None and stamp < previous:
                since=self.reorder_since.setdefault(kind,now)
                if (self.native_rc and previous-stamp<=NATIVE_REORDER_WINDOW_MS
                        and now-since<=NATIVE_REORDER_WINDOW_MS/1000):
                    self.reordered_frames.append(dict(kind=kind,at=now,stamp_ms=stamp,latest_ms=previous))
                    return  # Drop old bytes before either reducer or native RC can refresh.
                self.fail('fc_boot_clock_regressed_or_out_of_order')
                return
            if previous == stamp:
                return  # Replayed FC samples do not refresh their age.
            self.last_boot_by_kind[kind] = stamp
            self.reorder_since.pop(kind,None)
        if self.native_rc:self.native_rc.observe_message(msg,now)
        if kind == 'AUTOPILOT_VERSION' and self.version_request is not None:
            value = msg.flight_sw_version
            identity = (bytes(msg.uid2).hex(), f'{value >> 24}.{value >> 16 & 255}.{value >> 8 & 255}')
            if identity != (UID2, '1.14.3') or (self.identity and identity != self.identity):
                self.fail('fc_identity_changed_or_wrong')
            else:
                self.identity = identity
        elif kind == 'HEARTBEAT':
            if msg.autopilot != m.MAV_AUTOPILOT_PX4:
                self.fail('unexpected_autopilot')
                return
            self.hb = dict(at=now, armed=bool(msg.base_mode & m.MAV_MODE_FLAG_SAFETY_ARMED),
                           custom_mode=int(msg.custom_mode),
                           offboard=(msg.custom_mode >> 16 & 255) == 6,
                           manual_control_active=(msg.custom_mode >> 16) in (1, 2, 3, 7))
        elif kind == 'LOCAL_POSITION_NED':
            values = tuple(getattr(msg, key) for key in ('x', 'y', 'z', 'vx', 'vy', 'vz'))
            if not all(math.isfinite(x) for x in values):
                self.fail('non_finite_local_telemetry')
                return
            self.local = dict(at=now, boot_ms=int(msg.time_boot_ms),
                              position=values[:3], velocity=values[3:])
        elif kind == 'ATTITUDE_QUATERNION':
            q = tuple(getattr(msg, key) for key in ('q1', 'q2', 'q3', 'q4'))
            if not all(math.isfinite(x) for x in q) or not .95 <= math.sqrt(sum(x*x for x in q)) <= 1.05:
                self.fail('invalid_fc_attitude')
                return
            norm = math.sqrt(sum(x*x for x in q))
            w, x, y, z = (v/norm for v in q)
            self.attitude = dict(at=now, boot_ms=int(msg.time_boot_ms),
                                 yaw=math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)),q=(w,x,y,z))
        elif kind == 'EXTENDED_SYS_STATE':
            self.land = dict(at=now, landed=msg.landed_state == m.MAV_LANDED_STATE_ON_GROUND)
        elif (kind == 'COMMAND_ACK' and msg.command == 185
              and (msg.target_system, msg.target_component) == (250, self.component)
              and self.home_press is not None and now >= self.home_press):
            if msg.result == m.MAV_RESULT_ACCEPTED:
                self.home_ack = now

    def observe_gamepad(self, generation, present, now, *, event=False, home=None):
        self._clock(now)
        if not generation:
            raise ValueError('Actual opened gamepad generation required')
        if self.pad_generation is not None and generation != self.pad_generation:
            self.fail('gamepad_generation_changed')
        self.pad_generation = deepcopy(generation)
        self.pad_present = present is True
        if not self.pad_present:
            self.fail('gamepad_disconnected')
        if event:
            self.pad_event_at = now
        if home is not None:
            if type(home) is not bool:
                raise ValueError('Boolean observed HOME state required')
            self.home_down = home
            if home and self.home_press is None:
                self.home_press = now
            if not home and self.home_press is not None:
                self.home_release = now

    def observe_pose(self, sample, now_ros_ns, now_monotonic_ns, now):
        """Sample must already have passed existing PoseGate shape/frame checks."""
        self._clock(now)
        if sample is None:
            return
        source_age = (now_ros_ns-sample.stamp_ns)/1e9
        receive_age = (now_monotonic_ns-sample.received_monotonic_ns)/1e9
        if not 0 <= source_age <= .15 or not 0 <= receive_age <= .15:
            return
        if self.pose_stamp is not None and sample.stamp_ns <= self.pose_stamp:
            return
        self.pose_stamp = sample.stamp_ns
        self.pose_at = now-max(source_age, receive_age)

    def observe_actuator(self, snapshot, now, *, max_age=.5):
        self._clock(now)
        if type(max_age) not in (int,float) or max_age not in (.5,1.):
            raise ValueError('Explicit bounded actuator profile required')
        if self.native_rc:max_age=NATIVE_AUX_STATUS_MAX_AGE_S
        boot = self._boot_us(now)
        fields = ('armed', 'lockdown', 'manual_lockdown', 'force_failsafe')
        if (boot is None or not all(type(snapshot.get(k)) is bool for k in fields)
                or not 0 <= boot-snapshot['timestamp_us'] <= (max_age+(.5 if self.native_rc else 0))*1e6
                or not 0 <= snapshot['requested_at'] <= snapshot['at'] <= now
                or (self.actuator and (snapshot['timestamp_us'] < self.actuator['timestamp_us']
                    or snapshot['timestamp_us']==self.actuator['timestamp_us'] and
                    (not self.native_rc or any(snapshot[k]!=self.actuator[k] for k in fields))))):
            raise ValueError('Fresh current-boot actuator reply required')
        if not self.native_rc and (snapshot['manual_lockdown'] or snapshot['force_failsafe']):
            self.fail('actuator_failsafe')
        self.actuator = deepcopy(snapshot)
        if (self.home_ack is not None and self.home_release is not None
                and snapshot['requested_at'] >= max(self.home_ack, self.home_release)+1.5
                and snapshot['lockdown'] and not snapshot['armed']):
            self.home_proven = True

    def observe_audit(self, row, commander, *, requested_at, now):
        """A complete CURRENT transaction, not an archived passing assessment.

        Reuses the existing three-snapshot XYZ/yaw and control predicates. All
        audit timestamps must belong to the current boot. Measurement topics
        must advance; native selector/flags may repeat their unchanged 1 Hz
        publication within the existing source-age bound. The full
        audit/commander collection is NOT implemented by the read-only monitor;
        until its producer is attached, this input intentionally remains absent.
        """
        self._clock(now)
        boot = self._boot_us(now)
        diagnostic_age = NATIVE_AUDIT_MAX_AGE_S if self.native_rc else 2.
        if boot is None or not 0 <= requested_at <= now or now-requested_at > diagnostic_age:
            raise ValueError('Bounded current-boot audit transaction required')
        selected = row['selector']
        GroundEstimatorSettlement(now).observe(selected, now)
        topics = ('selector', 'flags', 'local', 'status', 'failsafe', 'aid', 'height_aid', 'yaw_aid')
        for topic in topics:
            stamp = row[topic]['timestamp']
            if type(stamp) is not int or not 0 <= boot-stamp <= diagnostic_age*1_000_000:
                raise ValueError('Audit FC topic timestamp stale/future/wrong boot')
            if self.audit_rows and stamp <= self.audit_rows[-1][topic]['timestamp']:
                # PX4 publishes these at ~1 Hz, or immediately on a change.
                # A faster new query can return the same current publication;
                # retain its source timestamp, never manufacture progression.
                same_periodic_state = (self.native_rc is not None
                    and topic in ('selector', 'flags')
                    and row[topic] == self.audit_rows[-1][topic])
                if not same_periodic_state:
                    raise ValueError('Cached/repeated audit is not live proof: '+topic)
        counters = tuple(row['local'][key] for key in
                         ('xy_reset_counter', 'z_reset_counter', 'heading_reset_counter'))
        if any(type(x) is not int or not 0 <= x <= 255 for x in counters):
            raise ValueError('Actual FC reset counters required')
        if (type(commander.get('pre_flight_checks_pass')) is not bool
                or type(commander.get('timestamp')) is not int
                or type(commander.get('arming_state')) is not int
                or type(commander.get('nav_state')) is not int
                or not 0 <= commander['arming_state'] <= 7
                or not 0 <= commander['nav_state'] <= 30
                or not 0 <= boot-commander['timestamp'] <= ((NATIVE_COMMANDER_MAX_AGE_S+.5)*1_000_000 if self.native_rc else 500_000)):
            raise ValueError('Fresh explicit Commander preflight result required')
        if self.selector and (selected['primary_instance'], selected['instance_changed_count']) != (
                self.selector['primary_instance'], self.selector['instance_changed_count']):
            self.audit_rows.clear()  # New pre-control selector needs three new rows.
        self.audit_rows.append(deepcopy({**row, 'commander': commander}))
        self.selector = deepcopy(selected)
        self.audit_at, self.audit_request = now, requested_at

    def build(self, now):
        """Return (Observation or None, reasons). Never fill unknowns with zeros."""
        self._clock(now)
        reasons = []
        if self.fault:
            reasons.append(self.fault)
        if self.identity != (UID2, '1.14.3'):
            reasons.append('current_fc_identity_missing')
        # PX4 1.14.3 set_message_interval explicitly rejects HEARTBEAT.
        # Keep the existing <2s heartbeat and .5s raw motion-feedback gates.
        # Native textual actuator/reset queries have a separate 1s budget.
        for key, row, limit in (('heartbeat', self.hb, 2.), ('local_position', self.local, .5),
                                ('fc_attitude', self.attitude, .5), ('landed_state', self.land, .5)):
            if row is None or age(now, row['at']) > limit or (key == 'heartbeat' and age(now, row['at']) >= 2):
                reasons.append(key+'_missing_or_stale')
        if age(now, self.pose_at) > .15:
            reasons.append('vicon_pose_missing_or_stale')
        native = self.pilot_status(now)
        if native is not None:
            reasons.extend(native['reasons'])
            manual=native['selected_manual_input'] or {}
            if not all(k in manual for k in ('roll','pitch','yaw','throttle','sticks_moving')):
                reasons.append('native_control_axes_missing')
            if native['kill_requested'] is not False or native['stop_active'] is not False:
                reasons.append('native_kill_active_or_unknown')
        else:
            if not self.pad_present or age(now, self.pad_event_at) > 30:
                reasons.append('gamepad_current_input_not_observed')
            if not self.home_proven:reasons.append('current_session_home_response_not_verified')
        boot = self._boot_us(now)
        actor_limit=NATIVE_AUX_STATUS_MAX_AGE_S if native is not None else .5
        if (self.actuator is None or age(now, self.actuator['at']) > actor_limit or boot is None
                or not 0 <= boot-self.actuator['timestamp_us'] <= (actor_limit+(.5 if native is not None else 0))*1e6):
            reasons.append('actuator_state_missing_or_stale')
        row=self.audit_rows[-1] if self.audit_rows else {}
        local_status=row.get('local',{})
        commander=row.get('commander',{})
        commander_at=self.audit_at
        if native is not None:
            # Slow full diagnostics use one shared budget. They do not replace
            # the current reset/Commander feedback checked independently below.
            if not self.audit_rows or age(now,self.audit_at)>NATIVE_AUDIT_MAX_AGE_S or boot is None:
                reasons.append('current_ekf_full_audit_missing_or_stale')
            for name,limit in (('local',NATIVE_AUX_STATUS_MAX_AGE_S*1e6),('commander',(NATIVE_COMMANDER_MAX_AGE_S+.5)*1_000_000)):
                item=self.critical_status.get(name)
                reply_age=NATIVE_COMMANDER_MAX_AGE_S if name=='commander' else NATIVE_AUX_STATUS_MAX_AGE_S
                if (not item or age(now,item['at'])>reply_age or boot is None or
                        not 0<=boot-item['value']['timestamp']<=limit):
                    reasons.append('current_'+name+'_status_missing_or_stale')
            if 'local' in self.critical_status:
                local_status=self.critical_status['local']['value']
                if not all(local_status[k] for k in ('xy_valid','z_valid','v_xy_valid','v_z_valid')):
                    reasons.append('current_local_estimate_invalid')
            if 'commander' in self.critical_status:
                commander=self.critical_status['commander']['value']
                commander_at=self.critical_status['commander']['at']
        elif (not self.audit_rows or age(now,self.audit_at)>.5 or boot is None
                or not 0<=boot-local_status['timestamp']<=500_000):
            reasons.append('current_ekf_audit_reset_counters_and_commander_missing_or_stale')
        if self.audit_rows and self.actuator:
            if (commander['arming_state'] == 2) != self.actuator['armed']:
                reasons.append('commander_and_actuator_armed_state_disagrees')
            if boot is None or not 0 <= boot-commander['timestamp'] <= ((NATIVE_COMMANDER_MAX_AGE_S+.5)*1_000_000 if native else 500_000):
                reasons.append('commander_state_missing_or_stale')
        fusion_ready = preflight = False
        if len(self.audit_rows) == 3:
            rows = list(self.audit_rows)
            try:
                primary = self.selector['primary_instance']
                fusion_ready = fusion.fusion_evidence_ok(rows, primary,
                    allow_in_air=self.offline_flight_observation,
                    require_baro_height=self.require_baro_height,
                    yaw_tolerance_deg=NATIVE_YAW_TOLERANCE_DEG if native is not None else 5.,
                    # Sequential diagnostics share the admitted row budget;
                    # live pose/local/actuator freshness is checked separately.
                    topic_max_separation_s=NATIVE_AUDIT_MAX_AGE_S if native is not None else 2.)
                preflight = (control.assess(rows, primary,
                    topic_max_separation_s=NATIVE_AUDIT_MAX_AGE_S if native is not None else 2.)['estimator_prerequisites_observed']
                             and commander.get('pre_flight_checks_pass') is True)
            except (KeyError, TypeError, ValueError):
                reasons.append('ekf_audit_malformed')
        if not fusion_ready:
            reasons.append('current_xyz_yaw_fusion_not_verified')
        if not preflight and not (self.offline_flight_observation and self.actuator
                                 and self.actuator['armed']):
            reasons.append('current_preflight_not_verified')
        if reasons:
            return None, reasons
        counters = tuple(local_status[key] for key in
                         ('xy_reset_counter', 'z_reset_counter', 'heading_reset_counter'))
        o = Observation(now, self.boot_session, self.rigid_revision, counters,
            self.local['position'], self.local['velocity'], self.attitude['yaw'],
            uid2=self.identity[0], firmware=self.identity[1],
            status_age_s=max(age(now,self.land['at']),
                state_feedback_age(now,commander_at,(boot-commander['timestamp'])/1e6,native=native is not None),
                state_feedback_age(now,self.actuator['at'],(boot-self.actuator['timestamp_us'])/1e6,native=native is not None)),
            local_age_s=age(now, self.local['at']), pose_age_s=age(now, self.pose_at),
            armed=self.actuator['armed'], landed=self.land['landed'], lockdown=self.actuator['lockdown'],
            offboard=commander['nav_state'] == 14 and not self.manual_mode_confirmed(now),
            manual_control_active=self.manual_mode_confirmed(now),
            fusion_ready=fusion_ready, preflight_ready=bool(preflight),
            gamepad_available=native is None, home_response_verified=self.home_proven, home_pressed=self.home_down,
            pilot_source='native_rc' if native is not None else 'gamepad',
            native_rc_ready=bool(native and native['native_input_observed']),
            native_sticks_ready=bool(native and native['sticks_ready']),
            native_stop=bool(native and (native['kill_requested'] is True or native['stop_active'] is True)))
        return o, []

    def manual_mode_confirmed(self, now):
        # PX4 HEARTBEAT is 1 Hz: use its existing <2 s receive budget,
        # independently of the faster attitude/actuator feedback.
        return bool(self.hb and 0 <= age(now, self.hb['at']) < 2
                    and self.hb.get('armed') and self.hb.get('manual_control_active'))

    def describe(self, now):
        observation, reasons = self.build(now)
        return {'scope': 'live receive-side input shadow, NOT flight authorization',
                'observation_complete': observation is not None, 'missing_conditions': reasons,
                'boot_session': self.boot_session, 'rigid_revision': self.rigid_revision,
                'crc_discarded_frames': self.bad_frames, 'audit_rows': len(self.audit_rows),
                'reordered_frame_tail': list(self.reordered_frames),
                'flight_authorized': False, 'mission_started': False,
                'full_audit_producer_attached': bool(self.audit_rows)}
