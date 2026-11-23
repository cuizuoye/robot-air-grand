"""Native RC state observation used by vicon_fixed_point_entry.

No I/O on import, no channel overrides, no telemetry-rate changes. The old
standalone gamepad watcher is retired: it inferred source from channel values
and changed RC_CHANNELS rate without restoring it. Keep one current entry.
"""

# Full textual EKF diagnostics share a slow shell with fast state reads.
# This is the Pocket mission diagnostic budget, not a control-feedback timeout.
# The 2026-11-10 flight recorded a 5.576s full-row gap. Keep margin for
# sequential replies; collector, admission and retained-row age share this cap.
NATIVE_AUDIT_MAX_AGE_S = 8.
# Query on the original 1s schedule, but tolerate a missed/delayed reply.
# The 11:20 run returned a valid Commander update after 1.015558596s.
# Keep refresh and expiry separate so added tolerance is not spent waiting.
NATIVE_COMMANDER_REFRESH_S = 1.
NATIVE_COMMANDER_MAX_AGE_S = 2.
NATIVE_RC_REFRESH_S = 1.
NATIVE_RC_MAX_AGE_S = 2.
# Mission-only tolerance for scheduling jitter and slow textual diagnostics.
# Refresh cadence and the 150ms age of transmitted Vicon samples stay separate.
NATIVE_DISPATCH_MAX_GAP_S = .4
NATIVE_AUX_STATUS_MAX_AGE_S = 1.
NATIVE_CLOCK_MAX_AGE_S = 2.
NATIVE_YAW_TOLERANCE_DEG = 10.
NATIVE_REORDER_WINDOW_MS = 250


def state_feedback_age(now, received_at, source_age_s, *, native=False):
    """Transport age for PX4 status topics, keeping source timestamps intact.

    PX4 1.14 Commander publishes unchanged actuator/vehicle status at 2 Hz.
    Native RC includes that publication phase in source age. The caller
    applies its reply budget (Commander 2s, actuator checked separately at
    1s). Frozen source age still grows despite rereads. Legacy unchanged.
    """
    import math
    if not all(math.isfinite(v) for v in (now,received_at,source_age_s)) or now<received_at or source_age_s<0:
        return math.inf
    return max(now-received_at,max(0.,source_age_s-(.5 if native else 0.)))

class NativeRCMonitor:
    """Pocket/TRS observation only: physical RC never becomes gamepad events.

    RC_CHANNELS contains no input-source identity. Use input_rc AND the selected
    manual_control_setpoint source; keep switch intent separate from FC state.
    Snapshot observations do not certify an airborne takeover or link-loss test.
    """
    PHYSICAL_SOURCES = frozenset(range(1, 16)) - {6, 7}  # MAVLink and QURT excluded.
    PARAMETER_TYPES = {'COM_RC_OVERRIDE':6,'COM_RC_STICK_OV':9,'RC_MAP_KILL_SW':6,'RC_MAP_FLTMODE':6}

    @classmethod
    def configuration_valid(cls, values, types):
        return bool(all(types.get(k)==v for k,v in cls.PARAMETER_TYPES.items())
            and type(values.get('COM_RC_OVERRIDE')) is int and values['COM_RC_OVERRIDE'] & 3 == 3
            and 0 < values.get('COM_RC_STICK_OV',0) <= 100
            and all(type(values.get(k)) is int and 1 <= values[k] <= 18 for k in ('RC_MAP_KILL_SW','RC_MAP_FLTMODE'))
            and values['RC_MAP_KILL_SW'] != values['RC_MAP_FLTMODE'])
    TOPICS = {
        'input_rc': (('timestamp', 'timestamp_last_signal', 'input_source', 'channel_count', 'rssi'),
                     ('rc_lost', 'rc_failsafe')),
        'manual_control_setpoint': (('timestamp', 'timestamp_sample', 'data_source'), ('valid',)),
        'manual_control_switches': (('timestamp', 'timestamp_sample', 'mode_slot',
                                    'arm_switch', 'offboard_switch', 'kill_switch'), ()),
        'actuator_armed': (('timestamp',), ('armed', 'manual_lockdown', 'lockdown', 'force_failsafe')),
        'failsafe_flags': (('timestamp',), ('manual_control_signal_lost',)),
    }

    def __init__(self):
        self.topics = {}
        self.channels = self.heartbeat = None
        self.mode_changes = []
        self.fc_clock = None
        self.fc_raw_boot_us = None

    def observe_clock(self, upper_boot_us, at, *, remote_boot_us=None):
        """Keep raw FC time separate from its RTT-dependent freshness bound.

        A shorter round trip can lower the upper bound without a FC reboot.
        Two-argument callers supply an exact clock (zero RTT).
        """
        import math
        remote_boot_us = upper_boot_us if remote_boot_us is None else remote_boot_us
        if (not all(math.isfinite(v) for v in (upper_boot_us, remote_boot_us))
                or not 0 < remote_boot_us <= upper_boot_us):
            raise ValueError('Positive FC boot clock required')
        if self.fc_raw_boot_us is not None and remote_boot_us < self.fc_raw_boot_us:
            raise RuntimeError('FC clock regressed; end observation, no reconnect')
        self.fc_raw_boot_us = remote_boot_us
        self.fc_clock = (upper_boot_us, at)

    def observe_topic(self, topic, text, at):
        from usb_fusion_policy import parse_topic
        ints, bools = self.TOPICS[topic]
        row = parse_topic(text, topic, integer_fields=ints, bool_fields=bools)
        if row and topic == 'manual_control_setpoint':
            controls = parse_topic(text, topic, bool_fields=('sticks_moving',),
                                   float_fields=('roll','pitch','yaw','throttle'))
            if controls:row.update(controls)
        # Failed parses invalidate previous evidence instead of retaining green.
        self.topics[topic] = {'received_at': at, 'value': row}
        return row

    def observe_message(self, msg, at):
        if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
            return
        if msg.get_type() == 'RC_CHANNELS':
            count = int(msg.chancount)
            self.channels = dict(received_at=at, time_boot_ms=msg.time_boot_ms,
                channel_count=count, values=[getattr(msg, f'chan{i}_raw') for i in range(1, min(count, 18)+1)],
                rssi=None if msg.rssi == 255 else msg.rssi,
                source_identity_proven=False)
        elif msg.get_type() == 'HEARTBEAT':
            row = dict(received_at=at, custom_mode=msg.custom_mode,
                       armed=bool(msg.base_mode & 128))
            if self.heartbeat and self.heartbeat['custom_mode'] != row['custom_mode']:
                self.mode_changes.append(row)
                self.mode_changes = self.mode_changes[-128:]
            self.heartbeat = row

    def topic_timing(self, name, now, max_age=.5):
        """Remaining query/source budget on the same conservative FC clock.

        Used by the existing shell scheduler; querying never renews source time.
        """
        import math
        item=self.topics.get(name)
        row=item.get('value') if item else None
        host_age=now-item['received_at'] if item else math.inf
        clock_current=bool(self.fc_clock and 0<=now-self.fc_clock[1]<=max_age)
        # Keep the same 1000ppm drift bound as LockClockProbe during holdover.
        elapsed=now-self.fc_clock[1] if clock_current else 0.
        current=self.fc_clock[0]+elapsed*1e6+math.ceil(elapsed*1000) if clock_current else None
        source_age=(current-row['timestamp'])/1e6 if current is not None and row else math.inf
        # Switch state is a 1Hz auxiliary report; failsafe is diagnostic.
        # Neither needs the 500ms stick-input reply budget. Physical receiver,
        # selected sticks and explicit loss flags retain their own checks.
        reply_limit=max(max_age,1.) if name in ('failsafe_flags','manual_control_switches') else max_age
        # One consistent rule: reply budget plus unchanged publication period.
        # A newly read snapshot is not necessarily newly published. Current
        # receiver traffic is still independently required from input_rc.
        source_limit=reply_limit+{'manual_control_setpoint':.2,
            'manual_control_switches':1.,'failsafe_flags':.5,'actuator_armed':.5}.get(name,0.)
        problems=[]
        if not row or row.get('timestamp',0)<=0:problems.append('missing_or_invalid_topic')
        if not 0<=host_age<=reply_limit:problems.append('reply_stale_or_future')
        if not clock_current:problems.append('fc_clock_missing_or_stale')
        if not 0<=source_age<=source_limit:problems.append('source_stale_or_future')
        return dict(host_age_s=host_age,reply_limit_s=reply_limit,
            source_age_upper_s=source_age,source_limit_s=source_limit,
            query_headroom_s=min(reply_limit-host_age,source_limit-source_age) if not problems else -math.inf,
            problems=problems)

    def status(self, now, max_age=2.):
        timing={name:self.topic_timing(name,now,max_age) for name in self.TOPICS}
        def fresh(name):
            return self.topics[name]['value'] if not timing[name]['problems'] else None
        rc = fresh('input_rc')
        manual = fresh('manual_control_setpoint')
        switches = fresh('manual_control_switches')
        actuator = fresh('actuator_armed')
        failsafe = fresh('failsafe_flags')
        reasons = []
        if not rc:
            reasons.append('input_rc_missing_or_stale')
        elif (rc['input_source'] not in self.PHYSICAL_SOURCES or rc['rc_lost'] or rc['rc_failsafe']
              or not 4 <= rc['channel_count'] <= 18
              or not 0 <= rc['timestamp']-rc['timestamp_last_signal'] <= max_age*1e6
              or rc['timestamp_last_signal'] <= 0 or rc['rssi'] == 0):
            reasons.append('physical_receiver_not_proven_or_signal_lost')
        if (not manual or manual['data_source'] != 1 or not manual['valid']
                or not 0 < manual['timestamp_sample'] <= manual['timestamp']
                or manual['timestamp']-manual['timestamp_sample'] > max_age*1e6):
            reasons.append('native_rc_not_selected_or_stale')
        if not failsafe or failsafe['manual_control_signal_lost']:
            reasons.append('fc_manual_control_loss_or_unknown')
        hb = self.heartbeat
        if not hb or not 0 <= now-hb['received_at'] <= max_age:
            hb = None
        loss_reported=bool(rc and (rc['rc_lost'] or rc['rc_failsafe'])
            or manual and not manual['valid'] or failsafe and failsafe['manual_control_signal_lost'])
        return dict(native_input_observed=not reasons, reasons=reasons,topic_timing=timing,
            loss_reported=loss_reported,
            sticks_ready=bool(manual and switches and switches['arm_switch'] in (0,3)
                and all(abs(manual.get(k, float('inf'))) <= .1 for k in ('roll','pitch','yaw'))
                and -1. <= manual.get('throttle',float('inf')) <= -.8 and manual.get('sticks_moving') is False),
            sticks_moving=bool(manual and manual.get('sticks_moving') is True),
            input_rc=rc, selected_manual_input=manual, switches=switches,
            kill_requested=None if not switches or switches['kill_switch'] not in (1,3) else switches['kill_switch'] == 1,
            output_lock_confirmed=None if not actuator else actuator['manual_lockdown'],
            stop_active=None if not actuator else any(actuator[k] for k in ('manual_lockdown','lockdown','force_failsafe')),
            actuator=actuator, actual_flight_mode=None if not hb else hb['custom_mode'],
            channels=self.channels,
            channels_fresh=bool(self.channels and self.fc_clock
                and 0 <= now-self.fc_clock[1] <= max_age
                and 0 <= now-self.channels['received_at'] <= max_age
                and 0 <= self.fc_clock[0]+(now-self.fc_clock[1])*1e6
                    -self.channels['time_boot_ms']*1000 <= max_age*1e6),
            mode_changes=list(self.mode_changes),
            raw_snapshots=dict(self.topics), flight_authorized=False,
            airborne_takeover_verified=False, receiver_loss_response_verified=False)

def review_parameters(readbacks):
    """Actual readback differences only; no invented channel mapping or writes."""
    required = {'COM_RC_IN_MODE':6, **NativeRCMonitor.PARAMETER_TYPES,
        **{name:6 for name in ('RC_MAP_ROLL','RC_MAP_PITCH','RC_MAP_YAW','RC_MAP_THROTTLE')}}
    missing, invalid, differences = [], [], []
    values, types = {}, {}
    for name, kind in required.items():
        row=readbacks.get(name)
        if not row:
            missing.append(name);continue
        value=row.get('value')
        if (row.get('type')!=kind or type(value) not in (int,float)
                or kind==6 and type(value) is not int):
            invalid.append(name+':type');continue
        values[name],types[name]=value,kind
    if 'COM_RC_IN_MODE' in values and values['COM_RC_IN_MODE']!=0:
        differences.append(dict(name='COM_RC_IN_MODE',original=readbacks['COM_RC_IN_MODE'],proposed_value=0))
    override=values.get('COM_RC_OVERRIDE')
    if override is not None and override>=0 and override & 3 != 3:
        differences.append(dict(name='COM_RC_OVERRIDE',original=readbacks['COM_RC_OVERRIDE'],proposed_value=override|3))
    elif override is not None and override<0: invalid.append('COM_RC_OVERRIDE:range')
    if 'COM_RC_STICK_OV' in values and not 0 < values['COM_RC_STICK_OV'] <= 100:
        invalid.append('COM_RC_STICK_OV:choose_valid_threshold')
    channel_names=[name for name in required if name.startswith('RC_MAP_')]
    channel_map={name:values[name] for name in channel_names if name in values}
    for name,channel in channel_map.items():
        if not 1<=channel<=18: invalid.append(name+':assign_actual_channel')
    mapped=[v for v in channel_map.values() if 1<=v<=18]
    if len(mapped)!=len(set(mapped)):invalid.append('channel_mapping_overlap')
    return dict(missing_parameters=missing,manual_attention=invalid,proposed_differences=differences,
        parameters_without_proposed_value=[n for n in values if n not in {d['name'] for d in differences}
                             and not any(x.startswith(n+':') for x in invalid)],
        observed_channel_map=channel_map,
        runtime_takeover_configuration_valid=NativeRCMonitor.configuration_valid(values,types),
        parameters_written=0,flight_authorized=False,
        scope='Readback review only; RC calibration, physical mapping and failsafe response still need observation')


def summarize_observation(samples):
    """Summarize only in-window fresh states, never certify a flight response."""
    channels, axes = {}, {}
    modes, slots, kills, locks, losses = set(),set(),set(),set(),set()
    native_count=0
    for item in samples:
        s=item['status'];native_count+=int(s['native_input_observed'])
        if s.get('channels_fresh'):
            for n,value in enumerate(s['channels']['values'],1):
                if 800<=value<=2200:channels.setdefault(n,[]).append(value)
        manual=s.get('selected_manual_input')
        if s['native_input_observed'] and manual:
            for name in ('roll','pitch','yaw','throttle'):
                value=manual.get(name)
                if type(value) in (float,int):axes.setdefault(name,[]).append(value)
        if s.get('actual_flight_mode') is not None:modes.add(s['actual_flight_mode'])
        if s.get('switches'):slots.add(s['switches']['mode_slot'])
        if s.get('kill_requested') is not None:kills.add(s['kill_requested'])
        if s.get('output_lock_confirmed') is not None:locks.add(s['output_lock_confirmed'])
        rc=s.get('input_rc')
        if rc:losses.add(bool(rc['rc_lost'] or rc['rc_failsafe']))
    def ranges(rows):return {str(k):dict(min=min(v),max=max(v),samples=len(v)) for k,v in rows.items()}
    return dict(samples=len(samples),native_input_samples=native_count,
        channel_ranges=ranges(channels),selected_axis_ranges=ranges(axes),
        mode_slots_observed=sorted(slots),actual_modes_observed=sorted(modes),kill_intents_observed=sorted(kills),
        output_lock_states_observed=sorted(locks),receiver_loss_flags_observed=sorted(losses),
        airborne_takeover_verified=False,receiver_loss_response_verified=False,flight_authorized=False,
        scope='Observed ranges/states only; no inferred stick direction, channel identity or loss-response timing')


if __name__ == '__main__':
    raise SystemExit('Use vicon_fixed_point_entry.py --handset pocket --ground-observe; this module opens no devices.')
