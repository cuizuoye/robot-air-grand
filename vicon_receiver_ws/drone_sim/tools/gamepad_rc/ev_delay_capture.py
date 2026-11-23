"""Offline preparation of matched Vicon/FC-IMU delay records. NO device API.

DelayRecorder reuses the bounded RAM recorder; no subscriptions, sends, port
opens, disk writes, approvals or live entrypoints are added. Future adapters
must supply real, atomic SDK frame/pose metadata, not the latest frame-number
topic. Existing static send guards are NOT a manual-motion capture permit.

FC visual samples and IMU samples use the SAME boot HRT. USB read/decode time
is retained only for diagnostics/matching, never used as measurement time.
Legacy reviewed SDK-rate intervals remain supported. Native ULog pairing uses
FC observation intervals and reports their variation against frame cadence;
microsecond resolution is not a claim of camera exposure/transport accuracy.
"""
from collections import Counter
from copy import deepcopy
import math
import hashlib
import json

import compact_vision
from uav_coord_bridge.mavlink_vision import transform_rigid_pose
from ground_link_diagnostics import TimingRecorder, KINDS
from ekf_ground_acceptance import result
from ev_delay_calibration import calibrate

CAPTURE_KINDS = {'calibration_vicon', 'calibration_send', 'calibration_visual',
                 'calibration_imu', 'calibration_tx', 'calibration_clock',
                 'calibration_sdk', 'calibration_ulog'}
FORBIDDEN_TX = {'PARAM_SET', 'PARAM_EXT_SET', 'SET_MODE', 'SET_ATTITUDE_TARGET',
                'SET_POSITION_TARGET_LOCAL_NED', 'SET_POSITION_TARGET_GLOBAL_INT'}


def pose_route(metadata):
    """Bind one declared capture transport; this is not live authorization.

    Historical documents default to TRS. WiFi must explicitly identify its
    fixed endpoints and source evidence; mixed-link samples never share a fit.
    """
    if not isinstance(metadata, dict):
        raise ValueError('capture_metadata_required')
    route = metadata.get('pose_route', 'trs')
    if route not in ('trs', 'wifi'):
        raise ValueError('unsupported_pose_route')
    if route == 'wifi':
        import ipaddress
        binding = metadata.get('pose_transport', {})
        if (not isinstance(binding, dict) or binding.get('kind') != 'fixed_peer_udp'
                or not isinstance(binding.get('evidence'), str) or not binding['evidence'].strip()):
            raise ValueError('wifi_transport_binding_required')
        for name in ('peer', 'bind'):
            endpoint = binding.get(name)
            if (not isinstance(endpoint, (list, tuple)) or len(endpoint) != 2
                    or not isinstance(endpoint[0], str) or type(endpoint[1]) is not int
                    or not 1 <= endpoint[1] <= 65535):
                raise ValueError('invalid_wifi_endpoint:'+name)
            address = ipaddress.IPv4Address(endpoint[0])
            if address.is_unspecified or address.is_multicast or address.is_loopback or int(address)==0xffffffff:
                raise ValueError('unicast_wifi_endpoint_required')
        if binding['peer'][0] == binding['bind'][0]:
            raise ValueError('distinct_wifi_peer_and_bind_required')
    return route


def fc_pairing_digest(events, boot_session):
    """Bind imported FC measurements, excluding host import/arrival times."""
    rows=[{k:v for k,v in e.items() if k!='at_ns'} for e in events
          if e['kind'] in ('calibration_visual','calibration_imu')]
    return hashlib.sha256(json.dumps([boot_session,rows],sort_keys=True,
        separators=(',', ':'),allow_nan=False).encode()).hexdigest()


class DelayRecorder(TimingRecorder):
    """Passive scalar callbacks ONLY; one original transport owner remains.

    record(kind, at_ns=original_host_observation_ns, **fields) accepts the field
    schema in prepare(). Metadata/evidence declarations are NOT verified by
    this object and cannot authorize hardware. Dump export() after stopping.
    """
    _allowed_kinds = KINDS | CAPTURE_KINDS

    def __init__(self, session, metadata, **kwargs):
        super().__init__(session, **kwargs)
        self.metadata = deepcopy(metadata)

    def export(self):
        return {**super().export(), **result(capture_metadata=deepcopy(self.metadata))}

    def observe_clock_decision(self, probe, *, at_ns):
        """Observe existing LockClockProbe AFTER request/ingest, no new request.

        Keep asymmetric offset bounds and original reply decision. RTT/2 is NOT
        declared to be clock accuracy; this callback creates no new reference.
        """
        if not probe.history:
            return False
        event=dict(probe.history[-1])
        original=event.pop('at_ns')
        if original>at_ns:
            raise ValueError('clock_observation_precedes_actual_event')
        event['original_event_ns']=original
        if probe.reference is not None and probe.reference['received_ns']==original:
            event.update(probe.reference)
        return self.record('calibration_clock',at_ns=at_ns,**event)

    def observe_highres_imu(self, message, *, at_ns, route, crc_valid=False):
        """Record real sensor-derived gyro; NOT fused ATTITUDE or delta_angle.

        HIGHRES_IMU uses vehicle_imu.timestamp_sample and delta_angle/dt minus
        estimated gyro bias in the pinned source. Its integration period,
        sensor ID and clipping are NOT in this MAVLink payload. Consequently
        these rows are retained but prepare() refuses to substitute them for
        the preferred fully-timestamped vehicle_imu integration records.
        """
        if (not crc_valid or message.get_type() != 'HIGHRES_IMU'
                or (message.get_srcSystem(), message.get_srcComponent()) != (1, 1)):
            return False
        if message.fields_updated & 0x38 != 0x38:
            return False
        return self.record('calibration_imu', at_ns=at_ns, source='HIGHRES_IMU',
            sample_us=message.time_usec, gx=message.xgyro, gy=message.ygyro,
            gz=message.zgyro, route=route, integration_us=None,
            sensor_id=None, clipping=None, sensor_derived=True)


def _integer(value, label, minimum=0):
    if type(value) is not int or value < minimum:
        raise ValueError('integer_required:' + label)
    return value


def _finite(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('finite_required:' + label)
    return value


def _quat(values):
    if len(values) != 4:
        raise ValueError('quaternion_length')
    values = [_finite(v, 'quaternion') for v in values]
    norm = math.sqrt(sum(v*v for v in values))
    if not .95 <= norm <= 1.05:
        raise ValueError('quaternion_norm')
    return [v/norm for v in values]


def _multiply(a, b):
    w, x, y, z = a; v, i, j, k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def _relative(a, b):
    q = _multiply([a[0], -a[1], -a[2], -a[3]], b)
    return [-v for v in q] if q[0] < 0 else q


def _distance(a, b):
    q = _relative(_quat(a), _quat(b))
    return 2*math.atan2(math.sqrt(sum(v*v for v in q[1:])), abs(q[0]))


def quaternion_rate(first_wxyz, second_wxyz, interval_ns):
    """SO(3) shortest relative rotation / acquisition interval, body FRD.

    A q/-q sign change is not motion. Midpoint assignment below compensates
    the finite-difference interval; no causal smoothing/group delay is added.
    """
    _integer(interval_ns, 'acquisition_interval', 1)
    q = _relative(_quat(first_wxyz), _quat(second_wxyz))
    s = math.sqrt(sum(v*v for v in q[1:]))
    angle = 2*math.atan2(s, q[0])
    factor = (2 if s < 1e-12 else angle/s)*1e9/interval_ns
    return [v*factor for v in q[1:]]


def _q(row):
    return _quat([row[k] for k in ('qw', 'qx', 'qy', 'qz')])


def prepare(document):
    """Build the existing calibrate() schema from a passive event export.

    Required capture_metadata: fc_boot_session, shared_fc_clock_evidence,
    rigid_body_revision_sha256, orientation_transform_evidence,
    sdk_frame_pose_evidence, sdk_rate_evidence, interval_uncertainty_ns,
    clock_uncertainty_ns, frame_pose_join='atomic_sdk_frame',
    timestamp_mode='fc_receive_time', body_orientation_in_rigid_xyzw,
    alignment_yaw_rad. Native same-ULog mode uses the importer-bound FC sample
    digest instead of host clock/SDK interval error declarations. All evidence
    must be reviewed; strings are not proof.

    vicon: frame, ros_stamp_ns, sdk_hz, qx/qy/qz/qw, occluded, at_ns;
           quaternion is the EXACT original world PoseStamped (SDK raw pose is
           separately retained as calibration_sdk). Callback at_ns is NOT an
           exposure timestamp. Optional source_observed_ns is SDK GetFrame end.
    send: frame, ros_stamp_ns, px/py/pz, qw/qx/qy/qz, route matching metadata
          pose_route ('trs' by default; 'wifi' requires pose_transport), at_ns;
          optional sent_ns/tx_start_ns retain actual wire-write times when an
          atomic side-channel arrives after the send; join time never replaces TX.
    visual: sample_us, published_us, px/py/pz, qw/qx/qy/qz, reset_counter,
            pose_frame=1, route='usb_diagnostic' or 'ulog', at_ns;
    imu: source='vehicle_imu', sample_us (integration END), integration_us,
         dx/dy/dz (delta_angle radians, FRD), sensor_id, clipping, at_ns;
    OR source='sensor_combined', sample_us (original timestamp), integration_us
         (gyro_integral_dt), gx/gy/gz (gyro_rad FRD), sensor_id joined from the
         contemporaneous sensor_selection record, clipping (gyro_clipping).
         Metadata sensor_selection_evidence is mandatory for that source.

    For ULog imports at_ns is explicitly import/observation time, not sample
    time, and is never used in the matching window or in calibration.
    """
    timing_diagnostics = None
    try:
        if document.get('recorder_fault') or document.get('dropped_events'):
            raise ValueError('capture_fault_or_overflow')
        meta = document['capture_metadata']
        transport = pose_route(meta)
        for name in ('fc_boot_session', 'shared_fc_clock_evidence',
                     'rigid_body_revision_sha256', 'orientation_transform_evidence',
                     'sdk_frame_pose_evidence', 'sdk_rate_evidence'):
            if not isinstance(meta.get(name), str) or not meta[name]:
                raise ValueError('missing_evidence:' + name)
        if (meta.get('frame_pose_join') != 'atomic_sdk_frame'
                or meta.get('timestamp_mode') != 'fc_receive_time'):
            raise ValueError('atomic_frame_join_and_receive_time_required')
        native = meta.get('timing_method') == 'native_fc_observation_intervals'
        if meta.get('motion_window_required'):
            window=meta.get('motion_window',{})
            go,start,cue,end,deadline=(_integer(window.get(k),'motion_'+k) for k in
                ('go_ns','started_ns','cue_ns','ended_ns','deadline_ns'))
            if not (go < start <= cue <= start+500_000_000
                    and deadline-start==15_000_000_000 and end>=deadline):
                raise ValueError('incomplete_or_unsynchronized_motion_window')
        if native:
            proof=meta.get('fc_time_pairing',{})
            artifact=proof.get('ulog_artifact') or {}
            if (proof.get('boot_session')!=meta['fc_boot_session']
                    or proof.get('measurement_sha256')!=fc_pairing_digest(document['events'],meta['fc_boot_session'])
                    or not isinstance(artifact.get('sha256'),str) or len(artifact['sha256'])!=64
                    or proof.get('timestamp_unit')!='microseconds'
                    or proof.get('timestamp_domain')!='same_ULog_FC_boot'):
                raise ValueError('native_FC_pairing_provenance_missing_or_changed')
            # HRT microsecond resolution only, NOT camera/transport accuracy.
            clock_error=interval_error=1000
        else:
            clock_error = _integer(meta.get('clock_uncertainty_ns'), 'clock_uncertainty')
            interval_error = _integer(meta.get('interval_uncertainty_ns'), 'interval_uncertainty')
        if clock_error > 5_000_000:
            raise ValueError('clock_uncertainty_over_5ms')
        if interval_error > 1_000_000:
            raise ValueError('sdk_interval_uncertainty_over_1ms')
        # Use exactly the existing frame conversion, not a second 45deg rotation.
        mount = meta['body_orientation_in_rigid_xyzw']
        alignment = _finite(meta['alignment_yaw_rad'], 'alignment_yaw')
        events = document['events']
        if not isinstance(events, list) or len(events) > 40000:
            raise ValueError('bounded_events_required')
        groups = {k: [] for k in CAPTURE_KINDS}
        last = -1
        for e in events:
            at = _integer(e['at_ns'], 'host_observation_time')
            if at < last:
                raise ValueError('host_clock_regression')
            last = at
            if e['kind'] in groups:
                groups[e['kind']].append(e)
        for e in groups['calibration_tx']:
            if e.get('message_type') in FORBIDDEN_TX or e.get('command') in (176, 241, 245, 246, 400):
                raise ValueError('forbidden_write_mode_or_actuation_in_capture')
        raw, previous_frame, previous_ros, hz = {}, -1, -1, None
        for e in groups['calibration_vicon']:
            frame = _integer(e['frame'], 'frame')
            stamp = _integer(e['ros_stamp_ns'], 'ros_stamp', 1)
            rate = _finite(e['sdk_hz'], 'sdk_hz')
            if not 1 <= rate <= 1000 or hz is not None and rate != hz:
                raise ValueError('sdk_rate_invalid_or_changed')
            if frame <= previous_frame or stamp <= previous_ros:
                raise ValueError('sdk_or_ros_epoch_changed')
            if e.get('occluded') is not False:
                raise ValueError('occluded_or_unknown_vicon')
            previous_frame, previous_ros, hz = frame, stamp, rate
            q = _q(e)
            _, body = transform_rigid_pose([0, 0, 0], [*q[1:], q[0]], [0, 0, 0], mount)
            _, body = compact_vision.aligned_pose([0, 0, 0], body, alignment)
            raw[(frame, stamp)] = (e, body)
        sends = []
        for e in groups['calibration_send']:
            key = (e['frame'], e['ros_stamp_ns'])
            if key not in raw or e.get('route') != transport:
                raise ValueError('missing_atomic_source_or_wrong_pose_route')
            if e.get('sent_ns',e['at_ns']) < raw[key][0].get('source_observed_ns',raw[key][0]['at_ns']):
                raise ValueError('send_precedes_source_observation')
            q = _q(e)
            if _distance(q, raw[key][1]) > 1e-5:
                raise ValueError('source_transform_does_not_match_actual_send')
            pos = [_finite(e[k], 'position') for k in ('px', 'py', 'pz')]
            sends.append((e, key, q, pos))
        matched, counters, last_us, last_frame = [], Counter(), -1, -1
        resets = set()
        for e in groups['calibration_visual']:
            if native and e.get('route')!='ulog':
                raise ValueError('native_pairing_requires_original_ULog_visual')
            sample = _integer(e['sample_us'], 'visual_sample', 1)
            publication = _integer(e['published_us'], 'visual_publication', 1)
            if sample <= last_us:
                raise ValueError('duplicate_visual_or_fc_boot_changed')
            last_us = sample
            if not 0 <= publication-sample <= 1000 or e.get('pose_frame') != 1:
                raise ValueError('not_verified_fc_receive_time_or_NED')
            resets.add(_integer(e['reset_counter'], 'visual_reset'))
            if len(resets) != 1:
                raise ValueError('visual_reset_during_capture')
            if e.get('route') not in ('ulog', 'usb_diagnostic'):
                raise ValueError('unreviewed_visual_diagnostic_route')
            q = _q(e); pos = [_finite(e[k], 'visual_position') for k in ('px', 'py', 'pz')]
            candidates = [(s, key) for s, key, sq, sp in sends
                if (e['route'] == 'ulog' or 0 <= e['at_ns']-s.get('sent_ns',s['at_ns']) <= 500_000_000)
                and _distance(q, sq) <= 1e-5
                and math.dist(pos, sp) <= 1e-5]
            if len(candidates) != 1:
                counters['ambiguous_or_unmatched_visual'] += 1
                continue
            _, key = candidates[0]
            if key[0] <= last_frame:
                raise ValueError('repeated_or_regressing_matched_source_frame')
            last_frame = key[0]
            matched.append((sample*1000, key[0], raw[key][1]))
        if len(matched) < 40:
            raise ValueError('at_least_40_unique_actual_visual_matches_required')
        if counters['ambiguous_or_unmatched_visual'] > .1*len(groups['calibration_visual']):
            raise ValueError('too_many_ambiguous_or_unmatched_visual_samples')
        # Empirical arrival variation relative to frame cadence, not absolute
        # exposure timing. A wide spread cannot support one precise delay.
        arrival_residuals=[a[0]-matched[0][0]-round((a[1]-matched[0][1])*1e9/hz)
                           for a in matched]
        arrival_span=max(arrival_residuals)-min(arrival_residuals)
        intervals=[dict(after_frame=a[1],before_frame=b[1],
            source_interval_ns=round((b[1]-a[1])*1e9/hz),fc_interval_ns=b[0]-a[0])
            for a,b in zip(matched,matched[1:])]
        invalid=[r for r in intervals if r['source_interval_ns']>50_000_000 or r['fc_interval_ns']>50_000_000]
        timing_diagnostics=dict(matched_visual_samples=len(matched),
            observed_arrival_timing_span_ns=arrival_span,
            maximum_source_interval_ns=max(r['source_interval_ns'] for r in intervals),
            maximum_fc_interval_ns=max(r['fc_interval_ns'] for r in intervals),
            interval_limit_ns=50_000_000,invalid_interval_count=len(invalid),invalid_intervals=invalid,
            scope='All matched intervals, including rejected ones; no trimming, repair or calibration approval')
        if invalid:
            raise ValueError('visual_gap_over_50ms_insufficient_recording_cadence')
        vision, visual_windows, arrival_interval_differences = [], [], []
        for a, b in zip(matched, matched[1:]):
            acquisition_ns = round((b[1]-a[1])*1e9/hz)
            interval_ns=b[0]-a[0] if native else acquisition_ns
            arrival_interval_differences.append(abs(b[0]-a[0]-acquisition_ns))
            vision.append(dict(t_ns=(a[0]+b[0])//2,
                rate_body_rad_s=quaternion_rate(a[2], b[2], interval_ns)))
            visual_windows.append(interval_ns)
        imu, ids, windows, last_end, last_mid = [], set(), [], -1, -1
        preferred_imu = [e for e in groups['calibration_imu'] if e.get('source') in ('vehicle_imu', 'sensor_combined')]
        if not preferred_imu:
            raise ValueError('vehicle_imu_required_HIGHRES_IMU_lacks_integration_period_and_identity')
        sources = {e['source'] for e in preferred_imu}
        if len(sources) != 1:
            raise ValueError('mixed_imu_sources_require_separate_review')
        source = next(iter(sources))
        if source == 'sensor_combined' and not meta.get('sensor_selection_evidence'):
            raise ValueError('sensor_combined_requires_sensor_selection_evidence')
        counters['imu_diagnostic_rows_not_used'] = len(groups['calibration_imu'])-len(preferred_imu)
        for e in preferred_imu:
            end = _integer(e['sample_us'], 'imu_sample', 1)*1000
            dt = _integer(e['integration_us'], 'imu_integration', 1)*1000
            sensor = _integer(e['sensor_id'], 'gyro_device_id', 1)
            ids.add(sensor)
            if len(ids) != 1 or end <= last_end or e.get('clipping') != 0:
                raise ValueError('imu_switch_boot_regression_or_clipping')
            midpoint = end-dt//2
            if midpoint <= last_mid or midpoint < 0 or dt > 20_000_000:
                raise ValueError('imu_integration_time_invalid')
            if last_end >= 0 and end-last_end > 50_000_000:
                raise ValueError('imu_sample_gap_over_50ms')
            last_end, last_mid = end, midpoint
            rates = ([_finite(e[k], 'gyro_rad') for k in ('gx', 'gy', 'gz')]
                if source == 'sensor_combined' else
                [_finite(e[k], 'delta_angle')*1e9/dt for k in ('dx', 'dy', 'dz')])
            imu.append(dict(t_ns=midpoint, rate_body_rad_s=rates))
            windows.append(dt)
        if len(imu) < 40:
            raise ValueError('raw_vehicle_imu_series_missing')
        capture = dict(alignment=dict(clock_domain='fc_boot_ns', timestamp_mode='fc_receive_time',
            vision_timestamp_field='vehicle_visual_odometry.timestamp_sample',
            imu_timestamp_field=('sensor_combined.timestamp' if source == 'sensor_combined'
                                 else 'vehicle_imu.timestamp_sample'), body_frame='FRD',
            clock_uncertainty_ns=clock_error,
            timing_method='native_fc_observation_intervals' if native else 'reviewed_sdk_acquisition_intervals',
            observation_matching_evidence='unique actual FC visual / '+transport+' send / atomic SDK frame matches',
            pose_route=transport, pose_transport=deepcopy(meta.get('pose_transport')),
            rigid_body_revision_sha256=meta['rigid_body_revision_sha256'],
            orientation_transform_evidence=meta['orientation_transform_evidence'],
            clock_mapping_evidence=('Same ULog native sample timestamps; payload matched to exact source frames; '+
                meta['fc_time_pairing']['ulog_artifact']['sha256'] if native else meta['shared_fc_clock_evidence']),
            imu_time_reference='integration_midpoint_from_timestamp_sample',
            vision_time_reference='midpoint_of_matched_FC_observation_samples'), imu=imu, vision=vision)
        return result(prepared=True, reasons=[], capture=capture,
            matched_visual_samples=len(matched), excluded_samples=dict(counters),
            uncertainty=dict(clock_mapping_ns=clock_error, sdk_interval_bound_ns=None if native else interval_error,
                native_interval_quantization_ns=interval_error if native else None,
                interval_bound_scope='native HRT quantization only' if native else 'reviewed SDK acquisition interval',
                host_timesync_used_for_fit=not native,
                max_arrival_vs_sdk_interval_difference_ns=max(arrival_interval_differences),
                observed_arrival_timing_span_ns=arrival_span,
                transport_jitter_removed=False,
                imu_integration_max_ns=max(windows), vision_difference_max_ns=max(visual_windows),
                search_grid_ns=1_000_000, is_statistical_confidence_interval=False,
                absolute_camera_exposure_delay_identified=False),
            parameter_approved=False, ev_delay_parameter_candidate_ms=None)
    except (KeyError, TypeError, ValueError, OverflowError, ZeroDivisionError) as exc:
        return result(prepared=False, reasons=[str(exc)], parameter_approved=False,
                      ev_delay_parameter_candidate_ms=None,timing_diagnostics=timing_diagnostics)


def analyze(document):
    """Fit and independent held-out time segment; never approve an EV_DELAY.

    Positive correlation lag means vision is late in its FC sample-time axis.
    PX4 setExtVisionData subtracts EV_DELAY*1000 AND dt_EKF/2. The total
    correlation lag is not blindly copied into the persistent parameter.
    Sampling kernels/half-EKF-period and uncertainty need human review.
    """
    prepared = prepare(document)
    if not prepared['prepared']:
        return {**prepared, 'candidate_identified': False}
    if (prepared['capture']['alignment']['timing_method']=='native_fc_observation_intervals'
            and prepared['uncertainty']['observed_arrival_timing_span_ns']>10_000_000):
        return {**prepared,'candidate_identified':False,
                'reasons':['native_arrival_variation_exceeds_plus_minus_5ms_budget']}
    capture = prepared['capture']
    lo = max(capture['imu'][0]['t_ns'], capture['vision'][0]['t_ns'])
    hi = min(capture['imu'][-1]['t_ns'], capture['vision'][-1]['t_ns'])
    split = lo + (hi-lo)*3//5
    fits = []
    for start, end in ((lo, split), (split, hi+1)):
        segment = dict(alignment=capture['alignment'], **{
            key: [r for r in capture[key] if start <= r['t_ns'] < end] for key in ('imu', 'vision')})
        fits.append(calibrate(segment))
    reasons = []
    if not all(r['candidate_identified'] for r in fits):
        reasons.append('fit_or_independent_holdout_unidentifiable')
    elif abs(fits[0]['effective_delay_candidate_ms']-fits[1]['effective_delay_candidate_ms']) > 5:
        reasons.append('independent_holdout_disagrees_over_5ms')
    return result(candidate_identified=not reasons, reasons=reasons,
        fit=fits[0], independent_holdout=fits[1], preparation=prepared,
        uncertainty=prepared['uncertainty'], parameter_approved=False,
        ev_delay_parameter_candidate_ms=None,
        sign_definition='positive lag: EV arrives late; PX4 subtracts EV_DELAY*1000 + dt_EKF/2',
        note='Effective lag candidate only; no hardware/maintenance approval, no separation of component delays.')
