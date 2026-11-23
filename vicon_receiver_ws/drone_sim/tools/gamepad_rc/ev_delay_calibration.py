"""OFFLINE effective EV-delay candidate from paired rotational excitation.

Input: actual FC observation timestamp_sample (not PC arrival) for transformed
Vicon angular rate, and IMU delta-angle/dt in the same FC monotonic clock/body
frame. Correlation yields an effective delay candidate, NOT component delays,
an approved parameter or evidence that an unobservable static run is calibrated.
No numpy/ROS/device dependencies. The bounded grid is for offline bench data.
"""
import bisect
import math

from ekf_ground_acceptance import result


def _series(rows):
    if not isinstance(rows, list) or not 40 <= len(rows) <= 20000:
        raise ValueError('40..20000 real timestamped samples required')
    output, last = [], -1
    for row in rows:
        stamp, value = row.get('t_ns'), row.get('rate_body_rad_s')
        if (type(stamp) is not int or stamp <= last or not isinstance(value, list) or len(value) != 3
                or any(type(v) not in (int,float) or not math.isfinite(v) for v in value)):
            raise ValueError('Progressing FC nanoseconds and finite body angular rate required')
        output.append((stamp, value))
        last = stamp
    return output


def _at(series, stamps, at, max_gap_ns):
    i = bisect.bisect_left(stamps, at)
    if i < len(stamps) and stamps[i] == at:
        return series[i][1]
    if i == 0 or i == len(stamps) or stamps[i]-stamps[i-1] > max_gap_ns:
        return None
    ratio = (at-stamps[i-1])/(stamps[i]-stamps[i-1])
    return [a+ratio*(b-a) for a,b in zip(series[i-1][1],series[i][1])]


def _score(pairs):
    if len(pairs) < 25:
        return None
    ma = [sum(a[k] for a,b in pairs)/len(pairs) for k in range(3)]
    mb = [sum(b[k] for a,b in pairs)/len(pairs) for k in range(3)]
    xx = yy = xy = 0.
    for a,b in pairs:
        for k in range(3):
            x,y = a[k]-ma[k],b[k]-mb[k]
            xx += x*x; yy += y*y; xy += x*y
    if min(xx,yy)/len(pairs) < .01:  # >=0.1rad/s RMS excitation, not a flight limit.
        return None
    return xy/math.sqrt(xx*yy)


def calibrate(capture):
    reasons = []
    # These are evidence declarations to review, NOT a mapping inferred from RTT.
    required = dict(clock_domain='fc_boot_ns', timestamp_mode='fc_receive_time',
        vision_timestamp_field='vehicle_visual_odometry.timestamp_sample',
        body_frame='FRD')
    meta = capture.get('alignment', {})
    if not isinstance(meta,dict):
        return result(candidate_identified=False,reasons=['alignment_missing'],parameter_approved=False)
    for key,value in required.items():
        if meta.get(key) != value:
            reasons.append('alignment_missing_or_wrong:' + key)
    # Both fields are real gyro measurement timestamps in the pinned source.
    # sensor_combined.timestamp is populated from vehicle_imu.timestamp_sample
    # by VotedSensorsUpdate, NOT from a fused attitude or host arrival clock.
    if meta.get('imu_timestamp_field') not in ('vehicle_imu.timestamp_sample', 'sensor_combined.timestamp'):
        reasons.append('alignment_missing_or_wrong:imu_timestamp_field')
    for key in ('observation_matching_evidence', 'rigid_body_revision_sha256',
                'orientation_transform_evidence', 'clock_mapping_evidence'):
        if not isinstance(meta.get(key),str) or not meta[key]:
            reasons.append('alignment_evidence_missing:' + key)
    uncertainty = meta.get('clock_uncertainty_ns')
    if type(uncertainty) is not int or not 0 <= uncertainty <= 5_000_000:
        reasons.append('clock_uncertainty_not_bounded_to_5ms')
    if reasons:
        return result(candidate_identified=False, reasons=reasons, parameter_approved=False)
    try:
        imu, vision = _series(capture.get('imu')), _series(capture.get('vision'))
    except (ValueError, TypeError, KeyError) as exc:
        return result(candidate_identified=False, reasons=[str(exc)], parameter_approved=False)
    # Use a fixed common domain at every lag to avoid edge-dependent sample bias.
    lo = max(imu[0][0],vision[0][0])
    hi = min(imu[-1][0],vision[-1][0]-300_000_000)
    if hi-lo < 4_000_000_000:
        return result(candidate_identified=False, reasons=['at_least_4s_common_excitation_required'], parameter_approved=False)
    stamps = [t for t,v in vision]
    segments = [[(t,v) for t,v in imu if a <= t < b] for a,b in ((lo,(lo+hi)//2),((lo+hi)//2,hi))]
    curves = []
    for segment in segments:
        curve = []
        for lag_ms in range(301):
            pairs = [(v,other) for t,v in segment
                     if (other := _at(vision,stamps,t+lag_ms*1_000_000,50_000_000)) is not None]
            score = _score(pairs)
            if score is not None and len(pairs) >= .9*len(segment):
                curve.append((lag_ms,score))
        curves.append(curve)
    if any(not c for c in curves):
        return result(candidate_identified=False, reasons=['insufficient_excitation_or_overlap'], parameter_approved=False)
    peaks = [max(c,key=lambda v:v[1]) for c in curves]
    spread = abs(peaks[0][0]-peaks[1][0])
    if any(s < .9 for lag,s in peaks):
        reasons.append('weak_correlation')
    if any(lag in (0,300) for lag,s in peaks):
        reasons.append('grid_boundary_requires_wider_independent_review')
    if spread > 5:
        reasons.append('split_half_delay_not_repeatable_within_5ms')
    support = []
    for curve,(lag,score) in zip(curves,peaks):
        near = [d for d,s in curve if s >= score-.005]
        support.append([min(near),max(near)])
        if max(near)-min(near) > 20:
            reasons.append('broad_or_ambiguous_correlation_peak')
        if any(abs(d-lag) > 20 and s >= score-.005 for d,s in curve):
            reasons.append('multiple_lag_candidates')
    return result(candidate_identified=not reasons, reasons=sorted(set(reasons)),
        effective_delay_candidate_ms=sum(p[0] for p in peaks)/2,
        split_half_peaks=[dict(delay_ms=d,correlation=s) for d,s in peaks],
        near_peak_support_ms=support, split_half_difference_ms=spread,
        component_delays_identified=False, parameter_approved=False,
        note='Candidate only; spectral excitation, transform, clock and independent holdout require human review. RTT is not EV_DELAY.')
