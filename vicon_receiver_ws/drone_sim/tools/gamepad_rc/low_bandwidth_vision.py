"""Opt-in ground candidate: standard unknown covariance, same effective noise.

No device access/parameter changes. MAVLink2 trims the trailing zero extension
bytes; NaN at covariance[0] explicitly means unknown, not a fabricated zero
measurement variance. Pinned PX4 1.14.3 EKF2.cpp falls back to EVP_NOISE for
the nonfinite position vector; all three angular variances remain bounded by
EVA_NOISE. Require exact float32 equality to the old effective six variances.
Never a flight authorization or a default encoding change.
"""
import math
import struct
import compact_vision as compact

PROFILE = 'unknown-covariance-equivalent-noise-ground-v1'
WIRE_BYTES = 48


def f32(value):
    return struct.unpack('<f', struct.pack('<f', value))[0]


def noise_evidence(parameters):
    if parameters.get('EKF2_EV_NOISE_MD') != 0:
        raise ValueError('Pinned message-noise mode 0 required')
    noise = [parameters.get(k) for k in ('EKF2_EVP_NOISE', 'EKF2_EVA_NOISE')]
    if any(type(v) not in (int, float) or not math.isfinite(v) or v <= 0 for v in noise):
        raise ValueError('Fresh finite positive EV position/angle noise parameters required')
    try:
        p, a = [f32(f32(v)*f32(v)) for v in noise]
    except (OverflowError, struct.error) as exc:
        raise ValueError('Noise outside float32 range') from exc
    covariance = compact.pose_covariance()
    old = [max(p, f32(covariance[i])) for i in (0, 6, 11)]
    old += [max(a, f32(covariance[i])) for i in (15, 18, 20)]
    new = [p]*3+[a]*3
    if not all(math.isfinite(v) and v > 0 for v in new) or old != new:
        raise ValueError('Unknown covariance would change effective PX4 measurement noise')
    return dict(profile=PROFILE, legacy_effective_variances=old,
        candidate_effective_variances=new, float32_equivalent=True,
        parameters={k:parameters[k] for k in ('EKF2_EV_NOISE_MD','EKF2_EVP_NOISE','EKF2_EVA_NOISE')},
        source='Installed PX4 1.14.3 receiver and EKF2.cpp position/orientation variance paths',
        hardware_verified=False,flight_authorized=False)


def message(enc, stamp_us, position, q_wxyz, alignment_yaw, parameters):
    noise_evidence(parameters)
    msg, p, q = compact.message(enc, stamp_us, position, q_wxyz, alignment_yaw)
    msg.covariance = [math.nan]+[0.]*20
    return msg, p, q


def pose_allowed(msg, parameters):
    try:
        noise_evidence(parameters)
        return bool(compact.pose_fields_allowed(msg) and len(msg.covariance)==21
            and math.isnan(msg.covariance[0]) and list(msg.covariance)[1:]==[0.]*20)
    except (ValueError, TypeError, KeyError, AttributeError):
        return False
