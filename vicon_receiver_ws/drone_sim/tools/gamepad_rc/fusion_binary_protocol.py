"""Versioned offline candidate over standard MAVLink V2_EXTENSION.

No serial owner, message-rate changes, parameter writes or flight permission.
65010 is a project-local type, not a globally allocated MAVLink message ID.
EVF1 remains an offline codec; EVF2 adds selector, resets and source times.
"""
import math
import struct

import fusion_binary_record as legacy

MESSAGE_TYPE = 65010
TARGET = (250, 213)
TAIL = struct.Struct('<IIIHBBBBBQQI')
FUSION_SIZE = legacy.SIZE + TAIL.size
VISUAL = struct.Struct('<4sIQQBB3f4fBb')
RESET_FIELDS = ('xy_reset_counter', 'z_reset_counter', 'vxy_reset_counter',
                'vz_reset_counter', 'heading_reset_counter')


def encode_fusion(row, sequence):
    s, local = row['selector'], row['local']
    healthy = s['healthy']
    if len(healthy) != 9 or any(type(v) is not bool for v in healthy):
        raise ValueError('Nine explicit health bits required')
    if healthy[s['primary_instance']] != s['primary_healthy']:
        raise ValueError('Inconsistent primary health')
    tail = TAIL.pack(*(legacy._integer(s[k], 32) for k in
                      ('instance_changed_count', 'accel_device_id', 'gyro_device_id')),
                     sum(int(v) << i for i, v in enumerate(healthy)),
                     *(legacy._integer(local[k], 8) for k in RESET_FIELDS),
                     legacy._integer(row['flags']['timestamp_sample'], 64),
                     legacy._integer(local['timestamp_sample'], 64),
                     legacy._integer(sequence, 32))
    return b'EVF2' + legacy.encode(row)[4:] + tail


def decode_fusion(raw):
    if len(raw) != FUSION_SIZE or raw[:4] != b'EVF2':
        raise ValueError('Exact EVF2 record required')
    unready = None
    try:
        row = legacy.decode(b'EVF1' + raw[4:legacy.SIZE])
    except legacy.AidNotReady as exc:
        # Still validate the ENTIRE schema before classifying it as not ready.
        row, unready = exc.row, exc.fields
    changed, accel, gyro, healthy, *rest = TAIL.unpack_from(raw, legacy.SIZE)
    if healthy >> 9:
        raise ValueError('Reserved selector bits')
    row['selector'].update(instance_changed_count=changed, accel_device_id=accel,
                           gyro_device_id=gyro, healthy=[bool(healthy & (1 << i)) for i in range(9)])
    s = row['selector']
    if s['primary_healthy'] != s['healthy'][s['primary_instance']]:
        raise ValueError('Inconsistent primary health')
    row['local'].update(zip(RESET_FIELDS, rest[:5]))
    row['flags']['timestamp_sample'], row['local']['timestamp_sample'], row['sequence'] = rest[5:]
    if any(row[key]['estimator_instance'] != s['primary_instance']
           for key in ('aid', 'height_aid', 'yaw_aid')):
        raise ValueError('Mixed estimator evidence')
    if unready:
        raise legacy.AidNotReady(row, unready)
    return row


def encode_visual(row, sequence):
    values = [*row['position'], *row['q']]
    if len(row['position']) != 3 or len(row['q']) != 4 or not all(math.isfinite(v) for v in values):
        raise ValueError('Finite complete visual pose required')
    return VISUAL.pack(b'EVR1', legacy._integer(sequence, 32),
                       legacy._integer(row['timestamp'], 64), legacy._integer(row['timestamp_sample'], 64),
                       legacy._integer(row['pose_frame'], 8), legacy._integer(row['velocity_frame'], 8),
                       *values, legacy._integer(row['reset_counter'], 8), row['quality'])


def decode_visual(raw):
    if len(raw) != 249 or any(raw[VISUAL.size:]):
        raise ValueError('Exact zero-padded EVR1 payload required')
    magic, seq, stamp, sample, pose, velocity, *values = VISUAL.unpack_from(raw)
    if magic != b'EVR1' or not all(math.isfinite(v) for v in values[:7]):
        raise ValueError('Invalid visual record')
    if not .95 <= math.sqrt(sum(v*v for v in values[3:7])) <= 1.05:
        raise ValueError('Invalid quaternion')
    return dict(sequence=seq, timestamp=stamp, timestamp_sample=sample, pose_frame=pose,
                velocity_frame=velocity, position=values[:3], q=values[3:7],
                reset_counter=values[7], quality=values[8])


def message(encoder, raw):
    """Offline/test packet helper. Does not write anything to a device."""
    if len(raw) > 249:
        raise ValueError('Extension payload exceeded')
    return encoder.v2_extension_encode(0, *TARGET, MESSAGE_TYPE, raw.ljust(249, b'\0'))
