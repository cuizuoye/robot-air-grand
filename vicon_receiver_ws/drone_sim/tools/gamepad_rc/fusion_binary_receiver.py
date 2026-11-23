"""Receive/replay adapter for the candidate; owns no hardware and authorizes no flight."""
import math
from copy import deepcopy
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'ros2_ws/src/uav_coord_bridge'))

from mavlink_crc_stream import CrcResyncParser
from fusion_binary_protocol import MESSAGE_TYPE, TARGET, RESET_FIELDS, decode_fusion, decode_visual
from fusion_binary_record import AidNotReady
from ground_estimator_settlement import require_frozen_selector
import usb_combined_fusion_policy as combined


class FusionBinaryReceiver:
    def __init__(self):
        self.parser = CrcResyncParser()
        self.latest = {}
        self.received_at = {}
        self.fault = None
        self.crc_rejections = 0
        self.signature = None
        self.last_now = None
        self.not_ready_records = 0
        self.last_not_ready = None

    def feed(self, raw, now, *, identity_verified):
        if not math.isfinite(now) or self.last_now is not None and now < self.last_now:
            self.fault = 'host_clock_regression'
            return []
        self.last_now = now
        if not identity_verified:
            self.fault = 'identity_or_boot_session_lost'
            return []
        if self.fault:
            return []
        accepted = []
        for msg in self.parser.parse_buffer(raw):
            if msg.get_type() == 'BAD_DATA':
                self.crc_rejections += 'invalid MAVLink CRC' in msg.reason
                continue  # Discard only; no timestamp/state refreshed by damaged data.
            if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
                continue
            if msg.get_type() != 'V2_EXTENSION' or msg.message_type != MESSAGE_TYPE:
                continue
            if (msg.target_network, msg.target_system, msg.target_component) != (0, *TARGET):
                continue
            data = bytes(msg.payload)
            try:
                kind = 'fusion' if data[:4] == b'EVF2' else 'visual'
                row = decode_fusion(data) if kind == 'fusion' else decode_visual(data)
                previous = self.latest.get(kind)
                if previous:
                    delta = (row['sequence'] - previous['sequence']) & 0xffffffff
                    if delta == 0:
                        continue
                    if delta >= 0x80000000:
                        raise ValueError('sequence_regression')
                    keys = ('selector','flags','local','aid','height_aid','yaw_aid')
                    stamps = tuple(row[k]['timestamp'] for k in keys) if kind == 'fusion' else (row['timestamp'],)
                    old_stamps = tuple(previous[k]['timestamp'] for k in keys) if kind == 'fusion' else (previous['timestamp'],)
                    if kind == 'fusion' and self.signature is not None:
                        require_frozen_selector(row['selector'], self.signature)
                        if any(row['selector'][k] != previous['selector'][k]
                               for k in ('accel_device_id', 'gyro_device_id')):
                            raise ValueError('selected_imu_changed')
                        if any(row['local'][k] != previous['local'][k] for k in RESET_FIELDS):
                            raise ValueError('local_position_reset')
                    same_estimator = (kind != 'fusion' or
                        (row['selector']['primary_instance'],row['selector']['instance_changed_count']) ==
                        (previous['selector']['primary_instance'],previous['selector']['instance_changed_count']))
                    if same_estimator and any(a < b for a,b in zip(stamps,old_stamps)):
                        raise ValueError('source_clock_regression')
                    if stamps == old_stamps:
                        continue  # Cached publications never refresh state age.
                self.latest[kind], self.received_at[kind] = row, now
                accepted.append((kind, deepcopy(row)))
            except AidNotReady as exc:
                self.not_ready_records += 1
                self.last_not_ready = dict(at=now, fields=list(exc.fields), sequence=exc.row['sequence'])
                if self.signature is not None:
                    self.fault = 'binary_fusion_became_unavailable'
                    break
                # No sample, source time, selector or freshness is refreshed.
                # Startup stays bounded by the existing missing-evidence timeout.
                continue
            except (ValueError, RuntimeError, KeyError, TypeError) as exc:
                self.fault = str(exc)
                break
        return accepted

    def snapshot(self, kind, now, *, fc_now_us, identity_verified):
        if not math.isfinite(now) or self.last_now is not None and now < self.last_now:
            self.fault = 'host_clock_regression'
        if not identity_verified:
            self.fault = 'identity_or_boot_session_lost'
        self.last_now = now
        if self.fault:
            raise RuntimeError(self.fault)
        row = self.latest.get(kind)
        if row is None or not 0 <= now-self.received_at[kind] < 1:
            if row is not None:
                self.fault = 'binary_state_missing_or_stale'
            raise RuntimeError('binary_state_missing_or_stale')
        stamps = ([row[key]['timestamp'] for key in ('selector', 'flags', 'local', 'aid', 'height_aid', 'yaw_aid')]
                  if kind == 'fusion' else [row['timestamp']])
        if type(fc_now_us) is not int or any(not 0 < t <= fc_now_us or fc_now_us-t >= 2_000_000 for t in stamps):
            self.fault = 'binary_source_time_missing_stale_or_future'
            raise RuntimeError('binary_source_time_missing_stale_or_future')
        return deepcopy(row)

    def freeze(self, signature):
        if self.signature is not None or 'fusion' not in self.latest:
            raise RuntimeError('Single selector freeze after settlement required')
        require_frozen_selector(self.latest['fusion']['selector'], signature)
        self.signature = signature


def fusion_evidence(rows, yaw_comparisons):
    """Reuse genuine policy on decoded evidence; no fabricated heading proof."""
    if len(rows) != len(yaw_comparisons) or len(rows) < 2:
        return False
    primary = rows[0]['selector']['primary_instance']
    signature = (primary, rows[0]['selector']['instance_changed_count'])
    try:
        for row in rows:
            require_frozen_selector(row['selector'], signature)
            s = row['selector']
            if not s['healthy'][primary] or s['gyro_fault_detected'] or s['accel_fault_detected']:
                return False
            if any(row['local'][k] != rows[0]['local'][k] for k in RESET_FIELDS):
                return False
            if any(s[k] != rows[0]['selector'][k] for k in ('accel_device_id', 'gyro_device_id')):
                return False
        return combined.fusion_evidence_ok([
            {**row, 'yaw_comparison': yaw} for row, yaw in zip(rows, yaw_comparisons)], primary)
    except (KeyError, RuntimeError, TypeError, ValueError):
        return False
