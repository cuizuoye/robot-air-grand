"""Capture-only regressions; reuse synthetic fixtures, never open devices."""
from dataclasses import replace
import json
import math

import pytest
from pymavlink.dialects.v20 import common as m

from ev_delay_device_io import Endpoint
from preparation_authorization import require_no_pending_recovery
from test_ev_delay_joint_device import execute, Port, Source, IDENTITY
from test_ev_delay_recording_session import session
from test_joint_rx_recovery_delta import BytesPort


def test_dynamic_difference_is_recorded_not_a_synchronous_alignment_test():
    s, obs = session()
    assert s.begin(obs)
    # Fresh samples can differ during motion because the lag is being measured.
    assert s.gate.check(replace(obs, now_ns=obs.now_ns+1, tilt_difference_deg=12.0))
    assert not s.gate.fault


@pytest.mark.parametrize('value', [6., float('nan'), float('inf'), -1., 181.])
def test_stationary_start_still_rejects_bad_alignment(value):
    s, obs = session()
    assert not s.begin(replace(obs, tilt_difference_deg=value))
    assert s.gate.fault == 'source_FC_tilt_disagreement'


@pytest.mark.parametrize('fault', ['nan', 'range', 'stale', 'unlocked'])
def test_dynamic_phase_retains_validity_freshness_and_lock_guards(fault):
    s, obs = session()
    assert s.begin(obs)
    update = dict(now_ns=obs.now_ns+1, tilt_difference_deg=12.)
    if fault == 'nan': update['tilt_difference_deg'] = float('nan')
    elif fault == 'range': update['tilt_difference_deg'] = 181.
    elif fault == 'stale': update['attitude_received_ns'] = obs.now_ns-500_000_000
    else: s.gate.lock.stop()
    assert not s.gate.check(replace(obs, **update))


@pytest.mark.parametrize('phase', ['rate_setup', 'capture'])
def test_usb_failure_does_not_block_healthy_trs_restore(tmp_path, monkeypatch, phase):
    original = Port.read
    def read(self, count):
        s = self.sim
        active = (len(s.ports) == 2 and s.ports[1].interval > 0 if phase == 'rate_setup'
                  else s.first_pose and s.now-s.first_pose > 600_000_000)
        if self.route == 0 and active:
            raise OSError('SYNTHETIC USB read failed')
        return original(self, count)
    monkeypatch.setattr(Port, 'read', read)
    sim, code, report = execute(tmp_path, monkeypatch, physical_overrides=dict(
        confirm_props_off_battery_disconnected_usb_only=False,
        confirm_props_installed_battery_disconnected_usb_only=True))
    assert code == 2 and not report['hardware_capture_completed']
    assert report['owner']['rates']['restoration_verified'], report['owner']['cleanup_errors']
    assert sim.ports[1].interval == 0
    if phase == 'capture':
        assert report['lifecycle']['keep_USB_power_required']
        assert not report['lifecycle']['logger'].get('file_closed_verified')
    else:
        assert sim.on_count == 0 and sim.first_pose is None
    assert 'SYNTHETIC USB' in report['pump_failure']
    assert all(not p.is_open for p in sim.ports)
    require_no_pending_recovery(tmp_path)


def startup(tmp_path, *, joint=True):
    now = [10_000_000_000]
    port = BytesPort()
    ep = Endpoint(port, 213, IDENTITY, clock=lambda: now[0],
        joint_rx_evidence=tmp_path/'rx.jsonl' if joint else None)
    fc = m.MAVLink(None, srcSystem=1, srcComponent=1)
    good = bytes(fc.heartbeat_encode(2, 12, 0, 0, 0).pack(fc))
    bad = bytearray(good); bad[-1] ^= 1
    return now, port, ep, good, bytes(bad)


def test_initial_bad_crc_can_resync_without_accepting_bad_state(tmp_path):
    now, port, ep, good, bad = startup(tmp_path)
    try:
        port.data = bad
        ep.poll()
        assert ep.heartbeat_ns is None and ep.last_valid_fc_ns is None
        assert not ep.safe()
        now[0] += 50_000_000
        port.data = good; ep.poll()
        assert ep.heartbeat_ns == now[0] and ep.sync.first_valid_ns == now[0]
        assert ep.evidence.identity is None  # heartbeat is not identity approval
        assert ep.parser.summary()['repaired_bytes'] == 0
    finally: ep.close_rx_evidence()
    rows = [json.loads(x) for x in (tmp_path/'rx.jsonl').read_text().splitlines()]
    assert b''.join(bytes.fromhex(x['wire_hex']) for x in rows if x['kind']=='serial_read') == bad+good


@pytest.mark.parametrize('failure', ['timeout', 'budget'])
def test_startup_corruption_has_original_time_and_byte_bounds(tmp_path, failure):
    now, port, ep, good, bad = startup(tmp_path)
    try:
        if failure == 'timeout':
            port.data = bad; ep.poll()
            now[0] += 2_000_000_000; port.data = good
        else: port.data = bad*300
        with pytest.raises(RuntimeError): ep.poll()
        assert ep.heartbeat_ns is None and not ep.safe()
    finally: ep.close_rx_evidence()


def test_startup_other_callers_keep_their_original_strict_policy(tmp_path):
    now, port, ep, good, bad = startup(tmp_path, joint=False)
    port.data = bad+good
    with pytest.raises(RuntimeError, match='CRC/framing'): ep.poll()
    assert ep.heartbeat_ns is None


def test_full_window_with_delayed_attitude_and_real_tilt(tmp_path, monkeypatch):
    original_append, original_poll = Port.append, Source.poll
    def angle(sim, lag=0):
        if sim.first_pose is None: return 0.
        t = max(0., (sim.now-sim.first_pose)/1e9-lag)
        return math.radians(15)*math.sin(2*math.pi*t)
    def append(self, msg):
        if msg.get_type() == 'ATTITUDE_QUATERNION':
            a = angle(self.sim, .12)
            msg.q1, msg.q2, msg.q3, msg.q4 = math.cos(a/2), math.sin(a/2), 0., 0.
            msg.time_boot_ms -= 120
        return original_append(self, msg)
    def poll(self):
        callback = self.on_frame
        def deliver(row, at):
            a = angle(self.sim)
            # Source is FLU, transformed body is FRD by existing mount logic.
            row['world_pose'][3:] = [math.sin(a/2), 0., 0., math.cos(a/2)]
            callback(row, at)
        self.on_frame = deliver
        try: return original_poll(self)
        finally: self.on_frame = callback
    monkeypatch.setattr(Port, 'append', append)
    monkeypatch.setattr(Source, 'poll', poll)
    sim, code, report = execute(tmp_path, monkeypatch)
    assert code == 0, (report['errors'], report.get('lifecycle'))
    assert sim.on_count == sim.off_count == 1
    assert sim.last_pose-sim.first_pose > 14_000_000_000
    assert report['owner']['dynamic_tilt_diagnostic']['max_difference_deg'] > 5
    assert 'first_failed_tilt_pair' not in report['owner']
    assert not report['calibration_approved'] and not report['flight_authorized']
    trace = json.loads(next(tmp_path.glob('joint_ev_delay_*/capture_raw.json')).read_text())
    w = trace['capture_metadata']['motion_window']
    assert w['ended_ns'] >= w['deadline_ns']
    assert w['deadline_ns']-w['started_ns'] == 15_000_000_000
    assert report['owner']['rates']['restoration_verified']
