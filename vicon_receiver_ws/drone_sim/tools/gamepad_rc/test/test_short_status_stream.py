"""Synthetic mixed-rate integration; not radio or EKF measurements."""
from dataclasses import replace
import pytest
from pymavlink.dialects.v20 import common as m
from test_trs_powered_fusion import observation
from test_lock_clock_probe import reply
from output_lock_status import MAGIC
from static_lock_gate import StaticLockGate
from trs_powered_fusion_policy import ShortStatusStreamGate


def fresh(gate, cycle):
    # FC is 4 seconds ahead; bounded 40 ms response. All messages traverse
    # the real MAVLink encoder/decoder before entering the candidate gate.
    sent = 1_000_000_000 + cycle * 200_000_000
    token = gate.lock_gate.clock.request(sent)
    assert gate.lock_gate.ingest(reply(remote=sent+4_000_000_000, token=token), sent+40_000_000)
    enc = m.MAVLink(None, srcSystem=1, srcComponent=1)
    packet = enc.named_value_int_encode(
        (sent+4_000_000_000)//1_000_000, b'OUTLOCK1', MAGIC|5).pack(enc)
    msg = m.MAVLink(None).parse_buffer(packet)[0]
    assert gate.lock_gate.ingest(msg, sent+45_000_000)


def initialized():
    gate = ShortStatusStreamGate(StaticLockGate())
    fresh(gate, 0)
    gate.start(observation(0), now_ns=1_050_000_000)
    return gate


def test_32hz_poses_with_5hz_short_state_and_clock_for_ten_seconds():
    gate = initialized()
    cycle = 0
    for index in range(320):
        t = index / 32
        latest = int(t / .2)
        if latest > cycle:
            fresh(gate, latest)
            cycle = latest
        assert gate.check(observation(t), now_ns=1_050_000_000+round(t*1e9))
        gate.actual_send(observation(t), now_ns=1_050_000_000+round(t*1e9))
    assert gate.sent == 320
    gate.stop()
    assert not gate.check(observation(10), now_ns=11_050_000_000)


@pytest.mark.parametrize('name,value', [
    ('identity_fresh', False), ('usb_absent', False), ('boot_unchanged', False),
    ('source_age', .151), ('attitude_age', .501), ('stationary', False),
    ('tilt_difference_deg', 5.01), ('parameters_fresh_match', False),
    ('gamepad_same_device', False), ('home_pressed', True)])
def test_other_guards_still_latch_and_recovery_cannot_resume(name, value):
    gate = initialized()
    assert not gate.check(replace(observation(.01), **{name:value}), now_ns=1_060_000_000)
    fresh(gate, 1)
    assert not gate.check(observation(.2), now_ns=1_250_000_000)
    with pytest.raises(ValueError):
        gate.start(observation(.2), now_ns=1_250_000_000)


def test_stale_text_cannot_override_short_state_or_clock_expiry():
    gate = initialized()
    # The old textual observation still says locked and fresh.
    assert not gate.check(observation(.5), now_ns=1_550_000_000)
    assert gate.first_failure['short_lock']['reasons']
    fresh(gate, 3)
    assert not gate.check(observation(.6), now_ns=1_650_000_000)


def test_old_text_missing_does_not_block_valid_short_state():
    gate = initialized()
    assert gate.check(replace(observation(.01), disarmed_locked=False, lock_age=float('inf')),
                      now_ns=1_060_000_000)


def test_invalid_time_domain_stops_both_gates():
    gate = initialized()
    with pytest.raises(ValueError):
        gate.check(observation(.01), now_ns=1.06)
    assert gate.blocked and gate.lock_gate.fault


def test_fusion_deadline_and_packet_gap_are_not_relaxed():
    gate = initialized()
    gate.begin_fusion(observation(.01), now_ns=1_060_000_000)
    gate.actual_send(observation(.01), now_ns=1_060_000_000)
    fresh(gate, 1)
    assert not gate.check(observation(.2), now_ns=1_250_000_000)
    assert 'send_gap_exceeded' in gate.first_failure['reasons']


def test_missing_other_prerequisite_does_not_consume_lock_start():
    gate = ShortStatusStreamGate(StaticLockGate())
    fresh(gate, 0)
    with pytest.raises(ValueError):
        gate.start(replace(observation(), stationary=False), now_ns=1_050_000_000)
    assert not gate.lock_gate.started
    gate.start(observation(), now_ns=1_050_000_000)


def test_send_boundary_rechecks_after_earlier_good_tick():
    gate = initialized()
    assert gate.check(observation(.01), now_ns=1_060_000_000)
    with pytest.raises(ValueError):
        gate.actual_send(observation(.5), now_ns=1_550_000_000)
    assert gate.sent == 0 and gate.blocked and gate.lock_gate.fault


def test_recorded_uncertain_row_is_not_retroactive_live_permission():
    # Recorded row 77 of trs_bounded_clock_deferred_01.json (2026-10-19).
    # Its successful later audit must NOT rewrite the live decision at arrival.
    value = StaticLockGate()
    value.started = True
    value.status.sample = dict(stamp=2508659, bits=1280508229,
                               received=16.775618949999625)
    value.clock.reference = dict(remote_ns=2508523079000, received_ns=16645814465,
                                lower_ns=2491877264535, upper_ns=2491927903165,
                                rtt_ns=50638630)
    detail = value.assess(16775618949, identity_verified=True)
    assert not detail['valid']
    assert detail['source_age_bounds_ms'][0] < 0
    assert value.fault == 'static_stream_lock_lost:source_age_uncertain'
    value.clock.reference = dict(remote_ns=2508721839000, received_ns=16840739729,
                                lower_ns=2491881099271, upper_ns=2491925171770,
                                rtt_ns=44072499)
    later = value.assess(16840739729, identity_verified=True)
    assert later['source_age_bounds_ms'][0] > 0
    assert not later['valid']  # No automatic restart after a live failure.


def test_recorded_65ms_gap_keeps_original_fresh_evidence_not_pending_row():
    value = StaticLockGate()
    value.status.sample = dict(stamp=2508153, bits=1280508229,
                               received=16.58458303400039)
    value.clock.reference = dict(remote_ns=2508122502000, received_ns=16235418053,
                                lower_ns=2491887083947, upper_ns=2491929193327,
                                rtt_ns=42109380)
    assert value.begin(16584583034, identity_verified=True)
    value.status.sample = dict(stamp=2508659, bits=1280508229,
                               received=16.775618949999625)
    value.clock.reference = dict(remote_ns=2508523079000, received_ns=16645814465,
                                lower_ns=2491877264535, upper_ns=2491927903165,
                                rtt_ns=50638630)
    pending = value.assess(16775618949, identity_verified=True)
    assert pending['valid']
    assert pending['evidence_selection'] == 'previous_still_fresh'
    assert pending['evidence_sample']['stamp'] == 2508153
    assert not pending['latest_pending']['valid']
    assert value.proven_sample['received'] == 16.58458303400039
    value.clock.reference = dict(remote_ns=2508721839000, received_ns=16840739729,
                                lower_ns=2491881099271, upper_ns=2491925171770,
                                rtt_ns=44072499)
    assert value.assess(16840739729, identity_verified=True)['valid']
    assert value.proven_sample['stamp'] == 2508659
    assert not value.fault
