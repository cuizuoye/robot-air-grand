from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from actuator_query import ActuatorQuery, terminal_prompt_seen


def reply(stamp=1_000_000, locked=False, armed=False):
    return ('nsh> listener actuator_armed -n 1\r\nTOPIC: actuator_armed\r\n'
            f' timestamp: {stamp}\r\n armed: {armed}\r\n lockdown: {locked}\r\n'
            ' manual_lockdown: False\r\n force_failsafe: False\r\n\r\nnsh> ')


def test_fragmented_reply_waits_for_complete_echo_and_prompt():
    q = ActuatorQuery()
    assert q.action(0, True) == 'request'
    text = reply()
    assert q.receive(text[:-5], .1, 1000) is None
    row = q.receive(text[-5:], .2, 1000)
    assert row['lockdown'] is False
    assert q.live_lockdown(.3) is False
    assert q.live_lockdown(2.201) is None
    assert not q.failures


def test_no_commands_or_polls_without_fresh_identity():
    q = ActuatorQuery()
    assert q.action(0, False) is None
    assert q.action(0, True) == 'request'
    assert q.action(1, False) is None
    assert q.action(1, True) == 'poll'
    assert q.action(1.1, True) is None


def test_timeout_does_not_throw_or_start_overlapping_command():
    q = ActuatorQuery()
    q.action(0, True)
    q.receive('nsh> listener actuator_armed -n 1\nTOPIC: actuator_armed\n', 1, 1000)
    assert q.action(6.01, True) == 'poll'
    assert q.fault == 'actuator_reply_timeout'
    assert q.pending_at is None and q.draining
    assert q.live_lockdown(6.01) is None
    assert q.action(7, True) == 'poll'
    assert q.action(12.02, True) is None
    assert q.disabled and not q.draining
    assert q.action(100, True) is None


def test_late_complete_reply_is_discarded_even_before_timeout_tick():
    q = ActuatorQuery()
    q.action(0, True)
    assert q.receive(reply(7_000_000, True), 7, 7000) is None
    assert q.last_snapshot is None
    assert q.fault == 'actuator_reply_timeout'
    assert not q.draining
    assert q.action(7.2, True) is None
    assert q.action(7.5, True) == 'request'
    row = q.receive(reply(8_000_000, True), 8, 8000)
    assert row['requested_at'] == 7.5
    assert q.live_lockdown(8) is True
    assert q.fault == 'actuator_reply_timeout'  # Recovery never erases history.


def test_corruption_drain_then_retry_never_joins_different_replies():
    q = ActuatorQuery()
    q.action(0, True)
    q.receive(reply()[:80], .1, 1000)
    q.damaged_packet(.2)
    assert q.receive(reply(1_000_000, True)[80:], .3, 1000) is None
    assert q.last_snapshot is None
    assert q.action(.8, True) == 'request'
    assert q.receive(reply(2_000_000, True), 1, 2000)['lockdown'] is True
    assert q.fault == 'actuator_reply_damaged_packet'
    assert 'TOPIC:' in q.failures[0]['incomplete_reply']


def test_retry_budget_is_not_reset_by_successful_recovery():
    q = ActuatorQuery()
    now = 0
    for attempt in range(q.max_failures):
        assert q.action(now, True) == 'request'
        q.receive('listener actuator_armed -n 1\nmissing fields\nnsh> ', now+.1, 1000)
        if attempt == 0:
            assert q.action(now+.7, True) == 'request'
            assert q.receive(reply(), now+.8, 1000)
            assert q.fault == 'actuator_reply_malformed'
            now += 3
        now += 1
    assert q.disabled and len(q.failures) == 3
    assert q.action(now, True) is None
    assert q.receive(reply(), now, 1000) is None


def test_old_observation_becomes_unknown_immediately_on_new_fault():
    q = ActuatorQuery()
    q.action(0, True)
    q.receive(reply(1_000_000, True), .1, 1000)
    assert q.live_lockdown(.2) is True
    q.next_request = .2
    q.action(.2, True)
    q.damaged_packet(.3)
    assert q.live_lockdown(.3) is None


@pytest.mark.parametrize('stamp,boot', [(1_000_000, 4000), (4_000_000, 1000), (1_000_000, None)])
def test_two_second_timestamp_guard_not_relaxed(stamp, boot):
    q = ActuatorQuery()
    q.action(0, True)
    assert q.receive(reply(stamp), .1, boot) is None
    assert q.fault == 'actuator_reply_stale_or_repeated'
    assert q.live_lockdown(.1) is None


def test_identical_snapshot_cannot_be_relabelled_as_new_query():
    q = ActuatorQuery()
    q.action(0, True)
    q.receive(reply(), .1, 1000)
    q.action(2.2, True)
    assert q.receive(reply(), 2.3, 1000) is None
    assert q.fault == 'actuator_reply_stale_or_repeated'


def test_unsafe_snapshot_remains_fatal_not_a_recoverable_link_error():
    q = ActuatorQuery()
    q.action(0, True)
    with pytest.raises(RuntimeError, match='Unsafe actuator'):
        q.receive(reply(armed=True), .1, 1000)


def test_oversize_reply_is_bounded_and_never_a_snapshot():
    q = ActuatorQuery()
    q.action(0, True)
    assert q.receive('x'*20000, .1, 1000) is None
    assert len(q.buffer) <= q.max_buffer
    assert q.fault == 'actuator_reply_overflow'
    assert q.receive('\nnsh> ', .2, 1000) is None
    assert not q.draining


def test_unsolicited_output_and_prompt_are_not_observations():
    q = ActuatorQuery()
    assert q.receive(reply(), 0, 1000) is None
    assert q.last_snapshot is None
    assert terminal_prompt_seen('\r\nnsh> \x1b[K')
    assert not terminal_prompt_seen('nsh> queued output')


def test_damaged_packet_before_any_query_does_not_invent_a_fault():
    q = ActuatorQuery()
    q.damaged_packet(0)
    assert not q.failures


def test_drain_timeout_cannot_be_undone_by_a_late_prompt():
    q = ActuatorQuery()
    q.action(0, True)
    q.action(6.1, True)
    assert q.draining
    assert q.receive(reply(13_000_000), 13, 13000) is None
    assert q.disabled
    assert q.action(14, True) is None


def test_complete_fields_without_command_echo_are_not_a_new_observation():
    q = ActuatorQuery()
    q.action(0, True)
    text = reply().split('TOPIC:')[1]
    assert q.receive('TOPIC:'+text, .1, 1000) is None
    q.action(6.1, True)
    assert q.last_snapshot is None
    assert q.fault == 'actuator_reply_timeout'


def test_source_age_is_not_reset_to_zero_by_pc_arrival():
    q = ActuatorQuery()
    q.action(0, True)
    q.receive(reply(1_000_000, True), .1, 2500)
    assert q.live_lockdown(.2) is True
    assert q.live_lockdown(.7) is None  # 1.5s at arrival + 0.6s elapsed.


def test_old_fields_before_current_echo_cannot_supply_missing_current_fields():
    q = ActuatorQuery();q.action(0, True)
    old = reply(1_000_000, True).split('TOPIC:')[1].split('nsh>')[0]
    text = 'TOPIC:' + old + 'listener actuator_armed -n 1\nmissing current fields\n\nnsh> '
    assert q.receive(text, .1, 1000) is None
    assert q.last_snapshot is None and q.fault == 'actuator_reply_malformed'


def test_old_armed_state_before_current_echo_is_not_current_actuator_state():
    q = ActuatorQuery();q.action(0, True)
    old = reply(900_000, True, armed=True).split('TOPIC:')[1].split('nsh>')[0]
    row = q.receive('TOPIC:' + old + reply(1_000_000, False), .1, 1000)
    assert row['armed'] is False and row['lockdown'] is False
