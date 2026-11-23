import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from diagnostic_transaction import DiagnosticTransaction

COMMAND = b'listener actuator_armed -n 1\n'
REPLY = COMMAND.decode() + 'TOPIC: actuator_armed\nfields\n\nnsh> '


def test_only_complete_clean_current_echo_and_prompt_can_be_accepted():
    d = DiagnosticTransaction()
    d.receive(REPLY, 0)  # Unsolicited output cannot start a transaction.
    d.begin(COMMAND, 1)
    d.receive('fields\n\nnsh> ', 1.1)
    assert d.ready_text() is None
    with pytest.raises(RuntimeError):d.accept()
    d.receive('\n' + REPLY, 1.2)
    assert d.ready_text() is not None
    d.accept()
    assert d.idle and d.ready_text() is None


def test_corrupted_reply_is_discarded_even_if_later_complete():
    d = DiagnosticTransaction();d.begin(COMMAND, 0)
    d.receive(COMMAND.decode() + 'partial', .1)
    d.damage('crc', .2)
    with pytest.raises(RuntimeError):d.begin(COMMAND, .21)
    d.receive('\n' + REPLY, .3)
    assert d.idle and d.ready_text() is None
    assert len(d.failures) == 1
    d.begin(COMMAND, .4)
    assert d.ready_text() is None
    d.receive(REPLY, .5)
    assert d.ready_text() is not None
    d.accept()


def test_corruption_after_complete_reply_still_invalidates_unaccepted_reply():
    d = DiagnosticTransaction();d.begin(COMMAND, 0)
    d.receive(REPLY, .1);d.damage('bad_prefix', .1)
    assert d.idle and d.ready_text() is None


def test_missing_prompt_permanently_blocks_new_commands():
    d = DiagnosticTransaction();d.begin(COMMAND, 0)
    d.damage('crc', .1);d.tick(3.2)
    assert d.phase == 'blocked'
    d.receive(REPLY, 3.3)
    with pytest.raises(RuntimeError):d.begin(COMMAND, 3.4)
    assert d.ready_text() is None


def test_timeout_then_late_reply_cannot_become_valid_evidence():
    d = DiagnosticTransaction();d.begin(COMMAND, 0)
    d.receive(REPLY, 3.1)
    assert d.idle and d.ready_text() is None
    assert d.failures[0]['reason'] == 'diagnostic_reply_timeout'


def test_poll_rate_limit_and_no_poll_after_clean_completion():
    d = DiagnosticTransaction();d.begin(COMMAND, 0)
    assert not d.poll_due(.24)
    assert d.poll_due(.25) and not d.poll_due(.26)
    d.damage('crc', .3)
    assert d.poll_due(.3) and not d.poll_due(.4)
    d.receive('\nnsh> ', .5)
    assert d.idle and not d.poll_due(.8)


def test_overflow_cannot_be_certified_or_reopen_without_boundary():
    d = DiagnosticTransaction();d.begin(COMMAND, 0)
    d.receive('x' * (d.max_buffer + 1), .1)
    assert d.phase == 'draining' and d.ready_text() is None
    assert len(d.buffer) <= d.max_buffer
    d.receive('\nnsh> ', .2)
    assert d.idle and d.ready_text() is None


def test_fast_lock_poll_is_bounded_and_still_requires_complete_reply():
    d = DiagnosticTransaction();d.poll_interval = .1;d.begin(COMMAND, 0)
    assert not d.poll_due(.099)
    assert d.poll_due(.1)
    assert not d.poll_due(.101)
    d.receive(COMMAND.decode() + 'partial', .15)
    assert d.ready_text() is None
    with pytest.raises(RuntimeError):d.accept()
    assert d.poll_due(.201)
    d.damage('crc', .21)
    assert d.ready_text() is None
    with pytest.raises(RuntimeError):d.begin(COMMAND, .22)
    d.receive('\nnsh> ', .25)
    assert d.idle and not d.poll_due(.4)


def test_fields_before_current_echo_are_not_exposed_to_parsers():
    d = DiagnosticTransaction();d.begin(COMMAND, 0)
    d.receive('TOPIC: actuator_armed\nold_fields\n\nnsh> \n' + REPLY, .1)
    assert 'old_fields' not in d.ready_text()
    assert 'fields' in d.ready_text()


def test_bad_data_outside_transaction_does_not_create_or_poison_one():
    d = DiagnosticTransaction();d.damage('unrelated_bad_frame', 0)
    assert d.idle and not d.failures
    d.begin(COMMAND, .1);d.receive(REPLY, .2)
    assert d.ready_text() is not None
