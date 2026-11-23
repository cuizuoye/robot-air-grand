"""Characterize the candidate clock estimator; NOT flight acceptance tests.

Passing these tests demonstrates limitations, not that the estimator is safe.
No serial I/O, parameter writes, or actuation.
"""
import pytest

from test_output_lock_status import feed, wire, OutputLockStatus


@pytest.mark.parametrize('clock_ms,clock_at,source_ms,received,expected', [
    (553534, 6.566374590, 553612, 6.637844730, -6.529860),
    (562139, 15.175894850, 562216, 15.239630842, -13.264008),
])
def test_recorded_negative_ages_reproduced(clock_ms, clock_at, source_ms, received, expected):
    state = OutputLockStatus()
    state.update_clock(clock_ms, clock_at)
    feed(state, wire(stamp=source_ms), received)
    result = state.assess(received, identity_verified=True)
    assert result['source_age_ms'] == pytest.approx(expected, abs=1e-5)
    assert result['reasons'] == ['source_future']


def test_host_wait_alone_can_clear_future_without_new_clock_evidence():
    state = OutputLockStatus()
    state.update_clock(10000, 1.0)
    feed(state, wire(stamp=10010), 1.0)
    assert not state.locked_disarmed(1.0, identity_verified=True)
    # Characterizes the current flaw: elapsed host time is not fresh FC proof.
    assert state.locked_disarmed(1.02, identity_verified=True)


def test_common_transport_backlog_is_not_measured_by_arrival_freshness():
    state = OutputLockStatus()
    # Synthetic FC current time is 13 s, but both packets were queued at 10 s.
    # The present API cannot observe that shared three-second transport delay.
    state.update_clock(10000, 5.0)
    feed(state, wire(stamp=10000), 5.0)
    result = state.assess(5.0, identity_verified=True)
    assert result['source_age_ms'] == 0
    assert result['valid']  # Known limitation, not a safe desired contract.
