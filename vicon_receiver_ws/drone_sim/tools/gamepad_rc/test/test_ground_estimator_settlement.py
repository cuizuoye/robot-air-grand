from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ground_estimator_settlement import GroundEstimatorSettlement as Gate, require_frozen_selector


def selector(instance=0, count=1):
    return dict(primary_instance=instance, instance_changed_count=count,
                instances_available=2, healthy=[True, True]+[False]*7,
                gyro_fault_detected=False, accel_fault_detected=False)


def test_healthy_pre_control_switch_requires_new_stability_period():
    gate = Gate(0)
    assert not gate.observe(selector(), .1)
    assert not gate.observe(selector(1, 2), 1.5)
    assert not gate.observe(selector(1, 2), 2)
    assert gate.observe(selector(1, 2), 3)
    assert gate.frozen_signature == (1, 2)
    require_frozen_selector(selector(1, 2), gate.frozen_signature)


@pytest.mark.parametrize('changed', [selector(0, 3), selector(1, 4)])
def test_after_freeze_switch_or_switch_away_and_back_is_always_fatal(changed):
    gate = Gate(0);gate.observe(selector(1, 2), 0)
    assert gate.observe(selector(1, 2), 2)
    with pytest.raises(ValueError, match='after freeze'):
        gate.observe(changed, 2.1)
    with pytest.raises(RuntimeError, match='after pre-control freeze'):
        require_frozen_selector(changed, gate.frozen_signature)


def test_never_settle_after_any_target_was_sent():
    with pytest.raises(ValueError, match='before targets'):
        Gate(0).observe(selector(), 0, targets_sent=1)


@pytest.mark.parametrize('change', [{'healthy': [False]*9}, {'gyro_fault_detected': True},
    {'accel_fault_detected': True}, {'primary_instance': 5}])
def test_health_checks_remain_strict(change):
    with pytest.raises(ValueError, match='explicitly healthy'):
        Gate(0).observe({**selector(), **change}, 0)


def test_repeated_switches_or_timeout_dont_become_a_retry_loop():
    gate = Gate(0)
    gate.observe(selector(0, 1), 0);gate.observe(selector(1, 2), .5)
    gate.observe(selector(0, 3), 1)
    with pytest.raises(ValueError, match='Repeated'):
        gate.observe(selector(1, 4), 1.5)
    with pytest.raises(ValueError, match='bounded'):
        Gate(0).observe(selector(), 6.1)
