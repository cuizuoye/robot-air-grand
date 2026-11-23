"""The same existing settlement rule now serves revision-static and ground audits."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from trs_static_fusion_bench import settle_estimator_before_audit
from ground_estimator_settlement import require_frozen_selector


def selected(instance=0,count=1):
    return dict(primary_instance=instance,instance_changed_count=count,instances_available=2,
        healthy=[True,True]+[False]*7,gyro_fault_detected=False,accel_fault_detected=False)


def test_shared_bounded_initialization_wait_handles_switch_without_control_commands():
    clock=[0.];calls=[]
    def listener(topic,**kw):
        calls.append(topic)
        return selected(1,2) if clock[0]>=.5 else selected()
    def pump():clock[0]+=.05
    gate=settle_estimator_before_audit(listener,pump,lambda:clock[0],lambda:0)
    assert 2<=clock[0]<=6 and gate.frozen_signature==(1,2)
    assert set(calls)=={'actuator_armed','selector'}
    with pytest.raises(RuntimeError):require_frozen_selector(selected(0,3),gate.frozen_signature)


def test_shared_wait_cannot_be_used_after_target_stream_started():
    with pytest.raises(ValueError,match='before targets'):
        settle_estimator_before_audit(lambda *a,**kw:selected(),lambda:None,lambda:0.,lambda:1)


def test_shared_wait_keeps_callers_deadline_errors_not_an_automatic_retry():
    def pump():raise RuntimeError('original stream deadline')
    with pytest.raises(RuntimeError,match='original stream deadline'):
        settle_estimator_before_audit(lambda *a,**kw:selected(),pump,lambda:0.,lambda:0)
