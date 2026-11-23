"""Single-owner input continuity tests; no ROS, serial, or flight execution."""
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mission_preparation_runtime import PreparationRuntime


def seed():
    runtime=PreparationRuntime('test-revision',body_origin_in_rigid=[0,0,0],
                               body_orientation_in_rigid=[0,0,0,1])
    # Isolate input handling from unrelated preparation telemetry requirements.
    runtime._owner_can_advance=lambda:False
    runtime.turn(1.,now_ros_ns=101_000_000_000,
                 pose=(101_000_000_000,'map',[0,0,0],[0,0,0,1]))
    runtime.prepare_requested=True
    return runtime


def test_returning_pose_does_not_hide_expired_source_or_resume():
    runtime=seed()
    runtime.turn(1.1,now_ros_ns=101_100_000_000)
    runtime.turn(1.16,now_ros_ns=101_160_000_000,
                 pose=(101_160_000_000,'map',[0,0,0],[0,0,0,1]))
    assert runtime.failure=='vicon_source_gap'
    runtime.turn(1.17,now_ros_ns=101_170_000_000,
                 pose=(101_170_000_000,'map',[0,0,0],[0,0,0,1]))
    assert runtime.failure=='vicon_source_gap'
    assert runtime.coordinator.core.phase=='BLOCKED'


@pytest.mark.parametrize('stamp,frame,position,reason',[
    (100_990_000_000,'map',[0,0,0],'vicon_source_clock_regression'),
    (101_010_000_000,'changed',[0,0,0],'vicon_frame_mismatch'),
    (101_010_000_000,'map',[1,0,0],'vicon_implausible_position_jump'),
])
def test_epoch_or_position_change_latches(stamp,frame,position,reason):
    runtime=seed()
    runtime.turn(1.01,now_ros_ns=101_010_000_000,
                 pose=(stamp,frame,position,[0,0,0,1]))
    assert runtime.failure==reason
    assert runtime.inputs.pose_stamp==101_000_000_000


def test_duplicate_does_not_refresh_but_does_not_immediately_abort():
    runtime=seed()
    runtime.turn(1.01,now_ros_ns=101_010_000_000,
                 pose=(101_000_000_000,'map',[0,0,0],[0,0,0,1]))
    assert runtime.failure is None
    assert runtime.inputs.pose_at==1.
