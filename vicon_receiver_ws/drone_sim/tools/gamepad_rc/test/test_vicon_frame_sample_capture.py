"""New FrameSample-only entry glue: pure offline rows; no ROS initialization."""
from copy import deepcopy
import importlib.util
from pathlib import Path
import pytest

from vicon_frame_sample_capture import analyze

def rows():
    return [dict(frame=100+i,sequence=1+i,sdk_hz=100.,subject='MircoAir',segment='MircoAir',
        sdk_pose=[1.,2.,3.,0.,0.,0.,1.],world_pose=[1.,2.,3.,0.,0.,0.,1.],
        sdk_ok=True,occluded=False,pose_published=True,epoch=0,queue_dropped=0,
        missing_frames=0,sdk_begin_ns=1_000_000_000+i*10_000_000,
        sdk_end_ns=1_001_000_000+i*10_000_000,callback_ns=1_002_000_000+i*10_000_000,
        ros_received_ns=2_001_000_000+i*10_000_000,ros_stamp_ns=2_001_100_000+i*10_000_000,
        callback_ros_ns=2_002_000_000+i*10_000_000,processing_ns=100_000,
        exposure_time_known=False) for i in range(1501)]

def test_good_pure_rows_never_create_authority():
    result=analyze(rows())
    assert result['passed'] and result['valid']==1501
    assert result['actual_host_observed_rate_hz']==100
    assert result['source_filtered_SDK_duplicate_count'] is None
    assert not result['flight_authorized'] and not result['EV_DELAY_approved']

@pytest.mark.parametrize('field,value,reason',[
    ('occluded',True,'invalid_occluded_or_unpublished_frame'),
    ('queue_dropped',1,'queue_or_RAM_overflow'),
    ('subject','car01','rigid_body_mismatch'),
    ('sdk_pose',[0.,0.,0.,0.,0.,0.,0.],'invalid_occluded_or_unpublished_frame'),
    ('epoch',1,'SDK_epoch_changed'),
    ('exposure_time_known',True,'unverified_exposure_timestamp_claim'),
])
def test_bad_sample_refused(field,value,reason):
    data=rows(); data[40][field]=value
    result=analyze(data)
    assert not result['passed'] and reason in result['reasons']

def test_missing_and_duplicate_frames_are_not_hidden():
    data=rows(); data[40]['frame']-=1
    result=analyze(data)
    assert result['received_duplicate_frames']==1
    assert result['inferred_missing_frame_numbers']==1
    assert not result['passed']

def test_queue_overflow_fault_and_short_window_refused():
    assert not analyze(rows(),ram_dropped=1)['passed']
    assert not analyze(rows(),adapter_fault='source_failed')['passed']
    assert not analyze(rows()[:100])['passed']
    assert not analyze([])['passed']

def test_timestamp_regression_refused():
    data=rows(); data[40]['callback_ns']=data[39]['callback_ns']
    assert not analyze(data)['passed']

def test_entry_does_not_import_or_open_hardware_on_import(monkeypatch):
    import subprocess
    monkeypatch.setattr(subprocess,'Popen',lambda *a,**kw:pytest.fail('live child during offline import'))
    spec=importlib.util.spec_from_file_location('vicon_frame_entry_offline',Path(__file__).parents[1]/'vicon_frame_sample_capture.py')
    spec.loader.exec_module(importlib.util.module_from_spec(spec))
