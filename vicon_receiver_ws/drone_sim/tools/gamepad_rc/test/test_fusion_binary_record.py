import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import pytest
from fusion_binary_record import FLAGS, SIZE, encode, decode


def sample():
    row={'flags':dict(timestamp=100,**{k:False for k in FLAGS}),
         'selector':dict(timestamp=101,primary_instance=0,instances_available=1,
                         primary_healthy=True,gyro_fault_detected=False,accel_fault_detected=False),
         'local':dict(timestamp=102,xy_valid=True,z_valid=True)}
    for key in ('aid','height_aid','yaw_aid'):
        row[key]=dict(timestamp=103,timestamp_sample=99,time_last_fuse=98,
            estimator_instance=0,fusion_enabled=True,fused=True,innovation_rejected=False,
            test_ratio=[.5,.25] if key=='aid' else .5,
            observation_variance=[1.,2.] if key=='aid' else 1.)
    return row


def test_roundtrip():
    r=sample();b=encode(r)
    assert len(b)==SIZE and SIZE<=249
    assert decode(b)==r


@pytest.mark.parametrize('n',range(SIZE))
def test_truncated(n):
    with pytest.raises(ValueError):decode(encode(sample())[:n])


def test_unknown_and_trailing():
    b=encode(sample())
    for bad in (b'BAD!'+b[4:],b+b'\0'):
        with pytest.raises(ValueError):decode(bad)


def test_nonfinite():
    r=sample();r['yaw_aid']['test_ratio']=float('nan')
    with pytest.raises(ValueError):encode(r)


def test_preserves_unhealthy_evidence():
    r=sample();r['flags']['reject_yaw']=True;r['yaw_aid']['fused']=False
    assert decode(encode(r))==r
