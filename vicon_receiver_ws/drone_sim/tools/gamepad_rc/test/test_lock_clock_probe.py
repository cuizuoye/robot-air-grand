from pathlib import Path
import sys
import pytest
from pymavlink.dialects.v20 import common as m
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from lock_clock_probe import LockClockProbe
from output_lock_status import MAGIC
from trs_message_interval_trial import allowed


def reply(remote=5_000_000_000,token=1_000_000_000,source=(1,1)):
    encoder=m.MAVLink(None,srcSystem=source[0],srcComponent=source[1])
    packet=encoder.timesync_encode(remote,token).pack(encoder)
    return m.MAVLink(None).parse_buffer(packet)[0]


def ready():
    clock=LockClockProbe()
    assert clock.request(1_000_000_000)==1_000_000_000
    assert clock.ingest(reply(),1_040_000_000)
    return clock


def sample(stamp=5005,bits=MAGIC|5):
    return dict(stamp=stamp,bits=bits,received=1.05)


def test_bounds_include_asymmetry_drift_and_quantization():
    result=ready().assess(sample(),1_050_000_000,identity_verified=True)
    assert result['valid']
    assert result['source_age_bounds_ms']==pytest.approx([3.95,45.05])


def test_old_sample_is_not_refreshed_by_recent_arrival():
    result=ready().assess(sample(2000),1_050_000_000,identity_verified=True)
    assert not result['valid']
    assert 'source_expired_or_uncertain' in result['reasons']


def test_fractional_nanosecond_receive_time_is_not_future():
    received=1.0568415870002354
    current=sample();current['received']=received
    result=ready().assess(current,int(received*1e9),identity_verified=True)
    assert result['valid']
    assert 'arrival_age' not in result['reasons']


def test_actual_future_arrival_and_half_second_expiry_remain_rejected():
    current=sample();current['received']=1.050001
    assert 'arrival_age' in ready().assess(current,1_050_000_000,identity_verified=True)['reasons']
    current['received']=.55
    assert 'arrival_age' in ready().assess(current,1_050_000_000,identity_verified=True)['reasons']


def test_duplicate_unsolicited_wrong_source_and_token():
    clock=LockClockProbe()
    assert not clock.ingest(reply(),1_040_000_000)
    clock.request(1_000_000_000)
    assert not clock.ingest(reply(source=(2,1)),1_040_000_000)
    assert not clock.ingest(reply(token=999),1_040_000_000)
    assert clock.ingest(reply(),1_040_000_000)
    assert not clock.ingest(reply(),1_050_000_000)


def test_timeout_consumed_and_old_reply_cannot_match_new_request():
    clock=LockClockProbe();clock.request(1_000_000_000)
    assert not clock.ingest(reply(),1_100_000_001)
    assert clock.reference is None and clock.pending is None
    clock.request(1_200_000_000)
    assert not clock.ingest(reply(),1_210_000_000)


def test_regression_latches_failure():
    clock=ready();clock.request(1_200_000_000)
    assert not clock.ingest(reply(remote=1_000_000,token=1_200_000_000),1_220_000_000)
    assert clock.fault=='remote_clock_regression'
    assert not clock.assess(sample(),1_230_000_000,identity_verified=True)['valid']


def test_audit_distinguishes_late_missing_and_accepted_replies():
    clock=LockClockProbe();clock.request(1_000_000_000)
    assert not clock.ingest(reply(),1_100_000_001)
    clock.request(1_200_000_000)
    clock.request(1_400_000_000)
    assert not clock.ingest(reply(token=1_200_000_000),1_420_000_000)
    assert clock.ingest(reply(token=1_400_000_000),1_440_000_000)
    counts=clock.summary()['counts']
    assert counts==dict(requests=3,rtt_rejected=1,expired_unanswered_request=1,
        unmatched_or_unsolicited=1,accepted=1)
    assert clock.reference['rtt_ns']==40_000_000


@pytest.mark.parametrize('stamp,bits,now,identity',[
    (5100,MAGIC|5,1_050_000_000,True),
    (5005,MAGIC|7,1_050_000_000,True),
    (5005,MAGIC|5,1_540_000_000,True),
    (5005,MAGIC|5,1_050_000_000,False),
])
def test_uncertain_unsafe_expired_or_unidentified_never_pass(stamp,bits,now,identity):
    assert not ready().assess(sample(stamp,bits),now,identity_verified=identity)['valid']


def test_probe_policy_is_explicit_and_matches_pending_request():
    msg=m.MAVLink(None).timesync_encode(0,123)
    opts=dict(safe=True,short_status=True,time_probe=True,pending_token=123)
    assert allowed(msg,**opts)
    for key,value in [('safe',False),('short_status',False),('time_probe',False),('pending_token',124)]:
        assert not allowed(msg,**{**opts,key:value})
    assert not allowed(m.MAVLink(None).timesync_encode(10,123),**opts)
