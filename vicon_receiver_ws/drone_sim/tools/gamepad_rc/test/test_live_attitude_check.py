import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from live_attitude_check import LiveAttitudeCheck, tilt_difference


def roll(degrees):
    half=math.radians(degrees)/2
    return [math.cos(half),math.sin(half),0,0]


def fill(check,start=0,count=20,dt=.1,source=(1,0,0,0),fc=(1,0,0,0)):
    for i in range(count):
        t=start+i*dt
        check.observe(source,fc,t,int((100+t)*1000))
    return start+(count-1)*dt


def test_twenty_fresh_pairs_required_and_no_flight_or_yaw_claim():
    c=LiveAttitudeCheck();t=fill(c,count=19)
    assert not c.summary(t)['observed']
    t=fill(c,start=1.9,count=1)
    r=c.summary(t)
    assert r['observed'] and r['pairs']==20
    assert not r['yaw_alignment_verified'] and not r['dynamic_extrinsics_verified']
    assert not r['imu_origin_verified']


def test_arbitrary_constant_yaw_offset_is_not_a_tilt_error():
    c=LiveAttitudeCheck();t=fill(c,source=(0,0,0,1))
    assert c.summary(t)['observed']
    assert tilt_difference((0,0,0,1),(1,0,0,0))==pytest.approx(0)


def test_quaternion_sign_flip_is_not_a_source_pose_jump():
    c=LiveAttitudeCheck();t=fill(c)
    c.observe((-1,0,0,0),(1,0,0,0),t+.1,102000)
    assert c.summary(t+.1)['observed'] and c.fault is None


@pytest.mark.parametrize('angle',[6,90,180])
def test_bad_tilt_latches_and_good_data_cannot_clear_it(angle):
    c=LiveAttitudeCheck();c.observe(roll(angle),(1,0,0,0),0,100000)
    assert c.fault=='source_fc_gravity_disagreement_over_5_deg'
    fill(c,start=.1)
    assert not c.summary(2)['observed']


def test_abrupt_yaw_flip_during_stationary_check_also_latches():
    c=LiveAttitudeCheck();fill(c)
    c.observe((0,0,0,1),(1,0,0,0),2,102000)
    assert c.fault=='source_attitude_step_during_static_check'


@pytest.mark.parametrize('bad',[(0,0,0,0),(1,2,3),None,(float('nan'),0,0,0)])
def test_invalid_quaternion_cannot_become_an_observation(bad):
    c=LiveAttitudeCheck();c.observe(bad,(1,0,0,0),0,100000)
    assert c.fault and not c.summary(0)['observed']


def test_duplicates_never_count_as_twenty_pairs_or_refresh_latest_age():
    c=LiveAttitudeCheck();c.observe((1,0,0,0),(1,0,0,0),0,100000)
    for i in range(1,30):c.observe((1,0,0,0),(1,0,0,0),i*.1,100000)
    r=c.summary(3)
    assert not r['observed'] and r['pairs']==1 and r['latest_pair_age_s']==3
    assert r['duplicate_fc_samples_ignored']==29


def test_reboot_or_boot_clock_regression_is_not_rebased():
    c=LiveAttitudeCheck();fill(c)
    c.observe((1,0,0,0),(1,0,0,0),2,100)
    assert c.fault=='attitude_boot_clock_regressed'


def test_stale_pairs_or_excessive_gap_do_not_pass():
    c=LiveAttitudeCheck();t=fill(c)
    assert not c.summary(t+.501)['observed']
    c=LiveAttitudeCheck();t=fill(c,count=10)
    t=fill(c,start=t+.6,count=10)
    assert not c.summary(t)['observed']


def test_low_rate_is_not_hidden_by_number_of_pairs():
    c=LiveAttitudeCheck();t=fill(c,dt=.4)
    assert not c.summary(t)['observed']


def test_history_is_bounded_and_normal_small_tilt_is_allowed():
    c=LiveAttitudeCheck();t=fill(c,count=100,source=roll(1))
    r=c.summary(t)
    assert r['observed'] and r['pairs']==40
    assert r['maximum_tilt_difference_deg']==pytest.approx(1)
