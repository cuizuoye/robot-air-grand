from pathlib import Path
import math
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import compact_vision as compact
import low_bandwidth_vision as low
import one_home_preflight_policy as policy


def parameters():
    return dict(EKF2_EV_NOISE_MD=0,EKF2_EVP_NOISE=low.f32(.1),EKF2_EVA_NOISE=low.f32(.1))


def test_actual_crc_valid_wire_preserves_pose_and_reduces_only_metadata():
    enc=m.MAVLink(None,srcSystem=250,srcComponent=213)
    args=(enc,1_000_000,[1.,2.,3.],[1.,0.,0.,0.],.3)
    old,p,q=compact.message(*args)
    msg,p2,q2=low.message(*args,parameters())
    assert p==p2 and q==q2
    assert len(old.pack(enc))==128 and len(msg.pack(enc))==48
    result=m.MAVLink(None).parse_buffer(msg.pack(enc))[0]
    assert low.pose_allowed(result,parameters())
    assert not compact.pose_allowed(result)  # Legacy profile stays strict.
    for key in ('usec','x','y','z','roll','pitch','yaw','reset_counter'):
        assert getattr(result,key)==pytest.approx(getattr(old,key),abs=1e-6)
    evidence=low.noise_evidence(parameters())
    assert evidence['legacy_effective_variances']==evidence['candidate_effective_variances']
    assert not evidence['flight_authorized']


@pytest.mark.parametrize('change',[
    dict(EKF2_EVP_NOISE=.001),dict(EKF2_EVA_NOISE=.001),dict(EKF2_EV_NOISE_MD=1),
    dict(EKF2_EVP_NOISE=0),dict(EKF2_EVP_NOISE=-1),dict(EKF2_EVP_NOISE=math.nan),
    dict(EKF2_EVA_NOISE=math.inf),dict(EKF2_EVA_NOISE=True),dict(EKF2_EVA_NOISE=1e100)])
def test_missing_or_different_effective_noise_blocks_candidate(change):
    with pytest.raises(ValueError):low.noise_evidence(parameters()|change)
    with pytest.raises(ValueError):low.noise_evidence({})


@pytest.mark.parametrize('change',['xyz_nan','yaw_nan','other_nan','other_nonzero','not_unknown','reset'])
def test_only_the_canonical_unknown_covariance_is_permitted(change):
    msg,_,_=low.message(m.MAVLink(None),1,[1,2,3],[1,0,0,0],0,parameters())
    if change=='xyz_nan':msg.x=math.nan
    if change=='yaw_nan':msg.yaw=math.nan
    if change=='other_nan':msg.covariance[6]=math.nan
    if change=='other_nonzero':msg.covariance[20]=.1
    if change=='not_unknown':msg.covariance[0]=0
    if change=='reset':msg.reset_counter=1
    assert not low.pose_allowed(msg,parameters())


def test_new_encoding_requires_its_explicit_profile_and_all_old_send_gates():
    enc=m.MAVLink(None)
    msg,_,_=low.message(enc,1,[1,2,3],[1,0,0,0],0,parameters())
    gates=dict(authorized=True,identity_fresh=True,locked_static=True)
    assert not policy.allowed(msg,**gates)
    options=dict(low_bandwidth_vision=True,vision_noise_parameters=parameters())
    assert policy.allowed(msg,**gates,**options)
    for key in gates:
        assert not policy.allowed(msg,**(gates|{key:False}),**options)
    assert not policy.allowed(msg,**gates,low_bandwidth_vision=True,vision_noise_parameters={})
    for command,param in ((400,1),(176,1),(22,0),(21,0),(185,0),(185,1),(246,1)):
        assert not policy.allowed(enc.command_long_encode(1,1,command,0,param,0,0,0,0,0,0),
            **gates,**options)
