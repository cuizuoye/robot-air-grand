from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from preflight_session_status import CHECKS, describe


def complete():
    return {name:True for name,_,_ in CHECKS}


def test_all_observations_still_never_authorize_or_start_a_flight():
    r=describe(complete())
    assert r['phase']=='STATIC_BENCH_INPUTS_OBSERVED'
    assert r['static_bench_inputs_observed'] and not r['missing_observations']
    assert not r['flight_authorized'] and not r['fusion_started']
    assert not r['hardware_mission_adapter_present'] and r['flight_blockers']
    assert r['next_step_requires_explicit_separate_bench']


@pytest.mark.parametrize('name,phase,hint',CHECKS)
def test_every_missing_observation_explains_its_own_stage(name,phase,hint):
    r=describe({**complete(),name:False})
    assert r['phase']==phase and not r['static_bench_inputs_observed']
    assert r['missing_observations']==[{'check':name,'phase':phase,'hint':hint}]


@pytest.mark.parametrize('value',[None,1,'true','False',[],{}])
def test_only_explicit_true_is_observed_evidence(value):
    r=describe({**complete(),'live_lockdown_confirmed':value})
    assert r['phase']=='WAIT_CURRENT_LOCK' and not r['static_bench_inputs_observed']


def test_degraded_session_does_not_become_ready_just_because_data_recovers():
    r=describe(complete(),fault='vicon_source_lost_after_ready')
    assert r['phase']=='DEGRADED' and not r['static_bench_inputs_observed']
    assert r['latched_fault']=='vicon_source_lost_after_ready'


def test_closed_session_never_claims_active_input_protection():
    r=describe(complete(),running=False)
    assert r['phase']=='CLOSED' and not r['static_bench_inputs_observed']
    assert not r['flight_authorized']


def test_all_missing_reasons_are_retained_not_just_first():
    r=describe({})
    assert r['phase']=='WAIT_LINK'
    assert len(r['missing_observations'])==len(CHECKS)
