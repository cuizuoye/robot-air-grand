"""Phase-2 observer integration with existing, unchanged safety predicates."""
import ast
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import ekf_static_profile as p
import ekf_ground_acceptance as a
from ground_link_diagnostics import TimingRecorder, summarize
from test_phase2_timing import trace
from fixed_profile_fixtures import approved_profile, current
from trs_powered_fusion_policy import FixedGroundPreparation
from test_trs_powered_fusion import observation
from test_fixed_ground_preparation import IDENTITY
from test_phase2_acceptance import no_grant, capture


@pytest.mark.parametrize('failure', ['home_unknown','disarmed_unknown','vicon_stale','timesync_stale',
                                     'attitude_501ms','parameter_timeout','reboot','identity_change'])
def test_diagnostic_observer_cannot_override_existing_ground_failures(failure):
    # New integration: even a clean diagnostic report cannot grant/revive
    # authority after the production ground gate rejects current evidence.
    doc=approved_profile();g=FixedGroundPreparation(p.fingerprint(doc))
    assert g.connect(IDENTITY,'SYNTHETIC-boot-1',10.)
    config=current()
    if failure=='parameter_timeout':config['current_configuration_verified']=False;config['timed_out']=['EKF2_EV_CTRL']
    if failure=='parameter_timeout':
        assert not g.verify_configuration(config,10.1)
    else:
        assert g.verify_configuration(config,10.1)
        obs=observation(10.2)
        args=dict(home=SimpleNamespace(verified=True,aborted=False),clock_lock={'valid':True},
                  identity=IDENTITY,boot_session='SYNTHETIC-boot-1')
        if failure=='home_unknown':args['home'].verified=False
        elif failure=='disarmed_unknown':obs=replace(obs,disarmed_locked=False)
        elif failure=='vicon_stale':obs=replace(obs,source_age=.151)
        elif failure=='timesync_stale':args['clock_lock']={'valid':False,'reasons':['clock_age']}
        elif failure=='attitude_501ms':obs=replace(obs,attitude_age=.501)
        elif failure=='reboot':args['boot_session']='different-boot'
        else:args['identity']={**IDENTITY,'uid2':'other'}
        assert not g.authorize_positions(obs,**args)
    clean=summarize(trace(),start_ns=1_000_000_000,end_ns=2_000_005_000)
    assert clean['timing_window_clear']
    assert not g.status()['position_authorized']
    no_grant(clean)
    assert not g.status()['physical_authorization']


def test_raw_receive_parse_and_application_delay_are_separate():
    t=dict(host_clock_session='SYNTHETIC',events=[
        dict(kind='read',at_ns=1_000_000_000,start_ns=999_000_000,bytes=100),
        dict(kind='parse',at_ns=1_001_000_000,start_ns=1_000_000_000,read_end_ns=1_000_000_000),
        dict(kind='application_update',at_ns=1_601_000_000,parse_end_ns=1_001_000_000)])
    r=summarize(t,start_ns=900_000_000,end_ns=1_700_000_000)
    assert r['parse_duration']['max_ns']==1_000_000
    assert r['parse_to_application']['max_ns']==600_000_000
    assert not r['review_complete']


def test_fault_is_latched_in_recorder_without_retry_or_io():
    r=TimingRecorder('SYNTHETIC')
    r.record('safety_revoke',at_ns=1,reason='attitude_age_exceeded')
    r.record('stream',at_ns=2,stream='VICON',accepted=True,source_ns=2)
    assert r.fault=='attitude_age_exceeded'
    assert not hasattr(r,'reset') and not hasattr(r,'restart')
    assert r.export()['recorder_fault']=='attitude_age_exceeded'


def test_receipt_boolean_cannot_replace_actual_matching_observations():
    c=capture();del c['receipt'];c['px4_receipt_verified']=True
    assert not a.first_fusion_review(c)['review_complete']


@pytest.mark.parametrize('filename', ['ekf_ground_acceptance.py','ground_link_diagnostics.py',
                                     'ev_delay_calibration.py','phase2_acceptance.py'])
def test_new_tools_have_no_hardware_or_command_api(filename):
    tree=ast.parse((Path(__file__).parents[1]/filename).read_text())
    imported=[]
    for node in ast.walk(tree):
        if isinstance(node,ast.Import):imported.extend(n.name for n in node.names)
        if isinstance(node,ast.ImportFrom):imported.append(node.module or '')
        if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute):
            assert node.func.attr not in ('recv_match','param_set_send','param_ext_set_send',
                'command_long_send','command_long_encode','serial_control_encode','write_parameter')
    assert not any(n.split('.')[0] in ('serial','rclpy','socket','subprocess','mavutil') for n in imported)


def test_existing_three_hardware_entry_locks_still_closed():
    import trs_powered_fusion_bench as bench
    import one_home_preflight_policy as one
    assert bench.FIXED_GROUND_LIVE_ENABLED is False
    with pytest.raises(PermissionError):
        bench.run(SimpleNamespace(fixed_ekf_profile='not-opened'))
    # Actual legacy consumer remains rejecting the phase-2 report's scope too.
    assert one.FIXED_SCOPE=='fixed-ekf-ground-offline-candidate-v1'


@pytest.mark.parametrize('value',[{},[],b'raw',float('nan'),'x'*513])
def test_receive_path_refuses_unbounded_or_mutable_diagnostic_payload(value):
    with pytest.raises(ValueError):TimingRecorder('x').record('read',at_ns=1,payload=value)
