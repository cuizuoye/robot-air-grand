"""SYNTHETIC observations/messages only. No hardware authorization is emitted."""
from dataclasses import replace
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import trs_powered_fusion_policy as p
import ekf_static_profile as profile
import one_home_preflight_policy as one
import trs_powered_fusion_bench as bench
from fusion_binary_offline import synthetic_row
from fixed_profile_fixtures import approved_profile,current,snapshot
from test_trs_powered_fusion import observation
from test_trs_powered_fusion_reactor import reactor
from trs_uart_readonly_check import UID2

IDENTITY={'uid2':UID2,'firmware':'1.14.3'}


def configured():
    gate=p.FixedGroundPreparation(profile.fingerprint(approved_profile()))
    assert gate.connect(IDENTITY,'SYNTHETIC-boot-1',10.)
    assert gate.verify_configuration(current(),10.1)
    return gate


def authorize(gate,obs=None,**changes):
    args=dict(home=NS(verified=True,aborted=False),clock_lock={'valid':True},
              identity=IDENTITY,boot_session='SYNTHETIC-boot-1')|changes
    return gate.authorize_positions(observation(10.2) if obs is None else obs,**args)


def rows():
    result=[synthetic_row(10_000_000+i*100_000) for i in range(3)]
    for row in result:row['yaw_comparison']=dict(fresh=True,fc_minus_vicon_wrapped_deg=0.)
    return result


def test_01_configured_only_is_not_home_position_or_fusion_permission():
    gate=configured();assert gate.phase=='CONFIG_VERIFIED'
    assert not gate.status()['position_authorized'] and not gate.ready_observed
    assert authorize(gate)
    assert gate.phase=='POSITION_AUTHORIZED' and not gate.ready_observed


def test_06_unknown_home_cannot_authorize_position():
    gate=configured();assert not authorize(gate,home=NS(verified=False,aborted=False))
    assert gate.phase=='FAILED' and not gate.status()['position_authorized']


@pytest.mark.parametrize('change',[
    dict(disarmed_locked=False),dict(source_age=.151),dict(attitude_age=.501),
    dict(identity_fresh=False),dict(gamepad_same_device=False),dict(boot_unchanged=False),
    dict(parameters_fresh_match=False),dict(home_pressed=True)])
def test_07_08_initial_guard_failure_is_terminal(change):
    gate=configured();assert not authorize(gate,replace(observation(10.2),**change))
    assert not authorize(gate)
    assert not gate.ready_observed


def test_09_timesync_reference_loss_revokes_even_with_fresh_vicon():
    gate=configured();assert authorize(gate)
    assert not gate.check(observation(10.3),clock_lock={'valid':False,'reasons':['clock_age']},
        identity=IDENTITY,boot_session='SYNTHETIC-boot-1')
    assert gate.phase=='AUTH_REVOKED'
    assert not gate.check(observation(10.4),clock_lock={'valid':True},identity=IDENTITY,boot_session='SYNTHETIC-boot-1')


def test_10_config_correct_and_position_received_do_not_prove_fusion():
    gate=configured();assert authorize(gate)
    assert not gate.verify_fusion([],0,reception_verified=True,observation=observation(10.3),
        clock_lock={'valid':True},identity=IDENTITY,boot_session='SYNTHETIC-boot-1')
    assert gate.phase=='AUTH_REVOKED' and not gate.ready_observed


def test_ready_requires_actual_three_progressing_audits_and_receipt():
    gate=configured();assert authorize(gate)
    assert gate.verify_fusion(rows(),0,reception_verified=True,observation=observation(10.3),
        clock_lock={'valid':True},identity=IDENTITY,boot_session='SYNTHETIC-boot-1')
    assert gate.phase=='READY' and not gate.status()['control_ready']
    assert not gate.status()['physical_authorization'] and not gate.status()['flight_authorized']
    gate.close(10.4)
    assert gate.phase=='AUTH_REVOKED' and not gate.status()['position_authorized']


@pytest.mark.parametrize('change',[dict(boot_session='SYNTHETIC-boot-2'),
    dict(identity={'uid2':'other','firmware':'1.14.3'})])
def test_11_14_reboot_or_identity_change_invalidates_old_success(change):
    gate=configured();assert authorize(gate)
    assert gate.verify_fusion(rows(),0,reception_verified=True,observation=observation(10.3),
        clock_lock={'valid':True},identity=IDENTITY,boot_session='SYNTHETIC-boot-1')
    args=dict(clock_lock={'valid':True},identity=IDENTITY,boot_session='SYNTHETIC-boot-1')|change
    assert not gate.check(observation(10.4),**args)
    assert not gate.status()['position_authorized']


def test_12_disconnect_revokes_without_a_recovery_sender():
    gate=configured();assert authorize(gate)
    assert not gate.check(replace(observation(10.3),identity_fresh=False),clock_lock={'valid':False},
        identity=IDENTITY,boot_session='SYNTHETIC-boot-1')
    assert gate.phase=='AUTH_REVOKED'
    assert not hasattr(gate,'writer') and not hasattr(gate,'restore')


@pytest.mark.parametrize('name,value,kind',[(n,v,k) for n,v,k in (
    ('EKF2_EV_CTRL',11,6),('EKF2_EV_CTRL',2,6),('EKF2_EV_DELAY',65.,9),('EKF2_EV_DELAY',0.,9),
    ('EKF2_HGT_REF',3,6))])
def test_13_15_writer_policies_deny_parameters_even_during_cleanup(name,value,kind):
    encoder=m.MAVLink(None)
    msg=encoder.param_set_encode(1,1,name.encode(),value,kind)
    flags=dict(fixed_config=True,authorized=True,identity_fresh=True,locked_static=True,
               receiving_verified=True,restore_safe=True,home_latched=True)
    assert not p.allowed(msg,**flags)
    assert not one.allowed(msg,**flags)


def test_new_mode_cannot_open_serial_even_with_all_cli_confirmations():
    assert not bench.FIXED_GROUND_LIVE_ENABLED
    with pytest.raises(PermissionError,match='offline-only'):
        bench.run(NS(fixed_ekf_profile='not-even-read.json',powered_locked_static_confirmed=True))


def test_new_mode_draft_cannot_be_consumed_as_old_dynamic_permit(tmp_path):
    file=tmp_path/'fixed.json';file.write_text(json.dumps(dict(scope=one.FIXED_SCOPE)))
    with pytest.raises(PermissionError,match='no legacy fallback'):
        one.consume(file,backup=tmp_path,output=tmp_path/'report.json',heading=tmp_path,board=tmp_path,baud=115200)
    assert not Path(str(file)+'.consumed.json').exists()


@pytest.mark.parametrize('scenario',['fixed_legacy_permit','fixed_wrong_profile_permit'])
def test_15_actual_runner_rejects_old_or_wrong_profile_grant_before_io(reactor,scenario):
    with pytest.raises(PermissionError,match='legacy or differently bound'):
        reactor(scenario=scenario,baud=115200,fixed_document=approved_profile())


def test_cli_fixed_offline_never_discovers_devices_or_accepts_file_as_ready(tmp_path,capsys,monkeypatch):
    import drone_preflight as cli
    def forbidden(*args,**kwargs):raise AssertionError('No real prerequisites/discovery allowed')
    monkeypatch.setattr(cli,'prerequisites',forbidden)
    file=tmp_path/'profile.json';file.write_text(json.dumps(approved_profile()))
    snap=tmp_path/'snapshot.json';snap.write_text(json.dumps(snapshot()))
    assert cli.main(['--fixed-ekf-profile',str(file),'--snapshot',str(snap)])==0
    result=json.loads(capsys.readouterr().out)
    assert not result['control_ready'] and not result['live_enabled'] and result['devices_opened']==0
    assert cli.main(['--fixed-ekf-profile',str(file),'--run-battery'])==2
    assert '不会退回' in capsys.readouterr().out
    assert cli.main(['--run-battery'])==2


def test_real_runner_fixed_happy_path_has_no_ekf_write_and_no_legacy_success_fields(reactor):
    code,report,sent=reactor(baud=115200,fixed_document=approved_profile())
    assert code==0,report['error'] or report['restore_errors']
    assert report['fixed_ground_acceptance_verified']
    assert report['fixed_ground_preparation']['ready_observed']
    assert report['fixed_ground_preparation']['phase']=='AUTH_REVOKED'
    assert not report['parameter_writes'] and report['restored']=={}
    assert report['parameter_restoration_verified'] is None
    assert report['fixed_configuration_unchanged_verified']
    assert not report['one_home_preflight_verified'] and not report['compact_ground_fusion_verified']
    assert not report['fixed_hardware_verified'] and not report['physical_authorization']
    assert not any(msg.get_type() in ('PARAM_SET','PARAM_EXT_SET') for _,msg in sent)
    assert any(msg.get_type()=='VISION_POSITION_ESTIMATE' for _,msg in sent)


@pytest.mark.parametrize('scenario',['radio_lost','short_clock_lost','one_home_abort','pad_lost',
    'source_lost','fc_reboot','binary_selector_change','binary_restore_echo_missing','binary_missing',
    'fixed_rogue_cleanup'])
def test_12_13_real_runner_fault_and_finally_remain_zero_parameter_writes(reactor,scenario):
    code,report,sent=reactor(scenario=scenario,baud=115200,fixed_document=approved_profile())
    assert code==2 and not report['fixed_ground_acceptance_verified']
    assert report['fixed_ground_preparation']['phase'] in ('FAILED','AUTH_REVOKED')
    assert not report['parameter_writes'] and not report['restored']
    assert not any(msg.get_type() in ('PARAM_SET','PARAM_EXT_SET') for _,msg in sent)


@pytest.mark.parametrize('scenario',['fixed_ctrl_mismatch','fixed_delay_mismatch','fixed_parameter_timeout'])
def test_02_03_05_real_runner_rejects_bad_configuration_before_any_pose(reactor,scenario):
    code,report,sent=reactor(scenario=scenario,baud=115200,fixed_document=approved_profile())
    assert code==2 and not report['fixed_ground_acceptance_verified']
    result=report['fixed_configuration_report']
    assert result and not result['current_configuration_verified']
    assert result['timed_out'] if scenario=='fixed_parameter_timeout' else result['mismatched']
    assert not any(msg.get_type() in ('VISION_POSITION_ESTIMATE','PARAM_SET','PARAM_EXT_SET') for _,msg in sent)


@pytest.mark.parametrize('time',[float('nan'),float('inf'),-1.,9.])
def test_invalid_or_regressed_time_revokes_without_inventing_replacement(time):
    gate=configured();assert authorize(gate)
    assert not gate.check(replace(observation(10.3),now=time),clock_lock={'valid':True},
        identity=IDENTITY,boot_session='SYNTHETIC-boot-1')
    assert gate.phase=='AUTH_REVOKED' and gate.history[-1]['at'] is None
