from dataclasses import replace
import datetime
import json
from pathlib import Path
import subprocess
import sys
import time

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import trs_powered_fusion_policy as p
import trs_powered_fusion_permit as permit
import trs_vision_reference_policy as old
from trs_powered_fusion_bench import boot_reference_matches


def observation(t=0):
    return p.Observation(t,True,True,True,True,.1,.01,.05,1,True,True,True,False)


@pytest.mark.parametrize('name,value', [
    ('identity_fresh',False),('usb_absent',False),('boot_unchanged',False),
    ('disarmed_locked',False),('lock_age',2.001),('source_age',.151),
    ('attitude_age',.501),('tilt_difference_deg',5.01),('stationary',False),
    ('parameters_fresh_match',False),('gamepad_same_device',False),('home_pressed',True),
    ('source_age',-1),('now',float('nan'))])
def test_every_live_failure_stops_and_latches_without_automatic_recovery(name,value):
    gate=p.StreamGate();gate.start(observation())
    assert not gate.check(replace(observation(.01),**{name:value}))
    assert gate.first_failure and gate.first_failure['reasons']
    original=gate.first_failure
    assert not gate.check(observation(.02))
    assert gate.first_failure is original
    with pytest.raises(ValueError):gate.start(observation(.03))


def test_actual_pose_stream_gap_and_bounded_packet_budget():
    gate=p.StreamGate();gate.start(observation());assert gate.check(observation(.01))
    gate.actual_send(.01)
    assert not gate.check(observation(.16001))
    assert 'send_gap_exceeded' in gate.first_failure['reasons']
    gate=p.StreamGate();gate.start(observation());gate.sent=639
    gate.actual_send(.01)
    assert not gate.check(observation(.02))


def test_wall_time_and_fusion_limits_cannot_be_extended_or_restarted():
    gate=p.StreamGate();gate.start(observation())
    assert not gate.check(observation(20))
    gate=p.StreamGate();gate.start(observation());gate.begin_fusion(observation(.1))
    with pytest.raises(ValueError):gate.begin_fusion(observation(.2))
    assert not gate.check(observation(12.1))


@pytest.mark.parametrize('command,param1',[(400,1),(400,0),(176,1),(21,0),(22,0),
                                          (185,0),(185,1),(185,3),(246,1),(511,1)])
def test_all_actuation_modes_rates_and_unlock_commands_refused(command,param1):
    msg=m.MAVLink(None).command_long_encode(1,1,command,0,param1,0,0,0,0,0,0)
    assert not p.allowed(msg,identity_fresh=True,authorized=True,locked_static=True,
                         receiving_verified=True,restore_safe=True,home_latched=True)


def test_stop_requires_genuine_latch_and_live_identity_even_in_new_scope():
    msg=m.MAVLink(None).command_long_encode(1,1,185,0,2,0,0,0,0,0,0)
    assert p.allowed(msg,identity_fresh=True,authorized=True,home_latched=True)
    assert not p.allowed(msg,identity_fresh=True,authorized=True)
    assert not p.allowed(msg,authorized=True,home_latched=True)


@pytest.mark.parametrize('name,candidate,baseline',[('EKF2_EV_CTRL',11,2),('EKF2_EV_DELAY',65,0)])
def test_separate_radio_write_scope_does_not_weaken_old_usb_policy(name,candidate,baseline):
    enc=m.MAVLink(None);msg=old.parameter_message(enc,name,candidate)
    gates=dict(identity_fresh=True,authorized=True,locked_static=True,receiving_verified=True)
    assert p.allowed(msg,**gates)
    for flag in gates:assert not p.allowed(msg,**{**gates,flag:False})
    assert not old.allowed(msg,route='trs',safe=True,authorized=True,backup_verified=True,
                           locked_static=True,receive_time_verified=True)
    msg=old.parameter_message(enc,name,baseline)
    assert p.allowed(msg,authorized=True,restore_safe=True)
    assert not p.allowed(msg,authorized=True)
    assert not p.allowed(msg,restore_safe=True)
    msg.param_type=9 if name=='EKF2_EV_CTRL' else 6
    assert not p.allowed(msg,authorized=True,restore_safe=True)


def test_pose_whitelist_and_fixed_shell_no_arbitrary_or_long_ground_command():
    enc=m.MAVLink(None);msg,*_=p.compact.message(enc,10,[1,2,-3],[1,0,0,0],0)
    assert p.allowed(msg,authorized=True,identity_fresh=True,locked_static=True)
    assert not p.allowed(msg,authorized=True,identity_fresh=True)
    for payload in (b'commander arm\n',p.COMMANDS['ground_visual_series']):
        msg=enc.serial_control_encode(10,2,0,0,len(payload),payload.ljust(70,b'\0'))
        assert not p.allowed(msg,identity_fresh=True,authorized=True,locked_static=True)
    payload=p.COMMANDS['actuator_armed']
    msg=enc.serial_control_encode(10,2,0,0,len(payload),payload.ljust(70,b'\0'))
    assert p.allowed(msg,identity_fresh=True)
    msg.flags=0;assert not p.allowed(msg,identity_fresh=True)


def test_reboot_or_old_lock_epoch_is_not_accepted_as_current_session():
    assert boot_reference_matches(110,100,1000,1010)
    assert not boot_reference_matches(1,100,1000,1010)
    assert not boot_reference_matches(101,100,1000,1010)
    assert not boot_reference_matches(110,100,1000,999)
    assert not boot_reference_matches(float('nan'),100,1000,1010)


def fixture(tmp_path,monkeypatch):
    import rigid_revision_static_permit as static
    from test_rigid_revision_static_permit import fixture as prior_fixture
    file,backup,output,h,b,_=prior_fixture(tmp_path,monkeypatch)
    static.consume(file,backup,output,h,b)
    import rigid_body_revision as revision
    import trs_runtime_monitor as runtime
    inspect=revision.inspect_current_calibration
    scoped_inspect=lambda h,b:inspect(h,b,registry_path=revision.REGISTRY)
    monkeypatch.setattr(permit,'inspect_current_calibration',scoped_inspect)
    monkeypatch.setattr(runtime,'inspect_current_calibration',scoped_inspect)
    monkeypatch.setattr(permit,'HOLDS',tmp_path/'known_incidents')
    static_proof=output
    output=tmp_path/'powered-output.json'
    static_proof.write_text(json.dumps(dict(rigid_body_revision_id='new',rigid_revision_static_validation=True,
        ekf_stationary_xyz_yaw_fusion_verified=True,error=None,restore_errors=[],
        restored={'EKF2_EV_CTRL':True,'EKF2_EV_DELAY':True},arming_mode_motion_or_unlock_commands_sent=False,
        actuator_after={'armed':False,'lockdown':True},identities={r:{'uid2':permit.UID} for r in ('usb','trs')},
        heading_candidate_path=str(h),board_center_candidate_path=str(b),vision_reference_permit_path=str(file))))
    attitude=tmp_path/'current-attitude.json';home=tmp_path/'current-home.json'
    moment=time.time()
    utc=lambda value:datetime.datetime.fromtimestamp(value,datetime.timezone.utc).isoformat()
    attitude.write_text(json.dumps(dict(captured_at_utc=utc(moment-2),static_observation_passed=True,error=None,
        identity={'uid2':permit.UID,'firmware':'1.14.3'},actual_sent_to_fc=0,median_gravity_axis_difference_deg=1,
        checks={'pass':True},heading_candidate=str(h),last_pair={'fc_boot_ms':100000})))
    home.write_text(json.dumps(dict(captured_at_utc=utc(moment),lockdown_after_release_verified=True,error=None,
        guarded_props_installed_stop_only_confirmed=True,fc_usb_present_at_end=False,
        evidence={'identity':{'uid2':permit.UID,'firmware':'1.14.3'}},passive_power={'reboot_observed':False},
        home_events=[{'value':1},{'value':0}],acks=[{'result':0,'source':[1,1],'target':[250,199]}],
        snapshots=[{'timestamp_us':102000000,'armed':False,'lockdown':True,'force_failsafe':False,'manual_lockdown':False}])))
    # Permit hashes are real code files; fixture-only holds remain isolated.
    expected,paths=permit.validate(backup,output,h,b,attitude,home,static_proof,baud=57600)
    record={**expected,'issued_unix':moment-1,'expires_unix':moment+599,
            'sha256':{k:permit.digest(v) for k,v in paths.items()}}
    new=tmp_path/'powered-permit.json';new.write_text(json.dumps(record))
    return new,dict(backup=backup,output=output,heading=h,board=b,baud=57600),record,attitude,home


def test_powered_permit_is_one_shot_and_never_changes_global_revision(tmp_path,monkeypatch):
    f,request,_,_,_=fixture(tmp_path,monkeypatch)
    record=permit.consume(f,**request)
    assert not record['flight_authorized'] and not record['global_holds_modified']
    with pytest.raises(FileExistsError):permit.consume(f,**request)


def test_permit_cannot_be_consumed_at_another_baud(tmp_path,monkeypatch):
    f,request,_,_,_=fixture(tmp_path,monkeypatch)
    with pytest.raises(ValueError,match='mismatch'):permit.consume(f,**(request|{'baud':115200}))
    assert not Path(str(f)+'.consumed.json').exists()


def test_permit_consumer_cannot_omit_baud(tmp_path,monkeypatch):
    f,request,_,_,_=fixture(tmp_path,monkeypatch)
    request.pop('baud')
    with pytest.raises(ValueError,match='Explicit TRS baud'):permit.consume(f,**request)
    assert not Path(str(f)+'.consumed.json').exists()


@pytest.mark.parametrize('missing',['trs_baud','rx_parser'])
def test_old_permits_are_rejected_without_automatic_upgrade(tmp_path,monkeypatch,missing):
    f,request,r,_,_=fixture(tmp_path,monkeypatch)
    r.pop(missing);r['schema']=1;f.write_text(json.dumps(r))
    with pytest.raises(ValueError,match='mismatch'):permit.consume(f,**request)
    assert not Path(str(f)+'.consumed.json').exists()


def test_transport_configuration_is_code_hash_bound(tmp_path,monkeypatch):
    f,request,r,_,_=fixture(tmp_path,monkeypatch)
    assert 'trs_transport_config.py' in r['sha256']
    assert r['trs_baud']==57600 and r['schema']==2
    r['sha256']['trs_transport_config.py']='0'*64;f.write_text(json.dumps(r))
    with pytest.raises(ValueError,match='mismatch'):permit.consume(f,**request)
    assert not Path(str(f)+'.consumed.json').exists()


@pytest.mark.parametrize('key,value',[('scope','flight'),('max_fusion_seconds',60),
    ('max_packets',10000),('flight_authorized',True),('commands_forbidden',[]),
    ('physical_conditions',['usb_only'])])
def test_no_scope_expansion_by_editing_permit(tmp_path,monkeypatch,key,value):
    f,request,r,_,_=fixture(tmp_path,monkeypatch);r[key]=value;f.write_text(json.dumps(r))
    with pytest.raises(ValueError):permit.consume(f,**request)
    assert not Path(str(f)+'.consumed.json').exists()


@pytest.mark.parametrize('target,key,value',[
    ('home','lockdown_after_release_verified',False),('home','fc_usb_present_at_end',True),
    ('home','home_events',[]),('attitude','static_observation_passed',False),
    ('attitude','median_gravity_axis_difference_deg',179)])
def test_failed_evidence_cannot_be_rehashed_into_approval(tmp_path,monkeypatch,target,key,value):
    f,request,r,a,h=fixture(tmp_path,monkeypatch);path=a if target=='attitude' else h
    obj=json.loads(path.read_text());obj[key]=value;path.write_text(json.dumps(obj))
    r['sha256'][target]=permit.digest(path);f.write_text(json.dumps(r))
    with pytest.raises(ValueError):permit.consume(f,**request)
    assert not Path(str(f)+'.consumed.json').exists()


def test_cli_without_physical_scope_has_no_device_or_output(tmp_path):
    runner=Path(__file__).resolve().parents[1]/'trs_powered_fusion_bench.py'
    result=subprocess.run([sys.executable,str(runner),'--trs-port','/missing','--permit','/missing',
        '--backup','/missing','--heading','/missing','--board','/missing','--output',str(tmp_path/'out')],
        capture_output=True,text=True,timeout=5)
    assert result.returncode==2 and 'powered-locked-static-confirmed' in result.stderr
    assert not (tmp_path/'out').exists()


def test_retained_stop_latch_blocks_before_permit_or_device_open(tmp_path,monkeypatch):
    from types import SimpleNamespace as NS
    import trs_powered_fusion_bench as runner
    monkeypatch.setattr(runner,'FC_USB',tmp_path/'missing-usb')
    output=tmp_path/'result.json';Path(str(output)+'.home.latch.json').write_text('{}')
    with pytest.raises(ValueError,match='Retained stop latch'):
        runner.run(NS(powered_locked_static_confirmed=True,output=output))


@pytest.mark.parametrize('seconds',[-1,120.1,float('nan'),float('inf')])
def test_stop_only_watch_budget_cannot_be_expanded_before_consuming_permit(seconds):
    from types import SimpleNamespace as NS
    import trs_powered_fusion_bench as runner
    with pytest.raises(ValueError,match='0..120'):
        runner.run(NS(home_watch_seconds=seconds))
