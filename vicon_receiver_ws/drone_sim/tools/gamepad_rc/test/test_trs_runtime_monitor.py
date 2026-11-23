from pathlib import Path
import sys
import pytest
from pymavlink.dialects.v20 import common as m
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import trs_runtime_monitor as p
from estop_only_policy import shell_message


@pytest.mark.parametrize('command,param1',[(400,1),(400,0),(176,1),(21,0),(22,0),(185,0),
    (185,1),(185,3),(246,1),(511,1)])
def test_all_flight_changes_unlock_and_rates_blocked(command,param1):
    msg=m.MAVLink(None).command_long_encode(1,1,command,0,param1,0,0,0,0,0,0)
    assert not p.allowed(msg,identity_fresh=True,home_latched=True)


def test_stop_requires_current_identity_and_home_latch():
    msg=m.MAVLink(None).command_long_encode(1,1,185,0,2,0,0,0,0,0,0)
    assert p.allowed(msg,identity_fresh=True,home_latched=True)
    assert not p.allowed(msg,identity_fresh=True)
    assert not p.allowed(msg,home_latched=True)


def test_fixed_reads_and_no_injection():
    e=m.MAVLink(None)
    assert p.allowed(shell_message(e),identity_fresh=True)
    assert not p.allowed(shell_message(e))
    for name in p.PARAMS:
        assert p.allowed(e.param_request_read_encode(1,1,name.encode(),-1),identity_fresh=True)
    assert not p.allowed(e.param_request_read_encode(1,1,b'UNRELATED',-1),identity_fresh=True)
    assert not p.allowed(e.param_set_encode(1,1,b'EKF2_EV_CTRL',11,6),identity_fresh=True,home_latched=True)
    assert not p.allowed(e.vision_position_estimate_encode(1,0,0,0,0,0,0),identity_fresh=True,home_latched=True)
    assert not p.allowed(e.set_position_target_local_ned_encode(1,1,1,1,2552,*([0]*11)),identity_fresh=True,home_latched=True)


def test_freshness_and_rate_are_observations_not_assumed():
    assert p.observation_age(1,None) is None
    assert p.observation_age(1,2) is None
    assert p.observation_age(1,.7)==pytest.approx(.3)
    assert p.observed_rate([]) is None
    assert p.observed_rate([0,1,2])==1
    assert p.observed_rate([0,.1,.2])==10


@pytest.mark.parametrize('changed',['inode','rdev','sysfs_device'])
def test_gamepad_replacement_cannot_reuse_event_number_or_old_fd(monkeypatch,changed):
    from types import SimpleNamespace
    original={'rdev':1,'inode':10,'sysfs_device':'/sys/old'}
    current={**original,changed:'/sys/new' if changed=='sysfs_device' else 20}
    monkeypatch.setattr(p,'gamepad_identity',lambda path:current)
    monkeypatch.setattr(p.os,'fstat',lambda fd:SimpleNamespace(st_rdev=1,st_ino=10))
    with pytest.raises(RuntimeError,match='re-enumerated'):
        p.require_gamepad_identity(999,'/fake/event21',original)


def test_open_race_with_different_inode_is_rejected(monkeypatch):
    from types import SimpleNamespace
    current={'rdev':1,'inode':20,'sysfs_device':'/sys/new'}
    monkeypatch.setattr(p,'gamepad_identity',lambda path:current)
    monkeypatch.setattr(p.os,'fstat',lambda fd:SimpleNamespace(st_rdev=1,st_ino=10))
    with pytest.raises(RuntimeError,match='new session required'):
        p.require_gamepad_identity(999,'/fake/event21',current)


def test_unchanged_gamepad_generation_is_accepted(monkeypatch):
    from types import SimpleNamespace
    current={'rdev':1,'inode':10,'sysfs_device':'/sys/old'}
    monkeypatch.setattr(p,'gamepad_identity',lambda path:current)
    monkeypatch.setattr(p.os,'fstat',lambda fd:SimpleNamespace(st_rdev=1,st_ino=10))
    p.require_gamepad_identity(999,'/fake/event21',current)


def test_physical_mode_mismatch_rejected(tmp_path,monkeypatch):
    monkeypatch.setattr(p,'FC_USB',tmp_path/'missing')
    p.check_physical('battery-only')
    with pytest.raises(RuntimeError):p.check_physical('usb-only')
    (tmp_path/'missing').touch()
    p.check_physical('usb-only')
    with pytest.raises(RuntimeError):p.check_physical('battery-only')


def test_recovered_heartbeat_or_source_does_not_clear_readiness_fault():
    args=dict(baseline_seen=True,joint_seen=True,identity_fresh=True,source_fresh=True)
    assert p.readiness_fault(None,**args) is None
    fault=p.readiness_fault(None,**{**args,'identity_fresh':False})
    assert fault=='fc_heartbeat_stale_after_ready'
    assert p.readiness_fault(fault,**args)==fault
    source=p.readiness_fault(None,**{**args,'source_fresh':False})
    assert source=='vicon_source_lost_after_ready'
    assert p.readiness_fault(source,**args)==source
    assert p.readiness_fault(None,baseline_seen=False,joint_seen=False,
                             identity_fresh=False,source_fresh=False) is None


def test_home_confirmation_timeout_is_a_fault_not_an_exception():
    assert p.home_observation_fault(20, None, False) is None
    assert p.home_observation_fault(10, 0, False) is None
    assert p.home_observation_fault(10.01, 0, False) == 'home_lock_confirmation_timeout'
    assert p.home_observation_fault(20, 0, True) is None


def test_isolated_frame_policy_never_certifies_damaged_reply_or_clears_history():
    from actuator_query import ActuatorQuery
    q=ActuatorQuery();q.action(0,True);q.damaged_packet(.1)
    assert p.query_readiness_fault(q)=='actuator_reply_damaged_packet'
    assert p.query_readiness_fault(q,True) is None
    assert q.live_lockdown(.2) is None and q.draining
    q.action(6.2,True)  # Lost terminal prompt still permanently blocks queries.
    assert q.disabled
    assert p.query_readiness_fault(q,True)=='actuator_reply_damaged_packet'
    assert len(q.failures)==1


@pytest.mark.parametrize('reason',['actuator_reply_timeout','actuator_reply_malformed',
                                  'actuator_reply_stale_or_repeated','actuator_reply_overflow'])
def test_tolerance_is_not_an_exception_for_non_crc_diagnostic_faults(reason):
    from actuator_query import ActuatorQuery
    q=ActuatorQuery();q.action(0,True);q.fail(reason,.1)
    assert p.query_readiness_fault(q,True)==reason


def test_retained_workspace_relative_evidence_works_from_another_directory(tmp_path,monkeypatch):
    import json
    root=tmp_path/'workspace';(root/'evidence').mkdir(parents=True)
    heading,board=root/'heading.json',root/'board.json'
    heading.write_text('{}');board.write_text('{}')
    proof=root/'evidence/proof.json';permit=root/'evidence/permit.json'
    permit.write_text(json.dumps({'output_path':'evidence/proof.json',
        'heading_sha256':p.digest(heading),'board_sha256':p.digest(board)}))
    Path(str(permit)+'.consumed.json').write_text(json.dumps({'permit_sha256':p.digest(permit)}))
    proof.write_text(json.dumps({'vision_reference_permit_path':'evidence/permit.json',
        'rigid_body_revision_id':'r1','rigid_revision_static_validation':True,
        'ekf_stationary_xyz_yaw_fusion_verified':True,'error':None,'restore_errors':[],
        'arming_mode_motion_or_unlock_commands_sent':False,
        'restored':{'EKF2_EV_CTRL':True,'EKF2_EV_DELAY':True},
        'actuator_after':{'armed':False,'lockdown':True},
        'identities':{r:{'uid2':p.UID2} for r in ('usb','trs')},
        'heading_candidate_path':'heading.json','board_center_candidate_path':'board.json'}))
    monkeypatch.setattr(p,'__file__',str(root/'tools/gamepad_rc/trs_runtime_monitor.py'))
    monkeypatch.setattr(p,'inspect_current_calibration',lambda *args:{'active_revision_id':'r1'})
    monkeypatch.chdir(tmp_path)
    assert p.inspect_evidence(heading,board,proof)['active_revision_id']=='r1'
