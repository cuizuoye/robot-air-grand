"""Only release-gate delta; existing peers reused, no hardware or old suites."""
from dataclasses import replace
from types import SimpleNamespace as NS
from pathlib import Path

import pytest
from flight_start_confirmation import FlightRunController
from test_shared_pilot_entry import prepared
import vicon_fixed_point_entry as entry


@pytest.mark.parametrize('mode,age,armed,grant_ok',[
    (6<<16,0,False,True), (3<<16,2.01,False,True),
    (3<<16,-.01,False,True), (3<<16,0,True,True),
    (3<<16,0,False,False),
])
def test_native_gate_rejects_without_commands(mode,age,armed,grant_ok):
    c=object.__new__(FlightRunController)
    c.runtime=NS(now=10.,inputs=NS(hb=dict(custom_mode=mode,at=10.-age,armed=armed)))
    c.review_slots=NS(execution=NS(start_available=lambda:grant_ok))
    with pytest.raises(PermissionError):c.require_native_start()


def test_review_required_before_preview_and_rechecked_before_confirm(tmp_path):
    p,c=prepared(tmp_path);r=c.runtime
    c.start.native_start_check=c.require_native_start
    r.inputs.hb.update(custom_mode=3<<16,at=r.now,armed=False)
    start=len(p.port.calls)
    with pytest.raises(PermissionError,match='execution review'):c.start.create()
    c.review_slots.execution=NS(consumed=True,fault=None)  # SYNTHETIC, no grant file.
    c.review_slots.execution.start_available=lambda:c.review_slots.execution.fault is None
    preview=c.start.create()  # Already Position: no duplicate 176 needed.
    assert len(p.port.calls)==start
    c.review_slots.execution.fault='synthetic changed review'
    with pytest.raises(PermissionError,match='execution review'):
        c.start.confirm(preview['confirmation_text'])
    c.review_slots.execution.fault=None
    r.inputs.hb['custom_mode']=6<<16
    with pytest.raises(PermissionError,match='Position'):
        c.start.confirm(preview['confirmation_text'])
    assert not c.start.used and r.start_requested_at is None
    assert len(p.port.calls)==start


def test_preflight_regression_blocks_preview_and_direct_output_enable(tmp_path):
    p,c=prepared(tmp_path);r=c.runtime
    o,missing=r.inputs.build(r.now)
    assert o is not None and not missing and o.preflight_ready
    # Explicit flag must stop start even if an adapter omits its missing label.
    r.inputs.build=lambda now:(replace(o,preflight_ready=False),[])
    start=len(p.port.calls)
    with pytest.raises(PermissionError):c.start.create()
    with pytest.raises(PermissionError):r.request_start(operator_confirmed=True)
    assert not c.start.used and r.start_requested_at is None
    assert len(p.port.calls)==start


def test_plan_exposes_actual_remaining_gates_without_approving():
    d=entry.plan()
    assert not d['profile_review']['approved']
    assert 'EKF2_EV_DELAY' in d['profile_review']['pending_calibration']
    assert d['devices_opened']==d['physical_tx_bytes']==0
    assert not d['flight_authorized'] and not d['current_session_ready']
    assert d['release_conditions'][0]['status']=='SOFTWARE_READY'
    assert d['profile_review']['initial_trial_eligible']
    assert 'position_preflight_pass' in d['historical_evidence']
    assert not d['historical_evidence']['position_preflight_pass']['current_session_readiness']
    assert d['operation_order']


def test_retained_trial_evidence_is_usable_without_fabricating_calibration():
    import ekf_static_profile as fixed
    from preparation_authorization import load_fixed_backup
    root=Path(entry.__file__).parent
    profile=fixed.load_json(root/'plan_b/fixed_point/vicon_profile.initial_trial.json')
    original=fixed.load_json(root/'plan_b/fixed_point/vicon_profile.unapproved.json')
    assert profile['parameters']==original['parameters']
    assert profile['persistence']==original['persistence']
    trial=profile['initial_trial']
    fixed.inspect_initial_trial_evidence(profile,**{k:trial[k] for k in ('revision','origin','orientation')})
    result=fixed.require_mission_profile(profile,pilot_source='native_rc',height=.3,hover_seconds=3)
    assert result['initial_trial_eligible'] and not result['configuration_approved']
    assert not result['current_configuration_verified'] and not result['flight_authorized']
    assert len(load_fixed_backup(Path(trial['evidence']['backup']['path']).parent,profile,0.))==34
    with pytest.raises(PermissionError):fixed.require_approved_profile(profile)


@pytest.mark.parametrize('change',['old_origin','old_orientation','wrong_revision','evidence_hash'])
def test_rebased_profile_rejects_mixed_old_and_new_frames(change):
    import ekf_static_profile as fixed
    profile=fixed.load_json(Path(entry.__file__).parent/'plan_b/fixed_point/vicon_profile.initial_trial.json')
    trial=profile['initial_trial'];transfer=trial['rigid_frame_transfer']
    old=fixed.load_json(transfer['baseline_profile']['path'])['initial_trial']
    if change=='old_origin':trial['origin']=old['origin']
    elif change=='old_orientation':trial['orientation']=old['orientation']
    elif change=='wrong_revision':trial['revision']=old['revision']
    else:transfer['sha256']='0'*64
    with pytest.raises(ValueError,match='transfer|extrinsics'):
        fixed.inspect_initial_trial_evidence(profile,**{k:trial[k] for k in ('revision','origin','orientation')})


def test_stationary_pose_wait_survives_start_expiry_without_authorizing_flight(tmp_path):
    from flight_execution_review import ReviewedGroundBootstrapGrant,ReviewedExecutionGrant
    from test_shared_pilot_entry import reviewed_fake_grant
    p,c=prepared(tmp_path);r=c.runtime
    msg=next(m for m in reversed(p.port.calls) if m.get_type()=='VISION_POSITION_ESTIMATE')
    ground=reviewed_fake_grant(ReviewedGroundBootstrapGrant,r)
    ground.wall=lambda:201
    assert not ground.start_available()
    assert ground(r,{'at':r.now},msg),ground.last_rejection
    arm=r.encoder.command_long_encode(1,1,400,0,1,0,0,0,0,0,0)
    assert not ground(r,{'at':r.now},arm)
    flight=reviewed_fake_grant(ReviewedExecutionGrant,r);flight.wall=lambda:201
    assert not flight(r,{'at':r.now},msg)
    assert flight.last_rejection['reason']=='start_review_expired'
    p.turn(with_pose=False)
    for _ in range(20):p.turn(with_pose=False)
    assert not ground(r,{'at':r.now},msg)


@pytest.mark.parametrize('pilot,height,hover',[
    ('gamepad',.3,3),('native_rc',.51,3),('native_rc',.3,4),
    ('native_rc',float('nan'),3),('native_rc',.3,float('inf'))])
def test_initial_trial_never_opens_unbounded_or_legacy_route(pilot,height,hover):
    from fixed_profile_fixtures import initial_trial_profile
    from ekf_static_profile import require_mission_profile
    with pytest.raises(PermissionError):
        require_mission_profile(initial_trial_profile(),pilot_source=pilot,height=height,hover_seconds=hover)


def test_existing_pocket_profile_accepts_half_metre_without_changing_default():
    from fixed_profile_fixtures import initial_trial_profile
    from ekf_static_profile import require_mission_profile
    result=require_mission_profile(initial_trial_profile(),pilot_source='native_rc',height=.5,hover_seconds=3)
    assert result['initial_trial_eligible'] and not result['flight_authorized']


def test_trial_still_requires_current_typed_readback_and_immutable_baseline():
    from fixed_profile_fixtures import initial_trial_profile,current
    from ekf_static_profile import require_mission_profile,validate_current,PARAMETER_TYPES
    p=initial_trial_profile()
    assert current(p)['current_configuration_verified']
    assert not current(p,EKF2_EV_CTRL=2)['current_configuration_verified']
    values={n:s['value'] for n,s in p['parameters'].items()}
    stale=validate_current(p,values,PARAMETER_TYPES,dict.fromkeys(values,1.),10.,
        identity={'uid2':p['firmware']['uid2'],'firmware':'1.14.3'})
    assert not stale['current_configuration_verified']
    p['parameters']['EKF2_EV_DELAY']['value']=99.
    with pytest.raises(PermissionError):
        require_mission_profile(p,pilot_source='native_rc',height=.3,hover_seconds=3)


def test_trial_full_native_flight_reuses_owner_and_restores_rate(tmp_path):
    from test_native_rc_mission_delta import NativePeer,start
    p=NativePeer(tmp_path,initial_trial=True);p.intervals[111]=0
    assert not p.prepare()['failure']
    assert p.runtime.core.phase=='WAIT_EXPLICIT_START'
    configuration=p.runtime.coordinator.fusion.configuration
    assert configuration['initial_trial_eligible'] and not configuration['configuration_approved']
    assert configuration['current_configuration_verified']
    start(p)
    for _ in range(5000):
        p.turn()
        if p.controller.done:break
    assert p.controller.done and p.runtime.core.phase=='COMPLETE',p.controller.status()
    assert p.controller.journal_finalized and p.intervals[111]==0
    assert not any(msg.get_type() in ('PARAM_SET','RC_CHANNELS_OVERRIDE','MANUAL_CONTROL')
        or msg.get_type()=='COMMAND_LONG' and msg.command==185 for _,msg in p.calls)


def test_trial_cannot_substitute_software_flag_for_physical_review(tmp_path):
    from test_native_rc_mission_delta import NativePeer
    from flight_execution_review import ReviewedExecutionGrant,flight_confirmations,required_code_paths
    p=NativePeer(tmp_path,authorized=False,initial_trial=True)
    for _ in range(100):p.turn()
    assert not p.runtime.core.arm_attempted
    assert not any(msg.get_type()=='VISION_POSITION_ESTIMATE'
        or msg.get_type()=='COMMAND_LONG' and msg.command in (176,185,400) for _,msg in p.calls)
    profile=p.runtime.coordinator.fusion.profile
    required=flight_confirmations(profile)
    assert 'dynamic_vicon_timing_verified' not in required
    assert 'approximate_layout_and_untuned_delay_accepted_for_initial_trial' in required
    grant=object.__new__(ReviewedExecutionGrant)
    grant.runtime=p.runtime
    grant.record={'evidence_hashes':{ref['path']:ref['sha256'] for ref in profile['initial_trial']['evidence'].values()}}
    assert not grant._trial_evidence_ok()  # Synthetic fixture cannot authorize a real session.
    assert not any(Path(path).name=='gamepad_calib.json' for path in required_code_paths('native_rc'))


def test_trial_review_resolves_migrated_evidence_alias(tmp_path,monkeypatch):
    from flight_execution_review import ReviewedExecutionGrant, digest
    import ekf_static_profile as fixed
    actual=tmp_path/'actual.json';actual.write_text('{"evidence":1}')
    alias=tmp_path/'previous.json';alias.symlink_to(actual)
    profile={'initial_trial':{'evidence':{'layout':{'path':str(alias),'sha256':digest(actual)}},
        'origin':'fixture','orientation':'fixture'}}
    grant=object.__new__(ReviewedExecutionGrant)
    grant.runtime=NS(fixed_configuration=True,coordinator=NS(fusion=NS(profile=profile)),
        inputs=NS(rigid_revision='fixture'))
    grant.record={'evidence_hashes':{str(actual.resolve()):digest(actual)}}
    inspected=[]
    monkeypatch.setattr(fixed,'inspect_initial_trial_evidence',lambda *a,**kw:inspected.append(kw))
    assert grant._trial_evidence_ok() and len(inspected)==1
    grant.record['evidence_hashes'][str(actual.resolve())]='0'*64
    assert not grant._trial_evidence_ok()
    assert len(inspected)==1


def test_trial_evidence_failure_precedes_device_open(tmp_path,monkeypatch):
    import json
    import ekf_static_profile as fixed
    root=Path(entry.__file__).parent
    profile=fixed.load_json(root/'plan_b/fixed_point/vicon_profile.initial_trial.json')
    profile['initial_trial']['evidence']['battery_report']['sha256']='0'*64
    path=tmp_path/'trial.json';path.write_text(json.dumps(profile))
    with pytest.raises(ValueError,match='evidence changed'):
        entry.open_pocket_session(profile_path=path,output=tmp_path/'run',peer=('127.0.0.1',14550),
            bind=('127.0.0.1',14551),open_devices=True)
    assert not (tmp_path/'run').exists()
