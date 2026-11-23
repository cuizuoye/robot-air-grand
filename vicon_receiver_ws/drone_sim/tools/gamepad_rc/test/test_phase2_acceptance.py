"""New phase-2 artifact review only. Existing device-open hard stop applies."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import struct

import pytest

import ekf_static_profile as p
import ekf_ground_acceptance as a
import phase2_acceptance as cli
from fixed_profile_fixtures import approved_profile, snapshot
from test_fixed_ground_preparation import rows as fusion_rows
from fusion_binary_protocol import RESET_FIELDS
from test_compact_vision import ground_inputs

SOURCE = Path(__file__).resolve().parents[3]/'micoair_shell_firmware_20261013_ikdyTI/source'
REBOOT = {'EKF2_EV_DELAY','EKF2_HGT_REF','EKF2_AID_MASK','EKF2_MAG_TYPE'}


def full(document):
    rows = snapshot(document)['records']
    rows += [dict(name=n,type=k,value=v) for n,k,v in (
        ('CAL_ACC0_ID',6,1),('SER_TEL1_BAUD',6,115200),('PWM_MAIN_FUNC1',6,101),
        ('OTHER_FLOAT',9,.123))]
    for i,row in enumerate(rows):
        row['value']=p.typed_value(row['value'],row['type'])
        row.update(index=i,param_count=len(rows),wire_value_hex=struct.pack(
            '<i' if row['type']==6 else '<f',row['value']).hex())
    return dict(complete=True,expected_count=len(rows),received_count=len(rows),missing_indices=[],records=rows,
                system_id=1,component_id=1,firmware={'version':p.VERSION})


def persistence():
    doc = approved_profile()
    # Test source-consistent defaults; not an approved real profile.
    doc['parameters']['EKF2_AID_MASK']['value'] = 0
    doc['parameters']['EKF2_HGT_REF']['value'] = 3
    doc['parameters']['EKF2_GPS_CTRL']['value'] = 0
    before = full(doc)
    after = deepcopy(before)
    identity = dict(uid2=doc['firmware']['uid2'],firmware=p.VERSION,
                    target=p.TARGET,artifact_sha256=doc['firmware']['artifact_sha256'])
    evidence = dict(before_sha256=p.fingerprint(before),after_sha256=p.fingerprint(after),
        identity_before=identity,identity_after=deepcopy(identity),boot_before='old',boot_after='new',
        host_clock_session='SYNTHETIC-HOST',before_complete_ns=1,save_started_ns=2,
        save_completed_ns=3,restart_observed_ns=4,after_complete_ns=5,
        save_method='nsh_param_save_default_sync',save_exit_code=0,save_transcript_sha256='a'*64,
        operator_reviewed=True)
    return doc,before,after,evidence


def no_grant(report):
    for name in ('physical_authorization','flight_authorized','control_ready','live_enabled'):
        assert report[name] is False
    assert report['devices_opened'] == 0


def capture():
    audits = fusion_rows()
    observations,sent=ground_inputs()
    c = dict(identity='SYNTHETIC_UID',boot='SYNTHETIC_BOOT',first_position_send_ns=1_000_000_000,
        coordinate_review=dict(reviewed=True,local_axes='x_y_z_down_yaw_aligned',
            transform_sha256='a'*64,rigid_body_revision_sha256='b'*64),
        receipt=dict(identity='SYNTHETIC_UID',boot='SYNTHETIC_BOOT',session_origin_ns=1_000_000_000,
                     observations=observations,actual_sends=sent), audits=audits,
        yaw_comparisons=[r['yaw_comparison'] for r in audits],
        audit_context=[dict(at_ns=1_800_000_000+i*100_000_000,identity='SYNTHETIC_UID',boot='SYNTHETIC_BOOT') for i in range(3)],
        limits=dict(reviewed=True,evidence='SYNTHETIC reviewed limits only',position_step_m=.1,
                    height_step_m=.05,yaw_step_rad=.1,settled_position_span_m=.02))
    c['samples'] = [dict(at_ns=at,source_us=at,identity=c['identity'],boot=c['boot'],
        position_local_m=[0.,0.,0.],yaw_rad=0.,ev_fused=at>=1_000_000_000,
        reset_counters=dict.fromkeys(RESET_FIELDS,0)) for at in (500_000_000,1_800_000_000,1_900_000_000,2_000_000_000)]
    return c


def test_inventory_matches_pinned_source_and_never_approves_example():
    doc=p.load_json(Path(__file__).parents[1]/'plan_b/fixed_profile.unapproved.example.json')
    r=a.inventory(doc,SOURCE)
    assert len(r['parameters'])==34 and r['approved_count']==0
    assert {v['name'] for v in r['parameters'] if v['reboot_required']} == REBOOT
    assert all(v['value_missing'] for v in r['parameters'])
    no_grant(r)


@pytest.mark.parametrize('values,issue',[
    ({'EKF2_HGT_REF':0,'EKF2_BARO_CTRL':0},'height_reference_would_enable_baro'),
    ({'EKF2_HGT_REF':1,'EKF2_GPS_CTRL':0},'height_reference_would_enable_gps'),
    ({'EKF2_HGT_REF':2,'EKF2_RNG_CTRL':0},'height_reference_would_enable_range'),
    ({'EKF2_HGT_REF':3,'EKF2_EV_CTRL':1},'height_reference_would_enable_ev_vertical'),
    ({'EKF2_GPS_CTRL':2},'gps_vertical_without_horizontal_would_be_cleared'),
    *[({'EKF2_AID_MASK':1<<b},'deprecated_aid_mask_would_migrate_and_commit') for b in (0,1,2,3,4,5,7,8)]])
def test_detects_px4_auto_commit_without_modifying_input(values,issue):
    old=deepcopy(values)
    assert issue in a.dependency_issues(values)
    assert values==old


def test_full_34_match_reboot_readback_review_not_hardware_proof():
    r=a.persistence_review(*persistence())
    assert r['review_complete'],r
    assert r['protected_parameters_unchanged']
    assert not r['persistence_proven_by_this_program']
    no_grant(r)


def test_historical_integral_json_float_requires_exact_typed_wire_not_rounding():
    doc,before,after,evidence=persistence()
    row=after['records'][0]
    row['value']=float(row['value'])
    assert a.full_snapshot(after)[row['name']]['value']==int(row['value'])
    row['value']+=.1
    with pytest.raises(ValueError):a.full_snapshot(after)


@pytest.mark.parametrize('change',[
    ('save_method','PARAM_VALUE'),('save_method','PREFLIGHT_STORAGE_ACK'),
    ('save_exit_code',1),('save_exit_code',True),('save_completed_ns',None),
    ('save_completed_ns',2),('save_transcript_sha256','bad'),('boot_after','old'),
    ('host_clock_session',None),('operator_reviewed',False),('after_sha256','0'*64)])
def test_save_ack_wait_claim_or_missing_reboot_never_persistence(change):
    doc,before,after,evidence=persistence()
    evidence[change[0]]=change[1]
    r=a.persistence_review(doc,before,after,evidence)
    assert not r['review_complete']
    no_grant(r)


@pytest.mark.parametrize('name', ['CAL_ACC0_ID','SER_TEL1_BAUD','PWM_MAIN_FUNC1','COM_RC_IN_MODE','EKF2_EV_DELAY'])
def test_post_reboot_unexpected_change_fails_even_after_successful_save(name):
    doc,before,after,evidence=persistence()
    row=next(r for r in after['records'] if r['name']==name)
    row['value']+=1
    row['wire_value_hex']=struct.pack('<i' if row['type']==6 else '<f',row['value']).hex()
    evidence['after_sha256']=p.fingerprint(after)
    r=a.persistence_review(doc,before,after,evidence)
    assert not r['review_complete']
    assert name in r['changed_parameters']


@pytest.mark.parametrize('fault',['missing','type','wire','duplicate','index','count'])
def test_incomplete_or_invalid_full_readback(fault):
    doc,before,after,evidence=persistence()
    if fault=='missing':after['records'].pop()
    elif fault=='type':after['records'][0]['type']=9
    elif fault=='wire':after['records'][0]['wire_value_hex']='ffff'
    elif fault=='duplicate':after['records'][1]['name']=after['records'][0]['name']
    elif fault=='index':after['records'][1]['index']=0
    else:after['complete']=False
    assert not a.persistence_review(doc,before,after,evidence)['review_complete']


def test_delay_unapproved_or_uid_changed_cannot_complete_review():
    for fault in ('delay','uid'):
        doc,before,after,evidence=persistence()
        if fault=='delay':doc['delay_calibration']['status']='pending_calibration'
        else:evidence['identity_after']['uid2']='other'
        r=a.persistence_review(doc,before,after,evidence)
        assert not r['review_complete']
        no_grant(r)


def test_first_fusion_reuses_actual_fusion_predicates_not_just_receipt():
    r=a.first_fusion_review(capture())
    assert r['review_complete'],r
    no_grant(r)


@pytest.mark.parametrize('fault',['receipt_only','jump','yaw','height','later_reset','no_baseline',
                                  'cached','boot','identity','unknown_limit','aid_rejected','audit_old',
                                  'reset_unknown','transform_unknown','two_audits'])
def test_initial_fusion_evidence_failures(fault):
    c=capture()
    if fault=='receipt_only':c['audits']=[]
    elif fault=='jump':c['samples'][1]['position_local_m'][0]=.5
    elif fault=='height':c['samples'][1]['position_local_m'][2]=.2
    elif fault=='yaw':c['samples'][1]['yaw_rad']=1.
    elif fault=='later_reset':c['samples'][-1]['reset_counters'][RESET_FIELDS[0]]=1
    elif fault=='no_baseline':c['samples']=c['samples'][1:]
    elif fault=='cached':c['samples'][-1]['source_us']=c['samples'][-2]['source_us']
    elif fault=='boot':c['samples'][-1]['boot']='reboot'
    elif fault=='identity':c['samples'][-1]['identity']='other'
    elif fault=='unknown_limit':c['limits']['reviewed']=False
    elif fault=='aid_rejected':c['audits'][-1]['aid']['innovation_rejected']=True
    elif fault=='reset_unknown':c['samples'][0]['reset_counters'][RESET_FIELDS[0]]=None
    elif fault=='transform_unknown':c['coordinate_review']['reviewed']=False
    elif fault=='two_audits':c['audits'].pop();c['audit_context'].pop();c['yaw_comparisons'].pop()
    else:c['audit_context'][0]['at_ns']=1
    r=a.first_fusion_review(c)
    assert not r['review_complete'],r
    no_grant(r)


def test_offline_cli_exclusive_output_and_no_live_options(tmp_path,capsys):
    f=tmp_path/'profile.json';f.write_text(json.dumps(approved_profile()))
    out=tmp_path/'new.json'
    args=['--output',str(out),'inventory','--profile',str(f),'--source-root',str(SOURCE)]
    assert cli.main(args)==0
    no_grant(json.loads(out.read_text()))
    with pytest.raises(SystemExit):cli.main(args)
    for option in ('--port','--connect','--home','--write','--arm','--offboard'):
        with pytest.raises(SystemExit):cli.main([option])


def test_phase1_safety_and_mission_files_not_changed():
    base=Path(__file__).parents[1]
    hashes={
        'mission_fusion_service.py':'980c85e0483f96d2e4cd0ecace895b36f0df5529e5f5143bb2e4334c67a12497',
        'flight_mission_core.py':'cd2f6d99838baf189d5738e1588c3a5fcb312f1fc70c7a593c6575430c14c835',
        'lock_clock_probe.py':'54c522b37c5886eb146b5ef509be4176cdc10e4492032a705b837b7a6b0c94ff',
        'output_lock_status.py':'14e95ba69c451d5c361243b302f449c1465cce9dfc4817f68d98a53a9cd91aff'}
    for name,digest in hashes.items():assert hashlib.sha256((base/name).read_bytes()).hexdigest()==digest
