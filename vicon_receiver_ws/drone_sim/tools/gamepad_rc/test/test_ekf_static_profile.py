from copy import deepcopy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import ekf_static_profile as p
from fixed_profile_fixtures import approved_profile, snapshot, current


def test_01_complete_typed_current_configuration_only_advances_configuration_stage():
    result=current()
    assert result['configuration_match'] and result['current_configuration_verified']
    assert not result['physical_authorization'] and not result['flight_authorized']


@pytest.mark.parametrize('name,value',[('EKF2_EV_CTRL',2),('EKF2_EV_DELAY',65.)])
def test_02_03_mismatch_is_not_timeout(name,value):
    result=current(**{name:value})
    assert not result['current_configuration_verified']
    assert result['mismatched'][0]['name']==name
    assert result['timed_out']==[]


def test_04_historical_delay_candidate_is_not_approved():
    profile=approved_profile()
    profile['parameters']['EKF2_EV_DELAY'].update(value=65.,status='pending_calibration')
    profile['delay_calibration']['status']='pending_calibration'
    result=current(profile)
    assert not result['current_configuration_verified']
    assert 'EKF2_EV_DELAY' in result['pending_calibration']


def test_05_read_timeout_does_not_become_a_value_mismatch():
    profile=approved_profile();rows=p.records_by_name(snapshot(profile))
    v={n:r['value'] for n,r in rows.items()}
    result=p.validate_current(profile,v,p.PARAMETER_TYPES,dict.fromkeys(v,1.),10.,
        identity=profile['firmware']|{'firmware':'1.14.3'})
    assert result['configuration_match'] and not result['mismatched']
    assert set(result['timed_out'])==set(p.READ_PARAMS)
    assert not result['current_configuration_verified']


@pytest.mark.parametrize('requested_time',[float('nan'),float('inf'),True,11.,None])
def test_unmatched_invalid_or_future_parameter_request_cannot_refresh_proof(requested_time):
    profile=approved_profile();v={n:s['value'] for n,s in profile['parameters'].items()}
    result=p.validate_current(profile,v,p.PARAMETER_TYPES,dict.fromkeys(v,10.),10.1,
        identity={'uid2':profile['firmware']['uid2'],'firmware':'1.14.3'},
        requested_at=dict.fromkeys(v,requested_time))
    assert not result['current_configuration_verified'] and result['timed_out']


@pytest.mark.parametrize('name',p.READ_PARAMS)
def test_each_required_parameter_is_mandatory_typed_and_reviewed(name):
    profile=approved_profile();rows=p.records_by_name(snapshot(profile));rows.pop(name)
    result=p.compare(profile,rows)
    assert dict(name=name,where='snapshot') in result['missing']
    assert not result['configuration_approved']
    rows=p.records_by_name(snapshot(profile));rows[name]['type']=9 if p.PARAMETER_TYPES[name]==6 else 6
    assert name in p.compare(profile,rows)['type_mismatch']
    profile['parameters'][name]['status']='unreviewed'
    assert not p.compare(profile,p.records_by_name(snapshot(profile)))['configuration_approved']


def test_offline_snapshot_and_parameter_echo_never_prove_current_reception_or_flash_save():
    profile=approved_profile()
    result=p.compare(profile,p.records_by_name(snapshot(profile)))
    assert result['configuration_approved'] and not result['current_configuration_verified']
    profile['persistence']['status']='unverified'
    assert not current(profile)['current_configuration_verified']


def test_change_after_persistence_receipt_invalidates_it():
    profile=approved_profile();profile['parameters']['EKF2_EV_DELAY']['value']=41.
    assert not current(profile)['persistence_evidence_present']


@pytest.mark.parametrize('value',[None,True,123,' '])
def test_approval_evidence_must_be_nonempty_text_not_a_truthy_placeholder(value):
    profile=approved_profile();profile['parameters']['EKF2_EV_DELAY']['evidence']=value
    assert not current(profile)['configuration_approved']
    profile=approved_profile();profile['delay_calibration']['evidence']=value
    assert not current(profile)['configuration_approved']


def test_old_boot_configuration_record_cannot_verify_a_new_ground_session():
    from trs_powered_fusion_policy import FixedGroundPreparation
    profile=approved_profile();gate=FixedGroundPreparation(p.fingerprint(profile))
    gate.connect(profile['firmware']|{'firmware':'1.14.3'},'new-boot',10.)
    assert not gate.verify_configuration(current(profile),10.1)


def test_draft_preserves_snapshot_values_without_auto_approving_11_or_65():
    doc=snapshot();next(r for r in doc['records'] if r['name']=='EKF2_EV_CTRL')['value']=2
    next(r for r in doc['records'] if r['name']=='EKF2_EV_DELAY')['value']=0.
    draft=p.draft_from_snapshot(doc)
    assert draft['parameters']['EKF2_EV_CTRL']['value']==2
    assert draft['parameters']['EKF2_EV_DELAY']['value']==0
    assert draft['parameters']['EKF2_EV_DELAY']['status']=='pending_calibration'
    assert not p.compare(draft,p.records_by_name(doc))['configuration_approved']


@pytest.mark.parametrize('path',['/dev/ttyUSB0','/dev/null','/proc/cpuinfo'])
def test_local_loader_cannot_treat_a_device_or_pseudo_file_as_a_snapshot(path):
    with pytest.raises((OSError,ValueError)):p.load_json(path)


def test_duplicate_names_keys_invalid_types_and_nan_are_rejected(tmp_path):
    doc=snapshot();doc['records'].append(deepcopy(doc['records'][0]))
    with pytest.raises(ValueError):p.records_by_name(doc)
    file=tmp_path/'bad.json';file.write_text('{"schema":1,"schema":1}')
    with pytest.raises(ValueError):p.load_json(file)
    file.write_text('{"value": NaN}')
    with pytest.raises(ValueError):p.load_json(file)
    profile=approved_profile();profile['parameters']['EKF2_EV_CTRL']['value']=True
    assert p.profile_errors(profile)


def test_float32_equivalence_and_wire_value_validation():
    profile=approved_profile();profile['parameters']['EKF2_EVP_NOISE']['value']=.1
    profile['persistence']['configuration_sha256']=p.configuration_digest(profile)
    rows=p.records_by_name(snapshot())
    assert p.compare(profile,rows)['configuration_match']
    rows['EKF2_EV_CTRL']['wire_value_hex']='00000000'
    assert 'EKF2_EV_CTRL' in p.compare(profile,rows)['invalid_records']


@pytest.mark.parametrize('change',['parameter','type','uid','image','evidence_path'])
def test_persistence_tracks_only_actual_fc_configuration(change):
    profile=approved_profile();before=p.configuration_digest(profile);host=p.fingerprint(profile)
    if change=='parameter':profile['parameters']['EKF2_HGT_REF']['value']=0
    elif change=='type':profile['parameters']['EKF2_EV_CTRL']['type']=9
    elif change=='uid':profile['firmware']['uid2']='0'*36
    elif change=='image':profile['firmware']['artifact_sha256']='c'*64
    else:profile['firmware']['installation_receipt']='SYNTHETIC relocated evidence'
    assert (p.configuration_digest(profile)==before)==(change=='evidence_path')
    assert p.fingerprint(profile)!=host


def test_14_different_aircraft_identity_rejects_even_matching_values():
    profile=approved_profile();rows=p.records_by_name(snapshot(profile));v={n:r['value'] for n,r in rows.items()}
    result=p.validate_current(profile,v,p.PARAMETER_TYPES,dict.fromkeys(v,10.),10.1,
        identity={'uid2':'wrong','firmware':'1.14.3'})
    assert not result['identity_match'] and not result['current_configuration_verified']


def test_complete_ev_bindings_match_current_actual_source_when_available():
    import re
    source=Path(__file__).resolve().parents[3]/'micoair_shell_firmware_20261013_ikdyTI/source'
    if not source.exists():pytest.skip('Primary firmware source not included in portable checkout')
    bindings=(source/'src/modules/ekf2/EKF2.hpp').read_text()
    for name in re.findall(r'px4::params::(EKF2_EV\w+)',bindings):
        assert name in p.PARAMETER_TYPES
    definitions='\n'.join(f.read_text() for f in (source/'src/modules').rglob('*params.c'))
    for name,kind in p.PARAMETER_TYPES.items():
        assert re.search(r'PARAM_DEFINE_'+('INT32' if kind==6 else 'FLOAT')+r'\('+name+r'\s*,',definitions),name
