"""Complete synthetic archives; never read a device or real parameter dump."""
import json
import os
from pathlib import Path
import struct
import sys
import time

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import preparation_authorization as auth
import ekf_static_profile as profile
from fixed_profile_fixtures import approved_profile,snapshot


@pytest.fixture
def backup(tmp_path):
    approved=approved_profile();rows=snapshot(approved)['records']
    for row in rows:
        row['wire_value_hex']=struct.pack('<i' if row['type']==6 else '<f',row['value']).hex()
    rows += [dict(name=f'SYNTHETIC_EXTRA_{i}',type=6,value=0,wire_value_hex='00000000')
             for i in range(926-len(rows))]
    for i,row in enumerate(rows):row.update(index=i,param_count=len(rows))
    path=tmp_path/'parameters_verified.json'
    record=dict(complete=True,expected_count=926,received_count=926,missing_indices=[],
        system_id=1,component_id=1,firmware={'version':'1.14.3'},records=rows)
    path.write_text(json.dumps(record))
    manifest=tmp_path/'parameters_manifest.json'
    manifest.write_text(json.dumps({'sha256':{path.name:auth.digest(path)}}))
    return approved,path,manifest,record


def test_full_fixed_backup_does_not_use_or_relax_old_dynamic_baseline(backup):
    approved,path,_,_=backup
    values=auth.load_fixed_backup(path.parent,approved,time.time())
    assert set(values)==set(profile.READ_PARAMS) and values['EKF2_EV_DELAY']==37.
    with pytest.raises(ValueError):auth.load_backup(path.parent,time.time())


@pytest.mark.parametrize('failure',['hash','duplicate_index','wrong_type','unapproved_delay',
    'missing_row','count_drift','missing_count','extra_wire_mismatch','extra_bad_name','count_bool'])
def test_bad_backup_never_becomes_configuration_approval(backup,failure):
    approved,path,manifest,record=backup
    if failure=='hash':manifest.write_text('{"sha256": {}}')
    elif failure=='unapproved_delay':approved['parameters']['EKF2_EV_DELAY']['status']='pending_calibration'
    else:
        if failure=='duplicate_index':record['records'][-1]['index']=0
        elif failure=='wrong_type':record['records'][0]['type']=9
        elif failure=='missing_row':record['records'].pop()
        elif failure=='count_drift':record['records'][-1]['param_count']=925
        elif failure=='missing_count':record['records'][-1].pop('param_count')
        elif failure=='extra_wire_mismatch':record['records'][-1]['value']=1
        elif failure=='extra_bad_name':record['records'][-1]['name']=None
        elif failure=='count_bool':record['expected_count']=True
        path.write_text(json.dumps(record))
        manifest.write_text(json.dumps({'sha256':{path.name:auth.digest(path)}}))
    with pytest.raises(ValueError):auth.load_fixed_backup(path.parent,approved,time.time())


@pytest.mark.parametrize('count',[921,926,930])
def test_complete_archived_backup_reused_but_never_current_readiness(backup,count):
    approved,path,manifest,record=backup
    rows=record['records'][:count]
    while len(rows)<count:
        rows.append(dict(name=f'NEW_SYNTHETIC_{len(rows)}',type=6,value=0,wire_value_hex='00000000'))
    for i,row in enumerate(rows):row.update(index=i,param_count=count)
    record.update(records=rows,expected_count=count,received_count=count)
    path.write_text(json.dumps(record));manifest.write_text(json.dumps({'sha256':{path.name:auth.digest(path)}}))
    os.utime(path,(time.time()-86400*30,time.time()-86400*30))
    values=auth.load_fixed_backup(path.parent,approved,time.time())
    # An intact archive is accepted, but aged or absent live replies are not.
    result=profile.validate_current(approved,values,profile.PARAMETER_TYPES,
        dict.fromkeys(values,10.),20.,identity={'uid2':approved['firmware']['uid2'],'firmware':'1.14.3'},boot_session='new')
    assert not result['current_configuration_verified'] and result['timed_out']
