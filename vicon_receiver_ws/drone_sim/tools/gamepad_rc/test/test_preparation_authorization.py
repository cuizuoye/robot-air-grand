"""Authorization/journal tests use temporary files and synthetic FC state only."""
from copy import deepcopy
from pathlib import Path
import json
import os
import struct
import sys
import time
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import preparation_authorization as p
import trs_runtime_monitor
from mission_preparation_io import PreparationIO
from test_mission_preparation_io import pending_parameter,attach,decode
from test_mission_preparation_runtime import prepared_peer
from test_trs_vision_reference import baseline


@pytest.fixture
def files(tmp_path,monkeypatch):
    heading,board,proof=[tmp_path/name for name in ('heading','board','proof')]
    for path in (heading,board,proof):path.write_text('{}')
    backup=tmp_path/'backup';backup.mkdir()
    values=baseline();rows=[]
    for i,(name,value) in enumerate({**values,**{f'EXTRA_{j}':0 for j in range(926-len(values))}}.items()):
        kind=9 if name=='EKF2_EV_DELAY' else 6
        rows.append(dict(name=name,index=i,type=kind,value=value,
                         wire_value_hex=struct.pack('<f' if kind==9 else '<i',value).hex()))
    doc=dict(complete=True,expected_count=926,received_count=926,system_id=1,component_id=1,
             firmware={'version':'1.14.3'},records=rows)
    (backup/'parameters_verified.json').write_text(json.dumps(doc))
    (backup/'parameters_manifest.json').write_text(json.dumps({'sha256':{'parameters_verified.json':p.digest(backup/'parameters_verified.json')}}))
    # Use temp holds/registry rather than modifying production incident records.
    incident,hold,registry=[tmp_path/name for name in ('incident','hold','registry')]
    incident.write_text('{"block_active_benches": true}')
    hold.write_text('{"block_props_off_benches": true}');registry.write_text('{}')
    original=p.evidence_paths
    def paths(h,b,s,back,registry_path=None):
        result=original(h,b,s,back)
        result.update(incident=incident,physical_hold=hold,registry=registry)
        return result
    monkeypatch.setattr(p,'evidence_paths',paths)
    monkeypatch.setattr(trs_runtime_monitor,'inspect_evidence',lambda *a:{'active_revision_id':'offline-revision'})
    return NS(root=tmp_path,heading=heading,board=board,proof=proof,backup=backup,
              incident=incident,hold=hold,run=tmp_path/'run')


def approved(files,*,mutate=None):
    doc=p.draft(files.run,files.heading,files.board,files.proof,files.backup)
    now=time.time();doc.update(user_authorized=True,issued_unix=now-1,expires_unix=now+120,
        physical_confirmations=dict(battery_disconnected=True,usb_only=True,stationary=True,props_treated_as_installed=True))
    if mutate:mutate(doc)
    path=files.root/'permit.json';path.write_text(json.dumps(doc))
    return path


def test_draft_is_not_a_live_approval(files):
    doc=p.draft(files.run,files.heading,files.board,files.proof,files.backup)
    assert doc['user_authorized'] is False and doc['issued_unix'] is None
    path=files.root/'draft.json';path.write_text(json.dumps(doc))
    peer=prepared_peer()
    with pytest.raises(ValueError,match='Unapproved'):
        p.consume(path,peer.runtime,files.run,files.heading,files.board,files.proof,files.backup)
    assert not Path(str(path)+'.consumed.json').exists()


def test_consumption_is_one_use_bound_and_never_clears_holds(files):
    peer=prepared_peer();path=approved(files)
    grant=p.consume(path,peer.runtime,files.run,files.heading,files.board,files.proof,files.backup)
    assert grant.boot==peer.runtime.inputs.boot_session
    assert json.loads(files.incident.read_text())['block_active_benches']
    with pytest.raises(FileExistsError):
        p.consume(path,peer.runtime,files.run,files.heading,files.board,files.proof,files.backup)


@pytest.mark.parametrize('change',[
    lambda d:d.update(scope='flight'),lambda d:d.update(user_authorized=False),
    lambda d:d.update(expires_unix=d['issued_unix']-1),
    lambda d:d['physical_confirmations'].update(battery_disconnected=False),
    lambda d:d['physical_confirmations'].update(battery_disconnected=1),
    lambda d:d['hashes'].update(heading='changed')])
def test_bad_scope_permission_time_physics_or_hash_is_rejected(files,change):
    path=approved(files,mutate=change);peer=prepared_peer()
    with pytest.raises(ValueError):p.consume(path,peer.runtime,files.run,files.heading,files.board,files.proof,files.backup)
    assert not Path(str(path)+'.consumed.json').exists()


def test_fresh_full_backup_is_required_not_archived_readiness(files):
    path=files.backup/'parameters_verified.json'
    os.utime(path,(time.time()-7200,time.time()-7200))
    with pytest.raises(ValueError,match='Fresh complete'):p.load_backup(files.backup,time.time())


def test_bad_backup_types_are_rejected_even_with_updated_manifest(files):
    path=files.backup/'parameters_verified.json';doc=json.loads(path.read_text())
    next(row for row in doc['records'] if row['name']=='EKF2_EV_DELAY')['type']=6
    path.write_text(json.dumps(doc))
    (files.backup/'parameters_manifest.json').write_text(json.dumps({'sha256':{path.name:p.digest(path)}}))
    with pytest.raises(ValueError,match='types'):p.load_backup(files.backup,time.time())


def test_evidence_seal_is_fresh_bounded_and_change_latches(tmp_path):
    path=tmp_path/'code';path.write_text('a');clock=NS(value=1.)
    seal=p.EvidenceSeal({'code':path},{'code':p.digest(path)},clock=lambda:clock.value)
    assert seal.refresh() and seal.fresh()
    clock.value=2.001;assert not seal.fresh()
    path.write_text('b');assert not seal.refresh() and seal.fault
    path.write_text('a');assert not seal.refresh()  # Never auto-recover changed code.


def test_grant_uses_current_context_and_wall_lag_not_a_true_callback(files):
    peer=prepared_peer();r=peer.runtime;path=approved(files)
    clock=NS(value=peer.now)
    grant=p.consume(path,r,files.run,files.heading,files.board,files.proof,files.backup,clock=lambda:clock.value)
    grant.record['expires_unix']=time.time()+120
    record=next(row for row in reversed(r.frames) if row['type']=='VISION_POSITION_ESTIMATE')
    msg=decode(bytes.fromhex(record['wire_hex']))
    assert grant(r,record,msg)
    clock.value+=.051;assert not grant(r,record,msg)
    clock.value=peer.now;r.inputs.fail('changed_boot')
    assert not grant(r,record,msg)


def test_journal_durable_before_write_and_uncertainty_cannot_be_forgotten(files):
    peer,record=pending_parameter();r=peer.runtime
    journal=p.RecoveryJournal(files.root/'recovery.json')
    io,port,_,_=attach(peer,authorize=lambda *a:True,journal=journal)
    port.short=True
    with pytest.raises(OSError):io._dispatch(record)
    state=json.loads(journal.path.read_text())
    assert state['needs_restore'] and state['events'][0]['name']=='EKF2_EV_DELAY'
    io.failure='partial_write';r.closed=True
    assert not journal.finalize(r,io)
    assert json.loads(journal.path.read_text())['needs_restore']


def test_recovery_scan_blocks_pending_and_corrupt_records(tmp_path):
    folder=tmp_path/'old';folder.mkdir();file=folder/'recovery.json'
    file.write_text('{"needs_restore": true}')
    with pytest.raises(ValueError,match='needs recovery'):p.require_no_pending_recovery(tmp_path)
    file.write_text('{"needs_restore": false}');p.require_no_pending_recovery(tmp_path)
    pending=folder/'recovery.json.pending';pending.write_text('{}')
    with pytest.raises(ValueError,match='Incomplete'):p.require_no_pending_recovery(tmp_path)


def test_parameter_authorization_is_rechecked_after_journal_delay():
    peer,record=pending_parameter();decisions=iter((True,False))
    io,port,_,_=attach(peer,authorize=lambda *a:next(decisions),journal=lambda row:True)
    with pytest.raises(PermissionError,match='before write'):io._dispatch(record)
    assert not port.writes and io.possibly_changed


def test_installed_ground_grant_accepts_only_current_pose_on_memory_port(files):
    peer=prepared_peer();r=peer.runtime;path=approved(files);clock=NS(value=peer.now)
    grant=p.consume(path,r,files.run,files.heading,files.board,files.proof,files.backup,clock=lambda:clock.value)
    journal=p.RecoveryJournal(files.root/'recovery.json')
    io,port,t,ns=attach(peer,authorize=grant,journal=journal)
    clock.value=t
    result=io.step(t,lambda:ns)
    assert result['failure'] is None and result['physical_tx_bytes']>0
    assert decode(port.writes[0]).get_type()=='VISION_POSITION_ESTIMATE'
    assert not result['flight_authorized']


def test_interrupt_during_possible_parameter_write_retains_recovery_state(files):
    peer,record=pending_parameter();journal=p.RecoveryJournal(files.root/'recovery.json')
    io,port,_,_=attach(peer,authorize=lambda *a:True,journal=journal)
    def interrupted(wire):raise KeyboardInterrupt
    port.write=interrupted
    with pytest.raises(KeyboardInterrupt):io._dispatch(record)
    assert io.failure and io.status()['parameter_recovery_required']
    assert json.loads(journal.path.read_text())['needs_restore']
