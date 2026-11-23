"""Full guarded writer + current synthetic FC replies; never real devices."""
from pathlib import Path
import json
import sys
from types import SimpleNamespace as NS

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from continuous_execution_io import ContinuousExecutionIO
from flight_execution_journal import FlightExecutionJournal
from flight_start_confirmation import FlightStartConfirmation,FlightRunController,build_guarded_session
from flight_execution_review import (draft,ReviewedExecutionGrant,REQUIRED,draft_ground,
    ReviewedGroundBootstrapGrant,GROUND_REQUIRED,ExecutionReviewSlots)
from preparation_authorization import require_no_pending_recovery
from synthetic_mission_peer import SyntheticPeer,synthetic_baseline
from mavlink_crc_stream import CrcResyncParser
from test_continuous_mission_runtime import prepared


class FakePort:
    timeout=write_timeout=0
    def __init__(self):self.records=[];self.calls=[];self.fail_arm=False;self.guard=None;self.rx=b''
    @property
    def in_waiting(self):return len(self.rx)
    def read(self,n):
        data,self.rx=self.rx[:n],self.rx[n:]
        return data
    def write(self,wire):
        msg=CrcResyncParser().parse_buffer(wire)[0]
        self.calls.append(msg)
        if self.guard:self.guard(msg)
        if self.fail_arm and msg.get_type()=='COMMAND_LONG' and msg.command==400:return len(wire)-1
        self.records.append(dict(wire_hex=wire.hex()))
        return len(wire)


class TransportPeer(SyntheticPeer):
    def output_records(self):return list(self.port.records),len(self.port.records)
    def run_owner_turn(self,now,**kwargs):
        self.input_args=kwargs
        self.port.rx=kwargs['rx']
        if getattr(self,'controller',None):self.controller.step(now)
        else:self.io.step(now)
        return self.runtime.status()


def wired(tmp_path,*,authorized=True,scenario='nominal',with_journal=True):
    p=TransportPeer(scenario=scenario);r=p.runtime;p.port=FakePort()
    journal=FlightExecutionJournal(tmp_path/'flight_recovery.json',boot_session=r.inputs.boot_session,
        rigid_revision=r.inputs.rigid_revision) if with_journal else None
    io=ContinuousExecutionIO(r,p.port,poll_gamepad=lambda:{k:p.input_args[k] for k in
        ('pad_generation','pad_present','pad_event','home_events')},
        poll_pose=lambda:p.input_args['pose'],now_ros_ns=lambda:p.input_args['now_ros_ns'],
        check_port_identity=lambda:None,usb_present=lambda:False,
        authorizer=(lambda *args:True) if authorized else None,journal=journal)
    p.io=io
    # Test authorizer is local to this fake peer only. No production grant or
    # physical-review file is created by this helper.
    return p,r,io,journal


def test_default_no_authorizer_cannot_send_a_home_or_prepare_flight(tmp_path):
    p,r,io,journal=wired(tmp_path,authorized=False)
    p.prepare()
    assert r.failure and not r.prepare_requested
    assert all(msg.get_type()!='COMMAND_LONG' or msg.command==512 for msg in p.port.calls)
    assert not journal.record['events']


def test_missing_durable_journal_cannot_be_replaced_by_true_authorizer(tmp_path):
    p,r,io,journal=wired(tmp_path,with_journal=False)
    p.prepare()
    assert r.failure and not any(msg.get_type()=='COMMAND_LONG' and msg.command==185 for msg in p.port.calls)


def test_nominal_real_wire_pipeline_full_journal_and_explicit_ticket(tmp_path):
    p,r,io,journal=wired(tmp_path)
    def durable(msg):
        if msg.get_type() in ('PARAM_SET','VISION_POSITION_ESTIMATE','SET_POSITION_TARGET_LOCAL_NED') or (
                msg.get_type()=='COMMAND_LONG' and msg.command!=512):
            data=json.loads(journal.path.read_text())
            assert data['needs_restore'] and data['needs_operator_review']
            assert not Path(str(journal.path)+'.pending').exists()
    p.port.guard=durable
    assert not p.prepare()['failure']
    p.controller=FlightRunController(r,io)
    ticket=p.controller.start;preview=ticket.create()
    assert r.core.phase=='WAIT_EXPLICIT_START' and not r.core.arm_attempted
    assert ticket.confirm(preview['confirmation_text'])
    for _ in range(2000):
        p.turn()
        if r.closed:break
    assert r.closed and r.core.phase=='COMPLETE' and p.params==synthetic_baseline()
    assert len(p.port.records)==io.complete_messages
    assert io.actual_tx_bytes==sum(len(bytes.fromhex(row['wire_hex'])) for row in p.port.records)
    assert r.writer is r.coordinator.writer is r.core.sink is r.coordinator.fusion.sink is io.writer
    assert p.controller.done and p.controller.journal_finalized and not journal.record['needs_restore']
    assert FlightExecutionJournal.inspect(journal.path)['verified_closed']
    with pytest.raises(PermissionError):ticket.confirm(preview['confirmation_text'])


@pytest.mark.parametrize('change',['wrong_text','expired','position','yaw','boot','home','selector'])
def test_start_ticket_is_current_one_use_and_cannot_rebase(change):
    p=prepared();r=p.runtime;t=FlightStartConfirmation(r);preview=t.create()
    if change=='wrong_text':text='TAKEOFF wrong'
    else:text=preview['confirmation_text']
    if change=='expired':t.preview['expires_at']=r.now-.01
    if change=='position':r.inputs.local['position']=(1.2,2.,-.1)
    if change=='yaw':r.inputs.attitude['yaw']=1.
    if change=='boot':r.inputs.boot_session='other'
    if change=='home':r.inputs.home_down=True
    if change=='selector':r.inputs.selector['instance_changed_count']+=1
    with pytest.raises(PermissionError):t.confirm(text)
    assert r.start_requested_at is None and not r.core.arm_attempted


def test_partial_arm_is_not_seen_as_an_ack_or_retried_and_recovery_stays_pending(tmp_path):
    p,r,io,journal=wired(tmp_path);assert not p.prepare()['failure']
    p.port.fail_arm=True;r.request_start(operator_confirmed=True)
    for _ in range(1000):
        p.turn()
        if io.delivery_uncertain and r.coordinator.fusion.phase=='BLOCKED':break
    arms=[msg for msg in p.port.calls if msg.get_type()=='COMMAND_LONG' and msg.command==400]
    assert len(arms)==1 and io.delivery_uncertain and not r.core.pending
    assert not journal.finalize(r,io) and journal.record['needs_restore']


def test_journal_sync_failure_blocks_before_any_effect(tmp_path,monkeypatch):
    p,r,io,journal=wired(tmp_path)
    def fail():raise OSError('disk full')
    monkeypatch.setattr(journal,'_save',fail)
    p.prepare()
    assert r.failure
    assert not any(msg.get_type()=='COMMAND_LONG' and msg.command==185 for msg in p.port.calls)


def test_current_usb_reappearance_blocks_privileged_writes(tmp_path):
    p,r,io,journal=wired(tmp_path);io.usb_present=lambda:True
    p.prepare()
    assert r.failure and not journal.record['events']


def test_authorization_is_rechecked_after_durability(tmp_path):
    p,r,io,journal=wired(tmp_path);calls=[]
    def grant(*args):
        calls.append(1)
        return len(calls)==1
    io.authorizer=grant;p.prepare()
    assert len(calls)>=2 and r.failure and journal.record['needs_restore']
    assert not any(msg.get_type()=='COMMAND_LONG' and msg.command==185 for msg in p.port.calls)


@pytest.mark.parametrize('field',['x','usec'])
def test_transport_rejects_fabricated_or_replayed_pose_even_with_test_authorizer(tmp_path,field):
    import compact_vision
    p,r,io,journal=wired(tmp_path);p.prepare();before=len(p.port.records)
    stamp,pos,q=r.coordinator.source_pose
    msg,_,_=compact_vision.message(r.encoder,stamp//1000,pos,q,r.coordinator.fusion.alignment)
    setattr(msg,field,getattr(msg,field)+1)
    with pytest.raises(PermissionError,match='exact current'):io.send(msg)
    assert len(p.port.records)==before


def test_pending_flight_journal_blocks_new_shared_preparations(tmp_path):
    run=tmp_path/'run';run.mkdir()
    p,r,io,journal=wired(run)
    p.prepare()
    with pytest.raises(ValueError,match='recovery'):require_no_pending_recovery(tmp_path)


def test_ground_cleanup_is_never_claimed_with_unknown_airborne_state(tmp_path):
    p,r,io,journal=wired(tmp_path);p.prepare();r.request_start(operator_confirmed=True)
    for _ in range(1000):
        p.turn()
        if r.core.phase=='HOVER':break
    assert r.inputs.actuator['armed']
    assert not journal.finalize(r,io)
    with pytest.raises(PermissionError,match='airborne'):io.close()


def test_review_draft_is_unapproved_and_cannot_be_consumed(tmp_path):
    p=prepared();r=p.runtime
    code=tmp_path/'code';code.write_text('fixture')
    evidence=tmp_path/'proof';evidence.write_text('not actual hardware evidence')
    record=draft(boot_session=r.inputs.boot_session,rigid_revision=r.inputs.rigid_revision,
        code_paths=[code],evidence_paths=[evidence])
    assert record['authorized'] is False and not any(record['confirmations'].values())
    path=tmp_path/'review.json';path.write_text(json.dumps(record))
    grant=ReviewedExecutionGrant(path,r,usb_present=lambda:False,check_port_identity=lambda:None)
    with pytest.raises(PermissionError):grant.consume()
    assert not Path(str(path)+'.consumed.json').exists()


@pytest.mark.parametrize('missing',REQUIRED)
def test_each_missing_physical_confirmation_blocks_consumption(tmp_path,missing):
    p=prepared();r=p.runtime
    code=tmp_path/'code';code.write_text('fixture')
    record=draft(boot_session=r.inputs.boot_session,rigid_revision=r.inputs.rigid_revision,
        code_paths=[code],evidence_paths=[code])
    record.update(authorized=True,issued_unix=99.,expires_unix=150.)
    record['confirmations']=dict.fromkeys(REQUIRED,True);record['confirmations'][missing]=False
    path=tmp_path/'review.json';path.write_text(json.dumps(record))
    grant=ReviewedExecutionGrant(path,r,usb_present=lambda:False,check_port_identity=lambda:None,
        wall=lambda:100.,clock=lambda:100.,origin_monotonic=100.-r.now)
    grant._physical_evidence_ok=lambda:True  # Isolate each confirmation predicate using synthetic tests only.
    with pytest.raises(PermissionError):grant.consume()


def approved_test_review(tmp_path):
    # Test-only review of synthetic observations. Files remain in pytest's
    # temporary directory and are NEVER usable as physical-flight evidence.
    p=prepared();r=p.runtime
    code=tmp_path/'fixture_code';code.write_text('fixture')
    record=draft(boot_session=r.inputs.boot_session,rigid_revision=r.inputs.rigid_revision,
        code_paths=[code],evidence_paths=[code])
    record.update(authorized=True,issued_unix=99.,expires_unix=150.)
    record['confirmations']=dict.fromkeys(REQUIRED,True)
    path=tmp_path/'TEST_ONLY_review.json';path.write_text(json.dumps(record))
    clock=NS(t=100.)
    grant=ReviewedExecutionGrant(path,r,usb_present=lambda:False,check_port_identity=lambda:None,
        wall=lambda:100.,clock=lambda:clock.t,origin_monotonic=100.-r.now)
    grant._physical_evidence_ok=lambda:True  # Test-only injection; no real hardware report exists.
    return p,r,grant,clock,code


def test_review_one_use_hashes_and_actual_elapsed_time_are_enforced(tmp_path):
    p,r,grant,clock,code=approved_test_review(tmp_path)
    assert grant.consume() is grant
    with pytest.raises(PermissionError):grant.consume()
    msg=r.encoder.command_long_encode(1,1,185,0,2,0,0,0,0,0,0)
    record=dict(at=r.now)
    assert grant(r,record,msg)
    clock.t+=.1
    assert grant(r,record,msg)  # No separate 50 ms cutoff while state remains current.
    clock.t+=.5
    assert not grant(r,record,msg)  # Actual link state has now expired; never restamp it.
    clock.t=100.
    ordinary=r.encoder.command_long_encode(1,1,400,0,1,0,0,0,0,0,0)
    assert grant(r,record,ordinary)
    code.write_text('changed')
    assert not grant(r,record,ordinary)
    # Existing safety path deliberately avoids a disk scan before STOP/LAND.
    assert grant(r,record,msg)


def test_mutating_cached_review_does_not_bypass_pinned_file_hash(tmp_path):
    p,r,grant,clock,code=approved_test_review(tmp_path);grant.consume()
    grant.record['code_hashes']={}
    msg=r.encoder.command_long_encode(1,1,185,0,2,0,0,0,0,0,0)
    assert not grant(r,dict(at=r.now),msg)


@pytest.mark.parametrize('delay,accepted',[(.051,True),(.17,False)])
def test_hash_duration_is_included_in_actual_send_age(tmp_path,delay,accepted):
    p,r,grant,clock,code=approved_test_review(tmp_path);grant.consume()
    check=grant._hashes_ok
    def slow_hash():
        result=check();clock.t+=delay
        return result
    grant._hashes_ok=slow_hash
    msg=r.encoder.command_long_encode(1,1,400,0,1,0,0,0,0,0,0)
    assert grant(r,dict(at=r.now),msg) is accepted
    if not accepted:
        assert grant.last_rejection['elapsed_since_turn_s']==pytest.approx(delay)
        assert grant.last_rejection['ages_s']['source_age']>.15


def test_review_consumption_has_no_separate_50ms_deadline(tmp_path):
    p,r,grant,clock,code=approved_test_review(tmp_path)
    clock.t+=.075
    assert grant.consume() is grant
    assert grant.start_available()


@pytest.mark.parametrize('command,param',[(185,0),(400,1),(176,1),(21,0),(511,1)])
def test_ground_bootstrap_never_upgrades_itself_to_flight(tmp_path,command,param):
    p=prepared();r=p.runtime;code=tmp_path/'fixture';code.write_text('fixture')
    record=draft_ground(boot_session=r.inputs.boot_session,rigid_revision=r.inputs.rigid_revision,
        code_paths=[code],evidence_paths=[code])
    assert not record['authorized'] and not any(record['confirmations'].values())
    record.update(authorized=True,issued_unix=99.,expires_unix=150.)
    record['confirmations']=dict.fromkeys(GROUND_REQUIRED,True)
    path=tmp_path/'TEST_ONLY_ground.json';path.write_text(json.dumps(record))
    g=ReviewedGroundBootstrapGrant(path,r,usb_present=lambda:False,check_port_identity=lambda:None,
        origin_monotonic=100.-r.now,clock=lambda:100.,wall=lambda:100.)
    g._physical_evidence_ok=lambda:True  # Only synthetic gate predicate test.
    g.consume()
    msg=r.encoder.command_long_encode(1,1,command,0,param,0,0,0,0,0,0)
    assert not g(r,dict(at=r.now),msg)


def test_empty_review_slots_deny_all_effects_and_require_explicit_phase_order(tmp_path):
    p,r,io,journal=wired(tmp_path)
    slots=ExecutionReviewSlots(r,io,origin_monotonic=0.,usb_present=lambda:False,check_port_identity=lambda:None)
    msg=r.encoder.command_long_encode(1,1,400,0,1,0,0,0,0,0,0)
    assert not slots(r,dict(at=0),msg)
    with pytest.raises(PermissionError):slots.install_execution(tmp_path/'missing')


def test_software_report_cannot_substitute_for_actual_battery_preflight(tmp_path):
    p,r,grant,clock,code=approved_test_review(tmp_path)
    del grant._physical_evidence_ok  # Exercise the actual production evidence parser.
    assert not grant._physical_evidence_ok()
    with pytest.raises(PermissionError):grant.consume()


def test_privileged_session_does_not_close_or_report_done_on_land_timeout(tmp_path):
    p,r,io,journal=wired(tmp_path,scenario='landing_timeout');p.prepare();r.request_start(operator_confirmed=True)
    for _ in range(5000):
        p.turn()
        if r.core.phase=='LAND_TIMEOUT':break
    controller=FlightRunController(r,io)
    status=controller.status()
    assert status['intervention_required'] and not status['done'] and not io.closed


def test_guarded_factory_opens_nothing_and_installs_empty_review_slots(tmp_path):
    port=FakePort()
    controller=build_guarded_session(revision='TEST_ONLY_revision',origin=(0,0,0),
        orientation=(1,0,0,0),port=port,poll_gamepad=lambda:{},poll_pose=lambda:None,
        now_ros_ns=lambda:0,check_port_identity=lambda:None,usb_present=lambda:False,
        origin_monotonic=10.,recovery_path=tmp_path/'run'/'flight_recovery.json')
    assert not port.calls and not controller.done
    assert controller.io.authorizer is controller.review_slots
    assert controller.io.journal.record['events']==[]
    r=controller.runtime
    msg=r.encoder.command_long_encode(1,1,400,0,1,0,0,0,0,0,0)
    assert not controller.io.authorizer(r,dict(at=0),msg)


def test_guarded_factory_refuses_unresolved_previous_journal(tmp_path):
    prior=tmp_path/'previous';prior.mkdir()
    (prior/'flight_recovery.json').write_text(json.dumps(dict(needs_restore=True)))
    with pytest.raises(ValueError,match='recovery'):
        build_guarded_session(revision='TEST_ONLY_revision',origin=(0,0,0),
            orientation=(1,0,0,0),port=FakePort(),poll_gamepad=lambda:{},poll_pose=lambda:None,
            now_ros_ns=lambda:0,check_port_identity=lambda:None,usb_present=lambda:False,
            origin_monotonic=10.,recovery_path=tmp_path/'new'/'flight_recovery.json')
    assert not (tmp_path/'new'/'flight_recovery.json').exists()
