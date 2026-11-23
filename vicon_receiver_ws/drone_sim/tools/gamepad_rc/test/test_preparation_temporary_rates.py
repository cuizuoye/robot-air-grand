"""Synthetic single-owner rates/fusion lifecycle; never opens hardware."""
from copy import deepcopy
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mission_preparation_runtime import PreparationRuntime
from preparation_authorization import RateRecoveryJournal, require_no_pending_recovery
from test_mission_preparation_runtime import Peer
from test_mission_preparation_io import attach
from trs_message_interval_trial import CANDIDATES
from test_preparation_authorization import files,approved
import preparation_authorization as authorization


class RatePeer(Peer):
    def __init__(self,persist=None,*,ground=False):
        super().__init__(fusion=True)
        self.saved=[]
        def save(record):
            self.saved.append(deepcopy(record))
            if persist:persist(record)
        self.runtime=PreparationRuntime('offline-revision',body_origin_in_rigid=[0,0,0],
            body_orientation_in_rigid=[0,0,0,1],enable_offline_fusion=True,
            enable_temporary_rates=True,rates_persist=save,ground_fusion_only=ground)
        self.values={32:1_000_000,230:2_000_000,245:1_000_000}
        self.original=dict(self.values)

    def inspect_output(self):
        before=len(self.calls);super().inspect_output()
        for _,msg in self.calls[before:]:
            if msg.get_type()!='COMMAND_LONG' or msg.command not in (510,511):continue
            key=int(msg.param1)
            if msg.command==510:
                response=self.encoder.message_interval_encode(interval_us=self.values[key],message_id=key)
            else:
                assert self.saved and key in self.saved[-1]['possibly_changed']
                self.values[key]=int(msg.param2)
                response=self.encoder.command_ack_encode(511,0,target_system=250,target_component=220)
            self.pending.append(self.pack(response))

    def turn(self,**kwargs):
        msg=self.encoder.estimator_status_encode(int((self.now+10)*1e6),0,*([0]*8))
        extra=kwargs.pop('extra_rx',b'')
        return super().turn(extra_rx=self.pack(msg)+extra,**kwargs)

    def rates_ready(self):
        self.home_check()
        for _ in range(250):
            self.turn()
            if self.runtime.rates.summary()['measured_profile_verified']:return
        pytest.fail(self.runtime.status())


def test_preparation_cannot_start_until_actual_profile_arrivals_verified():
    p=RatePeer();p.home_check()
    with pytest.raises(PermissionError):p.runtime.request_prepare(operator_confirmed=True)
    assert not p.runtime.prepare_requested
    p.rates_ready();p.runtime.request_prepare(operator_confirmed=True)
    for _ in range(300):
        state=p.turn()
        if state['failure'] or p.runtime.coordinator.fusion.phase=='HANDED_OFF':break
    assert state['failure'] is None,state
    assert p.runtime.coordinator.fusion.phase=='HANDED_OFF'
    p.runtime.request_close()
    for _ in range(200):
        state=p.turn()
        if state['closed']:break
    assert state['closed'] and p.values==p.original
    assert p.runtime.rates.summary()['restoration_verified']
    assert not any(msg.get_type()=='COMMAND_LONG' and msg.command in (176,400,21,22,246)
        for _,msg in p.calls)


def test_cancel_before_fusion_restores_rates_and_blocks_next_run_until_restored(tmp_path):
    run=tmp_path/'run';run.mkdir();journal=RateRecoveryJournal(run/'rates_recovery.json')
    p=RatePeer(journal);p.rates_ready()
    with pytest.raises(ValueError,match='needs recovery'):require_no_pending_recovery(tmp_path)
    p.runtime.request_close()
    for _ in range(150):
        p.turn()
        if p.runtime.closed:break
    assert p.runtime.closed and p.values==p.original
    assert journal.finalize(p.runtime,type('IO',(),{'failure':None})())
    require_no_pending_recovery(tmp_path)
    assert not json.loads(journal.path.read_text())['needs_restore']


def test_rate_commands_need_separate_grant_and_durable_journal():
    p=RatePeer();p.home_check()
    # Construct the next exact pending SET, without simulating its response.
    r=p.runtime;rates=r.rates
    rates.pending=rates.outgoing=None;rates.phase='SET_NEXT';rates.changed=[]
    rates.original=dict(p.original)
    msg=rates.action(p.now,fresh=True,locked=True)
    r._capture(msg);record=r.frames[-1]
    io,port,_,_=attach(p)
    with pytest.raises(PermissionError,match='authorized'):io._dispatch(record)
    io.authorize=lambda *args:True
    with pytest.raises(PermissionError,match='journal'):io._dispatch(record)
    assert not port.writes


def test_rate_restore_uncertainty_remains_a_recovery_block(tmp_path):
    run=tmp_path/'run';run.mkdir();journal=RateRecoveryJournal(run/'rates_recovery.json')
    p=RatePeer(journal);p.rates_ready()
    assert not journal.finalize(p.runtime,type('IO',(),{'failure':'uncertain write'})())
    with pytest.raises(ValueError,match='needs recovery'):require_no_pending_recovery(tmp_path)


def test_first_failure_records_cause_ages_before_later_pose_staleness():
    p=RatePeer();p.rates_ready();r=p.runtime
    r._fail('synthetic_first_failure')
    context=deepcopy(r.failure_context)
    r._fail('later_fault')
    assert r.failure=='synthetic_first_failure' and r.failure_context==context
    assert context['source_age']<=.15 and context['actuator_age']<=.5
    assert 'landed_message_age' in context


def test_long_human_wait_does_not_burn_diagnostic_budget_or_request_parameters():
    p=RatePeer()
    for _ in range(100):p.turn()
    assert not p.runtime.failure and not p.runtime.coordinator.audit.reads
    assert not p.runtime.sent_counts.get('SERIAL_CONTROL')
    assert not p.runtime.sent_counts.get('PARAM_REQUEST_READ')
    assert not p.runtime.sent_counts.get('VISION_POSITION_ESTIMATE')
    assert not p.runtime.rates.changed
    p.runtime.request_close()
    assert p.runtime.closed and p.runtime.rates.phase=='CLOSED'


def test_prepare_wait_clock_is_separate_from_active_check_deadline():
    from prepare_drone import HostSession
    p=RatePeer()
    class IdleIO:
        failure=None
        def step(self,now,clock):
            p.runtime.turn(now,now_ros_ns=1_000_000_000)
        def status(self):return {}
    session=HostSession(p.runtime,IdleIO(),prepare=True,seconds=120)
    session.turn(121,0)
    assert not session.done and session.close_deadline is None
    p.runtime.home_check_requested=True
    session.turn(122,0)
    assert session.active_started_at==122 and session.close_deadline is None
    session.turn(243,0)
    assert session.reason=='bounded_session_end'


def test_read_interval_remains_read_only_when_actor_ages_but_rate_set_is_denied(files):
    p=RatePeer();p.rates_ready();r=p.runtime;rates=r.rates
    token=approved(files)
    grant=authorization.consume(token,r,files.run,files.heading,files.board,files.proof,
        files.backup,clock=lambda:p.now)
    rates.pending=rates.outgoing=None;rates.phase='WAIT_APPLY';rates.current=32;rates.wait_until=p.now
    r.inputs.actuator['timestamp_us']-=700_000
    msg=rates.action(p.now,fresh=True,locked=False)
    assert msg.command==510
    r._capture(msg);frame=r.frames[-1]
    assert grant(r,frame,msg)  # GET never writes state; no actor-age privilege needed.
    io,port,_,_=attach(p,authorize=grant)
    io._dispatch(frame)
    assert port.writes
    rates.pending=rates.outgoing=None;rates.phase='SET_NEXT';rates.changed=[]
    assert rates.action(p.now,fresh=True,locked=False) is None


def test_rate_writes_are_not_captured_at_the_half_second_freshness_edge():
    p=RatePeer();p.rates_ready();r=p.runtime;rates=r.rates
    rates.pending=rates.outgoing=None;rates.phase='SET_NEXT';rates.changed=[]
    msg=rates.action(p.now,fresh=True,locked=True)
    r.inputs.actuator['timestamp_us']=r.inputs._boot_us(p.now)-450_000
    with pytest.raises(PermissionError):r._capture(msg)


def test_restore_tool_can_only_return_logged_originals_not_candidate_or_control():
    from restore_preparation_rates import allowed
    from trs_message_interval_trial import interval_message
    from pymavlink.dialects.v20 import common as m
    e=m.MAVLink(None,srcSystem=250,srcComponent=221)
    kwargs=dict(safe=True,locked=True,original={32:1_000_000,230:2_000_000,245:1_000_000},changed=[32])
    assert allowed(interval_message(e,32,1_000_000),**kwargs)
    assert not allowed(interval_message(e,32,100_000),**kwargs)
    assert not allowed(interval_message(e,245,1_000_000),**kwargs)
    assert not allowed(interval_message(e,32,1_000_000),**{**kwargs,'locked':False})
    for command in (176,185,400,21,22,246):
        assert not allowed(e.command_long_encode(1,1,command,0,1,0,0,0,0,0,0),**kwargs)


def test_failed_restore_exits_without_claiming_closed_or_spamming_ready():
    from prepare_drone import HostSession
    from test_preparation_host import DrivenIO
    p=RatePeer();p.rates_ready();r=p.runtime
    r.request_close();r.rates.phase='RESTORE_FAILED'
    session=HostSession(r,DrivenIO(p),prepare=True);session.close_deadline=p.now+35
    state=session.turn(round(p.now+.032,9),0)
    assert state['done'] and not r.closed
    assert r.rates.phase=='RESTORE_FAILED'
