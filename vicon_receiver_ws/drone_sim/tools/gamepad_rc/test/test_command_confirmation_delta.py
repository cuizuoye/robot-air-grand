"""Late ACK versus actual state: injected peers only, no physical I/O."""
import pytest
from pymavlink.dialects.v20 import common as m
from test_gamepad_takeover import peer, fc_ack, fc_mode
from test_flight_mission_core import offboard_wait, ack, tick
from test_continuous_mission_runtime import prepared


def test_late_mode_ack_allows_next_heartbeat_without_resending(monkeypatch):
    link,clock,sent=peer(monkeypatch)
    link.request_mode(2);start=clock[0]
    clock[0]=start+2.77;link.observe_mode(fc_ack())
    clock[0]=start+3.2;link.observe_mode(fc_mode(2))
    assert link.mode_request['state']=='CONFIRMED' and len(sent)==1


@pytest.mark.parametrize('accepted',[False,True])
def test_mode_progress_and_duplicate_ack_do_not_extend_deadline(monkeypatch,accepted):
    link,clock,sent=peer(monkeypatch)
    link.request_mode(2);start=clock[0]
    clock[0]=start+2.77;link.observe_mode(fc_ack(result=0 if accepted else 5))
    clock[0]=start+2.99;link.observe_mode(fc_ack(result=0 if accepted else 5))
    clock[0]=start+(3.78 if accepted else 3.01);link.check_mode_timeout()
    assert link.mode_request['state']=='TIMEOUT'
    assert link.mode_request['timeout_reason']==('state_confirmation_timeout' if accepted else 'ack_timeout')
    link.observe_mode(fc_mode(2))
    assert link.mode_request['state']=='TIMEOUT' and len(sent)==1


def waiting(command):
    core,sink=offboard_wait()
    if command==400:
        ack(core,176,4.3);tick(core,4.3,offboard=True)
    return core,sink


def advance(core,to,*,offboard=False,armed=False):
    while core.last_t<to:
        tick(core,min(to,core.last_t+.05),offboard=offboard,armed=armed)
        if core.phase in ('LAND','BLOCKED','COMPLETE'):break


@pytest.mark.parametrize('command',[176,400])
def test_core_late_acceptance_and_actual_state_are_separate(command):
    core,sink=waiting(command);start=core.pending['sent_at']
    advance(core,start+2.7,offboard=command==400)
    assert ack(core,command,start+2.77)
    advance(core,start+3.15,offboard=command==400)
    assert core.status()['command_confirmation']['waiting_for']=='state'
    tick(core,start+3.2,offboard=True,armed=command==400)
    assert core.phase==('WAIT_ARM' if command==176 else 'ASCEND')
    assert sum(x.get_type()=='COMMAND_LONG' and x.command==command for x in sink.messages)==1


@pytest.mark.parametrize('command',[176,400])
def test_core_duplicate_ack_cannot_keep_missing_state_alive(command):
    core,sink=waiting(command);start=core.pending['sent_at']
    advance(core,start+2.7,offboard=command==400)
    assert ack(core,command,start+2.77)
    advance(core,start+2.95,offboard=command==400)
    assert ack(core,command,start+2.99)
    advance(core,start+3.8,offboard=command==400)
    assert core.fault=='command_state_confirmation_timeout'
    assert core.phase==('BLOCKED' if command==176 else 'COMPLETE')  # Peer is still landed/disarmed.
    assert sum(x.get_type()=='COMMAND_LONG' and x.command==command for x in sink.messages)==1


def test_core_progress_is_not_rejection_or_acceptance_and_cannot_extend():
    core,sink=waiting(176);start=core.pending['sent_at']
    advance(core,start+2.7)
    assert ack(core,176,start+2.77,m.MAV_RESULT_IN_PROGRESS)
    assert not core.pending['accepted'] and not core.pending.get('rejected')
    advance(core,start+3.05)
    assert core.fault=='command_ack_timeout' and not core.arm_attempted


@pytest.mark.parametrize('state_arrives',[True,False])
def test_runtime_enable_late_ack_needs_new_unlocked_state(state_arrives):
    p=prepared();r=p.runtime
    r.request_start(operator_confirmed=True);start=r.start_requested_at
    p.inspect_output();p.locked=True
    # Withhold only the synthetic enable ACK; all fresh telemetry continues.
    p.pending.clear()
    while p.now<start+2.76:p.turn()
    p.turn(extra_rx=p.pack(p.encoder.command_ack_encode(185,0,target_system=250,target_component=220)))
    assert r.enable_ack_at is not None
    while p.now<start+3.1:p.turn()
    assert not r.failure and r.core.phase=='WAIT_EXPLICIT_START'
    if state_arrives:
        p.locked=False
        while p.now<start+3.6 and r.core.phase=='WAIT_EXPLICIT_START':p.turn()
        assert not r.failure and r.core.phase=='WARMUP'
    else:
        while p.now<start+3.85 and not r.failure:p.turn()
        assert r.failure=='output_enable_state_confirmation_timeout'
    assert sum(msg.get_type()=='COMMAND_LONG' and msg.command==185 and msg.param1==0
               for _,msg in p.calls)==1


def test_late_first_ack_cannot_create_new_grace():
    core,sink=waiting(176);start=core.pending['sent_at']
    advance(core,start+2.95)
    assert not ack(core,176,start+3.01)
    tick(core,start+3.02,offboard=True)
    assert core.fault=='command_ack_timeout' and not core.arm_attempted


def test_home_during_grace_cancels_pending_command():
    core,sink=waiting(176);start=core.pending['sent_at']
    advance(core,start+2.7);assert ack(core,176,start+2.77)
    advance(core,start+3.1)
    tick(core,start+3.15,home_pressed=True)
    assert core.stop_latched and core.pending is None and not core.arm_attempted


def test_console_reports_ack_to_state_transition_without_phase_change():
    import json
    from test_vicon_console import Controller
    from vicon_fixed_point_entry import serve_console
    c=Controller();original=c.status
    def status():
        row=original()
        row['command_confirmation']={'command':176,'waiting_for':'ack' if c.steps<2 else 'state'}
        row['failure']=None
        return row
    c.status=status;out=[]
    lines=iter([None,None,''])
    serve_console(c,lambda:next(lines,None),emit=out.append,clock=lambda:0,idle=lambda:None)
    rows=[json.loads(line) for line in out if line.startswith('{')]
    assert [r['command_confirmation']['waiting_for'] for r in rows[:2]]==['ack','state']
    assert rows[0]['phase']==rows[1]['phase']
