"""Offline late-ground confirmation and timeout takeover; reuse existing peers."""
import socket
import pytest
from test_flight_mission_core import airborne, tick, observation, selector, ack
from test_native_rc_mission_delta import prepared, airborne as native_airborne


@pytest.fixture(autouse=True)
def no_network(monkeypatch):
    monkeypatch.setattr(socket,'socket',lambda *a,**k:pytest.fail('Offline only'))


def timed_out():
    core,sink=airborne()
    tick(core,4.5,armed=True,landed=False,gamepad_available=False)
    core.tick(observation(40.,armed=True,landed=False),selector(),boot_ms=40000)
    assert core.phase=='LAND_TIMEOUT'
    return core,sink


def test_late_ground_closes_failed_task_without_control_or_restart():
    core,sink=timed_out();before=len(sink.messages);fault=core.fault
    core.tick(observation(40.1,armed=False,landed=True),selector(),boot_ms=40100)
    status=core.status()
    assert core.phase=='COMPLETE' and core.fault==fault
    assert not status['mission_sequence_passed_in_offline_sink']
    assert status['landing_confirmation']==dict(attempts=1,ack_accepted=False,
        waiting_for=None,timed_out_at=40.,ground_confirmed_at=40.1)
    assert len(sink.messages)==before
    core.tick(observation(41.,armed=False,landed=True),selector(),boot_ms=41000)
    assert len(sink.messages)==before


@pytest.mark.parametrize('change',[
    dict(armed=True),dict(landed=False),dict(status_age_s=.501),
    dict(boot_session='another_boot'),dict(uid2='other_aircraft'),
    dict(armed=0),dict(landed=1)])
def test_timeout_requires_actual_fresh_same_boot_ground(change):
    core,sink=timed_out();before=len(sink.messages)
    state=dict(armed=False,landed=True);state.update(change)
    core.tick(observation(40.1,**state),selector(),boot_ms=40100)
    assert core.phase=='LAND_TIMEOUT' and len(sink.messages)==before
    assert core.status()['landing_confirmation']['waiting_for']=='fresh_landed_and_disarmed'


def test_land_ack_alone_does_not_confirm_ground_or_remove_timeout():
    core,sink=airborne();tick(core,4.5,armed=True,landed=False,gamepad_available=False)
    assert ack(core,21,4.6)
    assert core.status()['landing_confirmation']['ack_accepted']
    core.tick(observation(40.,armed=True,landed=False),selector(),boot_ms=40000)
    assert core.phase=='LAND_TIMEOUT' and core.ground_confirmed_at is None


def test_timeout_still_observes_manual_takeover_without_commands():
    core,sink=timed_out();before=len(sink.messages)
    core.tick(observation(40.1,armed=True,landed=False,manual_control_active=True,offboard=False),
              selector(),boot_ms=40100)
    assert core.phase=='PILOT_CONTROL' and core.pilot_control
    assert len(sink.messages)==before and core.landing_timed_out_at==40.


@pytest.mark.parametrize('cancel',[False,True])
def test_pocket_late_landing_finishes_existing_journal_and_keeps_failure(tmp_path,cancel):
    p=prepared(tmp_path);native_airborne(p)
    r=p.runtime
    p.scenario='landing_timeout'
    if cancel: p.controller.cancel()
    for _ in range(5000):
        p.turn()
        if r.core.phase=='LAND_TIMEOUT':break
    assert r.core.phase=='LAND_TIMEOUT',r.status()
    status=p.controller.status()
    assert status['intervention_required'] and not status['done']
    fault=status['failure'];before=len(p.calls)
    # Only the simulator now allows actual landing/disarm observations.
    p.scenario='nominal'
    for _ in range(1000):
        p.turn()
        if p.controller.done:break
    assert p.controller.done and p.controller.journal_finalized,p.controller.status()
    assert p.controller.status()['failure']==fault
    assert not r.core.status()['mission_sequence_passed_in_offline_sink']
    assert not any(msg.get_type() in ('SET_POSITION_TARGET_LOCAL_NED','RC_CHANNELS_OVERRIDE')
        or msg.get_type()=='COMMAND_LONG' and msg.command in (21,176,185,400)
        for _,msg in p.calls[before:])


def test_console_reports_land_ack_change_without_phase_change():
    import json
    from test_vicon_console import Controller
    from vicon_fixed_point_entry import serve_console
    c=Controller();original=c.status
    def status():
        row=original()
        row['landing_confirmation']=dict(ack_accepted=c.steps>=2,
            waiting_for='fresh_landed_and_disarmed',timed_out_at=None)
        return row
    c.status=status;out=[]
    lines=iter([None,None,''])
    serve_console(c,lambda:next(lines,None),emit=out.append,clock=lambda:0,idle=lambda:None)
    rows=[json.loads(line) for line in out if line.startswith('{')]
    assert [r['landing_confirmation']['ack_accepted'] for r in rows[:2]]==[False,True]
