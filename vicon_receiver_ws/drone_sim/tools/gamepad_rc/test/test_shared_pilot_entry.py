"""New one-writer pilot wiring only; no old test suites or native opens."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace as NS
import json
import sys

import pytest
from pymavlink.dialects.v20 import common as m
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from test_vicon_fixed_entry import wired
from gamepad_rc import CODE_OF,KEY_NAME,Mapper
from px4_modes import custom_mode_from_param
from preparation_host_bindings import LinuxGamepad
from gamepad_estop_monitor import EVENT,HOME

CALIB=json.loads((Path(__file__).resolve().parents[1]/'gamepad_calib.json').read_text())


def pilot_peer(tmp_path,authorized=True):
    p,c=wired(tmp_path,authorized=authorized)
    r=c.runtime;r.enable_pilot(CALIB)
    p.pilot_state=dict(axes={code:CALIB['axes'][name]['center'] for name,code in CODE_OF.items()},buttons=(),edges=())
    p.io.poll_gamepad=lambda:{**{k:p.input_args[k] for k in
        ('pad_generation','pad_present','pad_event','home_events')},'pilot_state':deepcopy(p.pilot_state)}
    return p,c


def receipt(p,wrong=False):
    values=list(p.runtime.pilot_channels[:4]) if p.runtime.pilot_channels else [1500]*4
    if wrong:values[0]+=123
    return p.pack(p.encoder.rc_channels_encode(int((p.now+p.dt+p.boot_offset)*1000),
        4,*values,*([65535]*14),100))


def prepared(tmp_path):
    p,c=pilot_peer(tmp_path)
    result=p.prepare();assert not result['failure'],result
    p.turn(extra_rx=receipt(p))
    return p,c


def airborne(tmp_path):
    p,c=prepared(tmp_path)
    c.start.confirm(c.start.create()['confirmation_text'])
    for _ in range(500):
        p.turn()
        if c.runtime.core.phase=='ASCEND':break
    assert c.runtime.core.phase=='ASCEND',c.status()
    return p,c


def press(p,key):
    p.pilot_state['edges']=(key,)  # Complete press/release in one poll.
    p.turn()
    p.pilot_state['edges']=()


def mode_frames(p,start):
    return [msg for msg in p.port.calls[start:] if msg.get_type()=='COMMAND_LONG' and msg.command==176]


def test_neutral_rc_shares_writer_ignores_aux_and_needs_real_receipt(tmp_path):
    p,c=pilot_peer(tmp_path);assert not p.prepare()['failure']
    r=c.runtime
    assert r.writer is r.coordinator.writer is r.core.sink is r.coordinator.fusion.sink is p.io.writer
    assert r.pilot_link.mav is None  # No standalone Link/KeepAlive/reader opened.
    rc=[msg for msg in p.port.calls if msg.get_type()=='RC_CHANNELS_OVERRIDE']
    assert rc and all([getattr(msg,'chan%d_raw'%i) for i in range(1,9)]==[1500]*4+[65535]*4 for msg in rc)
    with pytest.raises(PermissionError,match='RC_CHANNELS'):c.start.create()
    p.turn(extra_rx=receipt(p,wrong=True))
    assert not r.pilot_rc_received()
    p.turn(extra_rx=receipt(p))
    assert r.pilot_rc_received() and c.start.create()['height_m']==.3
    assert sum(e['kind']=='RC_CHANNELS_OVERRIDE' for e in c.io.journal.record['events'])==1
    assert not any(msg.get_type()=='PARAM_SET' for msg in p.port.calls)


@pytest.mark.parametrize('button,parameter',[('L1',1),('A',2),('B',11)])
def test_buttons_yield_before_command_and_share_ack_reader(tmp_path,button,parameter):
    p,c=airborne(tmp_path);r=c.runtime;start=len(p.port.calls)
    seen=[]
    p.port.guard=lambda msg:seen.append(r.core.phase) if msg.get_type()=='COMMAND_LONG' and msg.command==176 else None
    press(p,button)
    requests=mode_frames(p,start)
    assert len(requests)==1 and seen==['PILOT_CONTROL']
    assert r.pilot_link.mode_request['parameter']==parameter
    ack=p.pack(p.encoder.command_ack_encode(176,0,target_system=250,target_component=220))
    hb=p.pack(p.encoder.heartbeat_encode(2,12,128,custom_mode_from_param(parameter),0))
    p.turn(extra_rx=ack+hb)
    assert r.pilot_link.mode_request['state']=='CONFIRMED'
    for _ in range(15):p.turn()
    assert r.core.phase=='PILOT_CONTROL'
    assert not any(msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED' for msg in p.port.calls[start:])
    assert any(msg.get_type()=='VISION_POSITION_ESTIMATE' for msg in p.port.calls[start:])
    assert not any(msg.get_type()=='COMMAND_LONG' and msg.command in (400,21) for msg in p.port.calls[start:])


def test_stick_motion_yields_before_rc_and_does_not_repeatedly_change_mode(tmp_path):
    p,c=airborne(tmp_path);r=c.runtime;start=len(p.port.calls)
    p.pilot_state['axes'][CODE_OF['ABS_RX']]=CALIB['axes']['ABS_RX']['max']
    phases=[]
    p.port.guard=lambda msg:phases.append(r.core.phase) if msg.get_type()=='RC_CHANNELS_OVERRIDE' else None
    p.turn()
    assert r.core.phase=='PILOT_CONTROL' and r.pilot_link.mode_request['parameter']==2
    for _ in range(20):p.turn()
    assert phases and set(phases)=={'PILOT_CONTROL'} and len(mode_frames(p,start))==1
    rc=[msg for msg in p.port.calls[start:] if msg.get_type()=='RC_CHANNELS_OVERRIDE']
    assert rc and rc[-1].chan1_raw==2000 and rc[-1].chan5_raw==rc[-1].chan6_raw==65535
    assert not any(msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED' for msg in p.port.calls[start:])


def test_native_event_snapshot_preserves_fast_buttons_and_home_without_new_input_owner(monkeypatch):
    import preparation_host_bindings as b
    pad=object.__new__(LinuxGamepad)
    pad.controls=True;pad.axes={code:0 for code in CODE_OF.values()};pad.buttons=set()
    pad.fd=-123;pad.path='SYNTHETIC';pad.generation=('synthetic',);pad.pending=b'';pad.initial=None
    chunks=[b''.join(EVENT.pack(0,0,*e) for e in [(3,0,123),(1,0x136,1),(1,0x136,0),(1,HOME,1),(1,HOME,0)])]
    monkeypatch.setattr(b,'require_gamepad_identity',lambda *a:None)
    monkeypatch.setattr(b.os,'read',lambda *a:chunks.pop())
    out=pad.poll()
    assert out['home_events']==(True,False)
    assert out['pilot_state']['axes'][0]==123
    assert out['pilot_state']['edges']==('L1','HOME') and not out['pilot_state']['buttons']


def test_home_burst_wins_over_sticks_and_buttons_and_release_never_resumes(tmp_path):
    p,c=airborne(tmp_path);start=len(p.port.calls)
    p.pilot_state['edges']=('L1','A','B')
    p.pilot_state['axes'][0]=CALIB['axes']['ABS_X']['max']
    p.turn(home=(True,False));p.pilot_state['edges']=()
    for _ in range(15):p.turn()
    sent=p.port.calls[start:]
    assert c.runtime.core.stop_latched
    assert any(x.get_type()=='COMMAND_LONG' and x.command==185 and x.param1==2 for x in sent)
    assert not any(x.get_type() in ('RC_CHANNELS_OVERRIDE','SET_POSITION_TARGET_LOCAL_NED') for x in sent)
    assert not mode_frames(p,start)


@pytest.mark.parametrize('failure',['rejected','timeout'])
def test_unconfirmed_takeover_never_retries_or_restores_auto(tmp_path,failure):
    p,c=airborne(tmp_path);press(p,'L1');start=len(p.port.calls)
    link=c.runtime.pilot_link
    if failure=='rejected':
        ack=p.pack(p.encoder.command_ack_encode(176,2,target_system=250,target_component=220))
        p.turn(extra_rx=ack)
        assert link.mode_request['state']=='REJECTED'
    else:
        for _ in range(310):p.turn()
        assert link.mode_request['state']=='TIMEOUT'
    ack=p.pack(p.encoder.command_ack_encode(176,0,target_system=250,target_component=220))
    hb=p.pack(p.encoder.heartbeat_encode(2,12,128,custom_mode_from_param(1),0))
    p.turn(extra_rx=ack+hb)
    assert link.mode_request['state']==('REJECTED' if failure=='rejected' else 'TIMEOUT')
    assert c.runtime.core.pilot_control and not mode_frames(p,start)
    assert c.status()['intervention_required']
    assert not any(msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED' for msg in p.port.calls[start:])


def test_missing_pilot_snapshot_stops_rc_not_cached_replay(tmp_path):
    p,c=airborne(tmp_path);press(p,'L1');start=len(p.port.calls)
    p.io.poll_gamepad=lambda:{k:p.input_args[k] for k in ('pad_generation','pad_present','pad_event','home_events')}
    p.turn()
    assert c.runtime.failure
    assert not any(m.get_type()=='RC_CHANNELS_OVERRIDE' for m in p.port.calls[start:])
    assert c.runtime.core.pilot_control


def test_short_mode_write_latches_and_stops_rc_retry(tmp_path):
    p,c=airborne(tmp_path);original=p.port.write;start=len(p.port.calls)
    def short(wire):
        n=original(wire);msg=p.port.calls[-1]
        return n-1 if msg.get_type()=='COMMAND_LONG' and msg.command==176 else n
    p.port.write=short
    press(p,'A')
    for _ in range(15):p.turn()
    assert c.io.delivery_uncertain and c.runtime.core.pilot_control
    assert len(mode_frames(p,start))==1
    assert not any(x.get_type()=='RC_CHANNELS_OVERRIDE' for x in p.port.calls[start:])
    assert any(x.get_type()=='VISION_POSITION_ESTIMATE' for x in p.port.calls[start:]), (c.status(), c.runtime.trace)


def test_no_review_does_not_gain_rc_or_mode_permission(tmp_path):
    p,c=pilot_peer(tmp_path,authorized=False);p.prepare()
    assert c.runtime.failure
    assert not any(x.get_type()=='RC_CHANNELS_OVERRIDE' for x in p.port.calls)
    assert not mode_frames(p,0)


def reviewed_fake_grant(cls,r):
    """Exercise changed per-frame predicates only, never create a grant file."""
    g=object.__new__(cls);g.runtime=r;g.consumed=True;g.fault=None
    g.origin=0;g.clock=lambda:r.now;g.wall=lambda:100;g.usb_present=lambda:False
    g.start_window=(0.,200.);g.last_rejection=None
    g.check_port_identity=lambda:None;g._hashes_ok=lambda:True;g._profile_ok=lambda:True
    g.record=dict(boot_session=r.inputs.boot_session,rigid_revision=r.inputs.rigid_revision,
                  issued_unix=99,expires_unix=200)
    g.original_record=json.dumps(g.record,sort_keys=True,allow_nan=False)
    return g


def test_manual_recovery_does_not_depend_on_vicon_freshness(tmp_path):
    from flight_execution_review import ReviewedExecutionGrant
    p,c=airborne(tmp_path);r=c.runtime
    for _ in range(20):p.turn(with_pose=False)
    assert r.failure and r.context().source_age>.15
    c.io.authorizer=reviewed_fake_grant(ReviewedExecutionGrant,r)
    start=len(p.port.calls)
    p.pilot_state['edges']=('L1',)
    p.turn(with_pose=False);p.pilot_state['edges']=()
    assert len(mode_frames(p,start))==1 and r.core.pilot_control
    for _ in range(10):p.turn(with_pose=False)
    assert any(msg.get_type()=='RC_CHANNELS_OVERRIDE' for msg in p.port.calls[start:])


def test_ground_review_only_accepts_centered_rc_and_direct_start_needs_receipt(tmp_path):
    from flight_execution_review import ReviewedGroundBootstrapGrant
    p,c=pilot_peer(tmp_path);assert not p.prepare()['failure'];r=c.runtime
    with pytest.raises(PermissionError,match='RC_CHANNELS'):r.request_start(operator_confirmed=True)
    grant=reviewed_fake_grant(ReviewedGroundBootstrapGrant,r)
    def allowed():
        msg=r.encoder.rc_channels_override_encode(1,1,*r.pilot_channels[:4],*[65535]*4)
        r._pilot_candidate=msg
        try:return grant(r,{'at':r.now},msg)
        finally:r._pilot_candidate=None
    assert allowed()
    r.pilot_channels=(2000,*r.pilot_channels[1:]);r.pilot_centered=False;r.core.pilot_control=True
    assert not allowed()


def test_native_factory_selects_full_pilot_binding_without_second_link(tmp_path,monkeypatch):
    import vicon_fixed_point_entry as e
    import continuous_device_host as host
    import preparation_host_bindings as b
    import mavlink_readonly_probe as probe
    from fixed_profile_fixtures import approved_profile
    path=tmp_path/'synthetic.json';path.write_text(json.dumps(approved_profile()))
    flags=[];calls=[]
    class Resource:
        def __init__(self,*a,**kw):flags.append(kw);self.port=object()
        def close(self):pass
        def check(self):pass
        def poll(self):pass
        def now_ns(self):return 0
    monkeypatch.setattr(host,'SerialOwner',Resource)
    monkeypatch.setattr(host,'configuration',lambda:dict(revision='SYNTHETIC',origin=[0,0,0],orientation=[0,0,0,1]))
    monkeypatch.setattr(b,'LinuxGamepad',Resource);monkeypatch.setattr(b,'RosLatestPose',Resource)
    monkeypatch.setattr(probe,'FC_USB',NS(exists=lambda:False))
    monkeypatch.setattr(e,'build_guarded_session',lambda **kw:(calls.append(kw)
        or NS(start=NS(),require_native_start=lambda:None)))
    c=e.open_session(profile_path=path,recovery_path=tmp_path/'run'/'flight_recovery.json',
        trs_port='SYNTHETIC',open_devices=True)
    assert flags==[{}, {'controls':True}, {}]
    assert len(calls)==1 and calls[0]['gamepad_calibration']==CALIB
    c.resources.close()


def test_input_failure_during_auto_keeps_land_and_localization_service(tmp_path):
    p,c=airborne(tmp_path)
    for _ in range(30):p.turn()
    start=len(p.port.calls)
    original=p.io.poll_gamepad
    p.io.poll_gamepad=lambda:{**original(),'pilot_state':None}
    for _ in range(15):p.turn()
    messages=p.port.calls[start:]
    assert c.runtime.pilot_fault and c.runtime.core.phase=='LAND'
    assert any(x.get_type()=='COMMAND_LONG' and x.command==21 for x in messages)
    assert any(x.get_type()=='VISION_POSITION_ESTIMATE' for x in messages)
    assert not any(x.get_type()=='RC_CHANNELS_OVERRIDE' for x in messages)


def test_observed_manual_landing_reuses_ground_close_without_auto_resume(tmp_path):
    p,c=airborne(tmp_path)
    for _ in range(30):p.turn()
    press(p,'B');start=len(p.port.calls)
    p.landing=True;p.offboard=False;p.target=None;p.controller=c
    for _ in range(500):
        p.turn()
        if c.done:break
    assert c.done and c.journal_finalized and c.runtime.core.pilot_control
    assert not p.armed and p.locked
    assert not any(msg.get_type() in ('PARAM_SET','SET_POSITION_TARGET_LOCAL_NED') for msg in p.port.calls[start:])
