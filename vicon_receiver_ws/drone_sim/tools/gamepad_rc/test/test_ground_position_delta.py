"""Only the new locked-ground A path; synthetic peers, no device access."""
import pytest
from test_shared_pilot_entry import prepared, pilot_peer, press, mode_frames, receipt, reviewed_fake_grant
from flight_execution_review import ReviewedGroundBootstrapGrant
from px4_modes import custom_mode_from_param


def ground_peer(tmp_path):
    p,c=prepared(tmp_path)
    c.io.authorizer=reviewed_fake_grant(ReviewedGroundBootstrapGrant,c.runtime)
    return p,c


def test_ground_a_uses_same_writer_without_takeover_and_needs_ack_and_mode(tmp_path):
    p,c=ground_peer(tmp_path);r=c.runtime;start=len(p.port.calls)
    press(p,'A')
    assert len(mode_frames(p,start))==1
    assert r.pilot_link.mode_request['takeover'] is False
    assert not r.core.pilot_control and r.core.phase=='WAIT_EXPLICIT_START'
    assert r.context().lockdown and not r.core.arm_attempted
    for action in (c.start.create,lambda:r.request_start(operator_confirmed=True)):
        with pytest.raises(PermissionError,match='ACK and actual mode'):action()
    ack=p.pack(p.encoder.command_ack_encode(176,0,target_system=250,target_component=220))
    p.turn(extra_rx=ack)
    assert not r.ground_mode_confirmed()
    hb=p.pack(p.encoder.heartbeat_encode(2,12,0,custom_mode_from_param(2),0))
    p.turn(extra_rx=hb+receipt(p))
    assert r.ground_mode_confirmed() and c.start.create()['height_m']==.3
    assert not any(x.get_type()=='COMMAND_LONG' and x.command in (185,400,21)
                   for x in p.port.calls[start:])
    assert not any(x.get_type() in ('PARAM_SET','SET_POSITION_TARGET_LOCAL_NED')
                   for x in p.port.calls[start:])


@pytest.mark.parametrize('condition',['armed','hb_armed','not_landed','unlocked','stale_land',
                                    'sticks','home','cancel','stale_pad'])
def test_ground_a_requires_locked_current_stationary_unarmed_state(tmp_path,condition):
    p,c=ground_peer(tmp_path);r=c.runtime
    if condition=='armed':r.inputs.actuator['armed']=True
    elif condition=='hb_armed':r.inputs.hb['armed']=True
    elif condition=='not_landed':r.inputs.land['landed']=False
    elif condition=='unlocked':r.inputs.actuator['lockdown']=False
    elif condition=='stale_land':r.inputs.land['at']=r.now-1
    elif condition=='sticks':r.pilot_centered=False
    elif condition=='home':r.inputs.home_down=True
    elif condition=='cancel':r.cancel_requested=True
    elif condition=='stale_pad':r.pilot_sample_at=r.now-.01
    assert not r.ground_position_allowed()
    msg=r.encoder.command_long_encode(1,1,176,0,1,3,0,0,0,0,0)
    r.pilot_link.mode_request=dict(parameter=2,takeover=False,state='WAITING',sent_at=r.now)
    r._pilot_candidate=msg
    try:
        assert not r.pilot_candidate_allowed(msg)
        assert not c.io.authorizer(r,{'at':r.now},msg)
    finally:r._pilot_candidate=None


@pytest.mark.parametrize('failure',['reject','timeout'])
def test_failed_ground_mode_has_no_automatic_retry_or_start(tmp_path,failure):
    p,c=ground_peer(tmp_path);r=c.runtime;start=len(p.port.calls)
    press(p,'A')
    if failure=='reject':
        p.turn(extra_rx=p.pack(p.encoder.command_ack_encode(176,2,target_system=250,target_component=220)))
    else:r.pilot_link.check_mode_timeout(r.now+3.01)
    for _ in range(4):p.turn()
    assert len(mode_frames(p,start))==1 and not r.core.pilot_control
    assert not r.ground_mode_confirmed()
    assert c.status()['intervention_required']
    with pytest.raises(PermissionError,match='ACK and actual mode'):c.start.create()


def test_ground_l1_b_do_not_gain_a_new_mode_permission(tmp_path):
    p,c=ground_peer(tmp_path);start=len(p.port.calls)
    press(p,'L1');press(p,'B')
    assert not mode_frames(p,start) and not c.runtime.core.pilot_control


def test_land_preflight_block_can_be_released_before_mission_handoff(tmp_path):
    p,c=pilot_peer(tmp_path);r=c.runtime
    original=p._reply_text
    def commander(key):
        text=original(key)
        request=r.pilot_link.mode_request
        if key=='commander' and not (request and request['state']=='CONFIRMED'):
            text=text.replace('pre_flight_checks_pass: True','pre_flight_checks_pass: False')
            text=text.replace('nav_state: 4','nav_state: 18')
        return text
    p._reply_text=commander
    for _ in range(10):p.turn()
    p.turn(home=(True,));p.turn(home=(False,))
    for _ in range(1200):
        p.turn()
        if r.coordinator.fusion.phase=='FUSING' and r.inputs.audit_rows:break
    assert not r.failure and not r._handed_off()
    assert 'current_preflight_not_verified' in r.inputs.build(r.now)[1]
    c.io.authorizer=reviewed_fake_grant(ReviewedGroundBootstrapGrant,r)
    start=len(p.port.calls);press(p,'A')
    assert len(mode_frames(p,start))==1 and not r.core.pilot_control
    ack=p.pack(p.encoder.command_ack_encode(176,0,target_system=250,target_component=220))
    hb=p.pack(p.encoder.heartbeat_encode(2,12,0,custom_mode_from_param(2),0))
    p.turn(extra_rx=ack+hb)
    for _ in range(1200):
        p.turn()
        if r._handed_off() or r.failure:break
    assert not r.failure and r.core.phase=='WAIT_EXPLICIT_START'
    assert not r.core.arm_attempted and r.context().lockdown
