"""Only cleanup-induced runtime changes; no hardware or new execution framework."""
import json
from types import SimpleNamespace as NS

import pytest
import ev_delay_vicon_source as source
from flight_execution_review import ReviewedExecutionGrant, UID2


@pytest.mark.parametrize('error',[None,'wrong_library'])
def test_startup_metadata_closed_once_and_failure_evidence_preserved(error):
    calls=[]
    s=object.__new__(source.ViconSource)
    s.count=0;s.report={'audits':[],'shared_before':None}
    s.worker=NS(submit=lambda *a:calls.append('submit'),
        wait=lambda:{'payload':{'error':error}},
        close=lambda:calls.append('close') or True,status=lambda:{'closed':True})
    s.metadata=NS(finish_and_export=lambda:calls.append('export') or ['raw-proof'])
    if error:
        with pytest.raises(RuntimeError,match=error):s.startup_audit()
    else:s.startup_audit()
    assert calls==['submit','export','close']
    assert s.worker is s.metadata is None
    assert s.report['audit_evidence']==['raw-proof']
    assert len(s.report['audits'])==1


def test_receiver_exit_still_stops_polling():
    s=object.__new__(source.ViconSource);s.stopped=False
    s.proc=NS(poll=lambda:1)
    with pytest.raises(RuntimeError,match='receiver exited'):s.poll()


def review():
    c=NS(boot_session='boot',rigid_revision='rigid',uid2=UID2,firmware='1.14.3',
        epoch_valid=True,actuator_age=0,heartbeat_age=0,gamepad_current=True,
        source_age=0,attitude_age=0)
    c.pilot_current=lambda:True  # Existing context API shared by gamepad and Pocket.
    r=NS(now=1,closing=False,start_requested_at=0.,pilot_source='gamepad',inputs=NS(pad_event_at=1.),
        core=NS(phase='HOVER',arm_attempted=True),
        context=lambda:c,pilot_candidate_allowed=lambda m:False,failure=None,stop_latched=False)
    g=object.__new__(ReviewedExecutionGrant)
    g.runtime=r;g.consumed=True;g.fault=None;g.origin=0
    g.start_window=(0.,101.);g.last_rejection=None
    g.clock=lambda:1;g.wall=lambda:100;g.usb_present=lambda:False
    g._profile_ok=lambda:True;g.check_port_identity=lambda:None
    g.record=dict(boot_session='boot',rigid_revision='rigid',issued_unix=99,expires_unix=200)
    g.original_record=json.dumps(g.record,sort_keys=True,allow_nan=False)
    return g,r,c


@pytest.mark.parametrize('kind,command,hashed,allowed',[
    ('VISION_POSITION_ESTIMATE',0,False,True),
    ('SET_POSITION_TARGET_LOCAL_NED',0,False,True),
    ('RC_CHANNELS_OVERRIDE',0,False,True),
    ('COMMAND_LONG',400,True,False),
    ('COMMAND_LONG',176,True,False),
    ('PARAM_SET',0,True,False),
    ('COMMAND_LONG',21,False,True),
    ('COMMAND_LONG',185,False,True),
])
def test_hashing_only_at_non_safety_state_change(kind,command,hashed,allowed):
    g,r,c=review();calls=[]
    g._hashes_ok=lambda:calls.append('hash') or False
    msg=NS(get_type=lambda:kind,command=command,param1=2)
    assert g(r,{'at':1},msg) is allowed
    assert bool(calls) is hashed


@pytest.mark.parametrize('fault',['stale_pose','mutated_review'])
def test_no_hash_does_not_bypass_live_state_or_memory_review(fault):
    g,r,c=review()
    g._hashes_ok=lambda:pytest.fail('stream packet must not scan files')
    if fault=='stale_pose':c.source_age=1
    else:g.record['issued_unix']=98
    assert not g(r,{'at':1},NS(get_type=lambda:'VISION_POSITION_ESTIMATE'))


def test_expired_start_window_and_flight_timeout_do_not_block_land_recovery():
    g,r,c=review();r.now=200.;r.inputs.pad_event_at=200.
    g.clock=lambda:200.;g.wall=lambda:500.
    g._hashes_ok=lambda:True
    assert not g(r,{'at':200.},NS(get_type=lambda:'SET_POSITION_TARGET_LOCAL_NED'))
    assert g.last_rejection['reason']=='flight_duration_timeout'
    assert g(r,{'at':200.},NS(get_type=lambda:'COMMAND_LONG',command=21,param1=0))
    c.actuator_age=.51
    assert not g(r,{'at':200.},NS(get_type=lambda:'COMMAND_LONG',command=21,param1=0))


def test_expired_preparation_review_keeps_verified_ground_restore_available():
    # Isolate the review time boundary. Exact original parameter/journal
    # matching is still enforced by the existing runtime and ground grant.
    g,r,c=review();r.now=200.;r.start_requested_at=None;r.closing=True
    r.inputs.pad_event_at=200.;r.inputs.land={'at':200.}
    g.scope='battery-locked-ground-bootstrap-v1'
    c.restore_ground=lambda:True
    g.clock=lambda:200.;g.wall=lambda:500.
    msg=NS(get_type=lambda:'PARAM_SET')
    assert g(r,{'at':200.},msg)
    r.inputs.land['at']=199.49
    assert not g(r,{'at':200.},msg)
    r.inputs.land['at']=200.;c.epoch_valid=False
    assert not g(r,{'at':200.},msg)
