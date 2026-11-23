"""Only new fixed-profile native wiring. All peers/resources are synthetic.

The inherited no_real_hardware fixture rejects actual serial/input opens.
No old test functions are collected or repeated by this test module.
"""
import json
from pathlib import Path
from types import SimpleNamespace as NS
import sys

import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import ekf_static_profile as fixed
import vicon_fixed_point_entry as entry
from fixed_profile_fixtures import approved_profile
from test_continuous_execution import TransportPeer, FakePort
from flight_start_confirmation import build_guarded_session, FlightRunController
from flight_execution_review import ReviewedExecutionGrant


def wired(tmp_path, authorized=False, profile=None):
    profile=approved_profile() if profile is None else profile
    p=TransportPeer(fixed_configuration=True,fixed_profile=profile)
    p.port=FakePort()
    c=build_guarded_session(revision='SYNTHETIC-ONLY-rigid-revision',origin=[0,0,0],
        orientation=[0,0,0,1],port=p.port,
        poll_gamepad=lambda:{k:p.input_args[k] for k in
            ('pad_generation','pad_present','pad_event','home_events')},
        poll_pose=lambda:p.input_args['pose'],now_ros_ns=lambda:p.input_args['now_ros_ns'],
        check_port_identity=lambda:None,usb_present=lambda:False,
        origin_monotonic=0,recovery_path=tmp_path/'run'/'flight_recovery.json',fixed_profile=profile)
    p.runtime=c.runtime;p.io=c.io
    if authorized:p.io.authorizer=lambda *args:True  # Synthetic ONLY; no approval file.
    return p,c


def test_same_wire_owner_replies_in_vicon_clock_domain_and_rejects_replay(tmp_path):
    from pymavlink.dialects.v20 import common as m
    profile=approved_profile()
    profile['timestamp_mode']='local_ros_timesync'
    profile['timestamp_evidence']='SYNTHETIC test only'
    profile['persistence']['configuration_sha256']=fixed.configuration_digest(profile)
    p,c=wired(tmp_path,profile=profile)
    for _ in range(10):p.turn()
    token=123456789
    request=p.encoder.timesync_encode(0,token)
    p.turn(extra_rx=p.pack(request))
    replies=[msg for msg in p.port.calls if msg.get_type()=='TIMESYNC' and msg.tc1>0]
    assert len(replies)==1
    assert replies[0].ts1==token
    assert replies[0].tc1==p.input_args['now_ros_ns']
    assert replies[0].tc1!=int(p.now*1e9)
    assert c.runtime.fixed_sync_reply_count==1
    assert c.runtime.fixed_sync_reply is None
    # A previous response cannot be resent through the normal writer.
    with pytest.raises(PermissionError):c.io.send(replies[0])
    p.turn(extra_rx=p.pack(request))
    wrong=m.MAVLink(None,srcSystem=2,srcComponent=1)
    p.turn(extra_rx=wrong.timesync_encode(0,token+1).pack(wrong))
    # Installed dialect has no target extension; exercise newer-dialect
    # metadata at the decoded-message boundary without inventing wire bytes.
    targeted=m.MAVLink(None).parse_char(p.pack(p.encoder.timesync_encode(0,token+2)))
    targeted.target_system=42
    c.runtime._ingest_owner_message(targeted,p.now)
    valid=m.MAVLink(None).parse_char(p.pack(p.encoder.timesync_encode(0,token+3)))
    c.runtime.fixed_ros_ns=None
    c.runtime._ingest_owner_message(valid,p.now)
    c.runtime.fixed_ros_ns=p.input_args['now_ros_ns']
    c.runtime.inputs.fault='fc_reboot'
    c.runtime._ingest_owner_message(valid,p.now)
    assert c.runtime.fixed_sync_reply_count==1
    assert not c.runtime.core.arm_attempted
    assert not any(msg.get_type() in ('PARAM_SET','VISION_POSITION_ESTIMATE',
        'SET_POSITION_TARGET_LOCAL_NED') for msg in p.port.calls)


def test_fixed_entry_wire_pipeline_and_confirmed_closure(tmp_path):
    p,c=wired(tmp_path,authorized=True)
    assert not p.prepare()['failure']
    p.controller=c
    preview=c.start.create()
    assert not c.runtime.core.arm_attempted
    c.start.confirm(preview['confirmation_text'])
    for _ in range(5000):
        p.turn()
        if c.done:break
    assert c.done and c.runtime.core.phase=='COMPLETE',c.status()
    assert c.journal_finalized
    assert p.io.actual_tx_bytes==sum(len(bytes.fromhex(r['wire_hex'])) for r in p.port.records)
    kinds={m.get_type() for m in p.port.calls}
    assert {'TIMESYNC','VISION_POSITION_ESTIMATE','SET_POSITION_TARGET_LOCAL_NED'}<=kinds
    assert not kinds & {'PARAM_SET','PARAM_EXT_SET'}
    c.close()
    assert c.io.closed


def test_empty_review_slots_still_deny_effects_but_allow_fixed_queries(tmp_path):
    p,c=wired(tmp_path)
    assert c.review_slots.bootstrap is c.review_slots.execution is None
    p.prepare()
    assert c.runtime.failure
    assert all(m.get_type() not in ('VISION_POSITION_ESTIMATE','PARAM_SET') for m in p.port.calls)
    assert not any(m.get_type()=='COMMAND_LONG' and m.command!=512 for m in p.port.calls)
    assert not c.io.journal.record['events']


def test_profile_rejected_before_resource_access_or_journal(tmp_path):
    class Untouchable:
        def __getattribute__(self,key):raise AssertionError('resource touched')
    profile=approved_profile();profile['delay_calibration']['status']='pending_calibration'
    with pytest.raises(PermissionError,match='unapproved'):
        build_guarded_session(revision='x',origin=[0,0,0],orientation=[0,0,0,1],port=Untouchable(),
            poll_gamepad=lambda:None,poll_pose=lambda:None,now_ros_ns=lambda:0,
            check_port_identity=lambda:None,usb_present=lambda:False,origin_monotonic=0,
            recovery_path=tmp_path/'flight_recovery.json',fixed_profile=profile)
    assert not list(tmp_path.iterdir())


def test_existing_reviews_cannot_authorize_fixed_profile_and_mutation_invalidates(tmp_path):
    p,c=wired(tmp_path)
    grant=object.__new__(ReviewedExecutionGrant)
    grant.runtime=c.runtime;grant.record={}
    assert not grant._profile_ok()
    grant.record['fixed_profile_sha256']=fixed.fingerprint(c.runtime.coordinator.fusion.profile)
    assert grant._profile_ok()
    c.runtime.coordinator.fusion.profile['profile_id']='changed'
    assert not grant._profile_ok()


def test_review_dispatches_fixed_backup_not_legacy(tmp_path,monkeypatch):
    p,c=wired(tmp_path)
    grant=object.__new__(ReviewedExecutionGrant)
    grant.runtime=c.runtime;grant.record={'backup':'synthetic'};grant.wall=lambda:123
    calls=[]
    import preparation_authorization as auth
    monkeypatch.setattr(auth,'load_fixed_backup',lambda *args:calls.append(args))
    grant._load_backup()
    assert calls==[('synthetic',c.runtime.coordinator.fusion.profile,123)]


def test_native_factory_rejects_default_and_current_candidate_without_opens(tmp_path):
    kwargs=dict(profile_path=Path(entry.__file__).parent/'plan_b/fixed_point/vicon_profile.unapproved.json',
                recovery_path=tmp_path/'flight_recovery.json',trs_port='/dev/serial/by-id/unused')
    with pytest.raises(PermissionError,match='open_devices'):
        entry.open_session(**kwargs)
    with pytest.raises(PermissionError,match='unapproved'):
        entry.open_session(**kwargs,open_devices=True)
    assert not list(tmp_path.iterdir())


def test_native_factory_uses_existing_resources_and_closes_on_construction_failure(tmp_path,monkeypatch):
    import continuous_device_host as host
    import preparation_host_bindings as bindings
    import mavlink_readonly_probe as probe
    profile_path=tmp_path/'synthetic.json';profile_path.write_text(json.dumps(approved_profile()))
    events=[]
    class Owner:
        def __init__(self,path):self.port=FakePort();events.append('serial')
        def check(self):pass
        def close(self):events.append('serial_closed')
    class Pad:
        def __init__(self, *, controls=False):
            assert controls
            events.append('pad')
        def poll(self):return {}
        def close(self):events.append('pad_closed')
    class Pose:
        def __init__(self):raise RuntimeError('synthetic ROS failure')
    monkeypatch.setattr(host,'SerialOwner',Owner)
    monkeypatch.setattr(host,'configuration',lambda:dict(revision='SYNTHETIC',origin=[0,0,0],orientation=[0,0,0,1]))
    monkeypatch.setattr(bindings,'LinuxGamepad',Pad);monkeypatch.setattr(bindings,'RosLatestPose',Pose)
    monkeypatch.setattr(probe,'FC_USB',NS(exists=lambda:False))
    with pytest.raises(RuntimeError,match='ROS failure'):
        entry.open_session(profile_path=profile_path,recovery_path=tmp_path/'run'/'flight_recovery.json',
            trs_port='SYNTHETIC',open_devices=True)
    assert events==['serial','pad','pad_closed','serial_closed']
    events.clear()
    class WorkingPose:
        def __init__(self):events.append('pose')
        def poll(self):return None
        def now_ns(self):return 0
        def close(self):events.append('pose_closed')
    monkeypatch.setattr(bindings,'RosLatestPose',WorkingPose)
    c=entry.open_session(profile_path=profile_path,recovery_path=tmp_path/'run'/'flight_recovery.json',
        trs_port='SYNTHETIC',open_devices=True)
    assert isinstance(c,FlightRunController) and c.runtime.fixed_configuration
    assert not c.io.port.records and c.review_slots.execution is None
    c.close()
    assert events==['serial','pad','pose','pose_closed','pad_closed','serial_closed']


def test_close_retains_native_resources_when_airborne(tmp_path):
    p,c=wired(tmp_path,authorized=True);assert not p.prepare()['failure']
    calls=[];c.resources=NS(close=lambda:calls.append('closed'))
    c.runtime.core.arm_attempted=True
    c.runtime.inputs.hb['armed']=True
    c.runtime.inputs.actuator['armed']=True
    with pytest.raises(PermissionError,match='airborne'):c.close()
    assert not calls and not c.io.closed


def test_plan_four_differences_and_no_flight_claim():
    plan=entry.plan()
    assert [(r['name'],r['old'],r['target']) for r in plan['changes']]==[
        ('EKF2_EV_CTRL',0,11),('EKF2_HGT_REF',2,3),('EKF2_RNG_CTRL',2,0),('EKF2_OF_CTRL',1,0)]
    assert plan['devices_opened']==plan['physical_tx_bytes']==0
    assert not plan['flight_authorized'] and plan['full_gamepad_takeover_connected']
    assert not plan['hardware_takeover_verified']
