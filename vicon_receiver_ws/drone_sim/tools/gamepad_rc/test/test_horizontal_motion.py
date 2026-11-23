"""Only new air-motion paths; synthetic telemetry and zero real device/network I/O."""
from copy import deepcopy
from dataclasses import replace
import json
import math
from pathlib import Path
import socket
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from follow_target_generator import HorizontalMotion, HorizontalMotionSpec
from flight_mission_core import MissionCore
from mission_wire_transport import MissionWireTransport
from flight_start_confirmation import FlightRunController
from flight_execution_review import ReviewedExecutionGrant, flight_scope
from test_flight_mission_core import observation, selector, Sink, ack
from test_native_rc_mission_delta import NativePeer, start


@pytest.fixture(autouse=True)
def no_network(monkeypatch):
    monkeypatch.setattr(socket, 'socket', lambda *a, **k: pytest.fail('No network during air-motion tests'))


def config(pattern='directions', **changes):
    return {**HorizontalMotionSpec(pattern=pattern).to_dict(), **changes}


@pytest.mark.parametrize('pattern', ['directions', 'circle', 'directions_circle'])
@pytest.mark.parametrize('yaw', [0., math.pi/2, -.73])
def test_reference_geometry_continuity_and_dynamic_limits(pattern, yaw):
    path = HorizontalMotion(config(pattern))
    center = (2.3, -1.7, -.6)
    path.start(0., center_ned=center, yaw_ned=yaw)
    end = center
    for i, segment in enumerate(path.segments):
        assert path.sample(i, 0.) == pytest.approx(end)
        duration = segment['duration_s']
        n = max(2, math.ceil(duration/.01))
        dt = duration/n
        points = [path.sample(i, k*dt) for k in range(n+1)]
        assert all(p[2] == center[2] for p in points)
        assert max(math.dist(p[:2], center[:2]) for p in points) <= path.spec.extent_m+1e-10
        if segment['kind'] == 'circle':
            assert all(math.dist(p[:2], center[:2]) == pytest.approx(path.spec.extent_m) for p in points)
        if duration:
            velocities = [tuple((b-a)/dt for a,b in zip(p,q)) for p,q in zip(points, points[1:])]
            assert max(math.dist(v, (0,0,0)) for v in velocities) <= path.spec.speed_mps+1e-5
            assert max(math.dist(v,w)/dt for v,w in zip(velocities, velocities[1:])) <= path.spec.acceleration_mps2+1e-4
            assert math.dist(velocities[0], (0,0,0)) < .001
            assert math.dist(velocities[-1], (0,0,0)) < .001
        end = points[-1]
    assert end == pytest.approx(center)
    forward = path.sample(1, path.segments[1]['duration_s'])
    assert forward == pytest.approx((center[0]+.4*math.cos(yaw), center[1]+.4*math.sin(yaw), center[2]))
    if pattern in ('directions', 'directions_circle'):
        left = path.sample(5, path.segments[5]['duration_s'])
        assert left == pytest.approx((center[0]+.4*math.sin(yaw), center[1]-.4*math.cos(yaw), center[2]))


@pytest.mark.parametrize('field,value', [('extent_m',0),('speed_mps',float('nan')),
    ('acceleration_mps2',-1),('dwell_seconds',True),('arrival_radius_m',.4),('pattern','unknown')])
def test_reject_invalid_task_configuration(field, value):
    with pytest.raises(ValueError):HorizontalMotionSpec.from_dict(config(**{field:value}))


def test_clock_alone_does_not_claim_aircraft_arrived():
    path=HorizontalMotion(config());path.start(0.,center_ned=(0,0,-.3),yaw_ned=0)
    for t in (0.,10.,20.):
        path.step(t,position_ned=(.36,0,-.3),velocity_ned=(0,0,0))
    assert path.index == 0 and not path.complete
    for t in (20.1,23.2):path.step(t,position_ned=(.29,0,-.3),velocity_ned=(0,0,0))
    assert path.index == 1  # accepted demonstrator drift, not a 5 cm precision gate
    with pytest.raises(ValueError):path.step(20.,position_ned=(0,0,-.3),velocity_ned=(0,0,0))
    with pytest.raises(ValueError):path.start(24.,center_ned=(1,1,-.3),yaw_ned=1)


def native_observation(t, **changes):
    return replace(observation(t), pilot_source='native_rc', native_rc_ready=True,
                   native_sticks_ready=True, gamepad_available=False, home_response_verified=False,
                   **{'lockdown':False, **changes})


def core_run(pattern, *, yaw=.63, drift=.29, failure=None, observation_hook=None, motion_settings=None):
    sink=Sink();core=MissionCore(sink,height=.3,hover_seconds=3.,pilot_source='native_rc',
                               horizontal_motion=(motion_settings or config(pattern)) if pattern else None)
    envelope=MissionWireTransport();envelope.attach(core)
    def write(msg):
        assert envelope.candidate_allowed(msg)
        return sink(msg)
    core.sink=write
    assert not core.prepare(native_observation(0.,yaw_ned=yaw),selector())
    assert core.prepare(native_observation(2.,yaw_ned=yaw),selector())
    core.start(native_observation(2.1,yaw_ned=yaw),selector(),operator_confirmed=True)
    position=core.origin.position_ned;offboard=armed=False;land_at=None;cursor=0;rows=[]
    injected=False
    for i in range(2600):
        t=2.1+i*.05
        if land_at is not None and t-land_at>=1.:armed=False
        previous=position
        targets=[msg for msg in sink.messages if msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED']
        if targets:
            last=targets[-1]
            offset=drift if core.phase in ('ASCEND','HOVER') else 0.
            position=(last.x+offset,last.y,last.z)
        velocity=tuple((b-a)/.05 for a,b in zip(previous,position))
        o=native_observation(t,position_ned=position,velocity_ned=velocity,yaw_ned=yaw,
                             armed=armed,landed=not armed,offboard=offboard)
        if failure and not injected and core.motion.index >= 1:
            injected=True
            if failure=='pose_loss':o=replace(o,pose_age_s=1.)
            if failure=='manual':o=replace(o,offboard=False,manual_control_active=True)
        if observation_hook is not None:o=observation_hook(core,o)
        core.cycle(o,selector(),boot_ms=int(t*1000),
                   vision=(int((100+t)*1e6),[1.,-2.,-3.],[1.,0.,0.,0.],0.))
        for msg in sink.messages[cursor:]:
            if msg.get_type()!='COMMAND_LONG':continue
            assert ack(core,msg.command,t)
            if msg.command==176:offboard=True
            elif msg.command==400:armed=True
            elif msg.command==21:land_at=t
        cursor=len(sink.messages)
        rows.append(dict(t=t,phase=core.phase,stage=core.motion.describe()['stage'] if core.motion else None,position=o.position_ned))
        if core.phase in ('COMPLETE','PILOT_CONTROL','BLOCKED'):break
    return core,sink,envelope,rows


@pytest.mark.parametrize('pattern',['directions','circle'])
def test_full_mavlink_sequence_accepts_drift_and_returns_to_center(pattern):
    core,sink,envelope,rows=core_run(pattern)
    assert core.phase=='COMPLETE' and core.fault is None,core.status()
    assert core.motion.complete
    assert list(dict.fromkeys(row['stage'] for row in rows)) == [
        s['name'] for s in core.motion.segments]+['complete']
    targets=[msg for msg in sink.messages if msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED']
    assert math.dist((targets[-1].x,targets[-1].y),core.origin.position_ned[:2])<1e-6
    assert max(math.dist((m.x,m.y),core.origin.position_ned[:2]) for m in targets)==pytest.approx(.4,abs=1e-4)
    assert max(math.dist((a.x,a.y),(b.x,b.y)) for a,b in zip(targets,targets[1:])) <= .01501
    assert all(m.yaw==pytest.approx(core.origin.yaw_ned,abs=1e-6) for m in targets)
    assert [m.command for m in sink.messages if m.get_type()=='COMMAND_LONG']==[176,400,21]
    assert not any(m.get_type() in ('PARAM_SET','RC_CHANNELS_OVERRIDE') for m in sink.messages)


def test_native_fixed_hover_uses_same_separate_arrival_budget():
    core,sink,_,rows=core_run(None)
    assert core.phase=='COMPLETE' and core.fault is None,core.status()
    assert core.preview.arrival_criteria()==dict(horizontal_tolerance_m=.35,
        vertical_tolerance_m=.10,settling_speed_mps=.20)
    assert 'HOVER' in [r['phase'] for r in rows]
    assert [m.command for m in sink.messages if m.get_type()=='COMMAND_LONG']==[176,400,21]


@pytest.mark.parametrize('pattern',[None,'directions','circle'])
def test_normal_finish_descends_to_resting_reference_before_land(pattern):
    core,sink,wire,rows=core_run(pattern)
    assert core.phase=='COMPLETE' and core.fault is None
    assert 'DESCEND' in [r['phase'] for r in rows]
    targets=[r for r in core.sent if r['type']=='SET_POSITION_TARGET_LOCAL_NED'
             and r['t']>=core.descent_at]
    assert targets and targets[-1]['message']['z']==pytest.approx(core.origin.position_ned[2])
    assert targets[-1]['t']-core.descent_at>=5.9  # 30 cm at 5 cm/s
    for a,b in zip(targets,targets[1:]):
        assert 0<=b['message']['z']-a['message']['z']<=.05*(b['t']-a['t'])+1e-7
        assert (a['message']['x'],a['message']['y'])==pytest.approx(core.origin.position_ned[:2])
    land=next(r for r in core.sent if r['type']=='COMMAND_LONG' and r['message']['command']==21)
    assert land['t']>=targets[-1]['t']
    assert [m.command for m in sink.messages if m.get_type()=='COMMAND_LONG']==[176,400,21]
    assert core.status()['normal_descent']['resting_z_ned']==2.3  # not world zero


def test_reference_arrival_or_elapsed_time_alone_cannot_handoff_land():
    observed=[]
    def lag(core,o):
        if core.phase=='DESCEND':
            elapsed=o.t-core.descent_at
            if 6<=elapsed<8:
                o=replace(o,position_ned=(*o.position_ned[:2],core.origin.position_ned[2]-.08))
            elif 8<=elapsed<10:
                o=replace(o,velocity_ned=(0,0,.25))
            observed.append((elapsed,o))
        return o
    core,_,_,_=core_run(None,observation_hook=lag)
    assert core.phase=='COMPLETE' and core.fault is None
    assert core.landing_at-core.descent_at>=10
    assert abs(observed[-1][1].position_ned[2]-core.origin.position_ned[2])<=.02


@pytest.mark.parametrize('failure',['pose_loss','manual','excursion'])
def test_slow_descent_does_not_delay_existing_fault_or_takeover(failure):
    injected=[]
    def interrupt(core,o):
        if core.phase=='DESCEND' and not injected:
            injected.append(o.t)
            if failure=='excursion':return replace(o,position_ned=(10.,10.,o.position_ned[2]))
            return (replace(o,pose_age_s=1.) if failure=='pose_loss' else
                    replace(o,offboard=False,manual_control_active=True))
        return o
    core,sink,_,_=core_run(None,observation_hook=interrupt)
    assert injected
    if failure!='manual':
        assert core.landing_at==injected[0] and core.fault
    else:
        assert core.phase=='PILOT_CONTROL'
        assert not any(m.get_type()=='COMMAND_LONG' and m.command==21 for m in sink.messages)


def test_slow_descent_wire_rejects_unplanned_height_and_disarm():
    core=MissionCore(Sink(),height=.3,pilot_source='native_rc')
    core.origin=native_observation(0.);core._begin_descent(native_observation(1.))
    wire=MissionWireTransport();wire.attach(core)
    core.sink=wire;core._setpoint(native_observation(1.),1000)
    msg=core.encoder.parse_char(bytes.fromhex(wire.records[-1]['wire_hex']))
    assert wire.candidate_allowed(msg)
    msg.z+=.1
    assert not wire.candidate_allowed(msg)
    assert not wire.candidate_allowed(core._command(400,1.,0.))


@pytest.mark.parametrize('position,velocity', [((.351,0,-.3),(0,0,0)),
    ((.29,0,-.199),(0,0,0)),((.29,0,-.401),(0,0,0)),((.29,0,-.3),(.201,0,0))])
def test_wider_xy_does_not_accept_wrong_height_or_unsettled_endpoint(position,velocity):
    path=HorizontalMotion(config());path.start(0.,center_ned=(0,0,-.3),yaw_ned=0)
    for t in (0.,3.,6.):path.step(t,position_ned=position,velocity_ned=velocity)
    assert path.index==0 and not path.complete


@pytest.mark.parametrize('failure',['pose_loss','manual'])
def test_motion_keeps_existing_land_and_manual_takeover(failure):
    core,sink,_,rows=core_run('circle',failure=failure)
    commands=[m.command for m in sink.messages if m.get_type()=='COMMAND_LONG']
    assert commands.count(400)==1
    if failure=='pose_loss':assert core.phase=='COMPLETE' and core.fault and commands[-1]==21
    else:
        assert core.phase=='PILOT_CONTROL' and 21 not in commands
        assert not core.motion.complete


def test_wire_boundary_rejects_an_in_area_but_unplanned_target():
    core=MissionCore(Sink(),height=.3,pilot_source='native_rc',horizontal_motion=config())
    core.origin=native_observation(0.)
    core.motion.start(2.,center_ned=(1,2,2),yaw_ned=.6)
    core.last_t=2.;core.phase='HOVER'
    envelope=MissionWireTransport();envelope.attach(core)
    from disarmed_offboard_policy import POSITION_YAW_MASK
    from pymavlink.dialects.v20 import common as m
    msg=core.encoder.set_position_target_local_ned_encode(2000,1,1,m.MAV_FRAME_LOCAL_NED,
        POSITION_YAW_MASK,1,2,2,0,0,0,0,0,0,.6,0)
    assert envelope.candidate_allowed(msg)
    msg.x+=.1
    assert not envelope.candidate_allowed(msg)


@pytest.mark.parametrize('pattern',['directions','circle'])
@pytest.mark.parametrize('disturbance',['slow_audit','jitter'])
def test_real_entry_runtime_with_synthetic_fc_and_recorded_audit_gap(tmp_path,pattern,disturbance):
    p=NativePeer(tmp_path,timestamp_mode='local_ros_timesync',initial_trial=True,
                 horizontal_motion=config(pattern))
    assert not p.prepare()['failure'],p.controller.status()
    preview=p.controller.start.create()
    assert preview['horizontal_motion']['settings']['pattern']==pattern
    assert preview['arrival_criteria']==preview['horizontal_motion']['arrival_criteria']
    assert not p.runtime.core.arm_attempted
    # Exercise the real per-send predicates with synthetic identity/review.
    from test_shared_pilot_entry import reviewed_fake_grant
    r=p.runtime
    grant=reviewed_fake_grant(ReviewedExecutionGrant,r)
    p.io.authorizer=grant
    # Replay measured slow-audit cadence and inject bounded scheduling/sync
    # gaps in the SAME complete path. Fast poses are current after each pause.
    sync_ingest=r.fixed_clock.ingest
    clock_pause=None;clock_held=False;jittered=set()
    def intermittent_sync(msg,now_ns):
        nonlocal clock_held
        if clock_pause is not None and 0<=p.now-clock_pause<1.2:
            clock_held=True
            return False
        return sync_ingest(msg,now_ns)
    r.fixed_clock.ingest=intermittent_sync
    collector=p.runtime.coordinator.audit
    take_audits=collector.take_audits
    last=p.runtime.inputs.audit_at;gaps=[]
    def slow_full_audits():
        nonlocal last
        rows=take_audits()
        if rows and p.now-last>=5.575506057:
            gaps.append(p.now-last);last=p.now
            return rows[-1:]
        return []
    if disturbance=='slow_audit':collector.take_audits=slow_full_audits
    start(p)
    for _ in range(16000):
        if disturbance=='jitter' and r.core.phase=='HOVER' and r.core.motion.center is not None:
            if clock_pause is None:clock_pause=p.now
            index=r.core.motion.index
            if index not in jittered:
                p.now=round(p.now+.21,9);jittered.add(index)
        p.turn()
        if p.controller.done:break
    (tmp_path/'injected_path_result.json').write_text(json.dumps(dict(
        pattern=pattern,clock_holdover_s=1.2,jittered_segments=sorted(jittered),
        audit_gaps=gaps,status=p.controller.status(),failure_snapshot=p.controller.failure_snapshot),indent=2))
    if disturbance=='jitter':assert clock_held and len(jittered)>=3
    assert p.controller.done and p.runtime.core.phase=='COMPLETE',p.controller.status()
    assert p.runtime.core.fault is None,p.controller.status()
    if disturbance=='slow_audit':assert len(gaps)>=5 and min(gaps)>=5.575506057
    assert p.runtime.core.motion.complete and p.controller.journal_finalized
    assert p.runtime.sync_rates.summary()['restore_errors']==[]
    assert not any(msg.get_type() in ('PARAM_SET','PARAM_EXT_SET','RC_CHANNELS_OVERRIDE') for _,msg in p.calls)


def test_direction_mission_survives_repeated_rtt_bound_drops(tmp_path):
    from collections import deque
    from mavlink_crc_stream import CrcResyncParser
    p=NativePeer(tmp_path,timestamp_mode='local_ros_timesync',initial_trial=True,
                 horizontal_motion=config())
    assert not p.prepare()['failure']
    original=p.inspect_output;delayed=[];count=0;bounds=[];failure=None

    def varying_uplink():
        nonlocal count
        original()
        queue=deque()
        while p.pending:
            wire=p.pending.popleft();msg=CrcResyncParser().parse_buffer(wire)[0]
            if msg.get_type()=='TIMESYNC' and msg.ts1==p.runtime.fixed_clock.pending:
                count+=1
                if count % 2:
                    # FC stamps after 60 ms uplink, delivered after 70 ms.
                    wire=p.pack(p.encoder.timesync_encode(msg.tc1+60_000_000,msg.ts1))
                    delayed.append((p.now+.06,wire))
                    continue
            queue.append(wire)
        p.pending=queue
        for at,wire in delayed[:]:
            if p.now+1e-9>=at:
                p.pending.append(wire);delayed.remove((at,wire))
        ref=p.runtime.fixed_clock.reference
        if ref and (not bounds or ref['received_ns']!=bounds[-1][0]):
            bounds.append((ref['received_ns'],ref['received_ns']+ref['upper_ns']))

    p.inspect_output=varying_uplink
    start(p)
    for _ in range(16000):
        p.turn()
        if p.runtime.core.fault and failure is None:
            state=p.runtime.inputs.native_rc.status(p.now,max_age=1.)
            failure=json.dumps({k:v for k,v in state['topic_timing'].items() if v['problems']})
        if p.controller.done:break
    assert sum(b[1]<a[1] for a,b in zip(bounds,bounds[1:]))>10
    assert p.controller.done and p.runtime.core.phase=='COMPLETE',p.controller.status()
    assert p.runtime.core.fault is None and p.runtime.core.motion.complete, failure
    assert p.controller.journal_finalized and not p.runtime.sync_rates.summary()['restore_errors']


def test_profile_binding_and_old_vertical_scope_cannot_expand(tmp_path):
    from ekf_static_profile import fingerprint, configuration_digest, load_json, require_mission_profile
    from test_shared_pilot_entry import reviewed_fake_grant
    p=NativePeer(tmp_path,horizontal_motion=config())
    profile=p.runtime.coordinator.fusion.profile
    baseline=deepcopy(profile);baseline.pop('horizontal_motion')
    assert configuration_digest(profile)==configuration_digest(baseline)
    assert fingerprint(profile)!=fingerprint(baseline)
    assert flight_scope('native_rc',profile)!=flight_scope('native_rc',baseline)
    grant=reviewed_fake_grant(ReviewedExecutionGrant,p.runtime)
    del grant._profile_ok
    grant.record['fixed_profile_sha256']=fingerprint(profile)
    assert grant._profile_ok()
    p.runtime.core.motion.spec=HorizontalMotionSpec(pattern='circle')
    assert not grant._profile_ok()
    assert grant._reject('test_only') is False


def test_task_duration_budget_uses_same_computed_route_budget(tmp_path):
    p=NativePeer(tmp_path,horizontal_motion=config(speed_mps=.04))
    assert FlightRunController.task_seconds(p.runtime)>120.
    assert FlightRunController.task_seconds(p.runtime)>p.runtime.core.motion.nominal_seconds


def test_shipped_profiles_are_offline_and_keep_fc_values_unchanged():
    from ekf_static_profile import load_json,configuration_digest,require_mission_profile
    root=Path(__file__).resolve().parents[1]/'plan_b/fixed_point'
    baseline=load_json(root/'vicon_profile.initial_trial.json')
    for pattern in ('directions','circle','directions_circle'):
        profile=load_json(root/f'vicon_profile.{pattern}.json')
        require_mission_profile(profile,pilot_source='native_rc',height=.3,hover_seconds=3.)
        assert profile['parameters']==baseline['parameters']
        assert configuration_digest(profile)==configuration_digest(baseline)
        assert profile['initial_trial']['flight_authorized'] is False
        assert profile['horizontal_motion']['pattern']==pattern


@pytest.mark.parametrize('pattern',['circle','directions_circle'])
def test_cli_motion_preview_is_offline(capsys,pattern):
    import vicon_fixed_point_entry as entry
    path=Path(__file__).resolve().parents[1]/f'plan_b/fixed_point/vicon_profile.{pattern}.json'
    assert entry.main(['--profile',str(path)])==0
    result=json.loads(capsys.readouterr().out)
    assert result['horizontal_motion']['settings']['pattern']==pattern
    assert result['horizontal_motion']['center_ned_m'] is None


def test_combined_demo_profile_completes_all_stages_before_slow_descent():
    path=Path(__file__).resolve().parents[1]/'plan_b/fixed_point/vicon_profile.directions_circle.json'
    settings=json.loads(path.read_text())['horizontal_motion']
    core,sink,wire,rows=core_run('directions_circle',motion_settings=settings)
    assert core.phase=='COMPLETE' and core.fault is None,core.status()
    assert core.motion.complete
    stages=list(dict.fromkeys(row['stage'] for row in rows))
    assert stages==['center_hold','forward','return_forward','backward','return_backward',
                    'left','return_left','right','return_right','circle_entry','circle_ccw',
                    'circle_return','complete']
    assert core.motion.nominal_seconds==pytest.approx(58.8495559215)
    assert core.motion.spec.speed_mps==.25 and core.motion.spec.acceleration_mps2==.3
    assert core.motion.maximum_excursion_m==pytest.approx(.85)
    assert 'DESCEND' in [r['phase'] for r in rows]
    desc=[r for r in core.sent if r['type']=='SET_POSITION_TARGET_LOCAL_NED' and r['t']>=core.descent_at]
    assert desc[-1]['t']-core.descent_at>=5.9
    for a,b in zip(desc,desc[1:]):
        assert 0<=b['message']['z']-a['message']['z']<=.05*(b['t']-a['t'])+1e-7
    assert desc[-1]['message']['z']==pytest.approx(core.origin.position_ned[2])
    assert (desc[-1]['message']['x'],desc[-1]['message']['y'])==pytest.approx(core.origin.position_ned[:2])
    assert [m.command for m in sink.messages if m.get_type()=='COMMAND_LONG']==[176,400,21]
    assert all(m.yaw==pytest.approx(core.origin.yaw_ned,abs=1e-6)
               for m in sink.messages if m.get_type()=='SET_POSITION_TARGET_LOCAL_NED')
    assert not any(m.get_type() in ('PARAM_SET','RC_CHANNELS_OVERRIDE') for m in sink.messages)


def test_combined_demo_reference_limits_at_increased_speed():
    path=Path(__file__).resolve().parents[1]/'plan_b/fixed_point/vicon_profile.directions_circle.json'
    motion=HorizontalMotion(json.loads(path.read_text())['horizontal_motion'])
    motion.start(0.,center_ned=(1.,2.,-.3),yaw_ned=.63)
    end=motion.center
    for i,segment in enumerate(motion.segments):
        assert motion.sample(i,0.)==pytest.approx(end)
        duration=segment['duration_s'];n=max(2,math.ceil(duration/.005))
        points=[motion.sample(i,duration*k/n) for k in range(n+1)]
        if duration:
            dt=duration/n
            vel=[tuple((b-a)/dt for a,b in zip(p,q)) for p,q in zip(points,points[1:])]
            assert max(math.dist(v,(0,0,0)) for v in vel)<=.25+1e-5
            assert max(math.dist(v,w)/dt for v,w in zip(vel,vel[1:]))<=.3+1e-4
        end=points[-1]
    assert end==pytest.approx(motion.center)
