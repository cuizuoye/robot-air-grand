"""Only new cooperative contracts; no vehicle, ROS graph or network access."""
from dataclasses import replace
import copy
import json
import math
from pathlib import Path
import socket
import subprocess
import sys

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from follow_target_generator import AirGroundSequence, TargetPose, _wrap
from replay_follow_targets import cooperative_demo, save_cooperative_demo
from drone_console import ConsoleModel,Session
import vicon_fixed_point_entry as entry


@pytest.fixture(autouse=True)
def no_network(monkeypatch):
    def denied(*a,**k):raise AssertionError('offline cooperative test opened a network or child process')
    monkeypatch.setattr(socket,'socket',denied)
    monkeypatch.setattr(subprocess,'Popen',denied)


@pytest.fixture
def config():
    return json.loads((ROOT/'plan_b/follow/air_ground_example.json').read_text())


@pytest.mark.parametrize('case',['wait','start','stale_pose','regression','airborne_gap'])
def test_ground_wait_refresh_is_not_a_control_integration_gap(config,case):
    seq=AirGroundSequence(config)
    def step(t,**changes):
        args=dict(now_ns=t,now_source_ns=t,frame_revision=config['frame_revision'],
            car=TargetPose('car01','map',t,t,(0.,0.,0.),(0.,0.,0.,1.),True,False),
            uav=TargetPose('MircoAir','map',t,t,(.03,.02,.15),(0.,0.,0.,1.),True,False),
            car_velocity=(0.,0.,0.),uav_velocity=(0.,0.,0.),
            car_ready=True,uav_ready=True,car_stopped=True,armed=False,landed=True)
        args.update(changes)
        return seq.step(**args)
    assert step(2_000_000_000)['phase']=='WAIT_START'
    # Live evidence: the full audit was unavailable for about 18 seconds,
    # while host polling continued and both vehicles remained stopped.
    changes={}
    if case=='start':changes['start']=True
    if case=='stale_pose':
        changes['car']=TargetPose('car01','map',2_000_000_000,2_000_000_000,
                                  (0.,0.,0.),(0.,0.,0.,1.),True,False)
    if case=='regression':changes['now_source_ns']=1_999_999_999
    if case=='airborne_gap':
        assert step(2_020_000_000,start=True)['phase']=='TAKEOFF'
        changes.update(armed=True,landed=False)
    state=step(20_000_000_000,**changes)
    if case in ('wait','start'):
        assert state['fault'] is None
        assert state['phase']==('TAKEOFF' if case=='start' else 'WAIT_START')
        assert state['car_action']=='stop'
    else:
        assert state['fault']
        if case!='stale_pose':assert state['fault']=='cooperative_clock_gap_or_regression'


def test_nominal_measured_sequence_and_return_to_relative_pose(config):
    report,rows=cooperative_demo(config)
    assert report['complete'] and not report['failure']
    assert [p['phase'] for p in report['phases']]==[
        'WAIT_START','TAKEOFF','FOLLOW','WAIT_CAR_STOP','ALIGN','DESCEND','LAND','COMPLETE']
    assert report['final_relative_xy_m']==pytest.approx([.03,.02],abs=.005)
    assert report['final_relative_yaw_rad']==pytest.approx(.2,abs=.01)
    assert report['actual_sent_to_fc']==report['actual_sent_to_car']==report['devices_opened']==0
    assert report['live_adapters_connected'] is False
    for row in rows:
        s=row['status']
        if s['car_action']=='route':assert row['observed_armed'] and not row['observed_landed']
        if s['phase'] in ('ALIGN','DESCEND','LAND','COMPLETE'):assert row['observed_car_stopped']
        if s['phase']=='COMPLETE':assert row['observed_landed'] and not row['observed_armed']
    # Target motion, including the turn at the end, is bounded without resetting
    # the original relative heading or independently shifting world coordinates.
    for a,b in zip(rows,rows[1:]):
        sa,sb=a['status'],b['status']; dt=(b['at_ns']-a['at_ns'])*1e-9
        if sa['uav_target_map_m'] is not None and sb['uav_target_map_m'] is not None:
            assert math.dist(sa['uav_target_map_m'][:2],sb['uav_target_map_m'][:2]) <= .15*dt+1e-8
            assert abs(_wrap(sb['uav_yaw_map_rad']-sa['uav_yaw_map_rad'])) <= .3*dt+1e-8


@pytest.mark.parametrize('initial',[(.6,-.5,.2,1.1),(-.5,.3,.1,-2.6)])
def test_arbitrary_initial_position_does_not_reanchor_fixed_route(config,initial):
    report,rows=cooperative_demo(config,initial_car=initial)
    assert report['complete'],report['failure']
    assert report['final_relative_xy_m']==pytest.approx([.03,.02],abs=.01)
    goals={tuple(r['status']['car_goal_map_xyyaw']) for r in rows if r['status']['car_action']=='route'}
    assert goals==set(map(tuple,config['route_world_xyyaw']))
    assert report['final_relative_yaw_rad']==pytest.approx(.2,abs=.01)


def test_world_heading_option_remains_explicit(config):
    config['landing_heading']='initial_world'
    report,rows=cooperative_demo(config)
    assert report['complete']
    assert rows[-1]['uav_yaw_rad']==pytest.approx(.2,abs=.01)
    assert rows[-1]['car_yaw_rad']==pytest.approx(math.pi/2,abs=.01)


def test_no_car_start_from_takeoff_intention_alone(config):
    def keep_ground(t,seq,inputs):
        inputs.update(armed=False,landed=True)
        inputs['uav']=replace(inputs['uav'],position=(.03,.02,.3))
    report,rows=cooperative_demo(config,duration_s=8,hook=keep_ground)
    assert not report['complete']
    assert not any(r['status']['car_action']=='route' for r in rows)


@pytest.mark.parametrize('stop_jitter',[False,True])
def test_ground_descent_tolerates_drift_and_never_reclimbs_after_pause(stop_jitter):
    plan=json.loads((ROOT/'plan_b/follow/air_ground_side_follow.current.json').read_text())
    # Preserve the previous permissive profile's contract independently of
    # the current landing-accuracy candidate.
    plan.update(descent_speed_mps=.05,landing_xy_tolerance_m=.3,landing_horizontal_speed_mps=.2,
                ground_descent_continuous=False)
    for key in ('landing_integral_gain_s_inv2','landing_velocity_damping',
                'landing_settling_speed_mps','landing_settle_seconds'):
        plan.pop(key,None)
    seq=AirGroundSequence(plan)
    def pose(source,p,t):return TargetPose(source,'map',t,t,tuple(p),(0.,0.,0.,1.),True,False)
    car=pose('car01',(0.,.7,.1),10**9);uav=pose('MircoAir',(0.,0.,.04),10**9)
    seq.resolve_start(car,uav,car,uav)
    seq._capture(car,uav)
    seq.phase='ALIGN';seq.target=(0.,0.,.54)
    seq.landing_car_anchor=car  # Already confirmed by WAIT_CAR_STOP.
    seq.generator.set_mode('HOVER',hold_xy=(0.,0.))
    zs=[];paused=[];first_descent=None
    for i in range(1000):
        t=10**9+i*20_000_000
        # 20 cm residual and 0.10 m/s: above BOTH old strict thresholds.
        # Brief 40 cm excursion midway must HOLD altitude, never climb.
        excursion=200<=i<230
        at=(.4 if excursion else .2,0.,seq.target[2])
        landing=seq.phase=='LAND'
        state=seq.step(now_ns=t,now_source_ns=t,frame_revision=plan['frame_revision'],
            car=pose('car01',car.position,t),uav=pose('MircoAir',at,t),
            car_velocity=(.12,0.,.1) if stop_jitter else (0.,0.,0.),uav_velocity=(.1,0.,-.05 if seq.phase=='DESCEND' else 0.),
            car_ready=True,uav_ready=True,car_stopped=not stop_jitter,armed=not landing,landed=landing)
        assert not state['fault'],state
        assert state['phase']!='WAIT_CAR_STOP'
        if state['phase'] in ('DESCEND','LAND'):
            if first_descent is None:first_descent=i
            zs.append(state['uav_target_map_m'][2])
            if excursion:paused.append(state['uav_target_map_m'][2]);assert state['descent_paused']
        if state['complete']:break
    assert state['complete'] and first_descent<200
    assert len(paused)==30 and max(paused)-min(paused)<1e-9
    assert all(0<=a-b<=.05*.02+1e-9 for a,b in zip(zs,zs[1:]))
    assert zs[-1]==pytest.approx(.04)


def test_optional_precision_landing_waits_for_xy_convergence_then_descends_without_abort():
    plan=json.loads((ROOT/'plan_b/follow/air_ground_side_follow.current.json').read_text())
    plan.update(ground_descent_continuous=False,landing_xy_tolerance_m=.05,
                landing_settling_speed_mps=.04,landing_settle_seconds=1.5)
    seq=AirGroundSequence(plan)
    def pose(source,p,t):return TargetPose(source,'map',t,t,tuple(p),(0.,0.,0.,1.),True,False)
    car=pose('car01',(0.,.7,.1),10**9);uav=pose('MircoAir',(0.,0.,.04),10**9)
    seq.resolve_start(car,uav,car,uav);seq._capture(car,uav)
    seq.phase='ALIGN';seq.target=(0.,0.,.54);seq.landing_car_anchor=car
    seq.generator.set_mode('HOVER',hold_xy=(0.,0.))
    paused=0;rates=[]
    for i in range(1200):
        t=10**9+i*20_000_000
        # Reproduce the 28 cm residual: wait at cruise, not an early descent.
        # Then settle, briefly drift 18 cm during descent, and converge again.
        excursion=330<=i<480
        x=.28 if i<150 else .18 if excursion else .04
        vx=.12 if 150<=i<180 else .02
        before=seq.target[2];landing=seq.phase=='LAND'
        state=seq.step(now_ns=t,now_source_ns=t,frame_revision=plan['frame_revision'],
            car=pose('car01',car.position,t),uav=pose('MircoAir',(x,0.,before),t),
            car_velocity=(0.,0.,0.),uav_velocity=(vx,0.,-.04),
            car_ready=True,uav_ready=True,car_stopped=True,armed=not landing,landed=landing)
        assert not state['fault'],state
        assert state['car_action']=='stop'
        after=state['uav_target_map_m'][2]
        if i<180:assert state['phase']=='ALIGN' and after==.54
        if excursion:
            assert state['phase']=='DESCEND' and state['descent_paused']
            assert after==before
            paused+=1
        rates.append((before-after)/.02)
        if state['complete']:break
    assert state['complete'] and paused==150
    assert min(rates)>=-1e-9
    assert max(rates)==pytest.approx(.08)
    assert state['uav_target_map_m'][2]==pytest.approx(.04)


def test_current_ground_descent_finishes_despite_recorded_residual_and_waits_for_touchdown():
    plan=json.loads((ROOT/'plan_b/follow/air_ground_side_follow.current.json').read_text())
    seq=AirGroundSequence(plan)
    def pose(source,p,t):return TargetPose(source,'map',t,t,tuple(p),(0.,0.,0.,1.),True,False)
    car=pose('car01',(0.,.7,.1),10**9);uav=pose('MircoAir',(0.,0.,.04),10**9)
    seq.resolve_start(car,uav,car,uav);seq._capture(car,uav)
    seq.phase='ALIGN';seq.target=(0.,0.,.54);seq.landing_car_anchor=car
    seq.generator.set_mode('HOVER',hold_xy=(0.,0.))
    zs=[]
    for i in range(500):
        t=10**9+i*20_000_000
        # Persistent 4..12 cm XY residual and 7 cm/s horizontal motion: the
        # last real flight hovered here. Neither now interrupts descent.
        x=.08+.04*math.sin(i*.08);before=seq.target[2]
        state=seq.step(now_ns=t,now_source_ns=t,frame_revision=plan['frame_revision'],
            car=pose('car01',car.position,t),uav=pose('MircoAir',(x,.02,before),t),
            car_velocity=(0.,0.,0.),uav_velocity=(.07,0.,-.08),
            car_ready=True,uav_ready=True,car_stopped=True,armed=True,landed=False)
        assert not state['fault'] and not state['descent_paused']
        zs.append(state['uav_target_map_m'][2])
        if state['phase']=='LAND':break
    assert state['phase']=='LAND' and i*.02<8
    assert all(-1e-9<=a-b<=.08*.02+1e-9 for a,b in zip(zs,zs[1:]))
    assert not state['complete'] and state['landing_waiting_for']=='touchdown'
    for landed,armed,expected in [(False,True,'touchdown'),(True,True,'px4_auto_disarm'),(True,False,None)]:
        i+=1;t=10**9+i*20_000_000
        state=seq.step(now_ns=t,now_source_ns=t,frame_revision=plan['frame_revision'],
            car=pose('car01',car.position,t),uav=pose('MircoAir',(.10,.02,.04),t),
            car_velocity=(0.,0.,0.),uav_velocity=(0.,0.,0.),car_ready=True,
            uav_ready=True,car_stopped=True,armed=armed,landed=landed)
        assert not state['fault'] and state['landing_waiting_for']==expected
        assert state['complete']==(landed and not armed)


@pytest.mark.parametrize('feedback',[False,True])
def test_landing_vicon_feedback_corrects_error_without_moving_frozen_target(feedback):
    plan=json.loads((ROOT/'plan_b/follow/air_ground_side_follow.current.json').read_text())
    if not feedback:
        for key in ('landing_position_feedback_gain_s_inv','landing_integral_gain_s_inv2','landing_velocity_damping'):
            plan.pop(key,None)
    seq=AirGroundSequence(plan)
    def pose(source,p,t):return TargetPose(source,'map',t,t,tuple(p),(0.,0.,0.,1.),True,False)
    car=pose('car01',(0.,.7,.1),10**9);uav=pose('MircoAir',(0.,0.,.04),10**9)
    seq.resolve_start(car,uav,car,uav);seq._capture(car,uav)
    seq.phase='ALIGN';seq.target=(0.,0.,.54);seq.landing_car_anchor=car
    seq.generator.set_mode('HOVER',hold_xy=(0.,0.))
    previous=(0.,0.);velocities=[]
    for i in range(300):
        t=10**9+i*20_000_000
        # Two opposite residuals; neither is close enough to start descent.
        p=(.3,.1,.54) if i<150 else (-.3,-.1,.54)
        state=seq.step(now_ns=t,now_source_ns=t,frame_revision=plan['frame_revision'],
            car=pose('car01',car.position,t),uav=pose('MircoAir',p,t),
            car_velocity=(0.,0.,0.),uav_velocity=(0.,0.,0.),
            car_ready=True,uav_ready=True,car_stopped=True,armed=True,landed=False)
        v=state['uav_velocity_map_mps'][:2];velocities.append(v)
        assert not state['fault'] and state['phase']=='ALIGN'
        assert state['uav_target_map_m']==(0.,0.,.54)
        assert tuple(state['resolved_start']['uav_ground_position_map_m'])==(0.,0.,.04)
        assert math.hypot(*v)<=plan['landing_horizontal_speed_mps']+1e-9
        assert math.dist(v,previous)<=plan['follow_limits']['acceleration_mps2']*.02+1e-9
        previous=v
    if feedback:
        assert velocities[149][0]<0 and velocities[149][1]<0
        assert velocities[-1][0]>0 and velocities[-1][1]>0
    else:assert all(v==(0.,0.) for v in velocities)
    # A stale pose must not keep sending the previous correction.
    state=seq.step(now_ns=t+20_000_000,now_source_ns=t+20_000_000,
        frame_revision=plan['frame_revision'],car=pose('car01',car.position,t+20_000_000),
        uav=uav,car_velocity=(0.,0.,0.),uav_velocity=(0.,0.,0.),
        car_ready=True,uav_ready=True,car_stopped=True,armed=True,landed=False)
    assert state['phase']=='HOLD' and state['uav_velocity_map_mps']==(0.,0.,0.)


def test_ground_landing_anchor_does_not_hide_actual_car_motion():
    plan=json.loads((ROOT/'plan_b/follow/air_ground_side_follow.current.json').read_text())
    seq=AirGroundSequence(plan)
    def pose(source,position):return TargetPose(source,'map',10**9,10**9,position,(0.,0.,0.,1.),True,False)
    car=pose('car01',(0.,.7,.1));uav=pose('MircoAir',(0.,0.,.04))
    seq.resolve_start(car,uav,car,uav);seq._capture(car,uav)
    seq.phase='DESCEND';seq.descent_started=True;seq.target=(0.,0.,.3);seq.landing_car_anchor=car
    state=seq.step(now_ns=10**9,now_source_ns=10**9,frame_revision=plan['frame_revision'],
        car=replace(car,position=(.15,.7,.1)),uav=replace(uav,position=(0.,0.,.3)),
        car_velocity=(0.,0.,0.),uav_velocity=(0.,0.,0.),car_ready=True,uav_ready=True,
        car_stopped=True,armed=True,landed=False)
    assert not state['fault'] and state['phase']=='WAIT_CAR_STOP'
    assert state['uav_target_map_m'][2]==.3


@pytest.mark.parametrize('lag',[.03,.06])
def test_descent_pause_captures_actual_height_and_resumes_without_downward_jump(lag):
    plan=json.loads((ROOT/'plan_b/follow/air_ground_side_follow.current.json').read_text())
    plan.update(ground_descent_continuous=False,landing_xy_tolerance_m=.05,
                landing_settling_speed_mps=.04,landing_settle_seconds=1.5)
    seq=AirGroundSequence(plan)
    def pose(source,p,t):return TargetPose(source,'map',t,t,tuple(p),(0.,0.,0.,1.),True,False)
    car=pose('car01',(0.,.7,.1),10**9);uav=pose('MircoAir',(0.,0.,.04),10**9)
    seq.resolve_start(car,uav,car,uav);seq._capture(car,uav)
    seq.phase='DESCEND';seq.descent_started=True;seq.target=(0.,0.,.12)
    seq.landing_car_anchor=car;seq.generator.set_mode('HOVER',hold_xy=(0.,0.))
    measured=.12+lag
    frozen=tuple(seq.resolved_start['uav_ground_position_map_m'])
    resumed=False
    for i in range(240):
        t=10**9+i*20_000_000
        # Sustained 18 cm error, then a single good sample that must NOT
        # restart descent, then stable convergence. Z noise must not reanchor.
        x=.02 if i==70 or i>=160 else .18
        actual_z=measured if i==0 else measured-.002*math.sin(i*.1)
        before=seq.target[2]
        s=seq.step(now_ns=t,now_source_ns=t,frame_revision=plan['frame_revision'],
            car=pose('car01',car.position,t),uav=pose('MircoAir',(x,0.,actual_z),t),
            car_velocity=(0.,0.,0.),uav_velocity=(0.,0.,0.),
            car_ready=True,uav_ready=True,car_stopped=True,armed=True,landed=False)
        assert not s['fault'] and s['phase']=='DESCEND'
        assert s['car_action']=='stop'
        assert tuple(s['resolved_start']['uav_ground_position_map_m'])==frozen
        if i==0:
            assert s['uav_target_map_m'][2]==pytest.approx(actual_z)
            assert s['uav_target_map_m'][2]>before
        if i<190:
            assert s['descent_paused'] and s['uav_target_map_m'][2]==pytest.approx(measured)
        elif not s['descent_paused']:
            resumed=True
            assert 0<=before-s['uav_target_map_m'][2]<=plan['descent_speed_mps']*.02+1e-9
    assert resumed


@pytest.mark.parametrize('what,reason',[
    ('car_stale','subject_stale:car01'),('wrong_uav','subject_or_world_frame_mismatch'),
    ('frame','world_or_rigid_revision_changed'),('car_not_ready','paired_backend_not_ready')])
def test_fault_stops_car_and_preserves_first_reason(config,what,reason):
    def fault(t,seq,i):
        if seq.phase!='FOLLOW':return
        if what=='car_stale':i['car']=replace(i['car'],stamp_ns=i['now_source_ns']-600_000_000)
        elif what=='wrong_uav':i['uav']=replace(i['uav'],source='car01')
        elif what=='frame':i['frame_revision']='rebuilt'
        else:i['car_ready']=False
    report,rows=cooperative_demo(config,hook=fault)
    assert report['failure']==reason and not report['complete']
    assert rows[-1]['status']['phase']=='HOLD' and rows[-1]['status']['car_action']=='stop'


def test_waits_for_observed_car_stop_not_route_end_timer(config):
    def moving(t,seq,i):
        if seq.phase=='WAIT_CAR_STOP':i['car_stopped']=False
    report,rows=cooperative_demo(config,duration_s=58,hook=moving)
    assert not report['complete']
    assert rows[-1]['status']['phase']=='WAIT_CAR_STOP'
    assert not any(r['status']['land_intent_issued'] for r in rows)


def test_car_motion_during_descent_cancels_descent(config):
    touched=[]
    def moving(t,seq,i):
        if seq.phase=='DESCEND' and not touched:
            touched.append((t,seq.target[2]));i.update(car_stopped=False,car_velocity=(.08,0.,0.))
    report,rows=cooperative_demo(config,hook=moving)
    assert touched
    row=next(r for r in rows if abs((r['at_ns']-10**9)*1e-9-touched[0][0])<1e-6)
    assert row['status']['phase']=='WAIT_CAR_STOP'
    assert row['status']['uav_target_map_m'][2]>=touched[0][1]


def test_landed_flag_alone_never_completes(config):
    def still_armed(t,seq,i):
        if seq.phase=='LAND':i.update(landed=True,armed=True)
    report,rows=cooperative_demo(config,duration_s=58,hook=still_armed)
    assert not report['complete'] and rows[-1]['status']['phase']=='LAND'


def test_floor_landing_below_platform_is_not_success(config):
    def floor(t,seq,i):
        if seq.phase=='LAND':
            i.update(armed=False,landed=True)
            p=i['uav'].position
            i['uav']=replace(i['uav'],position=(*p[:2],p[2]-.4))
    report,rows=cooperative_demo(config,hook=floor)
    assert not report['complete'] and report['failure']=='landed_outside_platform'


def test_rviz_uses_separate_shared_world_names_and_synthetic_labels(config,monkeypatch):
    from types import ModuleType,SimpleNamespace as N
    from drone_console import RvizView
    geometry=ModuleType('geometry_msgs.msg'); geometry.Point=lambda **kw:N(**kw)
    visuals=ModuleType('visualization_msgs.msg')
    class Marker:
        DELETEALL=3;LINE_STRIP=4;ADD=0;SPHERE=2;TEXT_VIEW_FACING=9;CUBE=1;CYLINDER=3
        def __init__(self):
            self.header=N();self.pose=N(position=N(),orientation=N());self.scale=N();self.color=N()
    visuals.Marker=Marker;visuals.MarkerArray=lambda:N(markers=[])
    monkeypatch.setitem(sys.modules,'geometry_msgs.msg',geometry)
    monkeypatch.setitem(sys.modules,'visualization_msgs.msg',visuals)
    m=ConsoleModel();_,rows=cooperative_demo(config,duration_s=.04)
    for row in rows:m.ingest('cooperative.jsonl',row)
    published=[];view=object.__new__(RvizView);view.label_size=.12
    view.pub=N(publish=published.append);view.node=N(get_clock=lambda:N(now=lambda:N(to_msg=lambda:None)))
    view.draw(m,'coop_uav',True)
    texts=[v.text for v in published[0].markers if hasattr(v,'text')]
    assert len(texts)==2 and all(t.startswith('DEMO-') for t in texts)
    names={v.ns for v in published[0].markers if hasattr(v,'ns')}
    assert {key+'_'+kind for key in ('coop_uav','coop_car') for kind in ('start','body','label')} <= names
    assert names <= {key+'_'+kind for key in ('coop_uav','coop_car') for kind in ('trace','start','body','label')}


def test_small_platform_rejected_before_takeoff(config):
    config['platform_size_m']=[.23,.23]
    report,rows=cooperative_demo(config)
    assert report['failure']=='initial_placement_outside_usable_platform'
    assert all(not r['observed_armed'] for r in rows)


def test_pilot_control_stops_car_and_never_resumes(config):
    def pilot(t,seq,i):
        if seq.phase=='FOLLOW':i['pilot_control']=True
    report,rows=cooperative_demo(config,hook=pilot)
    assert not report['complete']
    assert rows[-1]['status']['phase']=='PILOT_CONTROL'
    assert rows[-1]['status']['car_action']=='stop' and rows[-1]['status']['uav_action']=='manual'


@pytest.mark.parametrize('field',['now_ns','now_source_ns'])
def test_invalid_clock_preserves_first_fault_after_valid_observation(config,field):
    def bad_clock(t,seq,i):
        if t>.04:i[field]=None
    report,rows=cooperative_demo(config,hook=bad_clock)
    assert report['failure']=='invalid_cooperative_clock'
    assert rows[-1]['status']['uav_target_map_m'] is None


def test_console_shared_frame_is_explicit_and_simulated(tmp_path,config,capsys):
    assert save_cooperative_demo(config,tmp_path/'run')==0
    s=Session(tmp_path/'run')
    while not s.eof:s.pump(all_available=True)
    assert s.model.status['phase']=='COMPLETE'
    assert s.model.cooperative_synthetic is True
    assert set(s.model.positions)=={'coop_car','coop_uav','coop_target'}
    assert sum(t.errors for t in s.tails)==0
    row=json.loads((tmp_path/'run/cooperative.jsonl').read_text().splitlines()[0]);row['frame_revision']='other'
    with pytest.raises(ValueError,match='不能拼接'):s.model.ingest('cooperative.jsonl',row)


def test_paired_preparation_plan_is_file_only(tmp_path,capsys):
    dest=tmp_path/'prepared'
    assert entry.main(['--prepare-pair','--output',str(dest)])==0
    p=json.loads(capsys.readouterr().out)
    assert not dest.exists() and not p['ready'] and not p['live_cooperation_connected']
    assert 'wand_4wse_pose2000_20261105/entry.py' in p['car_entry']
    assert len(p['terminal_argvs'])==2
    assert all(c[:2]==['gnome-terminal','--window'] for c in p['terminal_argvs'])
    assert '--flight-console' in p['uav_argv']
    assert '--fusion-observe' not in p['uav_argv']
    assert 'TAKEOFF ' not in json.dumps(p) and not p['automatic_motion']
    with pytest.raises(SystemExit):entry.main(['--prepare-pair','--run','--output',str(dest)])
    assert not dest.exists()


def test_joint_launch_opens_only_existing_preparation_commands(tmp_path,monkeypatch,capsys):
    import shutil
    calls=[]
    monkeypatch.setattr(shutil,'which',lambda _: '/usr/bin/gnome-terminal')
    monkeypatch.setattr(subprocess,'Popen',lambda args,**kwargs:calls.append(args))
    assert entry.main(['--prepare-pair','--run','--site-ready','--output',str(tmp_path/'prep')])==0
    p=json.loads(capsys.readouterr().out)
    assert p['launch_requested'] and not p['ready'] and not p['automatic_motion']
    assert calls==p['terminal_argvs']


def landing_tuning_fixture(**settings):
    plan=json.loads((ROOT/'plan_b/follow/air_ground_side_follow.current.json').read_text())
    # Exercise the optional precision tuning in isolation from the current
    # normal-landing task, which intentionally no longer waits for 5 cm.
    plan.update(ground_descent_continuous=False,landing_xy_tolerance_m=.05,
                landing_settling_speed_mps=.04,landing_settle_seconds=1.5)
    plan.update(settings)
    seq=AirGroundSequence(plan)
    def pose(source,p,t):return TargetPose(source,'map',t,t,tuple(p),(0.,0.,0.,1.),True,False)
    car=pose('car01',(0.,.7,.1),10**9);uav=pose('MircoAir',(0.,0.,.04),10**9)
    seq.resolve_start(car,uav,car,uav);seq._capture(car,uav)
    seq.phase='ALIGN';seq.target=(0.,0.,.54);seq.landing_car_anchor=car
    seq.generator.set_mode('HOVER',hold_xy=(0.,0.))
    def tick(i,p=(.07,0.,.54),v=(0.,0.,0.),**changes):
        t=10**9+i*20_000_000
        args=dict(now_ns=t,now_source_ns=t,frame_revision=plan['frame_revision'],
            car=pose('car01',car.position,t),uav=pose('MircoAir',p,t),
            car_velocity=(0.,0.,0.),uav_velocity=v,car_ready=True,uav_ready=True,
            car_stopped=True,armed=True,landed=False)
        args.update(changes)
        return seq.step(**args)
    return seq,plan,tick


def test_landing_bias_is_bounded_and_does_not_integrate_duplicate_pose():
    seq,plan,tick=landing_tuning_fixture()
    previous=(0.,0.)
    for i in range(2000):
        s=tick(i)
        v=s['uav_velocity_map_mps'][:2]
        assert not s['fault'] and s['phase']=='ALIGN'
        assert s['uav_target_map_m']==(0.,0.,.54)
        assert math.hypot(*seq.landing_integral)<=.5*plan['landing_horizontal_speed_mps']+1e-9
        assert math.hypot(*v)<=plan['landing_horizontal_speed_mps']+1e-9
        assert math.dist(v,previous)<=seq.limits.acceleration_mps2*.02+1e-9
        previous=v
    assert seq.landing_integral[0]==pytest.approx(-.04)
    bias=seq.landing_integral
    cached=seq.seen['MircoAir']
    for i in range(2000,2020):tick(i,uav=cached)
    assert seq.landing_integral==bias
    # Opposite residual must unwind, not retain the previous bias forever.
    for i in range(2020,2620):tick(i,p=(-.07,0.,.54))
    assert seq.landing_integral[0]>0


@pytest.mark.parametrize('exit_kind',['pilot','stale','car_moves'])
def test_landing_bias_cleared_on_departure_or_loss(exit_kind):
    seq,plan,tick=landing_tuning_fixture()
    for i in range(400):tick(i)
    assert seq.landing_integral[0]<-.01
    changes={}
    if exit_kind=='pilot':changes['pilot_control']=True
    elif exit_kind=='stale':changes['uav']=replace(seq.seen['MircoAir'],stamp_ns=1,received_ns=1)
    else:changes['car']=replace(seq.seen['car01'],position=(.2,.7,.1),stamp_ns=9_000_000_000,received_ns=9_000_000_000)
    s=tick(400,**changes)
    assert s['phase']=={'pilot':'PILOT_CONTROL','stale':'HOLD','car_moves':'WAIT_CAR_STOP'}[exit_kind]
    assert seq.landing_integral==(0.,0.)
    assert s['uav_velocity_map_mps']==(0.,0.,0.)


def test_landing_damping_brakes_crossing_and_requires_sustained_settle():
    seq,plan,tick=landing_tuning_fixture()
    # Inside the XY tolerance but moving through the point: brake rather
    # than increasing correction toward the soon-to-be-overshot target.
    for i in range(60):s=tick(i,p=(.01,0.,.54),v=(-.07,0.,0.))
    assert s['phase']=='ALIGN'
    assert s['uav_velocity_map_mps'][0]>0
    for i in range(60,100):s=tick(i,p=(.01,0.,.54))
    assert s['phase']=='ALIGN'  # 0.8 s is insufficient for precision settle.
    s=tick(100,p=(.06,0.,.54))  # An excursion resets only settlement, not flight.
    assert s['phase']=='ALIGN' and not s['fault']
    for i in range(101,176):s=tick(i,p=(.01,0.,.54))
    assert s['phase']=='ALIGN'
    s=tick(176,p=(.01,0.,.54))
    assert s['phase']=='DESCEND' and not s['fault']


def test_landing_integral_does_not_wind_up_during_saturated_return():
    seq,plan,tick=landing_tuning_fixture()
    for i in range(1000):s=tick(i,p=(.8,0.,.54))
    assert seq.landing_integral==(0.,0.)
    assert math.hypot(*s['uav_velocity_map_mps'][:2])==pytest.approx(.08)
    # Moving reference: don't turn return-trajectory lag into landing bias.
    seq.generator.xy=(.3,0.)
    for i in range(1000,1005):s=tick(i,p=(.31,0.,.54))
    assert seq.generator.xy[0]>.2
    assert seq.landing_integral==(0.,0.)


def test_landing_integral_reduces_bias_in_synthetic_velocity_response():
    # Deliberately simple disturbance model, NOT PX4/propeller/ground-effect
    # validation: prove that the new state can reject a constant bias which
    # proportional feedback alone cannot. Actual 5 cm accuracy is unproven.
    residuals=[]
    for gain in (0.,.05):
        seq,plan,tick=landing_tuning_fixture(landing_integral_gain_s_inv2=gain)
        x=.15;vx=0.
        for i in range(5000):
            s=tick(i,p=(x,0.,.65),v=(vx,0.,0.)) # Keep ALIGN to isolate XY response.
            cmd=-.95*x+s['uav_velocity_map_mps'][0]+.08
            vx+=(cmd-vx)*.02/.4
            x+=vx*.02
            assert not s['fault'] and abs(x)<.3
        residuals.append(abs(x))
    assert residuals[0]>.05 and residuals[1]<.04


@pytest.mark.parametrize('settings',[
    {'landing_integral_gain_s_inv2':float('nan')},
    {'landing_integral_gain_s_inv2':-.1},
    {'landing_velocity_damping':float('inf')},
    {'landing_velocity_damping':True},
    {'landing_settle_seconds':0},
    {'landing_settling_speed_mps':-.1},
])
def test_invalid_precision_tuning_rejected(settings):
    with pytest.raises(ValueError):landing_tuning_fixture(**settings)


@pytest.mark.parametrize('body_yaw',[-2.8,0.,1.6])
@pytest.mark.parametrize('fixed',[False,True])
def test_route_direction_uses_frozen_axes_but_live_origin_and_body_heading(body_yaw,fixed):
    plan=json.loads((ROOT/'plan_b/follow/air_ground_side_follow.current.json').read_text())
    if not fixed:plan.pop('route_heading_map_rad')
    # Exercise forward AND left offsets, not just a one-dimensional route.
    plan['route_local_forward_left_m']=[[.3,0.],[.3,.2],[0.,0.]]
    yaw=plan.get('route_heading_map_rad',body_yaw)
    def pose(source,p):return TargetPose(source,'map',10**9,10**9,p,
        (0.,0.,math.sin(body_yaw/2),math.cos(body_yaw/2)),True,False)
    uav=pose('MircoAir',(.1,.2,.08))
    car=pose('car01',(.1-.7*math.sin(body_yaw),.2+.7*math.cos(body_yaw),.12))
    seq=AirGroundSequence(plan);s=seq.resolve_start(car,uav,car,uav)
    origin=s['takeoff_car_world_xyyaw']
    assert origin[2]==pytest.approx(body_yaw)
    assert s['uav_ground_position_map_m']==uav.position
    assert s['uav_heading_map_rad']==pytest.approx(body_yaw)
    assert s['route_heading_map_rad']==pytest.approx(yaw)
    for local,goal in zip(plan['route_local_forward_left_m'],s['route_world_xyyaw']):
        x,y=local
        assert goal[:2]==pytest.approx((origin[0]+x*math.cos(yaw)-y*math.sin(yaw),
                                       origin[1]+x*math.sin(yaw)+y*math.cos(yaw)))
        assert goal[2]==pytest.approx(body_yaw)
    assert s['route_world_xyyaw'][-1]==origin


@pytest.mark.parametrize('bad',[float('nan'),float('inf'),True,'1.72'])
def test_bad_fixed_route_direction_rejected_offline(bad):
    plan=json.loads((ROOT/'plan_b/follow/air_ground_side_follow.current.json').read_text())
    plan['route_heading_map_rad']=bad
    with pytest.raises(ValueError,match='fixed_route_heading'):
        AirGroundSequence(plan)


def test_circle_geometry_and_live_heading_binding():
    from follow_target_generator import circle_local_route
    plan=json.loads((ROOT/'plan_b/follow/air_ground_circle.candidate.json').read_text())
    seq=AirGroundSequence(plan)
    points=circle_local_route(1.5,48)
    assert points[11]==pytest.approx((.75,.75))
    assert points[23]==pytest.approx((0.,1.5))
    assert points[35]==pytest.approx((-.75,.75))
    assert points[-1]==(0.,0.)
    for point in points:assert math.dist(point,(0.,.75))==pytest.approx(.75)
    for mutation in ('heading','route'):
        invalid=copy.deepcopy(plan)
        if mutation=='heading':invalid['route_heading_map_rad']=0.
        else:invalid['route_local_forward_left_m'][0]=[3,0]
        with pytest.raises(ValueError,match='circle_requires'):
            AirGroundSequence(invalid)
