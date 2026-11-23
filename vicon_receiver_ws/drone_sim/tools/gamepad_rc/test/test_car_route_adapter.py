"""Delta tests: real car core and verifier, synthetic measurements, zero I/O."""
from dataclasses import replace, asdict
import json
import math
from pathlib import Path
import socket
import subprocess
import sys

import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from car_route_adapter import CarRouteAdapter, RouteIntent, Pose, ZERO
from follow_target_generator import TargetPose
from replay_follow_targets import cooperative_demo
from replay_follow_targets import car_motion_demo
from mecanum_wand_4wse_control.safety_core import Lease
from mecanum_wand_4wse_hardware.commissioning import TransportPolicy


@pytest.fixture(autouse=True)
def no_io(monkeypatch):
    def deny(*a,**kw): raise AssertionError('offline car route tried network/process I/O')
    monkeypatch.setattr(socket,'socket',deny)
    monkeypatch.setattr(subprocess,'Popen',deny)


@pytest.fixture
def config():
    return json.loads((ROOT/'plan_b/follow/air_ground_example.json').read_text())


class Harness:
    def __init__(self,config,yaw=0.):
        self.a=CarRouteAdapter(config,channel_key='37'*32)
        self.p=TransportPolicy('37'*32,self.a.config.stage)
        self.i=0;self.now=10.;self.ros=1_800_000_010.
        self.car=(0.,0.,self.a.config.ground_z_m)
        self.yaw=yaw;self.last=None;self.last_intent=None;self.out=None

    def tick(self,*,action='route',index=0,change=None,repeat_intent=False,graph_ok=True):
        self.i+=1;self.now=10.+self.i*.02;self.ros=1_800_000_000.+self.now
        c=ZERO if self.out is None else self.out
        active=bool(self.last and self.last['state']=='ACTIVE')
        held=(self.a.supervisor.state in ('ARMED','ACTIVE') and bool(self.a.supervisor.epoch)
              and self.a.gate.primed_epoch==self.a.supervisor.epoch and self.p.epoch==self.a.supervisor.epoch)
        intent=RouteIntent(self.i,self.now,self.ros,self.a.plan.config['frame_id'],
            self.a.plan.config['frame_revision'],action,index)
        if repeat_intent:intent=self.last_intent
        self.last_intent=intent
        kwargs=dict(steady=self.now,ros=self.ros,
            car=TargetPose('car01',self.a.plan.config['frame_id'],round(self.ros*1e9),round(self.now*1e9),self.car,
                (0.,0.,math.sin(self.yaw/2),math.cos(self.yaw/2)),True,False),
            frame_revision=self.a.plan.config['frame_revision'],
            feedback=dict(sequence=self.i,issued_steady=self.now,issued_ros=self.ros,
                output_zero=c==ZERO,enabled=active,speed=math.hypot(c.vx,c.vy),yaw_rate=abs(c.wz),fault_bits=0),
            lease=Lease('test_operator',self.i,self.now,self.ros,held),intent=intent,graph_ok=graph_ok)
        if change:change(kwargs)
        self.last=self.a.step(**kwargs)
        assert self.p.receive(self.last['signed_authority'],self.now,self.ros)
        self.out=self.p.command if self.p.active(self.now) else ZERO
        return self.last

    def start(self):
        for _ in range(70):self.tick()
        assert self.last['ready'],self.last
        assert self.a.prepare(self.now,self.ros)[0]
        self.tick();self.tick()  # neutral gate/transport entry, then ACTIVE zero
        assert self.last['state']=='ACTIVE' and self.out==ZERO,self.last


def test_complete_with_real_control_chain_and_transport(config):
    report,rows=cooperative_demo(config,car_control=True)
    assert report['complete'],report
    assert report['actual_sent_to_car']==report['actual_sent_to_fc']==report['devices_opened']==0
    assert not report['car_runtime_wired'] and not report['live_adapters_connected']
    assert 'TransportPolicy' in report['car_control_chain']
    moving=[r for r in rows if any(r['car_control']['command_body_mps_rps'])]
    assert moving and all(r['status']['phase']=='FOLLOW' for r in moving)
    assert all(r['car_control']['transport_accepted'] for r in rows)
    assert all(r['observed_car_stopped'] for r in rows if r['status']['phase'] in ('ALIGN','DESCEND','LAND','COMPLETE'))
    assert report['final_relative_yaw_rad']==pytest.approx(.2,abs=.01)
    for a,b in zip(rows,rows[1:]):
        u,v=a['car_control'],b['car_control'];cmd=v['command_body_mps_rps']
        assert math.hypot(*cmd[:2])<=config['car_speed_mps']+1e-9
        if u['state']==v['state']=='ACTIVE':
            assert math.dist(u['command_body_mps_rps'][:2],cmd[:2])<=config['car_acceleration_mps2']*.02+1e-7


@pytest.mark.parametrize('initial',[(.4,-.3,.15,1.1),(-.3,.3,.15,-2.6)])
def test_start_changes_do_not_reanchor_world_route(config,initial):
    report,rows=cooperative_demo(config,car_control=True,initial_car=initial)
    assert report['complete'],report
    assert {tuple(r['status']['car_goal_map_xyyaw']) for r in rows if r['status']['car_action']=='route'}==set(map(tuple,config['route_world_xyyaw']))
    assert report['final_relative_yaw_rad']==pytest.approx(.2,abs=.01)


def test_world_x_becomes_body_right_at_ninety_degree_heading(config):
    config['route_world_xyyaw'][0]=[.4,0.,math.pi/2]
    h=Harness(config,yaw=math.pi/2);h.start();h.tick()
    assert abs(h.out.vx)<1e-12 and h.out.vy<0 and abs(h.out.wz)<1e-12
    assert 'wand' not in h.a.supervisor.inputs.poses
    assert 'wand' not in h.a.gate.inputs.poses


def test_stop_revokes_signed_authority_and_requires_measured_disabled_stillness(config):
    config['route_world_xyyaw'][0]=[.4,0.,0.]
    h=Harness(config);h.start();h.tick()
    assert h.out!=ZERO
    out=h.tick(action='stop')
    assert h.out==ZERO and not out['permit'] and not out['stop_confirmed']
    for _ in range(40):out=h.tick(action='stop')
    assert out['state']=='PAUSED' and out['stop_confirmed']
    assert not h.a.prepare(h.now,h.ros)[0]


@pytest.mark.parametrize('fault', ['stale_pose','stale_feedback','lease_release','wrong_frame','wrong_source','graph_conflict','invalid_intent'])
def test_input_failure_revokes_output_and_preserves_first_cause(config,fault):
    config['route_world_xyyaw'][0]=[.4,0.,0.]
    h=Harness(config);h.start();h.tick()
    def change(k):
        if fault=='stale_pose':k['car']=replace(k['car'],stamp_ns=k['car'].stamp_ns-2_100_000_000)
        elif fault=='stale_feedback':k['feedback']['issued_steady']-=.6;k['feedback']['issued_ros']-=.6
        elif fault=='lease_release':k['lease']=replace(k['lease'],held=False)
        elif fault=='wrong_frame':k['frame_revision']='other'
        elif fault=='wrong_source':k['car']=replace(k['car'],source='Sign')
        elif fault=='graph_conflict':k['graph_ok']=False
        else:k['intent']=None
    out=h.tick(change=change)
    assert h.out==ZERO and not out['permit'],out
    reason=out['fault'] or out['reason']
    for _ in range(5):out=h.tick(action='stop')
    assert reason in (out['fault'],out['reason'])


def test_replayed_intent_never_refreshes_deadline(config):
    config['route_world_xyyaw'][0]=[.4,0.,0.]
    h=Harness(config);h.start();h.tick()
    old=h.a.supervisor.inputs.intent
    for _ in range(9):out=h.tick(repeat_intent=True)
    assert out['fault']=='ROUTE_INTENT_TIMEOUT'
    assert h.a.supervisor.inputs.intent is old and h.out==ZERO


def test_gate_rejects_goal_not_from_selected_route(config):
    h=Harness(config);h.start();h.tick()
    frame=replace(h.a.gate.frame,sequence=h.a.gate.frame.sequence+1,goal=Pose(1,1,h.car[2],0))
    assert h.a.gate.receive(frame,h.now,h.ros)
    assert h.a.gate.tick(h.now,h.ros)==ZERO
    assert h.a.gate.fault=='FINAL_REFERENCE_STEP_LIMIT'


def test_wrong_route_index_or_goal_does_not_select_other_route(config):
    h=Harness(config);h.start()
    out=h.tick(index=3)
    assert out['fault']=='ROUTE_INDEX_SKIPPED_OR_REVERSED' and h.out==ZERO
    status=h.a.plan.status();status.update(car_action='route',route_index=0,car_goal_map_xyyaw=[99,99,0])
    with pytest.raises(ValueError,match='differs'):
        RouteIntent.from_sequence(status,1,h.now,h.ros,h.a.plan)


def test_elevated_body_is_not_accepted(config):
    h=Harness(config);h.start();h.tick()
    out=h.tick(change=lambda k:k.update(car=replace(k['car'],position=(0,0,h.car[2]+.2))))
    assert 'CAR_LIFT_OR_TILT' in out['fault'] and h.out==ZERO


def test_no_implicit_prepare_or_motion(config):
    h=Harness(config)
    for _ in range(100):out=h.tick()
    assert out['ready'] and h.out==ZERO and not out['permit'] and not h.a.used


def test_car_controller_failure_reaches_joint_hold(config,monkeypatch):
    original=CarRouteAdapter.step
    def break_graph(self,**kw):
        if self.supervisor.state=='ACTIVE':kw['graph_ok']=False
        return original(self,**kw)
    monkeypatch.setattr(CarRouteAdapter,'step',break_graph)
    report,rows=cooperative_demo(config,car_control=True)
    assert not report['complete'] and report['failure']=='COMMAND_SOURCE_CONFLICT'
    assert rows[-1]['status']['phase']=='HOLD'
    assert rows[-1]['status']['car_action']=='stop'
    assert rows[-1]['car_control']['command_body_mps_rps']==(0.,0.,0.)
    assert rows[-1]['status']['uav_action']=='hold'


@pytest.fixture
def car_plan():
    return json.loads((ROOT/'plan_b/follow/car_motion.example.json').read_text())


@pytest.mark.parametrize('initial',[(-.25,-.15,0.),(.25,-.2,1.1),(0.,0.,0.)])
def test_car_loaded_approach_stop_continue_route_and_wait_landing(car_plan,initial):
    report,rows=car_motion_demo(car_plan,initial_car=initial)
    assert report['complete'] and not report['failure'],report
    assert report['all_transport_accepted']
    assert [r['phase'] for r in report['phases']]==[
        'WAIT_READY','WAIT_START','TO_TAKEOFF','WAIT_TAKEOFF','ROUTE','WAIT_LANDING','COMPLETE']
    assert all(not any(r['applied_body_mps_rps']) for r in rows if r['status']['phase']
               not in ('TO_TAKEOFF','ROUTE'))
    goals={tuple(r['status']['car_goal_map_xyyaw']) for r in rows if r['status']['car_goal_map_xyyaw']}
    assert goals=={tuple(car_plan['takeoff_car_world_xyyaw']),*map(tuple,car_plan['route_world_xyyaw'])}
    assert math.dist(report['final_car_xyyaw'][:2],car_plan['route_world_xyyaw'][-1][:2])<.025
    for a,b in zip(rows,rows[1:]):
        cmd=b['applied_body_mps_rps'];assert math.hypot(*cmd[:2])<=.05+1e-9
        if a['status']['car_control']['state']==b['status']['car_control']['state']=='ACTIVE':
            assert math.dist(a['applied_body_mps_rps'][:2],cmd[:2])<=.1*.02+1e-7
    departure=next(r for r in rows if r['status']['phase']=='ROUTE')
    assert departure['observed_flight']['clearance_confirmed']
    assert departure['measured_feedback']['output_zero'] and not departure['measured_feedback']['enabled']
    assert rows[-1]['status']['car_stopped']
    assert rows[-1]['observed_flight']['landed'] and not rows[-1]['observed_flight']['armed']


def test_car_waits_for_explicit_start_without_wand_or_drone_controller(car_plan):
    report,rows=car_motion_demo(car_plan,duration_s=5.,auto_start=False)
    assert not report['complete'] and report['phase']=='WAIT_START'
    assert all(not any(r['applied_body_mps_rps']) for r in rows)
    assert report['devices_opened']==report['actual_sent_to_car']==report['actual_sent_to_fc']==0


@pytest.mark.parametrize('auto_start',[False,True])
def test_used_healthy_allowance_can_prepare_but_cannot_move_before_bridge_ready(car_plan,auto_start):
    def used_allowance(t,task,data):
        if t<3.:
            data['transport'].update(ready=False,healthy_budget_used=True,
                                     native_disabled_zero=True)
    report,rows=car_motion_demo(car_plan,standalone=True,hook=used_allowance,
        auto_start=auto_start,duration_s=8.)
    assert not report['failure'],report
    assert 'WAIT_START' in [p['phase'] for p in report['phases']]
    assert all(not any(r['applied_body_mps_rps']) for r in rows if r['at_s']<3.)
    if auto_start:
        assert any(any(r['applied_body_mps_rps']) for r in rows if r['at_s']>=3.)
    else:
        assert report['phase']=='WAIT_START'
        assert all(not any(r['applied_body_mps_rps']) for r in rows)


def test_standalone_current_rectangle_has_no_fabricated_uav_handoffs(tmp_path):
    plan=json.loads((ROOT/'plan_b/follow/air_ground_loaded.current.json').read_text())
    plan['schema']='car_cooperative_motion_v1'
    initial=tuple(plan['takeoff_car_world_xyyaw'])
    report,rows=car_motion_demo(plan,initial_car=initial,standalone=True)
    assert report['complete'] and not report['failure'],report
    assert all(row['observed_flight'] is None for row in rows)
    assert [p['phase'] for p in report['phases']]==[
        'WAIT_READY','WAIT_START','TO_TAKEOFF','WAIT_ROUTE','ROUTE','WAIT_FINISH','COMPLETE']
    assert report['all_transport_accepted']
    assert rows[-1]['status']['car_stopped'] and not any(rows[-1]['applied_body_mps_rps'])
    assert math.dist(report['final_car_xyyaw'][:2],initial[:2])<=plan['car_arrival_m']
    assert report['devices_opened']==report['actual_sent_to_car']==report['actual_sent_to_fc']==0
    (tmp_path/'standalone_result.json').write_text(json.dumps(report,indent=2))


def test_standalone_operator_stop_does_not_resume_when_data_continue(car_plan):
    def stop(t,task,data):
        if t>=5.:data['cancel']=True
    report,rows=car_motion_demo(car_plan,standalone=True,hook=stop)
    assert report['phase']=='ABORTED' and report['failure']=='OPERATOR_STOP'
    assert rows[-1]['status']['car_stopped']
    assert all(not any(r['applied_body_mps_rps']) for r in rows if r['at_s']>=5.)
    assert all(r['observed_flight'] is None for r in rows)


def test_standalone_still_waits_for_start_and_rejects_uav_facts(car_plan):
    report,rows=car_motion_demo(car_plan,standalone=True,auto_start=False,duration_s=3.)
    assert report['phase']=='WAIT_START' and not report['complete']
    assert all(not any(r['applied_body_mps_rps']) for r in rows)
    from car_route_adapter import FlightProgress
    def wrong(t,task,data):
        data['flight']=FlightProgress(task.task_id,0,data['steady'],data['ros'],False,True,False)
    report,rows=car_motion_demo(car_plan,standalone=True,hook=wrong,duration_s=3.)
    assert 'does not accept UAV handoffs' in report['failure']
    assert all(not any(r['applied_body_mps_rps']) for r in rows)


def test_recorded_ground_run_uses_plane_without_rewriting_world_poses(tmp_path):
    from car_route_adapter import RouteInputs
    run=ROOT.parents[1]/'preparation_runs/car_rectangle_live_20261112_144911_retry01'
    plan=json.loads((ROOT/'plan_b/follow/air_ground_loaded.current.json').read_text())
    plan['schema']='car_cooperative_motion_v1'
    adapter=CarRouteAdapter(plan,channel_key='37'*32)
    old=RouteInputs(adapter.config,adapter.plan);new=RouteInputs(adapter.config,adapter.plan)
    rows=[json.loads(l) for l in (run/'car_motion.jsonl').read_text().splitlines()]
    rows=rows[next(i for i,r in enumerate(rows) if r['status']['phase']=='TO_TAKEOFF'):]
    rejected_old=0;residuals=[];previous_stamp=None;duplicate_log_rows=0
    for row in rows:
        # State changes may log the same source frame twice. Replay each
        # physical sample once rather than treating a duplicate as a new pose.
        if row['car_stamp_ns']==previous_stamp:
            duplicate_log_rows+=1;continue
        previous_stamp=row['car_stamp_ns']
        x,y,z=row['car_position_map_m'];yaw=row['car_yaw_rad']
        source=row['car_stamp_ns']*1e-9;steady=row['car_received_ns']*1e-9
        before=tuple(row['car_position_map_m'])
        residual=z-adapter.ground_z_at(x,y);residuals.append(residual)
        # Original source and receipt times; no roll/pitch in the old log,
        # so this explicitly isolates only the height-rejection branch.
        if not old.observe_pose('car',Pose(x,y,z+adapter.control_z_offset_m,yaw),source,steady,source):rejected_old+=1
        assert new.observe_pose('car',Pose(x,y,residual+adapter.config.ground_z_m,yaw),source,steady,source)
        assert tuple(row['car_position_map_m'])==before
    assert rejected_old>0 and max(map(abs,residuals))<.003
    (tmp_path/'recorded_height_replay.json').write_text(json.dumps(dict(
        scope='actual recorded XYZ/yaw and timestamps; height branch only, roll/pitch unavailable',
        records=len(rows),duplicate_log_rows=duplicate_log_rows,old_rejected=rejected_old,
        new_rejected=0,max_residual_m=max(map(abs,residuals)),
        controls_sent=0,physical_validated=False),indent=2))


def test_lift_remains_motion_fault_but_static_stop_can_be_confirmed(config):
    config['live_inputs']={'car_ground_z_map_m':.12,'car_ground_plane_map_abc':[.03,-.01,.12]}
    config['route_world_xyyaw'][0]=[.4,0.,0.]
    h=Harness(config);h.car=(0.,0.,.12);h.start();h.tick()
    h.car=(0.,0.,.32);out=h.tick()
    assert out['fault']=='CAR_LIFT_OR_TILT' and h.out==ZERO
    for _ in range(40):out=h.tick(action='stop')
    assert out['stop_confirmed'] and out['state']=='FAULT'
    assert out['stop_observation']=='stationary_geometry_and_wheels_after_motion_rejection'
    assert out['geometry']['height_above_reference_m']==pytest.approx(.2)
    assert not out['ready'] and not out['permit'] and not h.a.prepare(h.now,h.ros)[0]


@pytest.mark.parametrize('bad',['vertical_motion','tilt_motion','stale_pose','enabled_feedback','nonzero_wheels'])
def test_geometry_rejection_stop_does_not_accept_motion_or_bad_evidence(config,bad):
    config['route_world_xyyaw'][0]=[.4,0.,0.]
    h=Harness(config);h.start();h.tick();h.car=(0.,0.,h.car[2]+.2);h.tick()
    first_stamp=round(h.ros*1e9);first_received=round(h.now*1e9)
    def change(k):
        if bad=='vertical_motion':k['car']=replace(k['car'],position=(0.,0.,h.car[2]+.03*math.sin(h.i*.3)))
        elif bad=='tilt_motion':
            roll=.4+.03*math.sin(h.i*.3)
            k['car']=replace(k['car'],quaternion_xyzw=(math.sin(roll/2),0.,0.,math.cos(roll/2)))
        elif bad=='stale_pose':k['car']=replace(k['car'],stamp_ns=first_stamp,received_ns=first_received)
        elif bad=='enabled_feedback':k['feedback']['enabled']=True
        else:k['feedback'].update(output_zero=False,speed=.03)
    for _ in range(120):
        out=h.tick(action='stop',change=change)
        assert not out['stop_confirmed'] and not out['permit'] and h.out==ZERO
    assert out['fault']=='CAR_LIFT_OR_TILT'


@pytest.mark.parametrize('yaw,direction',[(0.,'forward'),(1.1,'forward'),(-.4,'left')])
def test_segment_servo_reduces_drift_under_biased_drive_without_higher_limits(car_plan,monkeypatch,tmp_path,yaw,direction):
    from car_route_adapter import RouteSession,Velocity,wrap_angle,coupled_shape
    car_plan.update(car_speed_mps=.2,car_acceleration_mps2=.1)
    theta=yaw+(math.pi/2 if direction=='left' else 0.)
    goal=(3*math.cos(theta),3*math.sin(theta),yaw)
    car_plan['takeoff_car_world_xyyaw']=list(goal)
    car_plan['route_world_xyyaw']=[list(goal)]
    original=RouteSession.tick
    def former_point_servo(self,now):
        dt=now-self.last_tick;self.last_tick=now
        if self.state!='ACTIVE' or dt<=0:return self.command
        p=self.inputs.intent
        if not p or p.action!='route':self.pause(True);return ZERO
        x,y,heading=self.inputs.plan.route[p.route_index];self.goal=Pose(x,y,self.goal.z,heading)
        dx,dy=x-self.car.x,y-self.car.y;c,s=math.cos(self.car.yaw),math.sin(self.car.yaw)
        desired=Velocity(self.config.kp_position*(c*dx+s*dy),self.config.kp_position*(-s*dx+c*dy),2*wrap_angle(heading-self.car.yaw))
        self.command=coupled_shape(desired,self.command,dt,self.limits);return self.command
    def run(tick):
        monkeypatch.setattr(RouteSession,'tick',tick)
        h=Harness(car_plan,yaw=yaw);h.start();errors=[];angles=[];last=ZERO
        for _ in range(2000):
            out=h.tick();cmd=h.out
            assert not out['fault'],out
            assert math.hypot(cmd.vx,cmd.vy)<=.2+1e-9
            assert math.hypot(cmd.vx-last.vx,cmd.vy-last.vy)<=.1*.02+1e-7
            assert abs(cmd.wz-last.wz)<=.4*.02+1e-7
            last=cmd
            # Synthetic 8% cross-coupling and small drive yaw bias. Same
            # disturbance for old/new controllers, not a real hardware fit.
            bx,by=cmd.vx+.08*cmd.vy,cmd.vy-.08*cmd.vx
            c,s=math.cos(h.yaw),math.sin(h.yaw)
            h.car=(h.car[0]+(c*bx-s*by)*.02,h.car[1]+(s*bx+c*by)*.02,h.car[2])
            h.yaw=wrap_angle(h.yaw+(cmd.wz+.02*math.hypot(bx,by))*.02)
            errors.append(abs(-math.sin(theta)*h.car[0]+math.cos(theta)*h.car[1]))
            angles.append(abs(wrap_angle(h.yaw-yaw)))
            if math.dist(h.car[:2],goal[:2])<.015 and math.hypot(cmd.vx,cmd.vy)<.02:break
        assert math.dist(h.car[:2],goal[:2])<.015
        return dict(max_cross_m=max(errors),max_yaw_rad=max(angles),elapsed_s=len(errors)*.02)
    before=run(former_point_servo);after=run(original)
    assert after['max_cross_m']<.04 and after['max_cross_m']<before['max_cross_m']*.5,(before,after)
    assert after['max_yaw_rad']<before['max_yaw_rad'],(before,after)
    (tmp_path/'biased_drive_comparison.json').write_text(json.dumps(dict(before=before,after=after,
        synthetic=True,drive_cross_coupling=.08,physical_validated=False),indent=2))


def test_car_replay_uses_configured_world_height_for_three_metre_return_route(car_plan):
    from follow_target_generator import _rotate_xy
    start=(.66,-1.04,1.73)
    car_plan.update(car_speed_mps=.2,live_inputs={'car_ground_z_map_m':-.28},
                    takeoff_car_world_xyyaw=list(start),route_world_xyyaw=[])
    for local in ((3.,0.),(3.,3.),(0.,3.),(0.,0.)):
        dx,dy=_rotate_xy(local,start[2])
        car_plan['route_world_xyyaw'].append([start[0]+dx,start[1]+dy,start[2]])
    report,rows=car_motion_demo(car_plan,initial_car=start)
    assert report['complete'] and report['failure'] is None,report
    assert report['all_transport_accepted']
    assert all(row['car_position_map_m'][2]==pytest.approx(-.28) for row in rows)
    goals={tuple(row['status']['car_goal_map_xyyaw']) for row in rows
           if row['status']['car_goal_map_xyyaw'] is not None}
    assert goals=={start,*map(tuple,car_plan['route_world_xyyaw'])}
    assert math.dist(report['final_car_xyyaw'][:2],start[:2])<car_plan['car_arrival_m']
    assert report['final_car_xyyaw'][2]==pytest.approx(start[2])
    assert max(math.hypot(*row['applied_body_mps_rps'][:2]) for row in rows)<=.2+1e-9
    assert rows[-1]['status']['car_stopped']
    assert report['devices_opened']==report['actual_sent_to_car']==report['actual_sent_to_fc']==0


@pytest.mark.parametrize('block',['not_clear','stale_clear','landed_still_armed'])
def test_car_handoffs_cannot_be_replaced_by_timer_or_incomplete_flight_state(car_plan,block):
    def hook(t,task,i):
        if task.phase=='WAIT_TAKEOFF' and block in ('not_clear','stale_clear'):
            i['flight']=replace(i['flight'],clearance_confirmed=False) if block=='not_clear' else replace(
                i['flight'],issued_steady=i['steady']-2,issued_ros=i['ros']-2)
        if task.phase=='WAIT_LANDING' and block=='landed_still_armed':
            i['flight']=replace(i['flight'],armed=True,landed=True)
    report,rows=car_motion_demo(car_plan,duration_s=50.,hook=hook)
    assert not report['complete'] and not report['failure'],report
    assert report['phase']==('WAIT_LANDING' if block=='landed_still_armed' else 'WAIT_TAKEOFF')
    assert not any(rows[-1]['applied_body_mps_rps'])


@pytest.mark.parametrize('fault',['cancel','car_pose','feedback','flight_loss','flight_task','manual'])
def test_car_motion_faults_stop_without_auto_resume(car_plan,fault):
    seen=[]
    def hook(t,task,i):
        if task.phase=='ROUTE' and not seen and task.last_car['state']=='ACTIVE':
            seen.append((task,i))
            if fault=='cancel':i['cancel']=True
            elif fault=='car_pose':i['car']=replace(i['car'],stamp_ns=i['car'].stamp_ns-3_000_000_000)
            elif fault=='feedback':i['feedback'].update(issued_steady=i['steady']-1.,issued_ros=i['ros']-1.)
            elif fault=='flight_loss':i['flight']=replace(i['flight'],issued_steady=i['steady']-2.,issued_ros=i['ros']-2.)
            elif fault=='flight_task':i['flight']=replace(i['flight'],task_id='another-task')
            else:i['flight']=replace(i['flight'],pilot_control=True)
    report,rows=car_motion_demo(car_plan,hook=hook)
    assert seen and report['phase']=='ABORTED' and report['failure'],report
    assert rows[-1]['status']['car_stopped'] and not any(rows[-1]['applied_body_mps_rps'])
    last=rows[-1]['status'];task=seen[0][0]
    steady=rows[-1]['at_ns']*1e-9
    assert not task.car.prepare(steady,1_800_000_000.+steady,continue_after_index=0)[0]
    assert last['fault']==report['failure']


def test_joint_ground_reference_overrides_only_old_chassis_height(config):
    original=CarRouteAdapter(config,channel_key='37'*32)
    config['live_inputs']['car_ground_z_map_m']=-.28
    h=Harness(config)
    h.car=(*h.car[:2],-.28)
    assert asdict(h.a.config)==asdict(original.config)  # sealed configuration unchanged
    assert h.a.car_ground_z_map_m==-.28
    assert h.a.control_z_offset_m==pytest.approx(original.config.ground_z_m+.28)
    assert h.a.ground_reference_source=='joint_plan.live_inputs.car_ground_z_map_m'
    # Both supervisor and gate now accept the correctly transformed chassis
    # at this new world height. Original lift detection still rejects a lift.
    h.start()
    h.car=(h.car[0],h.car[1],-.28+h.a.config.lift_tolerance_m+.01)
    result=h.tick()
    assert 'CAR_LIFT_OR_TILT' in (result['fault'] or result['reason'])
    assert h.out==ZERO


@pytest.mark.parametrize('value',[None,True,float('nan'),float('inf'),'0.12'])
def test_joint_ground_reference_never_falls_back_for_invalid_configuration(config,value):
    config['live_inputs']['car_ground_z_map_m']=value
    with pytest.raises(ValueError,match='car_ground_z_map_m'):
        CarRouteAdapter(config,channel_key='37'*32)


def test_joint_missing_ground_height_cannot_silently_use_old_profile(config):
    del config['live_inputs']['car_ground_z_map_m']
    with pytest.raises(ValueError,match='car_ground_z_map_m'):
        CarRouteAdapter(config,channel_key='37'*32)


def test_car_only_replay_retains_explicit_legacy_height_scope():
    cfg=json.loads((ROOT/'plan_b/follow/car_motion.example.json').read_text())
    a=CarRouteAdapter(cfg,channel_key='37'*32)
    assert a.ground_reference_source=='legacy_profile_offline_only'


def test_circle_lateral_miss_advances_but_approach_and_final_stop_still_required():
    from car_route_adapter import CarMotionTask
    from follow_target_generator import AirGroundSequence
    c=json.loads((ROOT/'plan_b/follow/air_ground_circle.candidate.json').read_text())
    car=TargetPose('car01','map',1,1,(0.,.51,.1),(0.,0.,0.,1.),True,False)
    uav=replace(car,source='MircoAir',position=(0.,0.,.1))
    resolved=AirGroundSequence(c).resolve_start(car,uav,car,uav)
    task=CarMotionTask(dict(c,schema='car_cooperative_motion_v1'),channel_key='37'*32)
    task.bind_resolved_start(resolved);task.phase='ROUTE';task.index=task.plan.takeoff_index+5
    start=task.plan.route[task.index-1];goal=task.plan.route[task.index]
    d=math.dist(start[:2],goal[:2]);tx,ty=(goal[0]-start[0])/d,(goal[1]-start[1])/d
    missed=replace(car,position=(goal[0]+.03*tx-.10*ty,goal[1]+.03*ty+.10*tx,.1))
    assert math.dist(missed.position[:2],goal[:2])>c['car_arrival_m']
    assert task._arrived(missed,(.25,0,0))
    assert not task._arrived(replace(missed,position=(start[0]-.03*tx-.10*ty,start[1]-.03*ty+.10*tx,.1)),(.25,0,0))
    for phase,index in [('TO_TAKEOFF',task.plan.takeoff_index),('ROUTE',len(task.plan.route)-1)]:
        task.phase=phase;task.index=index;goal=task.plan.route[index]
        at_goal=replace(car,position=(*goal[:2],.1))
        assert not task._arrived(at_goal,(.25,0,0))
        assert task._arrived(at_goal,(0,0,0))
