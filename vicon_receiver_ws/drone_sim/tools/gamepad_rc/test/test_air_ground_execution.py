"""Real owners/encoders/verifier with synthetic plant feedback; no device I/O."""
from copy import deepcopy
from dataclasses import replace
import json
import math
from pathlib import Path
import socket
import subprocess
from types import SimpleNamespace

import pytest

from test_horizontal_motion import native_observation
from test_flight_mission_core import selector, Sink, ack
from flight_mission_core import MissionCore
from mission_wire_transport import MissionWireTransport
from air_ground_execution import AirGroundExecution
from follow_target_generator import TargetPose, _wrap, _rotate_xy
from mecanum_wand_4wse_hardware.commissioning import TransportPolicy
from car_route_adapter import ZERO

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(autouse=True)
def no_io(monkeypatch):
    def deny(*a,**kw):raise AssertionError('offline joint execution attempted hardware/network/process I/O')
    monkeypatch.setattr(socket,'socket',deny)
    monkeypatch.setattr(subprocess,'Popen',deny)


def config():return json.loads((ROOT/'plan_b/follow/air_ground_example.json').read_text())

def loaded_config():return json.loads((ROOT/'plan_b/follow/air_ground_loaded_example.json').read_text())


def ground_config():
    c=loaded_config()
    for name in ('takeoff_car_world_xyyaw','route_world_xyyaw','platform_offset_car_m',
                 'platform_size_m','uav_landing_footprint_m','edge_margin_m'):
        c.pop(name)
    c.update(landing_mode='initial_ground',landing_heading='initial_world',
        start_alignment=dict(mode='stationary_uav',uav_rigid_offset_car_forward_left_m=[0.,-.2]),
        route_local_forward_left_m=[[1.,0.],[0.,0.]],car_speed_mps=.2,
        car_arrival_m=.08,car_yaw_tolerance_rad=.17,landing_xy_tolerance_m=.12)
    c['follow_limits'].update(speed_mps=.4,acceleration_mps2=.2)
    return c


class Plant:
    """Kinematic stand-in, deliberately NOT a PX4/vehicle dynamics simulator."""
    dt=.02
    def __init__(self,initial=(-.1,-.1,.4),*,plan=None,auto_car=True,auto_flight=True):
        self.now=0.;self.steady=10.;self.ros=1_800_000_010.;self.i=0
        self.packets=[];self.out=ZERO;self.cursor=0;self.rows=[];self.change=None
        self.sink=Sink();self.core=MissionCore(self.sink,pilot_source='native_rc',
                                            height=(plan or {}).get('clearance_m',.3))
        self.auto_car,self.auto_flight=auto_car,auto_flight
        self.joint=AirGroundExecution(config() if plan is None else plan,source_snapshot=self.snapshot,car_publish=self.publish,
                                     channel_key='39'*32,origin_monotonic=10.)
        self.policy=TransportPolicy('39'*32,self.joint.car.config.stage)
        self.joint.bind(self.core)
        self.wire=MissionWireTransport();self.wire.attach(self.core)
        self.core.sink=self.uav_send
        self.car=[initial[0],initial[1],self.joint.car.config.ground_z_m];self.cyaw=initial[2]
        dx,dy=_rotate_xy((.03,.02),self.cyaw)
        self.uav=[self.car[0]+dx,self.car[1]+dy,self.car[2]+.15];self.uyaw=self.cyaw+.2
        if self.joint.sequence.ground_start:
            dx,dy=_rotate_xy((.1,-.45),self.cyaw)
            self.uav=[self.car[0]+dx,self.car[1]+dy,self.car[2]+.05]
            self.uyaw=self.cyaw+.1
        self.ground_z=self.uav[2]
        self.cv=self.uv=(0.,0.,0.);self.armed=self.offboard=False;self.land=False
        self.goal=self.uav[:];self.yaw_goal=self.uyaw;self.stop_at=None

    def uav_send(self,msg):
        assert self.wire.candidate_allowed(msg),(self.core.phase,msg,self.joint.describe())
        return self.sink(msg)

    def publish(self,wire):
        accepted=self.policy.receive(wire,self.steady,self.ros)
        self.packets.append(dict(at=self.now,accepted=accepted,packet=json.loads(wire)))
        self.out=self.policy.command if self.policy.active(self.steady) else ZERO

    def pose(self,source,position,yaw):
        return TargetPose(source,'map',round(self.ros*1e9),round(self.steady*1e9),tuple(position),
                          (0.,0.,math.sin(yaw/2),math.cos(yaw/2)),True,False)

    def snapshot(self):
        data=dict(car=self.pose('car01',self.car,self.cyaw),uav=self.pose('MircoAir',self.uav,self.uyaw),
            frame_revision=self.joint.sequence.config['frame_revision'],car_velocity=self.cv,uav_velocity=self.uv,
            feedback=dict(sequence=self.i,issued_steady=self.steady,issued_ros=self.ros,
                output_zero=self.out==ZERO,enabled=self.policy.active(self.steady),
                speed=math.hypot(self.out.vx,self.out.vy),yaw_rate=abs(self.out.wz),fault_bits=0),
            transport=dict(issued_steady=self.steady,issued_ros=self.ros,
                ready=not self.policy.active(self.steady),fresh=True,local_stop=False,native_fault_bits=0,
                fault='',armed_epoch=self.policy.epoch,policy_state=(self.policy.envelope or {}).get('state')),
            graph_ok=True)
        if self.joint.sequence.ground_start:
            # Nonzero lever arms: samples are different from body control points.
            for kind,p,yaw,arm in (('car',self.car,self.cyaw,(.015,0.)),
                                   ('uav',self.uav,self.uyaw,(-.07,.002))):
                dx,dy=_rotate_xy(arm,yaw)
                data[kind+'_rigid']=replace(data[kind],position=(p[0]-dx,p[1]-dy,p[2]))
        if self.change:self.change(data)
        return data

    def tick(self,*,manual=False,clock_missing=False,reset=False,landed=None):
        self.i+=1;self.now=round(self.i*self.dt,8);self.steady=10.+self.now;self.ros=1_800_000_000.+self.steady
        old=self.uav[:];vx,vy=_rotate_xy((self.out.vx,self.out.vy),self.cyaw)
        self.car[0]+=vx*self.dt;self.car[1]+=vy*self.dt;self.cyaw=_wrap(self.cyaw+self.out.wz*self.dt)
        self.cv=(vx,vy,self.out.wz)
        if self.land and self.armed:
            ground=self.ground_z if self.joint.sequence.ground_start else self.car[2]+.15
            self.uav[2]=max(ground,self.uav[2]-.1*self.dt)
            if self.uav[2]<=ground+1e-9:self.armed=False
        elif self.armed:
            self.uav=[a+.3*(b-a) for a,b in zip(self.uav,self.goal)]
            self.uyaw=_wrap(self.uyaw+.3*_wrap(self.yaw_goal-self.uyaw))
        elif self.joint.car_task is not None and not self.land and not self.joint.sequence.ground_start:
            dx,dy=_rotate_xy((.03,.02),self.cyaw)
            self.uav=[self.car[0]+dx,self.car[1]+dy,self.car[2]+.15]
            self.uyaw=_wrap(self.cyaw+.2)
        self.uv=tuple((b-a)/self.dt for a,b in zip(old,self.uav))
        o=native_observation(self.now,position_ned=(self.uav[0],-self.uav[1],-self.uav[2]),
            velocity_ned=(self.uv[0],-self.uv[1],-self.uv[2]),yaw_ned=-self.uyaw,
            armed=self.armed,landed=not self.armed if landed is None else landed,offboard=self.offboard and not manual,
            manual_control_active=manual,reset_counters=(int(reset),0,0))
        self.o=o
        self.joint.update(o,now=self.now,ros_ns=None if clock_missing else round(self.ros*1e9),pilot_control=manual)
        if (self.auto_car and self.joint.car_task is not None and self.joint.car_task.phase=='WAIT_START'
                and not self.joint.fault):self.joint.request_car_start(o)
        if self.core.phase=='PREPARE':self.core.prepare(o,selector())
        if self.auto_flight and self.core.phase=='WAIT_EXPLICIT_START' and self.joint.last_status['phase']=='WAIT_START':
            self.core.start(o,selector(),operator_confirmed=True)
        self.core.cycle(o,selector(),boot_ms=int(self.now*1000),
            vision=(int((100+self.now)*1e6),[1.,-2.,-3.],[1.,0.,0.,0.],0.))
        for msg in self.sink.messages[self.cursor:]:
            if msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED':
                self.goal=[msg.x,-msg.y,-msg.z];self.yaw_goal=-msg.yaw
            elif msg.get_type()=='COMMAND_LONG':
                assert ack(self.core,msg.command,self.now)
                if msg.command==176:self.offboard=True
                elif msg.command==400:self.armed=True
                elif msg.command==21:self.land=True
        self.cursor=len(self.sink.messages)
        self.rows.append(dict(t=self.now,phase=self.core.phase,joint=deepcopy(self.joint.describe()),
            uav=self.uav[:],car=self.car[:],car_yaw=self.cyaw,uav_yaw=self.uyaw,
            command=(self.out.vx,self.out.vy,self.out.wz),armed=o.armed,landed=o.landed))
        return self.rows[-1]

    def until(self,condition,seconds=100.):
        for _ in range(int(seconds/self.dt)):
            self.tick()
            if condition():return
        pytest.fail(str(self.rows[-1]))


@pytest.mark.parametrize('initial',[(-.2,.1,0.),(.3,-.3,1.4),(-.3,.3,-1.2)])
def test_ground_alignment_and_side_follow_use_existing_owners(initial):
    h=Plant(initial,plan=ground_config())
    initial_uav=h.uav[:];initial_yaw=h.uyaw
    h.until(lambda:h.core.phase=='COMPLETE')
    assert h.joint.describe()['complete'] and not h.joint.fault
    frozen=h.joint.sequence.resolved_start
    goal=frozen['takeoff_car_world_xyyaw']
    rawdelta=_rotate_xy(tuple(frozen['uav_rigid_position_map_m'][i]-
                             frozen['car_rigid_goal_map_xy_m'][i] for i in range(2)),-initial_yaw)
    assert rawdelta==pytest.approx((0.,-.2))
    assert goal[2]==pytest.approx(initial_yaw)
    delta=_rotate_xy((frozen['route_world_xyyaw'][0][0]-goal[0],
                      frozen['route_world_xyyaw'][0][1]-goal[1]),-initial_yaw)
    assert delta==pytest.approx((1.,0.))
    assert frozen['route_world_xyyaw'][-1]==goal
    before=[r for r in h.rows if r['joint']['car_task_phase']=='TO_TAKEOFF']
    assert before and all(not r['armed'] and r['uav']==initial_uav for r in before)
    assert any(any(r['command']) and r['armed'] for r in h.rows)
    assert math.dist(h.uav,initial_uav)<.02
    assert abs(_wrap(h.uyaw-initial_yaw))<.02
    assert [m.command for m in h.sink.messages if m.get_type()=='COMMAND_LONG']==[176,400,21]


def test_current_side_follow_from_measured_aligned_placement():
    # Recorded 2026-11-18 final placement, with synthetic movement feedback.
    # This exercises the current plan; it is not airborne hardware evidence.
    plan=json.loads((ROOT/'plan_b/follow/air_ground_side_follow.current.json').read_text())
    h=Plant((.8894071881,-.4289365640,-3.09048),plan=plan)
    h.car[2]=.0974416
    h.uav=[.8153850405,.2367066346,.0416482347]
    h.uyaw=-3.0850026669
    h.ground_z=h.uav[2];h.goal=h.uav[:];h.yaw_goal=h.uyaw
    initial=h.uav[:]
    rigid_delta=_rotate_xy((.8241504496-h.uav[0],.2151496093-h.uav[1]),-h.uyaw)
    def current_references(data):
        # Real MCU retained a healthy prior run across the last alignment:
        # startup ready never becomes true, but disabled zero is current.
        if h.joint.car.supervisor.state not in ('ARMED','ACTIVE'):
            data['transport'].update(ready=False,healthy_budget_used=True,native_disabled_zero=h.out==ZERO)
        # The plant's existing ready/epoch feedback stands in for the bridge
        # completing its independent reset handshake after prepare/ARMED.
        data['car_rigid']=data['car']  # Current virtual reference is car01 origin.
        dx,dy=_rotate_xy(rigid_delta,h.uyaw)
        data['uav_rigid']=replace(data['uav'],position=(h.uav[0]+dx,h.uav[1]+dy,h.uav[2]+.000646))
    h.change=current_references
    h.until(lambda:h.core.phase=='COMPLETE')
    assert not h.joint.fault and h.joint.describe()['complete']
    resolved=h.joint.sequence.resolved_start
    route_delta=tuple(resolved['route_world_xyyaw'][0][i]-resolved['takeoff_car_world_xyyaw'][i] for i in (0,1))
    fixed=plan['route_heading_map_rad']
    assert route_delta==pytest.approx((math.cos(fixed),math.sin(fixed)))
    assert resolved['uav_heading_map_rad']==pytest.approx(-3.0850026669)
    assert resolved['uav_rigid_offset_car_forward_left_m']==pytest.approx((0.,-.51))
    goal=resolved['takeoff_car_world_xyyaw']
    actual_offset=_rotate_xy((.8241504496-goal[0],.2151496093-goal[1]),-resolved['uav_heading_map_rad'])
    assert actual_offset==pytest.approx((0.,-.51),abs=1e-6)
    follow=[r for r in h.rows if r['joint']['phase']=='FOLLOW']
    assert follow and any(any(r['command']) and r['armed'] for r in follow)
    assert max(math.dist(r['car'][:2],resolved['takeoff_car_world_xyyaw'][:2]) for r in follow)>.85
    assert math.dist(h.car[:2],resolved['takeoff_car_world_xyyaw'][:2])<plan['car_arrival_m']
    assert math.dist(h.uav,initial)<.02
    assert all(not any(r['command']) for r in h.rows if r['joint']['phase']=='DESCEND')
    rendezvous=[r for r in h.rows if r['joint']['phase']=='RENDEZVOUS']
    assert rendezvous and all(not any(r['command']) for r in rendezvous)
    assert all(r['uav'][2] >= initial[2]+plan['clearance_m']-.05 for r in rendezvous)
    assert h.joint.sequence.relative_xy == (0.,0.)
    assert math.dist(follow[0]['uav'][:2],follow[0]['car'][:2]) <= plan['follow_arrival_tolerance_m']
    # Return sideways before starting ground descent; never land on the car.
    descent=[r for r in h.rows if r['joint']['phase']=='DESCEND']
    assert descent and all(math.dist(r['uav'][:2],initial[:2]) <= plan['landing_xy_tolerance_m'] for r in descent)
    assert all(b['joint']['target_ned'][2] >= a['joint']['target_ned'][2]-1e-9
               for a,b in zip(descent,descent[1:]))
    assert [m.command for m in h.sink.messages if m.get_type()=='COMMAND_LONG']==[176,400,21]


def test_rectangle_candidate_completes_four_legs_at_configured_speed_offline():
    plan=json.loads((ROOT/'plan_b/follow/air_ground_rectangle.candidate.json').read_text())
    # Recorded 2026-11-20 start, with the REAL candidate bounds (no virtual
    # +/-5 m arena). This remains kinematic software evidence, not a flight.
    yaw=1.6136231551860178
    h=Plant(initial=(.5488163676761966,-.7294039231912179,1.6148),plan=plan)
    h.car[2]=.09838548263360586
    h.uav=[.9962577328252494,-.7053034455571914,.018772953191718504];h.uyaw=yaw
    h.ground_z=h.uav[2]
    a,b,c=plan['live_inputs']['car_ground_plane_map_abc']
    residual=h.car[2]-(a*h.car[0]+b*h.car[1]+c)
    uav_arm=_rotate_xy((.0215991375259169,.0084220999336449),-yaw)
    def route_references(data):
        # Follow the existing sloping floor model in this stand-in, rather
        # than inventing a lift from a constant map-Z over a three metre leg.
        h.car[2]=a*h.car[0]+b*h.car[1]+c+residual
        data['car']=replace(data['car'],position=tuple(h.car))
        data['car_rigid']=data['car']
        dx,dy=_rotate_xy(uav_arm,h.uyaw)
        data['uav_rigid']=replace(data['uav'],position=(h.uav[0]-dx,h.uav[1]-dy,h.uav[2]))
    h.change=route_references
    h.goal=h.uav[:];h.yaw_goal=h.uyaw
    h.until(lambda:h.core.phase=='COMPLETE',seconds=160.)
    assert h.joint.describe()['complete'] and not h.joint.fault and h.out==ZERO
    assert h.joint.sequence.config['car_speed_mps']==.25
    assert h.joint.sequence.config['route_local_forward_left_m']==[[3.,0.],[3.,2.],[0.,2.],[0.,0.]]
    indices={r['joint']['car_route_index'] for r in h.rows if r['joint']['phase']=='FOLLOW'}
    assert {1,2,3,4}<=indices
    assert all(abs(_wrap(r['car_yaw']-yaw))<plan['car_yaw_tolerance_rad'] for r in h.rows)
    assert [m.command for m in h.sink.messages if m.get_type()=='COMMAND_LONG']==[176,400,21]
    assert 'DESCEND' in {r['joint']['phase'] for r in h.rows}
    for previous,row in zip(h.rows,h.rows[1:]):
        if previous['joint']['phase']==row['joint']['phase']=='DESCEND':
            dz=row['joint']['target_ned'][2]-previous['joint']['target_ned'][2]
            assert -1e-9<=dz<=plan['descent_speed_mps']*h.dt+1e-8
    envelope=h.joint.sequence.resolved_start['reference_path_bounds_map_m']
    limits=plan['follow_limits']
    for axis in ('x','y'):
        # The accepted 0.51m side offset shifts the car rectangle slightly;
        # it does not expand the reviewed world bounds. Keep checking the
        # whole route is inside and the original return-point regression
        # retains its 0.40m margin (the old failure omitted that point).
        assert envelope[axis+'min']>=limits[axis+'min']
        assert envelope[axis+'max']<=limits[axis+'max']
        ground=h.joint.sequence.resolved_start['uav_ground_position_map_m']['xy'.index(axis)]
        assert ground-limits[axis+'min']>=.4-1e-6
        assert limits[axis+'max']-ground>=.4-1e-6


@pytest.mark.parametrize('forward,left', [(-.45,.90), (.35,.80), (0.,.50)])
def test_current_rectangle_alignment_waits_without_flight_from_left_positions(forward,left):
    """Current plan, varied initial placement; no automatic flight grant."""
    plan=json.loads((ROOT/'plan_b/follow/air_ground_rectangle.candidate.json').read_text())
    yaw=1.669436437026765
    uav=(.7523387190093587,-.709437204708787,.026870065699324697)
    dx,dy=_rotate_xy((forward,left),yaw)
    h=Plant(initial=(uav[0]+dx,uav[1]+dy,yaw),plan=plan,auto_flight=False)
    h.uav=list(uav);h.uyaw=yaw;h.ground_z=uav[2]
    h.goal=list(uav);h.yaw_goal=yaw
    a,b,c=plan['live_inputs']['car_ground_plane_map_abc']
    def references(data):
        h.car[2]=a*h.car[0]+b*h.car[1]+c
        data['car']=replace(data['car'],position=tuple(h.car))
        data['car_rigid']=data['car']
        # Virtual reference fixture: isolate placement from mounting offsets.
        data['uav_rigid']=data['uav']
    h.change=references
    h.until(lambda:h.core.phase=='WAIT_EXPLICIT_START' and h.joint.preparation_allowed(),seconds=35.)
    for _ in range(100):h.tick()
    assert not h.joint.fault and h.joint.car_task.phase=='WAIT_TAKEOFF'
    assert h.out==ZERO and not h.armed and not h.joint.started
    assert h.uav==list(uav)
    actual=_rotate_xy((h.car[0]-uav[0],h.car[1]-uav[1]),-yaw)
    assert math.dist(actual,(0.,.51))<=plan['car_arrival_m']
    assert abs(_wrap(h.cyaw-yaw))<=plan['car_yaw_tolerance_rad']
    assert not any(m.get_type()=='COMMAND_LONG' and m.command in (176,400,21)
                   for m in h.sink.messages)


def test_ground_alignment_freezes_goal_and_stops_if_uav_is_moved():
    h=Plant(plan=ground_config(),auto_flight=False)
    h.until(lambda:h.joint.car_task.phase=='TO_TAKEOFF')
    route=deepcopy(h.joint.car_task.plan.route)
    h.uav[0]+=.2;h.tick()
    assert h.joint.fault=='stationary_uav_moved_after_start_alignment'
    assert h.joint.car_task.plan.route==route and h.out==ZERO and not h.armed
    assert not any(m.get_type()=='COMMAND_LONG' and m.command==400 for m in h.sink.messages)


@pytest.mark.parametrize('left,accepted',[(.1996,True),(.2,True),(.194,False),(-.2,False)])
def test_ground_alignment_boundary_noise_does_not_change_goal(left,accepted):
    from follow_target_generator import AirGroundSequence
    sequence=AirGroundSequence(ground_config())
    car=TargetPose('car01','map',1,1,(-.58,left,.1),(0.,0.,0.,1.),True,False)
    uav=TargetPose('MircoAir','map',1,1,(0.,0.,.1),(0.,0.,0.,1.),True,False)
    if accepted:
        result=sequence.resolve_start(car,uav,car,uav)
        assert result['car_rigid_goal_map_xy_m']==pytest.approx((0.,.2))
    else:
        with pytest.raises(ValueError,match='car_must_start_on_clear_left'):
            sequence.resolve_start(car,uav,car,uav)


@pytest.mark.parametrize('forward,left,yaw,accepted',[
    (.022,.498,-.019,True), (0.,.498,0.,True),
    (.4,.498,0.,False), (.022,.498,.3,False),
    (0.,-.498,0.,False), (0.,0.,0.,False)])
def test_ground_alignment_outward_retreat_preserves_clearance(forward,left,yaw,accepted):
    from follow_target_generator import AirGroundSequence
    plan=ground_config()
    plan['start_alignment']['uav_rigid_offset_car_forward_left_m']=[0.,-.639]
    sequence=AirGroundSequence(plan)
    car=TargetPose('car01','map',1,1,(forward,left,.1),
                   (0.,0.,math.sin(yaw/2),math.cos(yaw/2)),True,False)
    uav=TargetPose('MircoAir','map',1,1,(0.,0.,.1),(0.,0.,0.,1.),True,False)
    if not accepted:
        with pytest.raises(ValueError,match='car_must_start_on_clear_left'):
            sequence.resolve_start(car,uav,car,uav)
        return
    goal=sequence.resolve_start(car,uav,car,uav)['car_rigid_goal_map_xy_m']
    assert goal==pytest.approx((0.,.639))
    distances=[math.hypot(forward+(goal[0]-forward)*i/100,
                          left+(goal[1]-left)*i/100) for i in range(101)]
    assert all(b>=a-1e-12 for a,b in zip(distances,distances[1:]))


@pytest.mark.parametrize('failure',['right_side','outside_area','old_rigid_sample','moving_uav','armed'])
def test_ground_alignment_rejects_unusable_start_without_changing_route(failure):
    h=Plant(plan=ground_config(),auto_car=False,auto_flight=False)
    if failure=='right_side':h.uav[1]=h.car[1]+.5
    if failure=='outside_area':h.joint.sequence.config['route_local_forward_left_m']=[[20.,0.],[0.,0.]]
    h.until(lambda:h.joint.car_task.phase=='WAIT_START')
    if failure=='old_rigid_sample':
        p=h.joint.latest['uav_rigid'];h.joint.latest['uav_rigid']=replace(p,stamp_ns=p.stamp_ns-1)
    if failure=='moving_uav':h.joint.latest['uav_velocity']=(.2,0.,0.)
    o=replace(h.o,armed=True) if failure=='armed' else h.o
    with pytest.raises(ValueError):h.joint.request_car_start(o)
    assert not h.joint.car_start_pending and h.joint.car_task.plan.route==[]
    assert h.joint.sequence.resolved_start is None and h.out==ZERO


def test_ground_alignment_does_not_refresh_expired_pose_at_start():
    h=Plant(plan=ground_config(),auto_car=False,auto_flight=False)
    h.until(lambda:h.joint.car_task.phase=='WAIT_START')
    # Last dispatch is current, but a pose source was already stale.
    p=h.joint.latest['uav'];h.joint.latest['uav']=replace(p,received_ns=p.received_ns-2_000_000_000)
    with pytest.raises(ValueError,match='subject_stale'):h.joint.request_car_start(h.o)
    assert h.joint.sequence.resolved_start is None


@pytest.mark.parametrize('mode',['ground','loaded','wrong_scope','expired'])
def test_console_ground_alignment_does_not_require_or_create_flight_grant(mode):
    from flight_start_confirmation import FlightRunController
    h=Plant(plan=loaded_config() if mode=='loaded' else ground_config(),auto_car=False,auto_flight=False)
    h.until(lambda:h.joint.car_task.phase=='WAIT_START')
    grant=SimpleNamespace(scope='other' if mode=='wrong_scope' else 'native-rc-protected-ground-bootstrap-v1',
        start_available=lambda:mode!='expired',_profile_ok=lambda:True,_hashes_ok=lambda:True,
        usb_present=lambda:False,check_port_identity=lambda:None)
    owner=SimpleNamespace(runtime=SimpleNamespace(core=h.core,now=h.now,failure=None,cancel_requested=False,
        stop_latched=False,closing=False,closed=False,coordinator=SimpleNamespace(fusion=SimpleNamespace(phase='READY')),
        inputs=SimpleNamespace(build=lambda t:(h.o,[]))),
        review_slots=SimpleNamespace(bootstrap=grant,execution=None))
    if mode=='ground':
        result=FlightRunController.start_car(owner)
        assert result['action']=='ground_alignment_requested' and result['flight_started'] is False
        assert h.joint.car_start_pending
    else:
        with pytest.raises(PermissionError):FlightRunController.start_car(owner)
        assert not h.joint.car_start_pending
    assert owner.review_slots.execution is None
    with pytest.raises(PermissionError,match='execution review'):FlightRunController.require_native_start(owner)


def test_small_accepted_heading_error_stops_turning_before_translation():
    h=Plant(plan=ground_config(),auto_flight=False)
    h.uyaw=h.cyaw+.052  # about 3 degrees, inside the existing 0.17 rad allowance
    publish=h.publish
    def with_static_turn_deadband(wire):
        publish(wire)
        # Measured failure: small turn requests, zero wheel response. Model
        # that response explicitly instead of assuming perfect kinematics.
        if h.out.vx==h.out.vy==0 and abs(h.out.wz)<.12:h.out=ZERO
    h.joint.car_publish=with_static_turn_deadband
    for _ in range(400):
        h.tick()
        if h.joint.car.supervisor.follower.approach_stage=='translate_aligned':break
        assert not h.joint.fault
    assert h.joint.car.supervisor.follower.approach_stage=='translate_aligned'
    assert not h.armed
    assert abs(h.joint.car.supervisor.follower.command.wz)<=h.joint.sequence.config['stopped_yaw_rate_rps']


def test_large_heading_alignment_rotates_before_approaching_grounded_uav():
    # Geometry of the failed 2026-11-16 run: about 117 degrees, not just the
    # small relative-yaw errors used in the initial kinematic tests.
    h=Plant((.901156,-.502396,1.133637),plan=ground_config(),auto_flight=False)
    h.uav[:2]=[.737574,.127230];h.uyaw=-3.099532
    h.goal=h.uav[:];h.yaw_goal=h.uyaw
    start=h.car[:2];turn_rows=[];translation_rows=[]
    for _ in range(4000):
        row=h.tick();stage=h.joint.car.supervisor.follower.approach_stage
        if stage=='turn_in_place':turn_rows.append(row)
        if stage=='translate_aligned':translation_rows.append(row)
        if h.joint.car_task.phase=='WAIT_TAKEOFF' and h.joint.car_status['stop_confirmed']:break
        assert not h.joint.fault,h.rows[-1]
    assert h.joint.car_task.phase=='WAIT_TAKEOFF' and h.joint.car_status['stop_confirmed']
    assert turn_rows and translation_rows
    assert max(math.dist(r['car'][:2],start) for r in turn_rows)<1e-9
    assert all(r['command'][:2]==(0.,0.) for r in turn_rows)
    assert abs(_wrap(translation_rows[0]['car_yaw']-h.uyaw))<h.joint.sequence.config['car_yaw_tolerance_rad']
    assert not any(r['armed'] for r in h.rows)
    assert not any(m.get_type()=='COMMAND_LONG' for m in h.sink.messages)


@pytest.mark.parametrize('initial',[(-.1,-.1,.4),(.2,-.2,-1.3)])
def test_existing_owners_complete_joint_task(initial,tmp_path):
    h=Plant(initial);h.until(lambda:h.core.phase=='COMPLETE')
    assert not h.core.fault and not h.joint.fault,h.rows[-1]
    assert h.joint.describe()['complete'] and h.joint.describe()['car_stopped']
    assert list(dict.fromkeys(r['joint']['phase'] for r in h.rows))==[
        'WAIT_READY','WAIT_START','TAKEOFF','FOLLOW','WAIT_CAR_STOP','ALIGN','DESCEND','LAND','COMPLETE']
    assert [m.command for m in h.sink.messages if m.get_type()=='COMMAND_LONG']==[176,400,21]
    assert all(p['accepted'] for p in h.packets)
    moving=[r for r in h.rows if any(r['command'])]
    assert moving and all(r['armed'] and r['joint']['phase']=='FOLLOW' for r in moving)
    assert all(not r['joint']['car_stopped'] for r in moving)
    assert all(r['joint']['car_stopped'] for r in h.rows if r['joint']['phase'] in ('ALIGN','DESCEND','LAND','COMPLETE'))
    assert _wrap(h.uyaw-h.cyaw)==pytest.approx(.2,abs=.015)
    delta=_rotate_xy((h.uav[0]-h.car[0],h.uav[1]-h.car[1]),-h.cyaw)
    assert delta==pytest.approx((.03,.02),abs=.02)
    (tmp_path/'execution_trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in h.rows))
    (tmp_path/'result.json').write_text(json.dumps(dict(scope='synthetic_kinematic_existing_owners',
        complete=True,actual_devices_opened=0,actual_sent_to_fc=0,actual_sent_to_car=0,
        duration_s=h.now,initial_car=initial,relative_xy_m=delta,relative_yaw_rad=_wrap(h.uyaw-h.cyaw),
        phases=list(dict.fromkeys(r['joint']['phase'] for r in h.rows))),indent=2))


def test_warmup_does_not_consume_ascent_or_enable_car():
    h=Plant();h.until(lambda:h.core.phase=='WAIT_ARM')
    assert not any(r['command']!= (0.,0.,0.) for r in h.rows)
    assert h.joint.sequence.target[2]==pytest.approx(h.uav[2])
    h.tick();h.tick()
    assert abs(h.goal[2]-h.joint.sequence.initial_uav.position[2])<.015


def test_land_handoff_and_touchdown_wait_never_send_motor_cutoff():
    h=Plant();h.until(lambda:h.core.phase=='LAND')
    # The normal handoff now follows the captured resting body height, not
    # the old 8 cm target plus the broad 5 cm ascent-arrival tolerance.
    assert h.joint.sequence.config['land_handoff_height_m']==0.
    assert abs(h.uav[2]-(h.car[2]+.15))<=.02
    assert abs(h.uv[2])<=.08
    # Hold the synthetic aircraft above the deck after accepting Land. Its
    # altitude, Land ACK and passage of time must not manufacture touchdown.
    h.land=False
    h.uav[2]=h.car[2]+.15+.03;h.goal=h.uav[:]
    after_land=len(h.sink.messages)
    for _ in range(100):h.tick()
    assert h.core.phase=='LAND' and h.armed and not h.joint.describe()['complete']
    assert h.joint.describe()['landing_waiting_for']=='touchdown'
    assert h.uav[2]>h.car[2]+.15+.02
    # Actual contact and final disarm arrive separately. Touchdown is supplied
    # by the synthetic FC; the PC does not disarm or substitute a power ramp.
    h.uav[2]=h.car[2]+.15;h.goal=h.uav[:]
    for _ in range(100):h.tick(landed=True)
    assert h.core.phase=='LAND' and h.armed
    assert h.joint.describe()['landing_waiting_for']=='px4_auto_disarm'
    assert h.out==ZERO
    assert any(m.get_type()=='VISION_POSITION_ESTIMATE' for m in h.sink.messages[after_land:])
    assert not any(m.get_type()=='COMMAND_LONG' and m.command in (185,400)
                   for m in h.sink.messages[after_land:])
    for force in (0.,21196.):
        assert not h.wire.candidate_allowed(h.core._command(400,h.now,0.,force))
    h.armed=False;h.tick(landed=True)  # observed PX4 auto-disarm, not a PC action
    assert h.core.phase=='COMPLETE' and h.joint.describe()['complete']
    assert h.joint.describe()['landing_waiting_for'] is None


def test_takeover_stops_car_and_never_sends_land_or_rearms():
    h=Plant();h.until(lambda:any((h.out.vx,h.out.vy,h.out.wz)))
    count=len(h.sink.messages);h.tick(manual=True)
    assert h.core.phase=='PILOT_CONTROL' and h.out==ZERO
    assert all(m.get_type()=='VISION_POSITION_ESTIMATE' for m in h.sink.messages[count:])
    h.tick(manual=True)
    assert h.out==ZERO and h.core.phase=='PILOT_CONTROL'


def test_bridge_disabled_ready_false_during_motion_is_not_a_link_failure():
    h=Plant();h.until(lambda:any((h.out.vx,h.out.vy,h.out.wz)))
    assert not h.snapshot()['transport']['ready']
    h.tick()
    assert not h.joint.fault and h.core.phase=='HOVER' and h.out!=ZERO
    h.change=lambda d:d['transport'].update(local_stop=True)
    h.tick()
    assert h.out==ZERO and h.core.phase=='LAND'


@pytest.mark.parametrize('failure',['clock','epoch','frame','publish'])
def test_runtime_failure_uses_existing_land_and_does_not_claim_platform_landing(failure):
    h=Plant();h.until(lambda:h.core.phase=='HOVER')
    if failure=='frame':h.change=lambda d:d.update(frame_revision='changed')
    if failure=='publish':
        def fail(wire):raise OSError('injected publish failure')
        h.joint.car_publish=fail
    h.tick(clock_missing=failure=='clock',reset=failure=='epoch')
    assert h.core.phase=='LAND'
    assert not h.joint.describe()['complete']
    if failure=='clock':
        assert not h.policy.active(h.steady+.11)  # original signed-command watchdog
        assert h.joint.fault=='cooperative_source_clock_missing'
    elif failure=='frame':assert h.out==ZERO
    else:assert h.joint.fault


def test_cooperative_failure_before_arm_cannot_start_motors():
    h=Plant();h.until(lambda:h.core.phase=='WARMUP')
    h.change=lambda d:d.update(graph_ok=False)
    h.tick()
    assert h.core.phase=='BLOCKED' and h.out==ZERO
    assert not any(m.get_type()=='COMMAND_LONG' and m.command==400 for m in h.sink.messages)


def test_current_joint_target_is_the_only_allowed_motion_packet():
    h=Plant();h.until(lambda:h.core.phase=='HOVER')
    msg=deepcopy(next(m for m in reversed(h.sink.messages) if m.get_type()=='SET_POSITION_TARGET_LOCAL_NED'))
    assert h.wire.candidate_allowed(msg)
    msg.x+=.1
    assert not h.wire.candidate_allowed(msg)
    msg.x-=.1;msg.yaw+=.1
    assert not h.wire.candidate_allowed(msg)


def test_bounded_velocity_reaches_same_mavlink_sender_and_rejects_alteration():
    plan=ground_config();plan['controller_velocity_feedforward']=True
    h=Plant(initial=(.2,-.2,1.2),plan=plan)
    h.until(lambda:h.joint.sequence.phase=='FOLLOW' and math.hypot(*h.joint.velocity_ned[:2])>.05
            and h.core.last_setpoint==h.now)
    msg=deepcopy(next(m for m in reversed(h.sink.messages) if m.get_type()=='SET_POSITION_TARGET_LOCAL_NED'))
    assert msg.type_mask==2496 and h.wire.candidate_allowed(msg)
    # This plant's map->NED is x,-y,-z despite its nonzero heading.
    vx,vy=h.joint.sequence.generator.velocity
    assert (msg.vx,msg.vy,msg.vz)==pytest.approx((vx,-vy,0.))
    assert math.hypot(msg.vx,msg.vy)<=plan['follow_limits']['speed_mps']
    warmup=[m for m in h.sink.messages if m.get_type()=='SET_POSITION_TARGET_LOCAL_NED' and m.type_mask==2552]
    assert warmup and all((m.vx,m.vy,m.vz)==(0.,0.,0.) for m in warmup)
    for field,value in [('vx',msg.vx+.1),('vy',float('nan')),('vz',-.1),('afx',.1),('type_mask',2552)]:
        bad=deepcopy(msg);setattr(bad,field,value)
        assert not h.wire.candidate_allowed(bad)
    h.until(lambda:h.core.phase=='COMPLETE')
    assert h.joint.describe()['complete'] and h.out==ZERO


def test_fixed_flight_review_does_not_cover_cooperative_plan():
    from flight_execution_review import ReviewedExecutionGrant,flight_scope
    h=Plant();g=object.__new__(ReviewedExecutionGrant)
    g.runtime=SimpleNamespace(core=h.core,fixed_configuration=False)
    g.record={}
    assert not g._profile_ok()
    g.record['air_ground_plan_sha256']=h.joint.plan_sha256
    assert not g._profile_ok()  # example dimensions cannot be a live plan
    assert flight_scope('native_rc',{},h.joint.sequence.config)=='single-air-ground-native-rc-flight-review-v1'


def test_landing_feedback_uses_existing_sender_and_yields_to_pilot():
    plan=ground_config()
    plan.update(controller_velocity_feedforward=True,landing_position_feedback_gain_s_inv=.5,
                landing_xy_tolerance_m=.05,landing_horizontal_speed_mps=.08,
                landing_integral_gain_s_inv2=.05,landing_velocity_damping=.4,
                landing_settling_speed_mps=.04,landing_settle_seconds=1.5)
    h=Plant(initial=(.2,-.2,1.2),plan=plan)
    h.until(lambda:h.joint.sequence.phase=='ALIGN')
    # A tracking disturbance is measured in map coordinates, not a target shift.
    h.uav[0]+=.25
    for _ in range(8):h.tick()
    h.until(lambda:h.core.last_setpoint==h.now)
    seq=h.joint.sequence
    assert not h.joint.fault
    assert math.dist(seq.target_velocity[:2],seq.generator.velocity)>1e-5
    msg=next(m for m in reversed(h.sink.messages) if m.get_type()=='SET_POSITION_TARGET_LOCAL_NED')
    assert msg.type_mask==2496 and h.wire.candidate_allowed(msg)
    vx,vy,_=seq.target_velocity
    assert (msg.vx,msg.vy,msg.vz)==pytest.approx((vx,-vy,0.))
    # Keep a synthetic Vicon residual once the returning reference is still;
    # verify the bounded bias also travels through this same owner/encoder.
    def residual(data):
        data['uav']=replace(data['uav'],position=(seq.target[0]+.07,seq.target[1],seq.target[2]))
        data['uav_velocity']=(0.,0.,0.)
    h.change=residual
    for _ in range(200):h.tick()
    h.until(lambda:h.core.last_setpoint==h.now)
    assert seq.landing_integral[0]<-.001
    msg=next(m for m in reversed(h.sink.messages) if m.get_type()=='SET_POSITION_TARGET_LOCAL_NED')
    vx,vy,_=seq.target_velocity
    assert msg.type_mask==2496 and h.wire.candidate_allowed(msg)
    assert (msg.vx,msg.vy,msg.vz)==pytest.approx((vx,-vy,0.))
    h.tick(manual=True)
    assert seq.phase=='PILOT_CONTROL' and seq.target_velocity==(0.,0.,0.)
    assert seq.landing_integral==(0.,0.)
    assert h.out==ZERO


def test_binding_is_optional_single_use_and_checks_same_time_origin(tmp_path):
    from flight_start_confirmation import build_guarded_session,FlightRunController
    from test_continuous_execution import FakePort
    from test_native_rc_mission_delta import NativePeer
    h=Plant()
    p=NativePeer(tmp_path/'profile')
    joint=AirGroundExecution(config(),source_snapshot=h.snapshot,car_publish=h.publish,
        channel_key='39'*32,origin_monotonic=10.)
    controller=build_guarded_session(revision='OFFLINE',origin=[0,0,0],orientation=[0,0,0,1],
        port=FakePort(),poll_gamepad=None,poll_pose=lambda:None,now_ros_ns=lambda:None,
        check_port_identity=lambda:None,usb_present=lambda:False,origin_monotonic=10.,
        recovery_path=tmp_path/'run/flight_recovery.json',pilot_source='native_rc',cooperative_execution=joint,
        fixed_profile=p.runtime.coordinator.fusion.profile)
    assert controller.runtime.core.cooperative is joint
    assert FlightRunController.task_seconds(controller.runtime)==600.
    assert controller.status()['cooperative']['plan_sha256']==joint.plan_sha256
    with pytest.raises(ValueError):controller.runtime.attach_cooperative(joint)


def test_full_continuous_io_dispatch_and_feedback(tmp_path):
    from test_native_rc_mission_delta import NativePeer,start
    h=Plant((0.,0.,0.));p=NativePeer(tmp_path)
    p.x,p.y,p.z=.03,-.02,-h.uav[2];p.ground_z=p.z
    cfg=config();cfg['route_world_xyyaw']=[[0.,0.,0.],[.1,0.,0.],[.1,.1,0.]]
    def snapshot():
        stamp=p.input_args['now_ros_ns']
        h.i+=1;h.now=h.steady=p.now;h.ros=stamp*1e-9
        vx,vy=_rotate_xy((h.out.vx,h.out.vy),h.cyaw)
        h.car[0]+=vx*p.dt;h.car[1]+=vy*p.dt;h.cyaw=_wrap(h.cyaw+h.out.wz*p.dt)
        h.cv=(vx,vy,h.out.wz);h.uav=[p.x,-p.y,-p.z];h.uyaw=0.
        h.uv=(p.vx,-p.vy,-p.velocity)
        data=h.snapshot()
        for source in ('car','uav'):data[source]=replace(data[source],stamp_ns=stamp)
        return data
    h.joint=AirGroundExecution(cfg,source_snapshot=snapshot,car_publish=h.publish,
                              channel_key='39'*32,origin_monotonic=0.)
    p.runtime.attach_cooperative(h.joint)
    assert not p.prepare()['failure'],p.controller.status()
    for _ in range(200):
        p.turn()
        if h.joint.last_status['phase']=='WAIT_START':break
    assert h.joint.last_status['phase']=='WAIT_START',p.controller.status()
    preview=p.controller.start.create()
    assert preview['cooperative']['plan_sha256']==h.joint.plan_sha256
    start(p)
    first_fault=None
    for _ in range(8000):
        p.turn()
        if first_fault is None and h.joint.describe()['fault']:
            o,missing=p.runtime.inputs.build(p.now)
            first_fault=dict(at=p.now,joint=h.joint.describe(),missing=missing,observation=str(o),
                car=h.joint.car_status,sequence=h.joint.last_status)
        if p.controller.done:break
    assert p.controller.done,p.controller.status()
    assert not p.runtime.failure and not p.runtime.core.fault,json.dumps(first_fault,indent=2)
    assert h.joint.describe()['complete'],json.dumps(first_fault or h.joint.describe(),indent=2)
    assert p.controller.journal_finalized
    commands=[m.command for m in p.port.calls if m.get_type()=='COMMAND_LONG']
    assert commands.count(400)==commands.count(21)==1
    assert any(q['packet']['payload']['state']=='ACTIVE' for q in h.packets)
    (tmp_path/'result.json').write_text(json.dumps(dict(scope='synthetic_full_continuous_io',
        complete=True,actual_devices_opened=0,actual_sent_to_fc=0,actual_sent_to_car=0,
        commands=commands,cooperative=h.joint.describe(),journal_finalized=True),indent=2))


def test_brief_standstill_window_gap_holds_target_without_false_motion_failure():
    h=Plant();h.until(lambda:h.core.phase=='ASCEND')
    h.joint.car_status['stop_confirmed']=False
    before=h.joint.sequence.target
    h.tick()
    assert h.joint.sequence.phase=='TAKEOFF' and not h.joint.fault
    assert h.joint.sequence.target==before and h.out==ZERO


def test_actual_car_motion_before_clearance_still_aborts():
    h=Plant();h.until(lambda:h.core.phase=='ASCEND')
    h.car[0]+=.04
    h.tick()
    assert h.core.phase=='LAND' and h.joint.fault=='car_moved_before_uav_clearance'


def test_rendezvous_standstill_gap_brakes_waits_and_resumes_without_short_deadline():
    plan=ground_config()
    plan.update(follow_position='above_car',controller_velocity_feedforward=True,
                airborne_offset_car_forward_left_m=[0.,0.],follow_arrival_tolerance_m=.15)
    h=Plant(plan=plan)
    h.until(lambda:h.joint.sequence.phase=='RENDEZVOUS')
    for _ in range(10):h.tick()
    seq=h.joint.sequence
    anchor=h.car[:]
    previous=seq.target
    previous_velocity=seq.generator.velocity
    # Live run: zero wheel feedback, only 4.7 mm car position variation,
    # but the stop window briefly became false. Extend it to 3 s here.
    for i in range(150):
        h.car[0]=anchor[0]+.0047*math.sin(i*.1)
        h.joint.car_status['stop_confirmed']=False
        h.tick()
        assert seq.phase=='RENDEZVOUS' and not h.joint.fault
        assert h.out==ZERO and not h.land
        assert math.dist(seq.target[:2],previous[:2])<=seq.limits.speed_mps*h.dt+1e-9
        assert math.dist(seq.generator.velocity,previous_velocity)<=seq.limits.acceleration_mps2*h.dt+1e-9
        previous,previous_velocity=seq.target,seq.generator.velocity
    assert math.hypot(*seq.generator.velocity)<.01
    h.car[:]=anchor
    h.until(lambda:seq.phase=='FOLLOW')
    assert not h.joint.fault
    h.until(lambda:h.core.phase=='COMPLETE')
    assert h.joint.describe()['complete'] and not h.joint.fault


@pytest.mark.parametrize('stop_confirmed',[False,True])
def test_rendezvous_actual_displacement_still_aborts(stop_confirmed):
    plan=ground_config()
    plan.update(follow_position='above_car',airborne_offset_car_forward_left_m=[0.,0.],
                follow_arrival_tolerance_m=.15)
    h=Plant(plan=plan)
    h.until(lambda:h.joint.sequence.phase=='RENDEZVOUS')
    h.car[0]+=plan['car_arrival_m']+.02
    h.joint.car_status['stop_confirmed']=stop_confirmed
    h.tick()
    assert h.core.phase=='LAND'
    assert h.joint.fault=='car_moved_before_overhead_rendezvous'
    assert h.out==ZERO


def test_ground_report_outside_platform_cannot_claim_joint_success():
    h=Plant();h.until(lambda:h.joint.sequence.phase=='LAND')
    h.uav[0]+=.7;h.uav[2]=h.car[2]+.15;h.armed=False
    h.tick()
    assert h.core.phase=='COMPLETE'  # FC ground confirmation is valid.
    assert not h.joint.describe()['complete']
    assert h.joint.fault=='landed_outside_platform'


def test_existing_car_bridge_callback_is_reused_without_constructing_a_node(monkeypatch):
    from air_ground_execution import existing_car_bridge_publisher
    from mecanum_wand_4wse_hardware import nodes
    h=Plant()
    # Exercise the actual on_signed implementation, but never __init__ (ROS,
    # sockets, startup or motors). Only its hardware tick is a recording stub.
    bridge=object.__new__(nodes.HardwareBridge)
    bridge.runtime=SimpleNamespace(motion_tag='4WSE');bridge.policy=h.policy
    bridge.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=round(h.ros*1e9)))
    ticks=[];bridge.safety_tick=lambda:ticks.append(True)
    monkeypatch.setattr(nodes.time,'monotonic',lambda:h.steady)
    publish=existing_car_bridge_publisher(bridge)
    assert not ticks
    publish(h.joint.car.signer.encode(ZERO,h.steady,h.ros,state='STANDBY',permit=False))
    assert ticks==[True] and h.policy.envelope['state']=='STANDBY'
    assert not h.policy.active(h.steady)


def test_source_startup_wait_is_not_a_latched_flight_failure():
    h=Plant();source=h.joint.source_snapshot
    h.joint.source_snapshot=lambda:None
    h.tick()
    assert not h.joint.fault and not h.packets and h.joint.last_at is None
    h.joint.source_snapshot=source
    h.until(lambda:h.core.phase=='WARMUP')
    assert not h.joint.fault


@pytest.mark.parametrize('initial',[(-.12,-.16,.5),(.15,-.08,-.7)])
def test_loaded_approach_flight_and_landing_use_one_car_owner(initial,tmp_path):
    cfg=loaded_config();cfg['route_world_xyyaw']=[[.12,0.,0.],[.12,.1,.35]]
    h=Plant(initial,plan=cfg)
    h.until(lambda:h.core.phase=='COMPLETE',seconds=140.)
    assert not h.core.fault and not h.joint.fault,h.rows[-1]
    assert h.joint.describe()['complete'] and h.joint.car_task.phase=='COMPLETE'
    assert h.joint.car is h.joint.car_task.car
    phases=list(dict.fromkeys(r['joint']['car_task_phase'] for r in h.rows))
    assert phases==['WAIT_READY','WAIT_START','TO_TAKEOFF','WAIT_TAKEOFF','ROUTE','WAIT_LANDING','COMPLETE']
    approach=[r for r in h.rows if r['joint']['car_task_phase']=='TO_TAKEOFF']
    assert any(any(r['command']) for r in approach)
    assert all(r['phase']=='PREPARE' and not r['armed'] for r in approach)
    assert math.dist(h.core.origin.position_ned[:2],(initial[0],-initial[1]))>.08
    takeoff=h.joint.sequence.initial_car.position
    assert math.dist(takeoff[:2],cfg['takeoff_car_world_xyyaw'][:2])<=cfg['car_arrival_m']
    assert h.core.origin.position_ned==pytest.approx((
        *h.joint.sequence.initial_uav.position[:1],-h.joint.sequence.initial_uav.position[1],
        -h.joint.sequence.initial_uav.position[2]),abs=1e-8)
    moving=[r for r in h.rows if any(r['command']) and r['joint']['car_task_phase']=='ROUTE']
    assert moving and all(r['armed'] and r['joint']['phase']=='FOLLOW' for r in moving)
    assert all(p['accepted'] for p in h.packets)
    assert h.joint.car_publish_calls==len(h.rows)
    assert [m.command for m in h.sink.messages if m.get_type()=='COMMAND_LONG']==[176,400,21]
    assert _wrap(h.uyaw-h.cyaw)==pytest.approx(.2,abs=.02)
    (tmp_path/'loaded_execution_trace.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in h.rows))


def test_loaded_approach_waits_for_explicit_car_then_separate_takeoff():
    h=Plant(plan=loaded_config(),auto_car=False,auto_flight=False)
    h.until(lambda:h.joint.car_task.phase=='WAIT_START')
    for _ in range(70):h.tick()
    assert h.out==ZERO and h.core.phase=='PREPARE' and h.core.origin is None
    with pytest.raises(ValueError):h.joint.check_start(h.o)
    h.joint.request_car_start(h.o)
    h.until(lambda:h.core.phase=='WAIT_EXPLICIT_START')
    for _ in range(70):h.tick()
    assert h.joint.car_task.phase=='WAIT_TAKEOFF' and h.out==ZERO
    assert not any(m.get_type()=='COMMAND_LONG' for m in h.sink.messages)
    with pytest.raises(ValueError):h.joint.request_car_start(h.o)


@pytest.mark.parametrize('invalid',[None,'zero_missing','link_stale','local_stop','mcu_fault'])
def test_loaded_handoff_uses_current_disabled_stop_after_startup_budget_used(invalid):
    h=Plant(plan=loaded_config(),auto_flight=False)
    def after_approach(data):
        if h.joint.car_task.phase!='WAIT_TAKEOFF':return
        # Real bridge keeps ready=False after started_ms becomes nonzero,
        # even when wheel feedback confirms PAUSED and disabled zero output.
        data['transport'].update(ready=False,healthy_budget_used=True,native_disabled_zero=True)
        if invalid=='zero_missing':data['transport'].pop('native_disabled_zero')
        elif invalid=='link_stale':data['transport']['fresh']=False
        elif invalid=='local_stop':data['transport']['local_stop']=True
        elif invalid=='mcu_fault':data['transport']['native_fault_bits']=1
    h.change=after_approach
    h.until(lambda:h.joint.car_task.phase=='WAIT_TAKEOFF')
    for _ in range(150):h.tick()
    assert h.out==ZERO and not h.armed
    assert not any(m.get_type()=='COMMAND_LONG' for m in h.sink.messages)
    if invalid is None:
        assert h.joint.car_status['stop_confirmed']
        assert h.joint.last_status['phase']=='WAIT_START'
        assert h.core.phase=='WAIT_EXPLICIT_START'
    else:
        assert h.core.phase!='WAIT_EXPLICIT_START'


@pytest.mark.parametrize('failure',['source_missing','flight_stale','cancel','clock_missing'])
def test_loaded_approach_failure_stops_without_aircraft_start(failure):
    h=Plant(plan=loaded_config(),auto_flight=False)
    h.until(lambda:h.out!=ZERO)
    if failure=='source_missing':h.joint.source_snapshot=lambda:None
    elif failure=='flight_stale':
        old=h.joint._flight_progress
        h.joint._flight_progress=lambda *a,**kw:replace(old(*a,**kw),issued_steady=h.steady-4.,issued_ros=h.ros-4.)
    elif failure=='cancel':h.core.fault='operator_cancel'
    h.tick(clock_missing=failure=='clock_missing')
    assert h.joint.fault
    # Clock loss withdraws renewal; the original transport expiry owns stop.
    if failure=='clock_missing':assert not h.policy.active(h.steady+1.)
    else:assert h.out==ZERO
    assert not any(m.get_type()=='COMMAND_LONG' for m in h.sink.messages)


def test_owner_turn_does_not_refresh_old_flight_handoff():
    h=Plant(plan=loaded_config(),auto_car=False);h.tick()
    proof=h.joint._flight_progress(replace(h.o,status_age_s=.7),h.snapshot(),h.steady+.2,h.ros+.2,False)
    assert proof.issued_steady==pytest.approx(h.steady-.7)
    assert proof.issued_ros==pytest.approx(h.ros-.7)


def test_loaded_task_through_continuous_owner_and_explicit_console_actions(tmp_path,monkeypatch):
    from test_native_rc_mission_delta import NativePeer,start
    from test_shared_pilot_entry import reviewed_fake_grant
    from flight_execution_review import ReviewedExecutionGrant
    import flight_execution_review as reviews
    h=Plant((-.12,-.1,0.));p=NativePeer(tmp_path,timestamp_mode='local_ros_timesync')
    cfg=loaded_config();cfg['route_world_xyyaw']=[[.1,0.,0.],[.1,.1,0.]]
    p.x,p.y,p.z=h.uav[0],-h.uav[1],-h.uav[2];p.ground_z=p.z
    def snapshot():
        stamp=p.input_args['now_ros_ns']
        h.i+=1;h.now=h.steady=p.now;h.ros=stamp*1e-9
        vx,vy=_rotate_xy((h.out.vx,h.out.vy),h.cyaw)
        h.car[0]+=vx*p.dt;h.car[1]+=vy*p.dt;h.cyaw=_wrap(h.cyaw+h.out.wz*p.dt)
        h.cv=(vx,vy,h.out.wz)
        if not p.armed and not p.landing:
            dx,dy=_rotate_xy((.03,.02),h.cyaw)
            p.x,p.y=h.car[0]+dx,-(h.car[1]+dy)
        h.uav=[p.x,-p.y,-p.z];h.uyaw=0.;h.uv=(p.vx,-p.vy,-p.velocity)
        data=h.snapshot()
        if h.joint.car_task is not None and h.joint.car_task.phase=='WAIT_TAKEOFF':
            data['transport'].update(ready=False,healthy_budget_used=True,native_disabled_zero=h.out==ZERO)
        for source in ('car','uav'):data[source]=replace(data[source],stamp_ns=stamp)
        return data
    h.joint=AirGroundExecution(cfg,source_snapshot=snapshot,car_publish=h.publish,
                              channel_key='39'*32,origin_monotonic=0.)
    p.runtime.attach_cooperative(h.joint)
    closing_ticks=[]
    maintain=h.joint.service_stopped
    def record_stop(now,ros_ns):
        closing_ticks.append(now)
        return maintain(now,ros_ns)
    h.joint.service_stopped=record_stop
    for _ in range(2500):
        p.turn()
        if p.runtime.failure or (p.runtime.coordinator.fusion.phase=='READY' and h.joint.car_task.phase=='WAIT_START'):break
    assert not p.runtime.failure,p.controller.status()
    assert p.runtime.coordinator.fusion.phase=='READY' and p.runtime.core.origin is None,p.controller.status()
    assert p.runtime.coordinator.fusion.session_limit==600.
    # Current fused readiness permits the SAME plan-bound review before the
    # loaded approach; flight origin has deliberately not been frozen yet.
    grant=reviewed_fake_grant(ReviewedExecutionGrant,p.runtime)
    grant.execution_io=p.io;grant.record['backup']='SYNTHETIC-ONLY';grant._load_backup=lambda:None
    registry=tmp_path/'registry.json';registry.write_text(json.dumps({'active_revision_id':p.runtime.inputs.rigid_revision}))
    monkeypatch.setattr(reviews,'REGISTRY',registry)
    assert grant._physical_evidence_ok()
    with pytest.raises(PermissionError):p.controller.start_car()
    p.controller.review_slots.execution=grant
    grant.fault='expired'
    with pytest.raises(PermissionError):p.controller.start_car()
    grant.fault=None
    assert p.controller.start_car()['flight_started'] is False
    for _ in range(6000):
        p.turn()
        if p.runtime.failure or p.runtime.core.phase=='WAIT_EXPLICIT_START':break
    assert not p.runtime.failure,p.controller.status()
    assert p.runtime.core.phase=='WAIT_EXPLICIT_START',p.controller.status()
    assert h.joint.car_task.phase=='WAIT_TAKEOFF' and h.out==ZERO
    assert not any(m.get_type()=='COMMAND_LONG' and m.command in (176,400,21) for m in p.port.calls)
    assert math.dist(p.runtime.core.origin.position_ned[:2],(-.12,.1))>.08
    origin=p.runtime.core.origin.position_ned
    start(p)
    for _ in range(8000):
        p.turn()
        if p.controller.done:break
    assert p.controller.done and not p.runtime.failure,p.controller.status()
    assert h.joint.describe()['complete'],h.joint.describe()
    assert p.controller.journal_finalized
    assert closing_ticks  # Rate restoration must not starve the car owner.
    assert h.joint.car_status['stop_confirmed'] and not h.joint.car_status['fault']
    assert all(b-a<=p.dt+1e-8 for a,b in zip(closing_ticks,closing_ticks[1:]))
    commands=[m.command for m in p.port.calls if m.get_type()=='COMMAND_LONG']
    assert commands.count(400)==commands.count(21)==1
    (tmp_path/'loaded_continuous_result.json').write_text(json.dumps(dict(
        scope='synthetic_full_continuous_owner',complete=True,car_phase=h.joint.car_task.phase,
        frozen_origin=origin,commands=commands,actual_devices_opened=0,actual_sent_to_fc=0,actual_sent_to_car=0),indent=2))


def test_carried_placement_is_retained_separately_from_new_takeoff_origin():
    h=Plant((-.12,-.1,.6),plan=loaded_config(),auto_flight=False)
    h.until(lambda:h.core.phase=='WAIT_EXPLICIT_START')
    c=h.joint.sequence
    assert c.loading_pose[0].position[:2]==pytest.approx((-.12,-.1))
    # A small settling offset after carriage must not redefine the intended
    # return placement. No I/O; exercise the same capture used by TAKEOFF.
    data=h.snapshot();uav=replace(data['uav'],position=(data['uav'].position[0]+.01,*data['uav'].position[1:]),
        quaternion_xyzw=(0.,0.,math.sin((h.uyaw+.05)/2),math.cos((h.uyaw+.05)/2)))
    c._capture(data['car'],uav)
    assert c.relative_xy==pytest.approx((.03,.02))
    assert c.relative_yaw==pytest.approx(.2)
    assert c.initial_uav.position==uav.position


def test_prearm_joint_source_failure_exits_through_continuous_cleanup(tmp_path):
    from test_native_rc_mission_delta import NativePeer
    p=NativePeer(tmp_path);packets=[]
    def failed_source():raise ValueError('synthetic_joint_source_failed')
    joint=AirGroundExecution(loaded_config(),source_snapshot=failed_source,car_publish=packets.append,
        channel_key='39'*32,origin_monotonic=0.)
    p.runtime.attach_cooperative(joint)
    for _ in range(120):
        p.turn()
        if p.controller.done:break
    assert p.runtime.failure=='synthetic_joint_source_failed'
    assert p.controller.done and p.runtime.core.phase=='BLOCKED',p.controller.status()
    assert packets and all(not json.loads(x)['payload']['permit'] for x in packets)
    assert not any(m.get_type()=='COMMAND_LONG' and m.command in (176,400,21) for m in p.port.calls)


@pytest.mark.parametrize('feedback_fresh',[True,False])
def test_missing_uav_audit_keeps_car_stop_service_alive(feedback_fresh):
    h=Plant(plan=loaded_config(),auto_flight=False)
    h.until(lambda:h.joint.car_task.phase=='WAIT_TAKEOFF' and h.joint.car_status['stop_confirmed'])
    assert h.joint.car_status['stop_confirmed'] and not h.armed
    h.core.request_takeover('native_rc_mode_switch_intent')
    if not feedback_fresh:
        h.change=lambda data:data['feedback'].update(issued_steady=h.steady-2.,issued_ros=h.ros-2.)
    before=len(h.packets)
    for _ in range(100):
        h.i+=1;h.now+=h.dt;h.steady=10.+h.now;h.ros=1_800_000_000.+h.steady
        h.joint.update(None,now=h.now,ros_ns=round(h.ros*1e9),
                       unavailable='current_commander_status_missing_or_stale',pilot_control=True)
    # Transition into telemetry restoration must not discover an unserviced
    # car clock and manufacture an ESTOP on an already stopped car.
    h.i+=1;h.now+=h.dt;h.steady=10.+h.now;h.ros=1_800_000_000.+h.steady
    h.joint.service_stopped(h.now,round(h.ros*1e9))
    assert h.joint.car.supervisor.clock.fault==''
    assert h.joint.car.gate.clock.fault==''
    assert h.joint.car_status['stop_confirmed'] is feedback_fresh
    packets=[p['packet']['payload'] for p in h.packets[before:]]
    assert packets and all(not p['permit'] and not any(p['command'].values()) for p in packets)
    assert not any(p['estop'] for p in packets)
    assert not h.armed and h.joint.fault=='current_commander_status_missing_or_stale'

# Arbitrary-side approach: synthetic kinematics, no hardware/network I/O.
def avoidance_config():
    c=ground_config()
    c['start_alignment'].update(uav_rigid_offset_car_forward_left_m=[0.,-.51],
        avoidance=dict(car_size_m=[.18,.16],uav_size_m=[.17,.17],
                       car_reference_margin_m=.05,uav_reference_margin_m=.03,path_margin_m=.15))
    c['follow_limits'].update(xmin=-4.,xmax=4.,ymin=-4.,ymax=4.)
    return c


@pytest.mark.parametrize('angle',[i*math.pi/4 for i in range(8)])
@pytest.mark.parametrize('car_heading',[0.,math.pi/2,math.pi,-math.pi/2])
def test_arbitrary_approach_polyline_and_turn_footprints(angle,car_heading):
    from follow_target_generator import AirGroundSequence,ground_approach_clear
    c=avoidance_config();seq=AirGroundSequence(c)
    car=TargetPose('car01','map',1,1,(1.2*math.cos(angle),1.2*math.sin(angle),.1),
                  (0.,0.,math.sin(car_heading/2),math.cos(car_heading/2)),True,False)
    uav=replace(car,source='MircoAir',position=(0.,0.,.1),quaternion_xyzw=(0.,0.,0.,1.))
    r=seq.resolve_start(car,uav,car,uav);a=r['ground_approach'];previous=(*car.position[:2],car_heading)
    for i,point in enumerate(a['waypoints']):
        for k in range(101):
            t=k/100;x=previous[0]+t*(point[0]-previous[0]);y=previous[1]+t*(point[1]-previous[1])
            yaw=previous[2]+t*_wrap(point[2]-previous[2])
            pose=replace(car,position=(x,y,.1),quaternion_xyzw=(0.,0.,math.sin(yaw/2),math.cos(yaw/2)))
            assert ground_approach_clear(pose,a),(angle,car_heading,i,point)
        if i in a['rotation_indices']:
            assert math.dist(previous[:2],point[:2])<1e-9
        previous=point
    assert previous==pytest.approx((0.,.51,0.))
    assert a['physical_validated'] is False


@pytest.mark.parametrize('initial',[(0.,-1.,math.pi),(.9,0.,-math.pi/2),(-.9,0.,math.pi/2),(0.,.8,0.)])
def test_arbitrary_approach_real_owner_chain_stops_without_flight(initial):
    from follow_target_generator import ground_approach_clear
    h=Plant(initial=initial,plan=avoidance_config(),auto_flight=False)
    h.uav=[0.,0.,h.ground_z];h.uyaw=0.;h.goal=h.uav[:];h.yaw_goal=0.
    def references(data):
        data['car_rigid']=data['car'];data['uav_rigid']=data['uav']
        frozen=h.joint.sequence.resolved_start
        if frozen:assert ground_approach_clear(data['car'],frozen['ground_approach'])
    h.change=references
    h.until(lambda:h.joint.preparation_allowed() and h.core.phase=='WAIT_EXPLICIT_START',seconds=100.)
    assert h.joint.car_task.phase=='WAIT_TAKEOFF'
    assert h.joint.car_task.index==h.joint.car_task.plan.takeoff_index
    assert math.dist(h.car[:2],(0.,.51))<.08
    assert abs(_wrap(h.cyaw))<.17
    assert h.out==ZERO and not h.armed and h.joint.fault is None
    assert not any(m.get_type()=='COMMAND_LONG' and m.command in (176,400,21) for m in h.sink.messages)


def test_arbitrary_approach_handoff_preserves_all_flight_route_legs():
    plan=avoidance_config();plan['route_local_forward_left_m']=[[1,0],[1,1],[0,1],[0,0]]
    h=Plant(initial=(0.,-1.,math.pi),plan=plan)
    h.uav=[0.,0.,h.ground_z];h.uyaw=0.;h.goal=h.uav[:];h.yaw_goal=0.
    def references(data):data.update(car_rigid=data['car'],uav_rigid=data['uav'])
    h.change=references
    h.until(lambda:h.joint.describe()['complete'],seconds=220.)
    takeoff=h.joint.car_task.plan.takeoff_index
    assert takeoff>0
    assert {r['joint']['car_route_index'] for r in h.rows if r['joint']['phase']=='FOLLOW'} >= {1,2,3,4}
    assert h.joint.fault is None and h.out==ZERO and not h.armed


@pytest.mark.parametrize('case',['overlap','spacing','boundary','missing_geometry'])
def test_arbitrary_approach_rejects_geometry_before_motion(case):
    from follow_target_generator import AirGroundSequence
    c=avoidance_config();car=TargetPose('car01','map',1,1,(0.,-1.,.1),(0.,0.,0.,1.),True,False)
    uav=replace(car,source='MircoAir',position=(0.,0.,.1))
    error={'overlap':'overlaps_vehicle','spacing':'spacing_too_small','boundary':'no_clear_path','missing_geometry':'approach_geometry_required'}[case]
    if case=='overlap':car=replace(car,position=(0.,.1,.1))
    if case=='spacing':c['start_alignment']['uav_rigid_offset_car_forward_left_m']=[0.,-.2]
    if case=='boundary':c['follow_limits'].update(xmin=-.2,xmax=.2,ymin=-2.,ymax=2.)
    if case=='missing_geometry':c['start_alignment']['avoidance']['uav_size_m']=None
    seq=AirGroundSequence(c)
    with pytest.raises(ValueError,match=error):seq.resolve_start(car,uav,car,uav)
    assert seq.resolved_start is None


def test_arbitrary_approach_rotates_world_and_handles_close_retreat():
    from follow_target_generator import AirGroundSequence,ground_approach_clear
    for heading in (-2.8,-.3,1.2):
        for local in [(0.,-.32),(.9,-.9)]:
            c=avoidance_config();seq=AirGroundSequence(c);center=(.3,-.2,.1)
            dx,dy=_rotate_xy(local,heading)
            q=(0.,0.,math.sin(heading/2),math.cos(heading/2))
            car=TargetPose('car01','map',1,1,(center[0]+dx,center[1]+dy,.1),q,True,False)
            uav=replace(car,source='MircoAir',position=center)
            r=seq.resolve_start(car,uav,car,uav);a=r['ground_approach'];p=car.position
            for waypoint in a['waypoints']:
                for k in range(101):
                    t=k/100;xy=tuple(p[i]+t*(waypoint[i]-p[i]) for i in (0,1))
                    assert ground_approach_clear(replace(car,position=(*xy,.1)),a)
                p=waypoint
            assert _rotate_xy((p[0]-center[0],p[1]-center[1]),-heading)==pytest.approx((0.,.51))


def test_arbitrary_approach_stops_on_measured_exclusion_violation():
    h=Plant(initial=(0.,-1.,math.pi),plan=avoidance_config(),auto_flight=False)
    h.uav=[0.,0.,h.ground_z];h.uyaw=0.;h.goal=h.uav[:];h.yaw_goal=0.
    def references(data):data.update(car_rigid=data['car'],uav_rigid=data['uav'])
    h.change=references
    h.until(lambda:h.joint.car_task.phase=='TO_TAKEOFF')
    h.car[:2]=[0.,.1];h.tick()
    assert h.joint.fault=='approach_vehicle_clearance_lost'
    assert h.out==ZERO and not h.armed


def test_nearly_aligned_car_does_not_make_unnecessary_retreat():
    from follow_target_generator import AirGroundSequence
    c=avoidance_config();seq=AirGroundSequence(c)
    car=TargetPose('car01','map',1,1,(0.,.45,.1),(0.,0.,math.sin(.001/2),math.cos(.001/2)),True,False)
    uav=replace(car,source='MircoAir',position=(0.,0.,.1),quaternion_xyzw=(0.,0.,0.,1.))
    a=seq.resolve_start(car,uav,car,uav)['ground_approach']
    assert a['length_m']==pytest.approx(.06)
    assert len(a['waypoints'])==2

@pytest.mark.parametrize('initial',[(1.2,-.7,0.),(-1.,-.8,1.2),(.8,.9,-2.),(-.8,.9,0.)])
def test_unpowered_uav_alignment_ends_at_stop_without_flight(initial):
    from car_route_adapter import CarMotionTask
    from follow_target_generator import AirGroundSequence,ground_approach_clear
    from mecanum_wand_4wse_hardware.commissioning import TransportPolicy
    c=avoidance_config();c.update(schema='car_cooperative_motion_v1',alignment_only=True)
    task=CarMotionTask(c,channel_key='39'*32,standalone=True)
    policy=TransportPolicy('39'*32,task.car.config.stage)
    x,y,yaw=initial;command=ZERO;seen=set();resolved=None
    for i in range(8000):
        steady=10+i*.02;ros=1800000000.+steady
        vx,vy=_rotate_xy((command.vx,command.vy),yaw)
        x+=vx*.02;y+=vy*.02;yaw=_wrap(yaw+command.wz*.02)
        car=TargetPose('car01','map',round(ros*1e9),round(steady*1e9),(x,y,task.car.ground_z_at(x,y)),
            (0.,0.,math.sin(yaw/2),math.cos(yaw/2)),True,False)
        uav=replace(car,source='MircoAir',position=(0.,0.,.1),quaternion_xyzw=(0.,0.,0.,1.))
        start=task.phase=='WAIT_START'
        if start:
            resolved=AirGroundSequence(dict(c,schema='air_ground_sequence_v1')).resolve_start(car,uav,car,uav)
            task.bind_resolved_start(resolved)
        status=task.step(steady=steady,ros=ros,car=car,car_velocity=(vx,vy,command.wz),
            frame_revision=c['frame_revision'],graph_ok=True,start=start,
            feedback=dict(sequence=i,issued_steady=steady,issued_ros=ros,output_zero=command==ZERO,
                enabled=policy.active(steady),speed=math.hypot(command.vx,command.vy),yaw_rate=abs(command.wz),fault_bits=0),
            transport=dict(issued_steady=steady,issued_ros=ros,ready=not policy.active(steady),fresh=True,
                local_stop=False,native_fault_bits=0,fault='',armed_epoch=policy.epoch,
                policy_state=(policy.envelope or {}).get('state')))
        assert policy.receive(status['car_control']['signed_authority'],steady,ros)
        command=policy.command if policy.active(steady) else ZERO
        seen.add(status['phase'])
        if resolved:assert ground_approach_clear(car,resolved['ground_approach'])
        if task.phase in task.terminal:break
    assert status['complete'] and status['car_stopped'],status
    assert command==ZERO and task.flight is None
    assert 'ROUTE' not in seen and 'WAIT_TAKEOFF' not in seen
    assert math.dist((x,y),resolved['takeoff_car_world_xyyaw'][:2])<=c['car_arrival_m']


@pytest.mark.parametrize('heading',[0.,1.3,-2.2])
def test_circle_full_owner_chain_continuous_loop_and_original_ground_return(heading,record_property):
    plan=json.loads((ROOT/'plan_b/follow/air_ground_circle.candidate.json').read_text())
    # Virtual arena/flat floor for arbitrary headings, not new physical bounds.
    plan['follow_limits'].update(xmin=-4,xmax=4,ymin=-4,ymax=4)
    plan['live_inputs'].pop('car_ground_plane_map_abc')
    plan['live_inputs']['car_ground_z_map_m']=.1
    h=Plant(initial=(0.,0.,heading),plan=plan)
    dx,dy=_rotate_xy((0.,-.51),heading)
    h.uav=[dx,dy,h.ground_z];h.uyaw=heading;h.goal=h.uav[:];h.yaw_goal=heading
    initial=h.uav[:]
    def refs(data):data.update(car_rigid=data['car'],uav_rigid=data['uav'])
    h.change=refs
    h.until(lambda:h.joint.describe()['complete'] or h.joint.fault,seconds=150.)
    assert h.joint.describe()['complete'] and h.joint.fault is None,h.joint.describe()
    r=h.joint.sequence.resolved_start;origin=r['takeoff_car_world_xyyaw']
    cx,cy=_rotate_xy((0.,.75),heading);center=(origin[0]+cx,origin[1]+cy)
    route=[r for r in h.rows if r['joint']['car_task_phase']=='ROUTE']
    assert set(range(1,49)) <= {r['joint']['car_route_index'] for r in route}
    # No waypoint stop in the interior, traverse all quadrants at cruise speed.
    middle=[r for r in route if 5<=r['joint']['car_route_index']<=43]
    assert min(math.hypot(*r['command'][:2]) for r in middle)>.20
    assert max(abs(math.dist(r['car'][:2],center)-.75) for r in middle)<.05
    positions=[_rotate_xy((r['car'][0]-origin[0],r['car'][1]-origin[1]),-heading) for r in route]
    assert max(p[0] for p in positions)>.70 and min(p[0] for p in positions)<-.70
    assert max(p[1] for p in positions)>1.45
    assert all(abs(_wrap(r['car_yaw']-heading))<.02 for r in route)
    assert math.dist(h.car[:2],origin[:2])<plan['car_arrival_m']
    assert math.dist(h.uav[:2],initial[:2])<.03 and not h.armed and h.out==ZERO
    assert [m.command for m in h.sink.messages if m.get_type()=='COMMAND_LONG']==[176,400,21]
    for a,b in zip(route,route[1:]):
        assert math.hypot(*b['command'][:2])<=plan['car_speed_mps']+1e-9
        assert math.dist(a['command'][:2],b['command'][:2])<=plan['car_acceleration_mps2']*h.dt+1e-7
    descent=[r for r in h.rows if r['joint']['phase']=='DESCEND']
    assert descent
    assert all(b['joint']['target_ned'][2]>=a['joint']['target_ned'][2]-1e-9 for a,b in zip(descent,descent[1:]))
    record_property('circle_metrics',json.dumps(dict(scope='synthetic kinematic plant; no hardware',
        heading_rad=heading,route_seconds=route[-1]['t']-route[0]['t'],
        interior_min_speed_mps=min(math.hypot(*r['command'][:2]) for r in middle),
        interior_max_radial_error_m=max(abs(math.dist(r['car'][:2],center)-.75) for r in middle),
        car_return_error_m=math.dist(h.car[:2],origin[:2]),
        uav_return_error_m=math.dist(h.uav[:2],initial[:2]),complete=True)))
