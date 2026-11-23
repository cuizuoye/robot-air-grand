"""Explicit cooperative route -> existing 4WSE control/authority contracts.

No ROS, sockets, subprocesses or device ownership. The caller supplies real
body-reference poses, wheel feedback and operator lease; no Sign samples are
invented. A live runner must deliver the returned signed envelope to the ONE
existing HardwareBridge. The replay uses its real TransportPolicy in memory.
"""
from collections import deque
from dataclasses import dataclass, replace
import hashlib
import importlib
import json
import math
from pathlib import Path
import sys
import uuid

CAR_ENTRY = Path('/home/czy/vicon_mecanum_ws/candidates/wand_4wse_pose2000_20261105/entry.py')
HOST = CAR_ENTRY.parent / 'host'
PACKAGES = HOST / 'runtime/src'
for directory in (CAR_ENTRY.parents[2]/'src/mecanum_vicon_control',
                  PACKAGES/'mecanum_wand_4wse_control', PACKAGES/'mecanum_wand_4wse_hardware'):
    sys.path.insert(0, str(directory))
for name in ('mecanum_wand_4wse_control', 'mecanum_wand_4wse_hardware'):
    module = importlib.import_module(name)
    if not Path(module.__file__).resolve().is_relative_to(PACKAGES):
        raise ImportError('existing 4WSE pose2000 package required: '+name)

from mecanum_wand_4wse_control.core import Pose, Velocity, wrap_angle
from mecanum_wand_4wse_control.safety_core import Supervisor, FinalGate, ZERO, fresh_envelope
from mecanum_wand_4wse_control.stop_feedback import StopFeedbackInputs
from mecanum_wand_4wse_control.idle_admission import zero_preparation_ready
from mecanum_wand_4wse_control.tracking_config import TrackingSafetyConfig
from mecanum_wand_4wse_control.rotation_servo import coupled_shape, wheel_rpm
from mecanum_wand_4wse_hardware.commissioning import AuthoritySigner
from follow_target_generator import AirGroundSequence, TargetPose, _finite, _yaw, circle_waypoint_passed


class CarRoutePlan:
    """Car-only fixed world route; no invented UAV/platform geometry."""
    def __init__(self, config):
        import copy
        self.config = c = copy.deepcopy(config)
        self.takeoff_index=0
        self.rotation_indices=set()
        if c.get('schema') != 'car_cooperative_motion_v1':
            raise ValueError('explicit car motion plan required')
        if (c.get('car_source') != 'car01' or c.get('frame_id') != 'map'
                or not isinstance(c.get('frame_revision'),str) or not c['frame_revision']):
            raise ValueError('car source and current world revision required')
        if c.get('start_alignment'):
            AirGroundSequence(dict(c,schema='air_ground_sequence_v1'))
            self.route = []  # Resolved once, by the paired owner before CAR_START.
        else:
            self.route = [_finite(c['takeoff_car_world_xyyaw'],3)] + [
                _finite(p,3) for p in c['route_world_xyyaw']]
            if len(self.route)<2:raise ValueError('fixed route after takeoff point required')
        for k in ('car_speed_mps','car_acceleration_mps2','car_arrival_m',
                  'car_yaw_tolerance_rad','stopped_speed_mps','stopped_yaw_rate_rps',
                  'flight_status_timeout_s','maximum_task_seconds'):
            if type(c[k]) not in (float,int) or not math.isfinite(c[k]) or c[k]<=0:
                raise ValueError('positive finite car setting required: '+k)
        if c['maximum_task_seconds']>600:raise ValueError('existing 600 second session limit')


@dataclass(frozen=True)
class FlightProgress:
    """Observed flight-owner handoff, not a takeoff command or timer signal.

    clearance_confirmed comes from measured clearance/stability in the UAV
    owner. Same task id and original clocks bind both handoffs.
    """
    task_id: str
    sequence: int
    issued_steady: float
    issued_ros: float
    armed: bool
    landed: bool
    clearance_confirmed: bool
    pilot_control: bool = False


@dataclass(frozen=True)
class RouteIntent:
    sequence: int
    issued_steady: float
    issued_ros: float
    frame_id: str
    frame_revision: str
    action: str
    route_index: int

    @classmethod
    def from_sequence(cls, status, sequence, steady, ros, plan):
        """Only an index into the reviewed fixed world route, never free goals."""
        action = status['car_action']
        index = status['route_index']
        if action == 'route':
            if (type(index) is not int or not 0 <= index < len(plan.route)
                    or tuple(status['car_goal_map_xyyaw']) != plan.route[index]
                    or status['car_speed_mps'] != plan.config['car_speed_mps']
                    or status['car_acceleration_mps2'] != plan.config['car_acceleration_mps2']):
                raise ValueError('cooperative route differs from loaded plan')
        return cls(sequence, steady, ros, status['frame_id'], status['frame_revision'], action, index)


class RouteInputs(StopFeedbackInputs):
    """Same car/feedback/lease rules; replace only the Sign source requirement."""
    def __init__(self, config, plan):
        super().__init__(config)
        self.plan, self.intent = plan, None
        self.mode = 'cooperative_route'
        self.graph_ok = False
        self.route_index = None
        self.stop_pose_history = deque(maxlen=512)
        self.stop_pose_received = None
        self.stop_geometry_observed = False

    def observe_pose(self, source, pose, source_ros, steady, ros, roll=0., pitch=0.):
        # Motion admissibility and observed standstill are different facts.
        # Keep finite, timestamped geometry even when the motion layer rejects
        # height/tilt. It can confirm a stop, never grant motion permission.
        if source == 'car':
            h=self.stop_pose_history
            valid=(pose.valid() and all(math.isfinite(v) for v in (source_ros,steady,ros,roll,pitch))
                   and -self.c.future_tolerance_s <= ros-source_ros <= self.c.pose_timeout_s)
            row=(source_ros,pose,roll,pitch)
            if not valid or (h and source_ros<=h[-1][0] and row!=h[-1]):
                h.clear();self.stop_pose_received=None
            elif not h or source_ros>h[-1][0]:
                if h and (source_ros-h[-1][0]>self.c.pose_timeout_s
                          or self.pose_step_issue('car',pose,{'pose':h[-1][1]})):
                    h.clear()
                h.append(row);self.stop_pose_received=steady
                while len(h)>2 and h[1][0]<=source_ros-self.c.stationary_window_s+1e-9:h.popleft()
        return super().observe_pose(source,pose,source_ros,steady,ros,roll,pitch)

    def update_stationarity(self, steady, ros):
        self.stop_geometry_observed=False
        if super().update_stationarity(steady,ros):return True
        h=self.stop_pose_history
        if (self.pose_errors.get('car')!='CAR_LIFT_OR_TILT' or len(h)<2
                or self.stop_pose_received is None or not self.feedback_still(steady,ros)
                or not fresh_envelope(self.stop_pose_received,h[-1][0],steady,ros,
                                      self.c.pose_timeout_s,self.c.future_tolerance_s)):
            return False
        span=h[-1][0]-h[0][0]
        if span<self.c.stationary_window_s-1e-9:return False
        # Include XYZ and tilt as well as yaw: lifting/rocking during a stop
        # must not look stationary merely because planar wheel speeds are zero.
        ranges=[max(getattr(r[1],k) for r in h)-min(getattr(r[1],k) for r in h) for k in ('x','y','z')]
        angles=[[wrap_angle(value-origin) for value in values] for values,origin in (
            ([r[1].yaw for r in h],h[0][1].yaw),([r[2] for r in h],h[0][2]),([r[3] for r in h],h[0][3]))]
        still=(math.sqrt(sum(v*v for v in ranges))/span<=.005
               and max(max(v)-min(v) for v in angles)/span<=.02)
        self.stop_geometry_observed=still
        if still:self.stationary_since=steady if self.stationary_since is None else self.stationary_since
        return still

    def observe_intent(self, intent, now, ros):
        if (type(intent) is not RouteIntent or type(intent.sequence) is not int or intent.sequence < 0
                or intent.frame_id != self.plan.config['frame_id']
                or intent.frame_revision != self.plan.config['frame_revision']
                or intent.action not in ('route', 'stop')
                or type(intent.route_index) is not int
                or not (0 <= intent.route_index < len(self.plan.route) or
                        not self.plan.route and intent.action == 'stop' and intent.route_index == 0)):
            self.event('ROUTE_INTENT_INVALID')
            return False
        if self.intent and intent.sequence <= self.intent.sequence:
            if intent.sequence == self.intent.sequence and intent != self.intent:
                self.event('ROUTE_SAME_SEQUENCE_CHANGED')
            return False  # duplicates never renew either clock
        if not fresh_envelope(intent.issued_steady, intent.issued_ros, now, ros, self.c.lease_timeout_s):
            self.event('ROUTE_INTENT_TIMEOUT')
            return False
        if intent.action == 'route':
            expected = 0 if self.route_index is None else self.route_index
            if intent.route_index not in (expected, expected+1) or (self.route_index is None and intent.route_index != 0):
                self.event('ROUTE_INDEX_SKIPPED_OR_REVERSED')
                return False
            self.route_index = intent.route_index
        self.intent = intent
        return True

    def health(self, steady, ros):
        if not self.graph_ok: return 'COMMAND_SOURCE_CONFLICT'
        if self.mode != 'cooperative_route': return 'CONTROL_MODE_CONFLICT'
        if not self.pose_fresh('car', steady, ros): return self.pose_errors.get('car', 'CAR_POSE_TIMEOUT')
        if not self.c.inside(self.poses['car']['pose']): return 'CAR_POSE_INVALID'
        if not self.feedback_fresh(steady, ros): return self.feedback_error or 'FEEDBACK_TIMEOUT'
        if self.actuator_fault_bits: return 'ACTUATOR_FAULT'
        if not self.lease_fresh(steady, ros): return self.lease_error or 'OPERATOR_LEASE_TIMEOUT'
        p = self.intent
        if not p or not fresh_envelope(p.issued_steady, p.issued_ros, steady, ros, self.c.lease_timeout_s):
            return 'ROUTE_INTENT_TIMEOUT'
        return ''


class RouteSession:
    """Straight legs or continuous circle pursuit, ONE existing output limiter."""
    def __init__(self, config, inputs, limits):
        self.inputs, self.config, self.limits = inputs, config, limits
        self.goal = self.car = None
        self.state, self.reason, self.command = 'STANDBY', '', ZERO
        self.last_tick = None
        self.segment_index = None
        self.segment_start = None
        self.cross_track_error_m = None
        self.approach_stage = None
        self.heading_settled_since = None

    def observe(self, source, pose, stamp, *, source_stamp=None):
        if source != 'car': raise ValueError('route mode does not accept Sign')
        self.car = pose

    def engage(self, now):
        if self.car is None: return False, 'CAR_POSE_REQUIRED'
        self.goal, self.command, self.last_tick = self.car, ZERO, now
        self.segment_index=None;self.segment_start=(self.car.x,self.car.y)
        self.cross_track_error_m=None
        self.approach_stage = ('turn_in_place' if self.inputs.plan.config.get('start_alignment')
                               and not self.inputs.plan.config['start_alignment'].get('avoidance')
                               and self.inputs.intent and self.inputs.intent.route_index==0 else None)
        self.heading_settled_since=None
        self.state = 'ACTIVE'
        return True, 'route neutral entry'

    def pause(self, stop=False):
        self.state, self.command = 'PAUSED', ZERO

    def _circle_velocity(self, index, speed, gain):
        # Pursue a point ahead on the reviewed polyline. Progress still comes
        # from measured waypoint arrival, not elapsed time. Approach waypoints
        # never enter this branch; only the final circle point requires a stop.
        route=self.inputs.plan.route
        start=route[index-1][:2];end=route[index][:2]
        dx,dy=end[0]-start[0],end[1]-start[1];length=math.hypot(dx,dy)
        t=max(0.,min(1.,((self.car.x-start[0])*dx+(self.car.y-start[1])*dy)/max(length*length,1e-12)))
        projection=(start[0]+t*dx,start[1]+t*dy)
        self.cross_track_error_m=math.dist((self.car.x,self.car.y),projection)
        points=[projection]+[p[:2] for p in route[index:]]
        remaining=sum(math.dist(a,b) for a,b in zip(points,points[1:]))
        ahead=self.inputs.plan.config['circle_route']['lookahead_m']
        target=points[-1]
        for a,b in zip(points,points[1:]):
            span=math.dist(a,b)
            if span>=ahead and span>1e-12:
                target=tuple(a[k]+(b[k]-a[k])*ahead/span for k in (0,1));break
            ahead-=span
        dx,dy=target[0]-self.car.x,target[1]-self.car.y
        distance=math.hypot(dx,dy)
        # Slow only near the completed loop. Retain endpoint correction even
        # after projecting beyond it, so stop confirmation cannot hide drift.
        demand=min(speed,gain*max(remaining,math.dist((self.car.x,self.car.y),route[-1][:2])))
        return (demand*dx/max(distance,1e-12),demand*dy/max(distance,1e-12))

    def tick(self, now):
        dt = now-self.last_tick
        self.last_tick = now
        if self.state != 'ACTIVE' or dt <= 0: return self.command
        intent = self.inputs.intent
        if not intent or intent.action != 'route':
            self.reason = 'ROUTE_STOP'
            self.pause(True)
            return ZERO
        x,y,yaw = self.inputs.plan.route[intent.route_index]
        self.goal = Pose(x,y,self.goal.z,yaw)
        if self.segment_index!=intent.route_index:
            if intent.route_index>0:self.segment_start=self.inputs.plan.route[intent.route_index-1][:2]
            self.segment_index=intent.route_index
        sx,sy=self.segment_start
        dx,dy=x-sx,y-sy;length=math.hypot(dx,dy)
        gain=self.config.kp_position;speed=self.limits.max_speed_mps
        circle=(self.inputs.plan.config.get('circle_route') and
                intent.route_index>getattr(self.inputs.plan,'takeoff_index',-1))
        if circle:
            wx,wy=self._circle_velocity(intent.route_index,speed,gain)
        elif length>1e-9:
            tx,ty=dx/length,dy/length
            along=(self.car.x-sx)*tx+(self.car.y-sy)*ty
            cross=-(self.car.x-sx)*ty+(self.car.y-sy)*tx
            self.cross_track_error_m=cross
            # Bound forward demand BEFORE combining corrections. Previously
            # a distant goal scaled yaw/lateral corrections almost to zero.
            lateral=max(-speed,min(speed,-gain*cross))
            forward_limit=math.sqrt(max(0.,speed*speed-lateral*lateral))
            forward=max(-forward_limit,min(forward_limit,gain*(length-along)))
            wx,wy=forward*tx-lateral*ty,forward*ty+lateral*tx
        else:
            self.cross_track_error_m=0.
            wx,wy=gain*(x-self.car.x),gain*(y-self.car.y)
            scale=min(1.,speed/max(math.hypot(wx,wy),1e-12));wx*=scale;wy*=scale
        c,s = math.cos(self.car.yaw),math.sin(self.car.yaw)
        wz=max(-self.limits.max_yaw_rate_radps,min(self.limits.max_yaw_rate_radps,2.*wrap_angle(yaw-self.car.yaw)))
        if intent.route_index in getattr(self.inputs.plan,'rotation_indices',set()):
            wx=wy=0.
            # Rotation is a separate reviewed waypoint in swept-clear space.
            if abs(wrap_angle(yaw-self.car.yaw))<=self.inputs.plan.config['car_yaw_tolerance_rad']:wz=0.
        if self.approach_stage == 'turn_in_place' and intent.route_index==0:
            # A far-away start with a large yaw error must not sweep toward the
            # grounded aircraft while turning. Reuse measured yaw/wheel rates
            # and the plan's existing settle interval, not a timed blind turn.
            wx=wy=0.
            cfg=self.inputs.plan.config;feedback=self.inputs.feedback
            heading_ok=abs(wrap_angle(yaw-self.car.yaw))<=cfg['car_yaw_tolerance_rad']
            # Once heading is inside the admitted tolerance, request a stop
            # so measured rates and the shaped command can settle. Continuing
            # proportional yaw here can hold command.wz above the stop limit
            # forever when a small request produces no physical wheel motion.
            if heading_ok:wz=0.
            aligned=(heading_ok
                and self.inputs.car_yaw_rate<=cfg['stopped_yaw_rate_rps']
                and feedback is not None and feedback['yaw_rate']<=cfg['stopped_yaw_rate_rps']
                and feedback['speed']<=cfg['stopped_speed_mps']
                and abs(self.command.wz)<=cfg['stopped_yaw_rate_rps'])
            if not aligned:self.heading_settled_since=None
            elif self.heading_settled_since is None:self.heading_settled_since=now
            elif now-self.heading_settled_since>=cfg['settle_seconds']:
                self.approach_stage='translate_aligned'
                # Rotation may have caused physical drift: start the approach
                # from the measured stopping location, not the old ideal point.
                self.segment_start=(self.car.x,self.car.y)
        desired = Velocity(c*wx+s*wy,-s*wx+c*wy,wz)
        self.command = coupled_shape(desired,self.command,dt,self.limits)
        return self.command


class RouteGate(FinalGate):
    def __init__(self, config, plan, limits):
        super().__init__(config)
        self.inputs = RouteInputs(config,plan)
        self.limits = limits

    def reference_valid(self, frame):
        if self.reference_sample is None:
            car = self.inputs.poses['car']['pose']
            return (frame.command == ZERO and math.dist((frame.goal.x,frame.goal.y,frame.goal.z),
                        (car.x,car.y,car.z)) <= .02 and abs(wrap_angle(frame.goal.yaw-car.yaw)) <= .025)
        p = self.inputs.intent
        if not p or p.action != 'route': return False
        x,y,yaw = self.inputs.plan.route[p.route_index]
        origin_z = self.reference_sample[0].z
        return (abs(frame.goal.x-x) < 1e-9 and abs(frame.goal.y-y) < 1e-9
                and abs(frame.goal.z-origin_z) < 1e-9 and abs(wrap_angle(frame.goal.yaw-yaw)) < 1e-9)

    def tick(self, steady, ros):
        previous, stamp = self.output, self.last_applied_stamp
        output = super().tick(steady,ros)
        f = self.frame
        if f and f.enabled and not self.fault and not self.hard_fault:
            dt = .02 if stamp is None else f.issued_steady-stamp
            c = self.limits
            if (math.hypot(output.vx,output.vy) > c.max_speed_mps+1e-9
                    or abs(output.wz) > c.max_yaw_rate_radps+1e-9
                    or max(map(abs,wheel_rpm(output))) > c.max_wheel_rpm+1e-6
                    or math.hypot(output.vx-previous.vx,output.vy-previous.vy) > c.max_accel_mps2*dt+1e-7
                    or abs(output.wz-previous.wz) > c.max_yaw_accel_radps2*dt+1e-7):
                self.trip('ROUTE_COMMAND_LIMIT')
        return self.output


class CarRouteAdapter:
    """Injectable bridge input; construction and step NEVER send anything.

    car is a calibrated body-reference TargetPose (not raw car01 unless its
    origin/axes match). feedback is the existing plant_feedback payload. lease
    is the existing Lease, generated by the real operator/session owner. graph_ok
    must come from the exclusive owner audit, not a static live default.
    """
    def __init__(self, config, *, channel_key):
        self.plan = (CarRoutePlan(config) if config.get('schema')=='car_cooperative_motion_v1'
                     else AirGroundSequence(config))
        if Path(config['car_entry']).resolve() != CAR_ENTRY:
            raise ValueError('confirmed car entry required')
        profile_path = HOST/'profiles/stage6.LONG600_NOT_APPROVED.json'
        self.profile_sha256 = hashlib.sha256(profile_path.read_bytes()).hexdigest()
        profile = json.loads(profile_path.read_text())
        c = TrackingSafetyConfig(**profile['safety'])
        binding=config.get('live_inputs')
        self.ground_reference_source='legacy_profile_offline_only'
        self.car_ground_z_map_m=c.ground_z_m
        self.car_ground_plane_map_abc=None
        self.geometry_observation=None
        if binding is not None:
            # The pose callback already maps the rebuilt rigid to the chassis
            # reference. Its resting world Z must use that SAME reference;
            # neither deck height above the floor nor the old rigid's Z fits.
            z=binding.get('car_ground_z_map_m')
            if type(z) not in (int,float) or not math.isfinite(z):
                raise ValueError('联合计划缺少有效的 car_ground_z_map_m；不能沿用旧刚体高度')
            self.car_ground_z_map_m=z
            self.ground_reference_source='joint_plan.live_inputs.car_ground_z_map_m'
            if binding.get('car_ground_plane_map_abc') is not None:
                self.car_ground_plane_map_abc=_finite(binding['car_ground_plane_map_abc'],3)
                self.ground_reference_source='joint_plan.live_inputs.car_ground_plane_map_abc'
        self.config=c
        # Translate ONLY private control Z, using the configured local ground
        # surface if present. World poses for following/landing stay unchanged.
        # No online fitting/tracking of height: an actual lift stays a residual.
        self.control_z_offset_m=c.ground_z_m-self.car_ground_z_map_m
        speed,accel = config['car_speed_mps'],config['car_acceleration_mps2']
        if speed > c.max_speed_mps or accel > c.max_accel_mps2:
            raise ValueError('route demand exceeds existing 4WSE envelope')
        self.limits = replace(c.follower_config(),max_speed_mps=speed,max_accel_mps2=accel,
                            max_yaw_rate_radps=min(.2,c.max_yaw_rate_radps),
                            max_yaw_accel_radps2=min(.4,c.max_yaw_accel_radps2))
        self.supervisor = Supervisor(c)
        self.supervisor.inputs = RouteInputs(c,self.plan)
        self.supervisor.follower = RouteSession(c.follower_config(),self.supervisor.inputs,self.limits)
        self.gate = RouteGate(c,self.plan,self.limits)
        self.signer = AuthoritySigner(channel_key,c.stage)
        self.first_fault = ''
        self.used = False
        self.paused_route_index = None

    def ground_z_at(self, x, y):
        if self.car_ground_plane_map_abc is None:return self.car_ground_z_map_m
        a,b,c=self.car_ground_plane_map_abc
        return a*x+b*y+c

    def prepare(self, steady, ros, *, continue_after_index=None):
        if self.used:
            s=self.supervisor;p=s.inputs.intent
            # Continue only the planned takeoff stop, never fault recovery.
            takeoff=getattr(self.plan,'takeoff_index',0)
            allowed=(isinstance(self.plan,CarRoutePlan) and type(continue_after_index) is int and continue_after_index==takeoff
                and self.paused_route_index==takeoff and not self.first_fault
                and s.state=='PAUSED' and s.reason=='ROUTE_STOP' and not s.fault_latched
                and not self.gate.fault and not self.gate.hard_fault
                and p is not None and p.action=='route' and p.route_index==takeoff+1)
            if not allowed:return False, 'route already consumed; no automatic restart'
        return self.supervisor.request_start(steady,ros)

    def step(self, *, steady, ros, car, frame_revision, feedback, lease, intent, graph_ok):
        s,g = self.supervisor,self.gate
        try:
            if (type(car) is not TargetPose or car.source != self.plan.config['car_source']
                    or car.frame_id != self.plan.config['frame_id']
                    or frame_revision != self.plan.config['frame_revision']
                    or car.tracked is not True or car.occluded is not False):
                raise ValueError('CAR_IDENTITY_FRAME_OR_VISIBILITY')
            if any(type(v) not in (int,float) or not math.isfinite(v) for v in (*car.position,*car.quaternion_xyzw)):
                raise ValueError('CAR_NONFINITE')
            x,y,z,w = car.quaternion_xyzw
            norm = math.sqrt(x*x+y*y+z*z+w*w)
            if not .99 <= norm <= 1.01: raise ValueError('CAR_QUATERNION')
            x,y,z,w = (v/norm for v in (x,y,z,w))
            roll = math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
            pitch = math.asin(max(-1.,min(1.,2*(w*y-z*x))))
            yaw = math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
            ground=self.ground_z_at(*car.position[:2])
            height=car.position[2]-ground
            self.geometry_observation=dict(world_z_m=car.position[2],expected_ground_z_m=ground,
                height_above_reference_m=height,roll_rad=roll,pitch_rad=pitch,
                lift_limit_m=self.config.lift_tolerance_m,tilt_limit_rad=self.config.tilt_limit_rad,
                ground_reference_source=self.ground_reference_source)
            pose = Pose(car.position[0],car.position[1],height+self.config.ground_z_m,yaw)
            if car.received_ns > round(steady*1e9): raise ValueError('CAR_RECEIPT_FUTURE')
            # Only reconcile <= 1 ns float/integer conversion roundoff after
            # the integer comparison above. Never renew an old receipt.
            received = min(steady,car.received_ns*1e-9)
            for core in (s,g):
                core.inputs.graph_ok = graph_ok is True
                core.inputs.observe_pose('car',pose,car.stamp_ns*1e-9,received,ros,roll,pitch)
                core.inputs.observe_feedback(steady=steady,ros=ros,**feedback)
                core.inputs.observe_lease(lease,steady,ros)
                core.inputs.observe_intent(intent,steady,ros)
        except (ValueError,TypeError,KeyError,AttributeError) as exc:
            s.request_stop(steady,str(exc),fault=True)
            g.trip(str(exc))
        if type(intent) is RouteIntent and intent.action == 'stop' and s.state in ('ARMED','ACTIVE'):
            self.paused_route_index=s.inputs.route_index
            s.request_stop(steady,'ROUTE_STOP')
        s.observe_gate(steady=steady,ros=ros,epoch=g.primed_epoch,fault=g.fault,hard_fault=g.hard_fault)
        frame = s.tick(steady,ros)
        self.used |= frame.enabled
        g.receive(frame,steady,ros)
        command = g.tick(steady,ros)
        if not self.first_fault:
            if s.fault_latched: self.first_fault = s.reason
            elif g.fault and g.fault != 'COMMAND_TIMEOUT': self.first_fault = g.fault
        permit = bool(frame.epoch and frame.epoch == g.primed_epoch and not g.fault
                      and not g.hard_fault and frame.state in ('ARMED','ACTIVE'))
        wire = self.signer.encode(command,steady,ros,epoch=frame.epoch,state=frame.state,
                                  permit=permit,estop=g.hard_fault)
        ready = bool(s.state in ('STANDBY','PAUSED') and not s.fault_latched and
                     s.stop_confirmed and s.ready_since is not None
                     and steady-s.ready_since >= s.c.ready_window_s-1e-9 and not s.inputs.health(steady,ros))
        # Supervisor.stop_confirmed is also a remembered start prerequisite.
        # Never expose that old True as current standstill while driving.
        stopped = bool(s.state in ('STANDBY','PAUSED','FAULT') and s.stop_confirmed
                       and s.inputs.update_stationarity(steady,ros))
        return dict(state=s.state,ready=ready,stop_confirmed=stopped,
                    car_ground_z_map_m=self.car_ground_z_map_m,
                    internal_z_offset_m=(self.config.ground_z_m-self.geometry_observation['expected_ground_z_m']
                        if self.geometry_observation else self.control_z_offset_m),
                    geometry=self.geometry_observation,cross_track_error_m=s.follower.cross_track_error_m,
                    approach_stage=s.follower.approach_stage,
                    stop_observation=('stationary_geometry_and_wheels_after_motion_rejection'
                        if stopped and s.inputs.stop_geometry_observed else 'original_pose_and_wheel_window'),
                    reason=self.first_fault or s.inputs.health(steady,ros) or s.reason, fault=self.first_fault,
                    command_body_mps_rps=(command.vx,command.vy,command.wz),
                    signed_authority=wire,permit=permit,actual_sent_to_car=0,
                    route_index=s.inputs.route_index,live_transport_connected=False)


class CarMotionTask:
    """Loaded approach -> stopped takeoff wait -> route -> stopped landing wait.

    Uses the original car Supervisor/Gate/signing chain. Opens no resources
    and never publishes. The existing bridge owner supplies measurements and
    transmits the returned signed envelope; UAV motor control stays elsewhere.
    """
    terminal=('COMPLETE','ABORTED')

    def __init__(self, config, *, channel_key, standalone=False):
        if type(standalone) is not bool:raise ValueError('explicit standalone car mode required')
        self.standalone=standalone
        self.alignment_only=config.get('alignment_only') is True
        if self.alignment_only and (not standalone or not config.get('start_alignment',{}).get('avoidance')):
            raise ValueError('alignment_only_requires_standalone_avoidance_plan')
        if standalone and config.get('start_alignment') and not self.alignment_only:
            raise ValueError('automatic_start_requires_paired_uav_measurements')
        self.plan=CarRoutePlan(config)
        self.car=CarRouteAdapter(config,channel_key=channel_key)
        self.task_id=uuid.uuid4().hex
        self.phase='WAIT_READY';self.index=0;self.started_at=None
        self.counter=0;self.last_car=None;self.first_fault=None
        self.flight=None;self.waiting_for='car_readiness';self.resume_requested=False

    def bind_resolved_start(self, resolved):
        """Bind the reviewed approach prefix without changing flight route indices."""
        if self.started_at is not None or self.phase not in ('WAIT_READY','WAIT_START'):
            raise ValueError('approach_already_started')
        approach=resolved.get('ground_approach')
        prefix=approach['waypoints'] if approach else [resolved['takeoff_car_world_xyyaw']]
        for plan in (self.plan,self.car.plan):
            plan.route=[tuple(p) for p in prefix+list(resolved['route_world_xyyaw'])]
            plan.takeoff_index=len(prefix)-1
            plan.rotation_indices=set(approach['rotation_indices'] if approach else ())

    def _flight(self, value, steady, ros):
        if value is not None:
            if (type(value) is not FlightProgress or value.task_id!=self.task_id
                    or type(value.sequence) is not int or value.sequence<0
                    or any(type(v) is not bool for v in (value.armed,value.landed,
                            value.clearance_confirmed,value.pilot_control))):
                raise ValueError('FLIGHT_HANDOFF_IDENTITY_OR_STATUS_INVALID')
            if self.flight and value.sequence==self.flight.sequence and value!=self.flight:
                raise ValueError('FLIGHT_HANDOFF_DUPLICATE_CHANGED')
            if self.flight is None or value.sequence>self.flight.sequence:self.flight=value
        f=self.flight
        return f if f and fresh_envelope(f.issued_steady,f.issued_ros,steady,ros,
            self.plan.config['flight_status_timeout_s']) else None

    def _arrived(self, car, velocity):
        c=self.plan.config;goal=self.plan.route[self.index]
        pass_through=(bool(c.get('circle_route')) and self.phase=='ROUTE'
                      and self.plan.takeoff_index<self.index<len(self.plan.route)-1)
        close=(circle_waypoint_passed(car.position,self.plan.route[self.index-1],goal,
                   c['car_arrival_m'],c['circle_route']['lookahead_m']) if pass_through else
               math.dist(car.position[:2],goal[:2])<=c['car_arrival_m'])
        return (close
            and abs(wrap_angle(_yaw(car.quaternion_xyzw)-goal[2]))<=c['car_yaw_tolerance_rad']
            and (pass_through or math.hypot(*velocity[:2])<=c['stopped_speed_mps']
                 and abs(velocity[2])<=c['stopped_yaw_rate_rps']))

    def step(self, *, steady, ros, car, car_velocity, frame_revision, feedback,
             transport, graph_ok, flight=None, start=False, cancel=False):
        from mecanum_wand_4wse_control.safety_core import Lease
        self.counter+=1;c=self.plan.config
        link_ok=False
        try:
            if type(start) is not bool or type(cancel) is not bool:
                raise ValueError('explicit car start/stop intent required')
            velocity=_finite(car_velocity,3)
            link_ok=bool(fresh_envelope(transport['issued_steady'],transport['issued_ros'],steady,ros,.5)
                and transport['fresh'] is True and transport['local_stop'] is False
                and transport['native_fault_bits']==0 and not transport['fault'])
            if self.standalone and flight is not None:
                raise ValueError('standalone car route does not accept UAV handoffs')
            f=None if self.standalone else self._flight(flight,steady,ros)
            if self.phase not in self.terminal:
                if cancel:raise ValueError('OPERATOR_STOP')
                if self.started_at is not None:
                    if not link_ok:raise ValueError('CAR_TRANSPORT_LOST')
                    if steady-self.started_at>c['maximum_task_seconds']:raise ValueError('CAR_TASK_TIMEOUT')
                    if f and f.pilot_control:raise ValueError('UAV_PILOT_TAKEOVER')
                if self.phase in ('WAIT_READY','WAIT_START'):
                    ready=bool(self.last_car and self.last_car['ready'] and link_ok
                               and zero_preparation_ready(transport))
                    self.phase='WAIT_START' if ready else 'WAIT_READY'
                    self.waiting_for='explicit_start' if ready else 'car_readiness'
                    if start and ready:
                        if not self.plan.route:raise ValueError('automatic_start_not_resolved')
                        if not self.standalone and (f is None or f.armed or not f.landed):
                            raise ValueError('CARRIED_UAV_DISARMED_LANDED_REQUIRED')
                        self.started_at=steady;self.phase='TO_TAKEOFF';self.waiting_for=None
                elif self.phase=='TO_TAKEOFF':
                    if not self.standalone and (f is None or f.armed or not f.landed):
                        raise ValueError('CARRIED_UAV_STATE_LOST')
                    if self._arrived(car,velocity):
                        if self.index<self.plan.takeoff_index:self.index+=1
                        else:
                            self.phase=('WAIT_FINISH' if self.alignment_only else
                                        'WAIT_ROUTE' if self.standalone else 'WAIT_TAKEOFF')
                            self.waiting_for='measured_car_stop'
                elif self.phase in ('WAIT_TAKEOFF','WAIT_ROUTE'):
                    stopped=bool(self.last_car and self.last_car['stop_confirmed'])
                    self.waiting_for=('route_ready' if self.standalone else 'uav_clearance') if stopped else 'measured_car_stop'
                    if stopped and (self.standalone or f and f.armed and not f.landed and f.clearance_confirmed):
                        self.index=self.plan.takeoff_index+1;self.phase='ROUTE';self.waiting_for=None;self.resume_requested=True
                elif self.phase=='ROUTE':
                    if not self.standalone and (f is None or not f.armed or f.landed):raise ValueError('AIRBORNE_UAV_STATE_LOST')
                    if self._arrived(car,velocity):
                        if self.index+1<len(self.plan.route):self.index+=1
                        else:
                            self.phase='WAIT_FINISH' if self.standalone else 'WAIT_LANDING'
                            self.waiting_for='measured_car_stop'
                elif self.phase in ('WAIT_LANDING','WAIT_FINISH'):
                    stopped=bool(self.last_car and self.last_car['stop_confirmed'])
                    self.waiting_for=('route_complete' if self.standalone else 'uav_landed_and_disarmed') if stopped else 'measured_car_stop'
                    if stopped and (self.standalone or f and f.landed and not f.armed):
                        self.phase='COMPLETE';self.waiting_for=None
        except (ValueError,TypeError,KeyError,AttributeError) as exc:
            self.first_fault=self.first_fault or str(exc)
            self.phase='STOPPING';self.waiting_for='measured_car_stop'

        action='route' if self.phase in ('TO_TAKEOFF','ROUTE') and not self.first_fault else 'stop'
        s,g=self.car.supervisor,self.car.gate
        held=bool(action=='route' and link_ok and s.epoch and g.primed_epoch==s.epoch
            and transport.get('armed_epoch')==s.epoch and
            (transport.get('ready') is True if s.state=='ARMED' else
             s.state=='ACTIVE' and transport.get('policy_state')=='ACTIVE'))
        lease=Lease('car_cooperative_task',self.counter,steady,ros,held)
        intent=RouteIntent(self.counter,steady,ros,c['frame_id'],c['frame_revision'],action,self.index)
        self.last_car=self.car.step(steady=steady,ros=ros,car=car,frame_revision=frame_revision,
            feedback=feedback,lease=lease,intent=intent,graph_ok=graph_ok is True and link_ok)
        health=s.inputs.health(steady,ros) if self.started_at is not None else ''
        if self.last_car['fault'] or health:
            self.first_fault=self.first_fault or self.last_car['fault'] or health;self.phase='STOPPING'
        if self.phase=='COMPLETE' and not self.last_car['stop_confirmed']:
            self.phase='WAIT_FINISH' if self.standalone else 'WAIT_LANDING'
            self.waiting_for='measured_car_stop'
        if self.first_fault and self.last_car['stop_confirmed']:
            self.phase='ABORTED';self.waiting_for=None
        if action=='route' and self.last_car['ready'] and not self.first_fault:
            ok,reason=self.car.prepare(steady,ros,continue_after_index=self.plan.takeoff_index if self.resume_requested else None)
            if not ok:
                self.first_fault=reason;self.phase='STOPPING'
            else:self.resume_requested=False
        return self.status()

    def status(self):
        return dict(phase=self.phase,task_id=self.task_id,fault=self.first_fault,
            standalone=self.standalone,
            waiting_for=self.waiting_for,route_index=self.index,
            takeoff_car_world_xyyaw=self.plan.route[self.plan.takeoff_index] if self.plan.route else None,
            car_goal_map_xyyaw=self.plan.route[self.index] if self.phase in ('TO_TAKEOFF','ROUTE') else None,
            car_stopped=bool(self.last_car and self.last_car['stop_confirmed']),
            car_control=self.last_car,complete=self.phase=='COMPLETE',
            actual_sent_to_car=0,actual_sent_to_fc=0,devices_opened=0,
            scope=('standalone car route; no UAV control or fabricated handoffs' if self.standalone else
                   'injected car controller; observed flight handoffs required; no resource owner'))
