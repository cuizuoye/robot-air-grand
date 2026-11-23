"""Offline, transport-free horizontal references for an already airborne UAV.

Not an estimator, flight authorizer, ROS node or MAVLink sender. Self-localization
remains in the existing Vicon/PX4 path. FollowTargetGenerator keeps fixed world
height. AirGroundSequence adds explicit platform-relative mission intentions;
it does not dispatch them. Limits bound references, not actual tracking error.
"""
from dataclasses import dataclass, asdict
from enum import Enum
import math
import heapq

# Normal landing references only; PX4 still owns contact detection and disarm.
NORMAL_DESCENT_SPEED_MPS = .05
LAND_HANDOFF_TOLERANCE_M = .02
LAND_HANDOFF_MAX_VERTICAL_SPEED_MPS = .08


def _box_segment_clear(a, b, half):
    """Exact slab intersection with an open axis-aligned forbidden rectangle."""
    lo, hi = 0., 1.
    for x, y, h in zip(a, b, half):
        h -= 1e-9  # touching the planned padded boundary is permitted
        d = y-x
        if abs(d) < 1e-12:
            if abs(x) >= h:return True
        else:
            t0,t1=sorted(((-h-x)/d,(h-x)/d))
            lo=max(lo,t0);hi=min(hi,t1)
            if lo>=hi:return True
    return not (hi>0. and lo<1. and lo<hi)


def _box_detour(start, goal, half, inside):
    """Shortest visible polyline around one padded rectangle, within site bounds."""
    nodes=[tuple(start),tuple(goal),(-half[0],-half[1]),(-half[0],half[1]),
           (half[0],-half[1]),(half[0],half[1])]
    if not inside(start) or not inside(goal):return None
    if any(abs(p[0])<half[0]-1e-9 and abs(p[1])<half[1]-1e-9 for p in (start,goal)):
        return None
    todo=[(0.,0)];cost={0:0.};prev={}
    while todo:
        distance,i=heapq.heappop(todo)
        if distance>cost[i]+1e-12:continue
        if i==1:
            path=[]
            while i:path.append(nodes[i]);i=prev[i]
            return list(reversed(path))
        for j,p in enumerate(nodes):
            if j==i or not inside(p) or not _box_segment_clear(nodes[i],p,half):continue
            value=distance+math.dist(nodes[i],p)
            if value<cost.get(j,float('inf'))-1e-12:
                cost[j]=value;prev[j]=i;heapq.heappush(todo,(value,j))
    return None


def ground_avoidance_geometry(settings):
    """Conservative half extents about each calibrated control reference."""
    if not isinstance(settings,dict):raise ValueError('approach_geometry_required')
    result=[]
    for name in ('car','uav'):
        try:size=_finite(settings.get(name+'_size_m'),2)
        except ValueError as exc:raise ValueError('approach_geometry_required: '+name+'_size_m') from exc
        offset=settings.get(name+'_reference_margin_m')
        if min(size)<=0 or type(offset) not in (float,int) or not math.isfinite(offset) or offset<0:
            raise ValueError('approach_geometry_required')
        result.append(tuple(v/2+offset for v in size))
    margin=settings.get('path_margin_m')
    if type(margin) not in (float,int) or not math.isfinite(margin) or margin<=0:
        raise ValueError('approach_geometry_required')
    return *result,margin


def ground_approach_clear(car, approach):
    """Measured footprint clearance during ground approach, including yaw."""
    center=approach['obstacle_position_map_m'];yaw=approach['obstacle_yaw_rad']
    x,y=_rotate_xy((car.position[0]-center[0],car.position[1]-center[1]),-yaw)
    a,b=approach['car_half_extents_m'];u,v=approach['uav_half_extents_m']
    theta=_yaw(car.quaternion_xyzw)-yaw;c,s=abs(math.cos(theta)),abs(math.sin(theta))
    return abs(x)>=u+c*a+s*b+.02 or abs(y)>=v+s*a+c*b+.02


def plan_ground_approach(car, uav, goal, settings, limits, arrival):
    """Translate with fixed heading, turn in open space, then approach left goal.

    No hardware I/O. One UAV exclusion footprint only; site bounds are never
    expanded. Extents include uncertainty between body reference and center.
    Each translation uses a Minkowski rectangle; the turn uses swept circles.
    """
    car_half,uav_half,margin=ground_avoidance_geometry(settings)
    if margin<arrival+.02:raise ValueError('approach_margin_below_arrival_allowance')
    yaw=_yaw(uav.quaternion_xyzw);cyaw=_yaw(car.quaternion_xyzw)
    center=uav.position[:2]
    def local(p):return _rotate_xy((p[0]-center[0],p[1]-center[1]),-yaw)
    def world(p):
        x,y=_rotate_xy(p,yaw);return (center[0]+x,center[1]+y)
    def inside(p):return limits.contains(world(p))
    def extent(angle,padding):
        c,s=abs(math.cos(angle-yaw)),abs(math.sin(angle-yaw))
        return (uav_half[0]+c*car_half[0]+s*car_half[1]+padding,
                uav_half[1]+s*car_half[0]+c*car_half[1]+padding)
    start=local(car.position);end=local(goal)
    raw=extent(cyaw,.02)
    if abs(start[0])<raw[0] and abs(start[1])<raw[1]:
        raise ValueError('approach_start_overlaps_vehicle_envelope')
    initial=extent(cyaw,margin);final=extent(goal[2],margin)
    if abs(end[0])<final[0] and abs(end[1])<final[1]:
        raise ValueError('approach_final_spacing_too_small')
    # A close but non-overlapping start may retreat straight out through the
    # nearest face. Preserve its current yaw; never turn beside the aircraft.
    starts=[(start,[])]
    if abs(start[0])<initial[0] and abs(start[1])<initial[1]:
        starts=[]
        for axis in (0,1):
            for sign in (-1,1):
                point=list(start);point[axis]=sign*initial[axis];point=tuple(point)
                if inside(point) and _box_segment_clear(start,point,raw):
                    starts.append((point,[point]))
    best=None
    # A rotation-independent exclusion radius protects every intermediate yaw.
    radius=math.hypot(*car_half)+math.hypot(*uav_half)+margin+arrival
    turn_needed=abs(_wrap(cyaw-goal[2]))>1e-6
    # Avoid a long retreat for an already nearly parallel car. The rotation
    # arc is enclosed by initial extents plus r*|delta yaw|, not endpoints only.
    angle=abs(_wrap(cyaw-goal[2]))
    sweep=tuple(v+math.hypot(*car_half)*angle for v in initial)
    if turn_needed and angle<=.17 and (abs(start[0])>=sweep[0] or abs(start[1])>=sweep[1]):
        direct=_box_detour(start,end,final,inside)
        if direct is not None:
            legs=[(*world(start),goal[2])]+[(*world(p),goal[2]) for p in direct]
            score=sum(math.dist(a[:2],b[:2]) for a,b in zip([car.position]+legs,legs))
            best=(score,legs,[0])
    stages=[(radius*math.cos(i*math.pi/16),radius*math.sin(i*math.pi/16)) for i in range(32)]
    if math.hypot(*start)>=radius:stages.append(start)
    for escaped,prefix in starts:
        if not turn_needed:
            direct=_box_detour(escaped,end,initial,inside)
            if direct is not None:
                legs=[(*world(p),cyaw) for p in prefix+direct]
                score=sum(math.dist(a[:2],b[:2]) for a,b in zip([car.position]+legs,legs))
                if best is None or score<best[0]:best=(score,legs,[])
            continue
        for stage in stages:
            first=_box_detour(escaped,stage,initial,inside)
            last=_box_detour(stage,end,final,inside)
            if first is None or last is None:continue
            legs=[(*world(p),cyaw) for p in prefix+first]
            turn=len(legs);legs.append((*world(stage),goal[2]))
            legs += [(*world(p),goal[2]) for p in last]
            score=sum(math.dist(a[:2],b[:2]) for a,b in zip([car.position]+legs,legs))
            if best is None or score<best[0]:best=(score,legs,[turn])
    if best is None:raise ValueError('approach_no_clear_path_in_task_area')
    return dict(waypoints=best[1],rotation_indices=best[2],length_m=best[0],
                car_half_extents_m=car_half,uav_half_extents_m=uav_half,
                path_margin_m=margin,turn_clearance_radius_m=radius,
                obstacle_position_map_m=tuple(center),obstacle_yaw_rad=yaw,
                physical_validated=False)


class Mode(str, Enum):
    HOVER = 'HOVER'
    WAND_FOLLOW = 'WAND_FOLLOW'
    CAR_FOLLOW_ABOVE = 'CAR_FOLLOW_ABOVE'
    CAR_FOLLOW_OFFSET = 'CAR_FOLLOW_OFFSET'


def _finite(values, count):
    if (not isinstance(values, (tuple, list)) or len(values) != count
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in values)):
        raise ValueError('finite_vector_required')
    return tuple(float(v) for v in values)


@dataclass(frozen=True)
class TargetPose:
    """World-frame rigid/reference point; xyzw quaternion. Header != exposure.

    stamp_ns and now_source_ns share the ROS/source clock; received_ns and
    now_ns share host MONOTONIC. Neither clock may be substituted for the other.
    The caller must supply the genuine subject/point identity and validity;
    a PoseStamped without occlusion metadata is not proof of visibility.
    """
    source: str
    frame_id: str
    stamp_ns: int
    received_ns: int
    position: tuple
    quaternion_xyzw: tuple
    tracked: bool = False
    occluded: bool = True


@dataclass(frozen=True)
class Limits:
    xmin: float
    xmax: float
    ymin: float
    ymax: float
    speed_mps: float
    acceleration_mps2: float
    target_age_s: float = .2
    loss_timeout_s: float = 1.
    max_step_s: float = .1

    def __post_init__(self):
        _finite(tuple(self.__dict__.values()), 9)
        if not (self.xmin < self.xmax and self.ymin < self.ymax
                and min(self.speed_mps, self.acceleration_mps2, self.target_age_s,
                        self.loss_timeout_s, self.max_step_s) > 0
                and self.loss_timeout_s >= self.target_age_s):
            raise ValueError('invalid_offline_limits')

    def contains(self, xy):
        return self.xmin <= xy[0] <= self.xmax and self.ymin <= xy[1] <= self.ymax


class FollowTargetGenerator:
    """Four modes, one bounded XY governor; never changes flight safety gates.

    Per-axis caps are total caps/sqrt(2), so Euclidean speed/acceleration stay
    within total limits, including diagonal motion and target/mode changes.
    A stopping envelope bounds the REFERENCE inside an explicit rectangle.
    Target loss brakes smoothly, latches after a bounded interval and returns
    advisory text only. A latched fault requires a new offline instance.
    """
    def __init__(self, *, initial_xy, ground_z_map_m, height_m, yaw_map_rad,
                 limits, wand_source, car_source, self_source='MircoAir', frame_id='map'):
        self.xy = _finite(initial_xy, 2)
        ground, height, self.yaw = _finite((ground_z_map_m, height_m, yaw_map_rad), 3)
        if (type(limits) is not Limits or not limits.contains(self.xy) or height <= 0
                or not frame_id or any(not isinstance(x, str) or not x for x in
                                      (wand_source, car_source, self_source))
                or len({wand_source, car_source, self_source}) != 3):
            raise ValueError('explicit_distinct_sources_height_and_area_required')
        self.z = ground + height
        self.limits, self.frame_id = limits, frame_id
        self.sources = {'wand': wand_source, 'car': car_source}
        self.mode, self.hold_xy, self.offset = Mode.HOVER, self.xy, (0., 0.)
        self.velocity = (0., 0.)
        self.last_ns = self.last_source_now = self.loss_since = None
        self.seen = {}
        self.fault = None

    def set_mode(self, mode, *, hold_xy=None, offset_xy=(0., 0.)):
        if self.fault:
            raise ValueError('offline_fault_latched')
        mode = Mode(mode)
        offset = _finite(offset_xy, 2)
        if mode is Mode.HOVER:
            # 未来接入任务时由既有有效自身定位显式提供，不使用杆/车的位置。
            hold = _finite(hold_xy, 2) if hold_xy is not None else self.xy
            if not self.limits.contains(hold):
                raise ValueError('hover_outside_area')
            self.hold_xy = hold
        self.mode, self.offset = mode, offset
        # Mode switches do not reset velocity, loss deadline or source history.

    def _target(self, sample, now_ns, now_source_ns):
        if self.mode is Mode.HOVER:
            return self.hold_xy, None
        kind = 'wand' if self.mode is Mode.WAND_FOLLOW else 'car'
        if type(sample) is not TargetPose or sample.source != self.sources[kind]:
            raise ValueError('target_missing_or_wrong_identity')
        if sample.frame_id != self.frame_id:
            raise ValueError('target_frame_mismatch')
        if sample.tracked is not True or sample.occluded is not False:
            raise ValueError('target_untracked_or_occluded')
        if any(type(t) is not int or t <= 0 for t in (sample.stamp_ns, sample.received_ns)):
            raise ValueError('invalid_target_timestamp')
        ages = (now_source_ns-sample.stamp_ns, now_ns-sample.received_ns)
        if any(age < 0 or age > self.limits.target_age_s*1e9 for age in ages):
            raise ValueError('target_stale_or_future')
        p = _finite(sample.position, 3)
        q = _finite(sample.quaternion_xyzw, 4)
        norm = math.sqrt(sum(v*v for v in q))
        if not .99 <= norm <= 1.01:
            raise ValueError('invalid_target_quaternion')
        previous = self.seen.get(sample.source)
        if previous:
            if sample.stamp_ns < previous.stamp_ns or sample.received_ns < previous.received_ns:
                raise ValueError('target_clock_regression')
            if sample.stamp_ns == previous.stamp_ns and sample != previous:
                raise ValueError('conflicting_duplicate_target')
        self.seen[sample.source] = sample
        xy = p[:2]
        if self.mode is Mode.CAR_FOLLOW_OFFSET:
            x, y, z, w = (v/norm for v in q)
            # Use the projected car forward axis, not roll/pitch or UAV yaw.
            fx, fy = 1-2*(y*y+z*z), 2*(x*y+w*z)
            if math.hypot(fx, fy) < .1:
                raise ValueError('car_heading_unobservable')
            yaw = math.atan2(fy, fx)
            c, s = math.cos(yaw), math.sin(yaw)
            dx, dy = self.offset
            xy = (xy[0]+c*dx-s*dy, xy[1]+s*dx+c*dy)
        if not self.limits.contains(xy):
            raise ValueError('target_outside_area')
        return xy, {'source_age_ns': ages[0], 'receive_age_ns': ages[1],
                    'source': sample.source, 'stamp_ns': sample.stamp_ns,
                    'stamp_semantics': 'source_header_not_verified_exposure'}

    def step(self, *, now_ns, now_source_ns, target=None, target_velocity_xy=(0., 0.)):
        if any(type(t) is not int or t <= 0 for t in (now_ns, now_source_ns)):
            raise ValueError('explicit_integer_clocks_required')
        if self.fault:
            raise ValueError('offline_fault_latched:'+self.fault)
        dt = 0 if self.last_ns is None else (now_ns-self.last_ns)*1e-9
        if (self.last_ns is not None and (dt <= 0 or dt > self.limits.max_step_s)
                or self.last_source_now is not None and now_source_ns < self.last_source_now):
            self.fault = 'replay_or_source_clock_gap_regression'
            raise ValueError(self.fault)
        self.last_ns, self.last_source_now = now_ns, now_source_ns
        reason, metadata = None, None
        try:
            goal, metadata = self._target(target, now_ns, now_source_ns)
            self.loss_since = None
        except ValueError as exc:
            reason = str(exc)
            goal = self.xy
            if self.loss_since is None:
                self.loss_since = now_ns
            if now_ns-self.loss_since >= self.limits.loss_timeout_s*1e9:
                self.fault = 'target_loss_timeout:'+reason
        # Measured target velocity removes the governor's ~v/2 steady lag.
        # It passes through the SAME speed, acceleration and boundary limits;
        # stale/invalid targets brake and must never retain feed-forward.
        feedforward = _finite(target_velocity_xy, 2)
        if self.mode is Mode.HOVER or reason or self.fault:
            feedforward = (0., 0.)
        a = self.limits.acceleration_mps2/math.sqrt(2)
        vmax = self.limits.speed_mps/math.sqrt(2)
        old_v = self.velocity
        next_v, next_xy = [], []
        bounds = ((self.limits.xmin, self.limits.xmax), (self.limits.ymin, self.limits.ymax))
        for p, v, g, ff, (lo, hi) in zip(self.xy, old_v, goal, feedforward, bounds):
            if not dt:
                nv = v
            else:
                # p_next=p+v_next*dt; future braking <= v_next**2/(2*a).
                left = -a*dt+math.sqrt((a*dt)**2+2*a*max(0., p-lo))
                right = -a*dt+math.sqrt((a*dt)**2+2*a*max(0., hi-p))
                low, high = max(-vmax, v-a*dt, -left), min(vmax, v+a*dt, right)
                if low > high+1e-10:
                    self.fault = 'reference_braking_envelope_infeasible'
                    raise ValueError(self.fault)
                wanted = 0. if reason or self.fault else ff+2*(g-p)
                if self.mode is Mode.HOVER and not reason and not self.fault:
                    # A fixed landing point needs the same stopping-distance
                    # envelope as an area edge. Proportional pursuit alone
                    # brakes too late and overshoots on the return from the car.
                    # Preserve current velocity and the acceleration bound;
                    # never snap the reference or alter moving-car feedforward.
                    distance = g-p
                    stop_speed = -a*dt+math.sqrt((a*dt)**2+2*a*abs(distance))
                    wanted = math.copysign(min(abs(wanted),stop_speed),distance)
                nv = min(high, max(low, wanted))
            next_v.append(nv)
            next_xy.append(p+nv*dt)
        self.xy, self.velocity = tuple(next_xy), tuple(next_v)
        return {'mode': self.mode.value, 'frame_id': self.frame_id,
                'position_map_m': (*self.xy, self.z), 'yaw_map_rad': self.yaw,
                'velocity_xy_mps': self.velocity,
                'acceleration_xy_mps2': tuple((v-o)/dt if dt else 0. for v, o in zip(self.velocity, old_v)),
                'desired_xy_m': goal, 'source': metadata,
                'status': 'AUTHORITY_ABSENT_FAULT' if self.fault else 'TARGET_BRAKING' if reason else 'REFERENCE_ONLY',
                'reason': self.fault or reason, 'at_monotonic_ns': now_ns,
                'source_clock_now_ns': now_source_ns, 'flight_authorized': False,
                'live_dispatch_permitted': False, 'actual_sent_to_fc': 0,
                'scope': 'offline_reference_only_not_takeoff_or_aircraft_safety_authority'}


@dataclass(frozen=True)
class HorizontalMotionSpec:
    """One finite demonstration in the heading frozen at takeoff, metres/seconds."""
    pattern: str = 'directions'
    extent_m: float = .4
    speed_mps: float = .15
    acceleration_mps2: float = .2
    dwell_seconds: float = 1.
    arrival_radius_m: float = .35

    def __post_init__(self):
        values = (self.extent_m, self.speed_mps, self.acceleration_mps2,
                  self.dwell_seconds, self.arrival_radius_m)
        _finite(values, 5)
        if (self.pattern not in ('directions', 'circle', 'directions_circle') or min(values) <= 0
                or self.arrival_radius_m >= self.extent_m):
            raise ValueError('Motion needs directions/circle/directions_circle, positive limits and arrival radius below excursion')

    @classmethod
    def from_dict(cls, value):
        if not isinstance(value, dict) or set(value) != set(cls.__dataclass_fields__):
            raise ValueError('Explicit complete horizontal_motion settings required')
        return cls(**value)

    def to_dict(self):
        return asdict(self)


class HorizontalMotion:
    """Smooth relative targets, no transport. The existing MissionCore owns flight.

    Each leg stops before the next one. A circle has a radial entry and exit;
    its centre is the frozen hover target, never the drifting measured pose.
    """
    vertical_tolerance_m = .10
    settling_speed_mps = .20

    def __init__(self, spec, *, hover_seconds=3.):
        self.spec = spec if isinstance(spec, HorizontalMotionSpec) else HorizontalMotionSpec.from_dict(spec)
        if type(hover_seconds) not in (int, float) or not math.isfinite(hover_seconds) or hover_seconds <= 0:
            raise ValueError('Positive finite hover duration required')
        self.hover_seconds = hover_seconds
        self.segments = []
        r = self.spec.extent_m
        point = (0., 0.)  # forward, left in the frozen takeoff heading
        self._add('center_hold', 'line', point, point, hover_seconds)
        if self.spec.pattern in ('directions', 'directions_circle'):
            for name, end in (('forward', (r, 0.)), ('backward', (-r, 0.)),
                              ('left', (0., r)), ('right', (0., -r))):
                self._add(name, 'line', point, end, self.spec.dwell_seconds)
                self._add('return_'+name, 'line', end, point, self.spec.dwell_seconds)
        if self.spec.pattern in ('circle', 'directions_circle'):
            self._add('circle_entry', 'line', point, (r, 0.), self.spec.dwell_seconds)
            self._add('circle_ccw', 'circle', (r, 0.), (r, 0.), self.spec.dwell_seconds)
            self._add('circle_return', 'line', (r, 0.), point, self.spec.dwell_seconds)
        self.segments[-1]['dwell_s'] = hover_seconds
        self.center = self.yaw = self.target_ned = None
        self.index = 0
        self.since = self.settled_since = self.last_t = None
        self.complete = False

    def _add(self, name, kind, start, end, dwell):
        s = self.spec
        # Quintic time law: max s'=1.875, max |s''|=10/sqrt(3).
        # Circle acceleration includes both tangential and centripetal terms.
        length = 2*math.pi*s.extent_m if kind == 'circle' else math.dist(start, end)
        acc_term = length*10/math.sqrt(3)
        if kind == 'circle':
            acc_term += s.extent_m*(2*math.pi*1.875)**2
        duration = max(length*1.875/s.speed_mps, math.sqrt(acc_term/s.acceleration_mps2))
        if not math.isfinite(duration):
            raise ValueError('Motion duration must remain finite')
        self.segments.append(dict(name=name, kind=kind, start=start, end=end,
                                  duration_s=duration, dwell_s=dwell))

    @property
    def nominal_seconds(self):
        return sum(s['duration_s']+s['dwell_s'] for s in self.segments)

    @property
    def maximum_excursion_m(self):
        return self.spec.extent_m+self.spec.arrival_radius_m+.10

    def _clock(self, t):
        if not math.isfinite(t) or t < 0 or self.last_t is not None and t < self.last_t:
            raise ValueError('Finite monotonic motion clock required')
        self.last_t = t

    def start(self, t, *, center_ned, yaw_ned):
        if self.since is not None:
            raise ValueError('Motion cannot restart or rebase')
        self._clock(t)
        self.center = _finite(center_ned, 3)
        self.yaw, = _finite((yaw_ned,), 1)
        self.target_ned = self.center
        self.since = t

    def sample(self, index, elapsed):
        if self.center is None:
            raise ValueError('Frozen centre and heading required')
        segment = self.segments[index]
        duration = segment['duration_s']
        u = min(1., max(0., elapsed/duration)) if duration else 1.
        q = u*u*u*(10+u*(-15+6*u))
        if segment['kind'] == 'circle':
            theta = 2*math.pi*q
            forward, left = self.spec.extent_m*math.cos(theta), self.spec.extent_m*math.sin(theta)
        else:
            forward, left = tuple(a+(b-a)*q for a, b in zip(segment['start'], segment['end']))
        # NED heading: forward=(cos(yaw),sin(yaw)), left=(sin(yaw),-cos(yaw)).
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (self.center[0]+c*forward+s*left, self.center[1]+s*forward-c*left, self.center[2])

    def arrived(self, position, velocity):
        return (math.dist(position[:2], self.target_ned[:2]) <= self.spec.arrival_radius_m
                and abs(position[2]-self.target_ned[2]) <= self.vertical_tolerance_m
                and math.dist(velocity, (0., 0., 0.)) <= self.settling_speed_mps)

    def step(self, t, *, position_ned, velocity_ned):
        self._clock(t)
        position, velocity = _finite(position_ned, 3), _finite(velocity_ned, 3)
        if self.since is None:
            raise ValueError('Motion has not started')
        if self.complete:
            return self.target_ned
        segment = self.segments[self.index]
        self.target_ned = self.sample(self.index, t-self.since)
        # Clock reaching a waypoint is not proof that the aircraft reached it.
        # Existing whole-flight deadline handles a vehicle that cannot settle.
        if t-self.since >= segment['duration_s'] and self.arrived(position, velocity):
            if self.settled_since is None:
                self.settled_since = t
            if t-self.settled_since >= segment['dwell_s']:
                if self.index == len(self.segments)-1:
                    self.complete = True
                else:
                    self.index += 1
                    self.since, self.settled_since = t, None
        else:
            self.settled_since = None
        return self.target_ned

    def describe(self):
        return dict(settings=self.spec.to_dict(),
                    arrival_criteria=dict(horizontal_tolerance_m=self.spec.arrival_radius_m,
                        vertical_tolerance_m=self.vertical_tolerance_m, settling_speed_mps=self.settling_speed_mps),
                    stage='complete' if self.complete else self.segments[self.index]['name'],
                    center_ned_m=self.center, target_ned_m=self.target_ned,
                    nominal_motion_seconds=self.nominal_seconds,
                    maximum_excursion_m=self.maximum_excursion_m,
                    direction_reference='frozen_takeoff_heading', yaw_behavior='fixed',
                    stages=[dict(name=s['name'], duration_s=s['duration_s'], dwell_s=s['dwell_s'])
                            for s in self.segments])


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw(q):
    x, y, z, w = _finite(q, 4)
    if not .99 <= math.sqrt(x*x+y*y+z*z+w*w) <= 1.01:
        raise ValueError('invalid_target_quaternion')
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def _rotate_xy(xy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c*xy[0]-s*xy[1], s*xy[0]+c*xy[1]


def circle_local_route(diameter_m, segments):
    """Start on the circle, center to the left, forward tangent, fixed yaw."""
    if (type(diameter_m) not in (float,int) or not math.isfinite(diameter_m)
            or diameter_m<=0 or type(segments) is not int or not 24<=segments<=180):
        raise ValueError('invalid_circle_diameter_or_segments')
    r=diameter_m/2
    return [(r*math.sin(2*math.pi*i/segments),r*(1-math.cos(2*math.pi*i/segments)))
            for i in range(1,segments)]+[(0.,0.)]


def circle_waypoint_passed(position, start, goal, arrival_m, lookahead_m):
    """Do not strand continuous pursuit behind a waypoint missed laterally."""
    if math.dist(position[:2],goal[:2])<=arrival_m:return True
    dx,dy=goal[0]-start[0],goal[1]-start[1];length=math.hypot(dx,dy)
    if length<1e-12:return False
    px,py=position[0]-start[0],position[1]-start[1]
    along=(px*dx+py*dy)/length
    cross=abs(-px*dy+py*dx)/length
    return along>=length and cross<=max(arrival_m,lookahead_m)


class AirGroundSequence:
    """Measured-state sequencing, without ROS, sockets or actuator ownership.

    The car adapter must convert route/stop intentions through its existing
    single writer. The UAV adapter must use MissionCore's existing arm/land
    transactions. Those live adapters are NOT implemented by this planner.
    All positions are calibrated body references in one explicit, Z-up frame.
    A level, rigid platform is assumed; landing restores relative XY and yaw,
    while PX4 continues to stabilize roll/pitch. No time-based fake completion.
    """
    terminal = ('COMPLETE', 'HOLD', 'PILOT_CONTROL')

    def __init__(self, config):
        import copy
        self.config = c = copy.deepcopy(config)
        if c.get('schema') != 'air_ground_sequence_v1':
            raise ValueError('unknown_air_ground_plan')
        if (not c.get('frame_id') or not c.get('frame_revision') or
                not c.get('car_source') or not c.get('uav_source') or
                c['car_source'] == c['uav_source']):
            raise ValueError('explicit_shared_frame_and_distinct_subjects_required')
        if c.get('landing_heading') not in ('initial_relative_car', 'initial_world'):
            raise ValueError('explicit_landing_heading_required')
        self.limits = Limits(**c['follow_limits'])
        self.ground_start = c.get('landing_mode') == 'initial_ground'
        self.overhead = c.get('follow_position', 'initial_offset') == 'above_car'
        if c.get('follow_position', 'initial_offset') not in ('initial_offset', 'above_car'):
            raise ValueError('unknown_follow_position')
        if self.overhead and not self.ground_start:
            raise ValueError('above_car_follow_requires_separate_ground_landing')
        if self.overhead:
            _finite(c['airborne_offset_car_forward_left_m'], 2)
            if not 0 < c['follow_arrival_tolerance_m'] < 1:
                raise ValueError('invalid_follow_arrival_tolerance')
        if type(c.get('follow_velocity_feedforward', False)) is not bool:
            raise ValueError('explicit_follow_velocity_feedforward_required')
        if type(c.get('controller_velocity_feedforward', False)) is not bool:
            raise ValueError('explicit_controller_velocity_feedforward_required')
        if 'landing_position_feedback_gain_s_inv' in c:
            gain=c['landing_position_feedback_gain_s_inv']
            if (type(gain) not in (int,float) or not math.isfinite(gain) or gain<=0
                    or not c.get('controller_velocity_feedforward',False) or not self.ground_start):
                raise ValueError('ground_landing_feedback_requires_positive_gain_and_velocity_output')
        for name in ('landing_integral_gain_s_inv2','landing_velocity_damping'):
            value=c.get(name,0.)
            if (type(value) not in (int,float) or not math.isfinite(value) or value<0
                    or value and not c.get('landing_position_feedback_gain_s_inv')):
                raise ValueError('invalid_ground_landing_feedback_tuning')
        self.start_alignment = c.get('start_alignment')
        if 'route_heading_map_rad' in c:
            heading=c['route_heading_map_rad']
            if (type(heading) not in (int,float) or not math.isfinite(heading)
                    or not self.ground_start or not self.start_alignment):
                raise ValueError('fixed_route_heading_requires_finite_ground_start_direction')
        if c.get('landing_mode', 'platform') not in ('platform', 'initial_ground'):
            raise ValueError('unknown_landing_mode')
        if self.start_alignment is not None:
            a = self.start_alignment
            if not self.ground_start or a.get('mode') != 'stationary_uav':
                raise ValueError('stationary_uav_alignment_requires_ground_landing')
            offset = _finite(a['uav_rigid_offset_car_forward_left_m'], 2)
            local = [_finite(p, 2) for p in c['route_local_forward_left_m']]
            if offset[0] != 0 or offset[1] >= 0 or not local or local[-1] != (0., 0.):
                raise ValueError('right_side_offset_and_closed_local_route_required')
            if 'route_world_xyyaw' in c or 'takeoff_car_world_xyyaw' in c:
                raise ValueError('automatic_start_must_not_mix_old_world_waypoints')
        elif self.ground_start:
            raise ValueError('ground_start_requires_stationary_uav_alignment')
        if 'circle_route' in c:
            circle=c['circle_route']
            expected=circle_local_route(circle['diameter_m'],circle['segments'])
            if (not self.start_alignment or 'route_heading_map_rad' in c
                    or circle.get('direction')!='counterclockwise'
                    or circle.get('heading')!='fixed'
                    or len(local)!=len(expected)
                    or any(math.dist(a,b)>1e-9 for a,b in zip(local,expected))):
                raise ValueError('circle_requires_live_heading_and_closed_left_circle')
            lookahead=circle.get('lookahead_m')
            if (type(lookahead) not in (int,float) or not math.isfinite(lookahead)
                    or not 0<lookahead<circle['diameter_m']/2):
                raise ValueError('invalid_circle_lookahead')
        self.route = [] if self.start_alignment else [_finite(p, 3) for p in c['route_world_xyyaw']]
        if not self.start_alignment and (not self.route or any(not self.limits.contains(p[:2]) for p in self.route)):
            raise ValueError('route_required_inside_explicit_area')
        if ('takeoff_car_world_xyyaw' in c and
                not self.limits.contains(_finite(c['takeoff_car_world_xyyaw'],3)[:2])):
            raise ValueError('takeoff_point_required_inside_explicit_area')
        if not self.ground_start:
            self.platform_offset = _finite(c['platform_offset_car_m'], 3)
            self.platform_size = _finite(c['platform_size_m'], 2)
            self.footprint = _finite(c['uav_landing_footprint_m'], 2)
        names = ('clearance_m','car_speed_mps','car_acceleration_mps2',
                 'car_arrival_m','car_yaw_tolerance_rad','stopped_speed_mps',
                 'stopped_yaw_rate_rps','settle_seconds','climb_speed_mps',
                 'descent_speed_mps','yaw_rate_rps','vertical_tolerance_m',
                 'landing_xy_tolerance_m','landing_yaw_tolerance_rad')
        if not self.ground_start:names += ('edge_margin_m',)
        if 'landing_horizontal_speed_mps' in c:names += ('landing_horizontal_speed_mps',)
        for name in ('landing_settling_speed_mps','landing_settle_seconds'):
            if name in c:names += (name,)
        if type(c.get('ground_descent_continuous',False)) is not bool:
            raise ValueError('ground_descent_continuous_must_be_boolean')
        if any(type(c[k]) not in (float,int) or not math.isfinite(c[k]) or c[k] <= 0 for k in names):
            raise ValueError('positive_finite_cooperative_settings_required')
        if (type(c['land_handoff_height_m']) not in (float,int) or
                not math.isfinite(c['land_handoff_height_m']) or c['land_handoff_height_m'] < 0):
            raise ValueError('nonnegative_land_handoff_height_required')
        if (not self.ground_start and min(*self.platform_size,*self.footprint) <= 0 or
                c['car_speed_mps'] >= self.limits.speed_mps/math.sqrt(2) or
                c['land_handoff_height_m'] >= c['clearance_m']):
            raise ValueError('invalid_platform_or_follow_speed_budget')
        self.phase = 'WAIT_READY'
        self.fault = None
        self.index = 0
        self.last_ns = self.last_source_ns = None
        self.seen = {}
        self.settled_since = None
        self.generator = None
        self.target = self.yaw = self.relative_xy = self.relative_z = None
        self.relative_yaw = self.initial_yaw = None
        self.initial_car = self.initial_uav = None
        self.loading_pose = None
        self.resolved_start = None
        self.land_intent_issued = False
        self.descent_started = False
        self.descent_paused = False
        self.descent_hold_z = None
        self.target_velocity = (0., 0., 0.)
        self.landing_car_anchor = None
        self.landing_waiting_for = None
        self.landing_integral = (0.,0.)
        self.landing_feedback_stamp_ns = None
        self.complete = False

    def _pose(self, sample, source, now, source_now):
        if type(sample) is not TargetPose or sample.source != source or sample.frame_id != self.config['frame_id']:
            raise ValueError('subject_or_world_frame_mismatch')
        if sample.tracked is not True or sample.occluded is not False:
            raise ValueError('subject_not_visible:'+source)
        if any(type(t) is not int or t <= 0 for t in (sample.stamp_ns,sample.received_ns)):
            raise ValueError('invalid_subject_timestamp')
        if any(t < 0 or t > self.limits.target_age_s*1e9 for t in
               (source_now-sample.stamp_ns, now-sample.received_ns)):
            raise ValueError('subject_stale:'+source)
        _finite(sample.position,3); _yaw(sample.quaternion_xyzw)
        previous = self.seen.get(source)
        if previous and (sample.stamp_ns < previous.stamp_ns or sample.received_ns < previous.received_ns or
                         sample.stamp_ns == previous.stamp_ns and sample != previous):
            raise ValueError('subject_clock_or_duplicate_changed:'+source)
        self.seen[source] = sample

    def _transition(self, phase):
        self.phase = phase
        self.settled_since = None
        if phase not in ('ALIGN','DESCEND'):
            self.landing_integral = (0.,0.)
            self.landing_feedback_stamp_ns = None

    def _settled(self, condition, now, duration_s=None):
        if not condition:
            self.settled_since = None
            return False
        if self.settled_since is None:
            self.settled_since = now
        return now-self.settled_since >= (self.config['settle_seconds'] if duration_s is None else duration_s)*1e9

    def _point(self, car):
        if self.ground_start and self.phase in ('WAIT_CAR_STOP','ALIGN','DESCEND','LAND','COMPLETE'):
            return tuple(self.resolved_start['uav_ground_position_map_m'])
        dx,dy = _rotate_xy(self.relative_xy, _yaw(car.quaternion_xyzw))
        z = self.resolved_start['uav_ground_position_map_m'][2] if self.ground_start else car.position[2]+self.relative_z
        return car.position[0]+dx, car.position[1]+dy, z

    def resolve_start(self, car, uav, car_rigid, uav_rigid):
        """Freeze a session anchor; no I/O, no change to calibration or world limits.

        Rigid centers define the configured separation. Calibrated body heading
        defines forward/left. Car control still targets the chassis reference.
        An explicit avoidance profile supports approaches from all sides.
        Legacy profiles retain their already validated left-side segment.
        """
        if not self.start_alignment or self.resolved_start is not None or self.phase not in ('WAIT_READY','WAIT_START'):
            raise ValueError('automatic_start_already_frozen_or_not_configured')
        for body,rigid in ((car,car_rigid),(uav,uav_rigid)):
            if (type(rigid) is not TargetPose or rigid.source != body.source or
                    rigid.frame_id != body.frame_id or not rigid.tracked or rigid.occluded or
                    (rigid.stamp_ns,rigid.received_ns) != (body.stamp_ns,body.received_ns)):
                raise ValueError('matching_current_rigid_and_body_samples_required')
            _finite(rigid.position,3)
        yaw = _yaw(uav.quaternion_xyzw)
        offset = _finite(self.start_alignment['uav_rigid_offset_car_forward_left_m'],2)
        forward,left = _rotate_xy((car_rigid.position[0]-uav_rigid.position[0],
                                   car_rigid.position[1]-uav_rigid.position[1]),-yaw)
        # A stationary reference fluctuates by sub-mm amounts at this boundary.
        # Keep the target unchanged; permit only 5 mm of start-side measurement
        # slack, never a crossing from the UAV's right side. A nearly aligned
        # car may also retreat outward: its straight segment must increase
        # rigid-center clearance from the first instant, not cut closer first.
        outward = (abs(_wrap(_yaw(car.quaternion_xyzw)-yaw)) <= .17 and
                    forward*(-offset[0]-forward)+left*(-offset[1]-left) >= 0)
        avoidance=self.start_alignment.get('avoidance')
        if not avoidance and (left <= 0 or (left + .005 < -offset[1] and not outward)):
            raise ValueError('car_must_start_on_clear_left_approach_side')
        delta = _rotate_xy(offset,yaw)
        rigid_goal = tuple(uav_rigid.position[i]-delta[i] for i in range(2))
        # Rotate the existing calibrated rigid->body lever arm with the car.
        arm = _rotate_xy(tuple(car.position[i]-car_rigid.position[i] for i in range(2)),
                         _wrap(yaw-_yaw(car.quaternion_xyzw)))
        goal = (rigid_goal[0]+arm[0],rigid_goal[1]+arm[1],yaw)
        approach=(plan_ground_approach(car,uav,goal,avoidance,self.limits,self.config['car_arrival_m'])
                  if avoidance else None)
        relative = _rotate_xy(tuple(uav.position[i]-goal[i] for i in range(2)),-yaw)
        # A user-frozen task direction controls route translation only. Keep
        # live aircraft heading for side-by-side approach and landing heading;
        # never reuse the direction snapshot's historical XY/Z as an origin.
        route_yaw = self.config.get('route_heading_map_rad',yaw)
        route = []
        for local in self.config['route_local_forward_left_m']:
            dx,dy = _rotate_xy(local,route_yaw)
            route.append((goal[0]+dx,goal[1]+dy,yaw))
        airborne_relative = (_finite(self.config['airborne_offset_car_forward_left_m'], 2)
                             if self.overhead else relative)
        follow_offset = _rotate_xy(airborne_relative,yaw)
        points = [car.position[:2],uav.position[:2],goal[:2]] + [p[:2] for p in route]
        if approach:points += [p[:2] for p in approach['waypoints']]
        points += [(p[0]+follow_offset[0],p[1]+follow_offset[1]) for p in route]
        if any(not self.limits.contains(p) for p in points):
            raise ValueError('resolved_car_or_uav_route_outside_task_area')
        result = dict(takeoff_car_world_xyyaw=goal,route_world_xyyaw=route,
            ground_approach=approach,
            # Include BOTH aircraft ground point and car path in preparation
            # geometry. This is diagnostic, not automatic site expansion.
            reference_path_bounds_map_m=dict(xmin=min(p[0] for p in points),
                xmax=max(p[0] for p in points),ymin=min(p[1] for p in points),
                ymax=max(p[1] for p in points)),
            route_heading_map_rad=route_yaw,
            uav_rigid_offset_car_forward_left_m=offset,follow_body_offset_car_m=relative,
            airborne_follow_offset_car_m=airborne_relative,
            uav_ground_position_map_m=uav.position,uav_heading_map_rad=yaw,
            uav_rigid_position_map_m=uav_rigid.position,car_rigid_goal_map_xy_m=rigid_goal,
            car_source_stamp_ns=car.stamp_ns,uav_source_stamp_ns=uav.stamp_ns)
        self.route = route
        self.resolved_start = result
        return result

    def _inside_platform(self, car, uav):
        c = self.config
        cyaw = _yaw(car.quaternion_xyzw)
        relative = _rotate_xy((uav.position[0]-car.position[0],uav.position[1]-car.position[1]), -cyaw)
        yaw = _wrap(_yaw(uav.quaternion_xyzw)-cyaw)
        co,si = abs(math.cos(yaw)),abs(math.sin(yaw))
        occupied = (co*self.footprint[0]+si*self.footprint[1],
                    si*self.footprint[0]+co*self.footprint[1])
        room = [(size-foot)/2-c['edge_margin_m'] for size,foot in zip(self.platform_size,occupied)]
        return all(r > 0 and abs(x-o) <= r for x,o,r in zip(relative,self.platform_offset[:2],room))

    def _capture(self, car, uav):
        if not self.ground_start and not self._inside_platform(car,uav):
            raise ValueError('initial_placement_outside_usable_platform')
        if self.ground_start and self.resolved_start is None:
            raise ValueError('automatic_start_not_resolved')
        self.initial_car = car
        self.initial_uav = uav
        # Loaded approach preserves the placement BEFORE carriage motion, but
        # freezes the actual takeoff pose only AFTER arriving and stopping.
        reference_car,reference_uav=self.loading_pose or (car,uav)
        cyaw = _yaw(reference_car.quaternion_xyzw)
        self.relative_xy = _rotate_xy((reference_uav.position[0]-reference_car.position[0],
                                      reference_uav.position[1]-reference_car.position[1]),-cyaw)
        self.relative_z = reference_uav.position[2]-reference_car.position[2]
        if self.ground_start:
            # Do not turn the residual car arrival error into a new setpoint.
            self.relative_xy = tuple(self.resolved_start['airborne_follow_offset_car_m'])
        if not self.ground_start and self.relative_z < self.platform_offset[2]:
            raise ValueError('uav_body_reference_below_platform_surface')
        self.initial_yaw = _yaw(reference_uav.quaternion_xyzw)
        self.yaw = _yaw(uav.quaternion_xyzw)
        self.relative_yaw = _wrap(self.initial_yaw-cyaw)
        if self.ground_start:self.relative_yaw = 0.
        self.target = tuple(uav.position)
        self.generator = FollowTargetGenerator(initial_xy=uav.position[:2],
            ground_z_map_m=uav.position[2],height_m=self.config['clearance_m'],
            yaw_map_rad=self.yaw,limits=self.limits,wand_source='unused_cooperative_wand',
            car_source=self.config['car_source'],self_source=self.config['uav_source'],
            frame_id=self.config['frame_id'])
        self.generator.set_mode(Mode.CAR_FOLLOW_OFFSET,offset_xy=self.relative_xy)

    def step(self, *, now_ns, now_source_ns, frame_revision, car, uav,
             car_velocity, uav_velocity, car_ready, uav_ready, car_stopped,
             armed, landed, start=False, pilot_control=False,
             car_route_index=None, car_route_finished=False):
        # Explicit inputs are observations from the eventual adapters, not
        # inferred from a successful start command or an elapsed timer.
        previous_target_velocity = self.target_velocity
        self.target_velocity = (0., 0., 0.)
        if self.phase in self.terminal:
            return self.status()
        if pilot_control is True:
            self.landing_waiting_for = None
            self._transition('PILOT_CONTROL')
            return self.status()
        c = self.config
        try:
            if any(type(t) is not int or t <= 0 for t in (now_ns,now_source_ns)):
                raise ValueError('invalid_cooperative_clock')
            dt = 0. if self.last_ns is None else (now_ns-self.last_ns)*1e-9
            # Full FC diagnostics can withhold observations while the owner
            # continues polling at ground idle. No reference is integrated in
            # WAIT_*: resume from fresh poses rather than treating that wait as
            # an airborne control gap. Clock reversal and pose ages still apply.
            ground_wait = (self.phase in ('WAIT_READY','WAIT_START')
                           and armed is False and landed is True)
            if (self.last_ns is not None and (dt <= 0 or
                    dt > self.limits.max_step_s and not ground_wait) or
                    self.last_source_ns is not None and now_source_ns < self.last_source_ns):
                raise ValueError('cooperative_clock_gap_or_regression')
            self.last_ns,self.last_source_ns = now_ns,now_source_ns
            if frame_revision != c['frame_revision']:
                raise ValueError('world_or_rigid_revision_changed')
            if any(type(v) is not bool for v in (car_ready,uav_ready,car_stopped,armed,landed,start,pilot_control)):
                raise ValueError('explicit_observed_status_required')
            self._pose(uav,c['uav_source'],now_ns,now_source_ns)
            self._pose(car,c['car_source'],now_ns,now_source_ns)
            cv,uv = _finite(car_velocity,3),_finite(uav_velocity,3)
            stopped = car_stopped and math.hypot(*cv[:2]) <= c['stopped_speed_mps'] and abs(cv[2]) <= c['stopped_yaw_rate_rps']
            # Adjacent ground landing does not target the car deck. After the
            # initial measured stop, retain that confirmation while fresh car
            # positions remain at the stop anchor. Pose differentiation noise
            # must not repeatedly restart landing. Platform rules stay strict.
            if (self.ground_start and self.landing_car_anchor is not None
                    and self.phase in ('ALIGN','DESCEND','LAND')):
                anchor = self.landing_car_anchor
                stopped = (math.dist(car.position[:2],anchor.position[:2]) <= c['car_arrival_m']
                           and abs(_wrap(_yaw(car.quaternion_xyzw)-_yaw(anchor.quaternion_xyzw))) <= c['car_yaw_tolerance_rad'])
            if not self.limits.contains(uav.position[:2]) or not self.limits.contains(car.position[:2]):
                raise ValueError('observed_position_outside_task_area')
            if self.phase in ('WAIT_READY','WAIT_START'):
                ready = car_ready and uav_ready and stopped and landed and not armed
                self.phase = 'WAIT_START' if ready else 'WAIT_READY'
                if start and ready:
                    self._capture(car,uav)
                    self._transition('TAKEOFF')
                return self.status()
            if not car_ready or not uav_ready:
                raise ValueError('paired_backend_not_ready')
            point = self._point(car)
            desired_yaw = (_wrap(_yaw(car.quaternion_xyzw)+self.relative_yaw)
                           if c['landing_heading']=='initial_relative_car' else self.initial_yaw)
            if self.ground_start:desired_yaw = self.resolved_start['uav_heading_map_rad']
            self.yaw = _wrap(self.yaw+max(-c['yaw_rate_rps']*dt,min(c['yaw_rate_rps']*dt,_wrap(desired_yaw-self.yaw))))
            if self.phase in ('TAKEOFF', 'RENDEZVOUS'):
                if (math.dist(car.position,self.initial_car.position) > c['car_arrival_m'] or
                        math.hypot(*cv[:2]) > c['stopped_speed_mps'] or abs(cv[2]) > c['stopped_yaw_rate_rps'] or
                        abs(_wrap(_yaw(car.quaternion_xyzw)-_yaw(self.initial_car.quaternion_xyzw))) > c['car_yaw_tolerance_rad']):
                    raise ValueError('car_moved_before_uav_clearance' if self.phase == 'TAKEOFF'
                                     else 'car_moved_before_overhead_rendezvous')
            if self.phase == 'TAKEOFF':
                if not stopped:
                    # A temporarily incomplete standstill window is not
                    # observed motion. Hold this target and keep the car
                    # disabled; actual displacement above still stops flight.
                    self.settled_since=None
                    return self.status()
                # Mode/arm warmup is not ascent time. Keep the deck target
                # until an actual armed observation; otherwise the first
                # airborne setpoint would jump to the full clearance.
                z = (min(point[2]+c['clearance_m'],self.target[2]+c['climb_speed_mps']*dt)
                     if armed else self.target[2])
                self.target = (*self.initial_uav.position[:2],z)
                clear = (armed and not landed and uav.position[2] >= point[2]+c['clearance_m']-c['vertical_tolerance_m']
                         and abs(uv[2]) <= c['stopped_speed_mps'] and z >= point[2]+c['clearance_m']-1e-9)
                if self._settled(clear,now_ns):
                    self._transition('RENDEZVOUS' if self.overhead else 'FOLLOW')
                return self.status()
            if self.phase == 'LAND':
                # Land hands vertical control to PX4; it is not a motor-stop
                # command. Near-deck position or an ACK never means touchdown.
                self.landing_waiting_for = ('touchdown' if not landed else
                    'px4_auto_disarm' if armed else 'platform_position_confirmation')
                if not stopped:
                    raise ValueError('car_moved_during_land_handoff')
                if landed and not armed:
                    inside = (math.dist(uav.position[:2],point[:2]) <= c['landing_xy_tolerance_m']
                              if self.ground_start else self._inside_platform(car,uav))
                    if (not inside or abs(uav.position[2]-point[2]) > c['vertical_tolerance_m'] or
                            abs(_wrap(_yaw(uav.quaternion_xyzw)-desired_yaw)) > c['landing_yaw_tolerance_rad']):
                        raise ValueError('landed_outside_ground_target' if self.ground_start else 'landed_outside_platform')
                    self.complete = True
                    self.landing_waiting_for = None
                    self._transition('COMPLETE')
                return self.status()
            if not armed or landed:
                raise ValueError('airborne_state_lost_before_land')
            if self.phase == 'RENDEZVOUS':
                # An incomplete standstill window is not measured car motion.
                # Reuse HOVER to brake smoothly; keep stepping the governor so
                # a prolonged wait neither ages its clock nor hides a jump.
                # No new grace timer: fresh poses and actual motion above decide.
                if not stopped:
                    self.settled_since = None
                    if self.generator.mode is not Mode.HOVER:
                        self.generator.set_mode(Mode.HOVER,hold_xy=self.target[:2])
                else:
                    self.generator.set_mode(Mode.CAR_FOLLOW_OFFSET,offset_xy=self.relative_xy)
            if self.ground_start and self.phase in ('WAIT_CAR_STOP','ALIGN','DESCEND'):
                self.generator.set_mode(Mode.HOVER,hold_xy=self.resolved_start['uav_ground_position_map_m'][:2])
            ff = (0., 0.)
            if c.get('follow_velocity_feedforward', False) and self.phase == 'FOLLOW':
                dx,dy = _rotate_xy(self.relative_xy, _yaw(car.quaternion_xyzw))
                ff = (cv[0]-cv[2]*dy, cv[1]+cv[2]*dx)
            out = self.generator.step(now_ns=now_ns,now_source_ns=now_source_ns,target=car,
                                      target_velocity_xy=ff)
            if out['reason']:
                raise ValueError(out['reason'])
            xy = tuple(out['position_map_m'][:2])
            if self.phase == 'RENDEZVOUS':
                # Ascend beside the car FIRST, then cross above its reference.
                # The existing car task remains WAIT_TAKEOFF until FOLLOW.
                ready = (stopped and math.dist(uav.position[:2], point[:2]) <= c['follow_arrival_tolerance_m']
                         and abs(uav.position[2]-(point[2]+c['clearance_m'])) <= c['vertical_tolerance_m'])
                if self._settled(ready, now_ns):
                    self._transition('FOLLOW')
            elif self.phase == 'FOLLOW':
                if car_route_index is not None:
                    # Loaded-approach tasks have one route owner, CarMotionTask.
                    # Do not advance a second, independent waypoint counter.
                    if (type(car_route_index) is not int or not self.index<=car_route_index<=self.index+1
                            or car_route_index>=len(self.route) or type(car_route_finished) is not bool
                            or car_route_finished and car_route_index!=len(self.route)-1):
                        raise ValueError('car_route_progress_invalid')
                    self.index=car_route_index
                    if car_route_finished:self._transition('WAIT_CAR_STOP')
                else:
                    goal = self.route[self.index]
                    passing=bool(c.get('circle_route')) and self.index<len(self.route)-1
                    close=(circle_waypoint_passed(car.position,self.route[self.index-1],goal,
                              c['car_arrival_m'],c['circle_route']['lookahead_m']) if passing else
                           math.dist(car.position[:2],goal[:2]) <= c['car_arrival_m'])
                    reached = (close and
                               abs(_wrap(_yaw(car.quaternion_xyzw)-goal[2])) <= c['car_yaw_tolerance_rad'] and
                               (passing or
                                math.hypot(*cv[:2]) <= c['stopped_speed_mps'] and abs(cv[2]) <= c['stopped_yaw_rate_rps']))
                    if reached:
                        if self.index+1 < len(self.route):self.index += 1
                        else:self._transition('WAIT_CAR_STOP')
            elif self.phase == 'WAIT_CAR_STOP':
                if self._settled(stopped,now_ns):
                    self.landing_car_anchor = car
                    self._transition('ALIGN')
            elif self.phase in ('ALIGN','DESCEND'):
                if not stopped:
                    # Return to a stopped-car rendezvous; never continue a
                    # descent toward a deck that has started moving again.
                    self._transition('WAIT_CAR_STOP')
            # Once descent starts, realignment holds the last height instead
            # of commanding a climb all the way back to cruise altitude.
            zgoal = self.target[2] if self.descent_started else point[2]+c['clearance_m']
            landing_speed = c.get('landing_horizontal_speed_mps', c['stopped_speed_mps'])
            settling_speed = c.get('landing_settling_speed_mps',landing_speed)
            settling_time = c.get('landing_settle_seconds',c['settle_seconds'])
            aligned = ((self.ground_start or self._inside_platform(car,uav)) and
                       math.dist(uav.position[:2],point[:2]) <= c['landing_xy_tolerance_m'] and
                       abs(_wrap(_yaw(uav.quaternion_xyzw)-desired_yaw)) <= c['landing_yaw_tolerance_rad'] and
                       math.hypot(*uv[:2]) <= settling_speed)
            if self.phase == 'ALIGN':
                if self._settled(aligned and abs(uav.position[2]-zgoal) <= c['vertical_tolerance_m'],now_ns,settling_time):
                    self._transition('DESCEND')
                    self.descent_started = True
            was_paused = self.descent_paused
            self.descent_paused = False
            if self.phase != 'DESCEND':self.descent_hold_z = None
            if self.phase == 'DESCEND':
                if self.ground_start and c.get('ground_descent_continuous',False):
                    # Once over the adjacent ground landing area, finish the
                    # vertical ramp despite ordinary XY/velocity residuals.
                    # Keep bounded XY correction and actual touchdown checks;
                    # this is never a motor-stop or platform-landing shortcut.
                    self.descent_hold_z = None
                    zgoal = point[2]+c['land_handoff_height_m']
                elif not aligned and not self.ground_start:
                    self._transition('ALIGN')
                elif self.ground_start and (not aligned or
                        was_paused and not self._settled(aligned,now_ns,settling_time)):
                    # Freezing a reference BELOW the measured aircraft would
                    # still command descent while correcting XY. Capture the
                    # actual altitude once; don't chase subsequent pose noise
                    # or return to cruise height. Resume after the SAME existing
                    # settle interval, not a new deadline or abort timer.
                    if not aligned:self.settled_since = None
                    if self.descent_hold_z is None:
                        self.descent_hold_z = max(point[2]+c['land_handoff_height_m'],uav.position[2])
                    zgoal = self.descent_hold_z
                    self.descent_paused = True
                elif aligned:
                    self.descent_hold_z = None
                    zgoal = point[2]+c['land_handoff_height_m']
            rate = c['descent_speed_mps'] if zgoal < self.target[2] else c['climb_speed_mps']
            z = (zgoal if self.descent_paused else
                 self.target[2]+max(-rate*dt,min(rate*dt,zgoal-self.target[2])))
            self.target = (*xy,z)
            if c.get('controller_velocity_feedforward', False):
                # Send the bounded reference's velocity, not noisy raw car
                # differences. Vertical control keeps the proven position ramp.
                self.target_velocity = (*out['velocity_xy_mps'], 0.)
                gain=c.get('landing_position_feedback_gain_s_inv',0.)
                if gain and self.phase in ('ALIGN','DESCEND'):
                    # Correct measured Vicon tracking error via the EXISTING
                    # velocity feed-forward channel. Never move the frozen
                    # landing point or reset the FC/map alignment to hide error.
                    error=tuple(g-p for g,p in zip(xy,uav.position[:2]))
                    proportional=tuple(gain*e-c.get('landing_velocity_damping',0.)*v
                                       for e,v in zip(error,uv[:2]))
                    # Learn only persistent landing error, once per fresh pose,
                    # after the reference has arrived. Do not accumulate the
                    # return flight's tracking lag or replay one stale sample.
                    previous_stamp=self.landing_feedback_stamp_ns
                    self.landing_feedback_stamp_ns=uav.stamp_ns
                    integral_dt=(min(dt,(uav.stamp_ns-previous_stamp)*1e-9)
                                 if previous_stamp is not None else 0.)
                    if (integral_dt>0 and math.dist(xy,point[:2])<=c['landing_xy_tolerance_m']
                            and math.hypot(*out['velocity_xy_mps'])<=settling_speed):
                        candidate=tuple(v+c.get('landing_integral_gain_s_inv2',0.)*e*integral_dt
                                        for v,e in zip(self.landing_integral,error))
                        # Integral bias gets at most half the EXISTING correction
                        # budget. Conditional integration prevents windup while
                        # the correction or total output is saturated/slewing.
                        scale=min(1.,.5*landing_speed/max(math.hypot(*candidate),1e-12))
                        candidate=tuple(v*scale for v in candidate)
                        change=tuple(v-p for v,p in zip(candidate,self.landing_integral))
                        raw=tuple(p+i for p,i in zip(proportional,candidate))
                        delta=tuple(v+r-p for v,r,p in zip(out['velocity_xy_mps'],raw,previous_target_velocity[:2]))
                        limited=(math.hypot(*raw)>landing_speed and sum(d*r for d,r in zip(change,raw))>0
                                 or math.hypot(*delta)>self.limits.acceleration_mps2*dt
                                 and sum(d*r for d,r in zip(change,delta))>0)
                        if not limited:self.landing_integral=candidate
                    correction=tuple(p+i for p,i in zip(proportional,self.landing_integral))
                    scale=min(1.,landing_speed/max(math.hypot(*correction),1e-12))
                    desired=tuple(v+scale*e for v,e in zip(out['velocity_xy_mps'],correction))
                    scale=min(1.,self.limits.speed_mps/max(math.hypot(*desired),1e-12))
                    desired=tuple(v*scale for v in desired)
                    delta=tuple(v-p for v,p in zip(desired,previous_target_velocity[:2]))
                    scale=min(1.,self.limits.acceleration_mps2*dt/max(math.hypot(*delta),1e-12))
                    self.target_velocity=(*(p+scale*d for p,d in zip(previous_target_velocity[:2],delta)),0.)
            handoff_aligned = aligned or (self.ground_start and c.get('ground_descent_continuous',False))
            if (self.phase == 'DESCEND' and handoff_aligned and not self.descent_paused and abs(z-zgoal)<1e-8 and
                    abs(uav.position[2]-zgoal) <= LAND_HANDOFF_TOLERANCE_M and
                    abs(uv[2]) <= LAND_HANDOFF_MAX_VERTICAL_SPEED_MPS):
                self.land_intent_issued = True
                self.landing_waiting_for = 'touchdown'
                self._transition('LAND')
            return self.status()
        except (ValueError,TypeError,KeyError) as exc:
            self.target_velocity = (0., 0., 0.)
            self.landing_waiting_for = None
            self.fault = str(exc)
            observed=self.seen.get(c['uav_source'])
            # Target loss must not turn the previous moving goal into a 'hold'.
            if (observed and type(now_ns) is int and type(now_source_ns) is int and
                    0 <= now_ns-observed.received_ns <= self.limits.target_age_s*1e9 and
                    0 <= now_source_ns-observed.stamp_ns <= self.limits.target_age_s*1e9):
                self.target=tuple(observed.position)
                self.yaw=_yaw(observed.quaternion_xyzw)
            else:
                self.target=None
            self._transition('HOLD')
            return self.status()

    def status(self):
        car_action = 'route' if self.phase=='FOLLOW' else 'stop'
        actions = {'WAIT_READY':'none','WAIT_START':'none','TAKEOFF':'takeoff',
                   'RENDEZVOUS':'align_above_car',
                   'FOLLOW':'follow','WAIT_CAR_STOP':'hold_above_car','ALIGN':'align',
                   'DESCEND':'descend','LAND':'land','COMPLETE':'none',
                   'HOLD':'hold','PILOT_CONTROL':'manual'}
        return dict(phase=self.phase,fault=self.fault,complete=self.complete,
                    car_action=car_action,car_goal_map_xyyaw=self.route[self.index] if car_action=='route' else None,
                    car_speed_mps=self.config['car_speed_mps'],car_acceleration_mps2=self.config['car_acceleration_mps2'],
                    route_index=self.index,route_count=len(self.route),
                    uav_action=('existing_failsafe' if self.phase=='HOLD' and self.target is None else actions[self.phase]),uav_target_map_m=self.target,uav_yaw_map_rad=self.yaw,
                    uav_velocity_map_mps=self.target_velocity,
                    frame_id=self.config['frame_id'],frame_revision=self.config['frame_revision'],
                    initial_relative_xy_m=self.relative_xy,initial_relative_z_m=self.relative_z,
                    initial_relative_yaw_rad=self.relative_yaw,landing_heading=self.config['landing_heading'],
                    landing_mode='initial_ground' if self.ground_start else 'platform',
                    resolved_start=self.resolved_start,
                    land_intent_issued=self.land_intent_issued,
                    follow_position=self.config.get('follow_position','initial_offset'),
                    descent_paused=self.descent_paused,
                    landing_bias_velocity_map_mps=(*self.landing_integral,0.),
                    landing_waiting_for=self.landing_waiting_for,
                    flight_authorized=False,live_dispatch_permitted=False,
                    actual_sent_to_fc=0,actual_sent_to_car=0,
                    scope='offline_cooperative_intentions_not_live_adapter_or_physical_validation')
