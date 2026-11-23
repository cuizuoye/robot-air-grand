"""Cooperative intentions wired to existing car and UAV owners.

Resources are injected, never opened here. source_snapshot supplies calibrated
body references, original timestamps and existing car feedback/status. The car
publisher addresses the sole existing 4WSE bridge; UAV effects stay entirely in
MissionCore -> ContinuousExecutionIO. No independent MAVLink encoder/sender.
"""
import hashlib
import json
import math
from pathlib import Path
import sys
from threading import get_ident
from types import SimpleNamespace

from follow_target_generator import AirGroundSequence, _yaw, _wrap, ground_approach_clear
from car_route_adapter import CarRouteAdapter, CarMotionTask, FlightProgress, RouteIntent, ZERO
from mecanum_wand_4wse_control.safety_core import Lease, fresh_envelope
from mecanum_wand_4wse_control.idle_admission import zero_preparation_ready
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.target_mapping import (make_alignment,map_target_to_controller,
                                           yaw_target_controller,checked_alignment)


def existing_car_bridge_publisher(bridge):
    """Bind an ALREADY owned bridge; never construct or open another one.

    Its on_signed method retains TransportPolicy, startup handshake, watchdogs
    and the exclusive UDP socket. Calling this binder alone sends nothing.
    """
    from mecanum_wand_4wse_hardware.nodes import HardwareBridge
    if type(bridge) is not HardwareBridge or bridge.runtime.motion_tag!='4WSE':
        raise ValueError('existing exact 4WSE HardwareBridge required')
    owner=get_ident()
    def publish(wire):
        if get_ident()!=owner: raise RuntimeError('car bridge owner thread changed')
        bridge.on_signed(SimpleNamespace(data=wire))
    return publish


class AirGroundExecution:
    def __init__(self,config,*,source_snapshot,car_publish,channel_key,origin_monotonic,host_clock_ns=None):
        if not callable(source_snapshot) or not callable(car_publish):
            raise ValueError('existing source and car owner bindings required')
        if not math.isfinite(origin_monotonic) or origin_monotonic<0:
            raise ValueError('explicit host monotonic origin required')
        if host_clock_ns is not None and not callable(host_clock_ns):raise ValueError('host clock must be callable')
        self.host_clock_ns=host_clock_ns
        self.sequence=AirGroundSequence(config)
        self.car_task=(CarMotionTask(dict(config,schema='car_cooperative_motion_v1'),channel_key=channel_key)
                       if 'takeoff_car_world_xyyaw' in config or config.get('start_alignment') else None)
        self.car=self.car_task.car if self.car_task else CarRouteAdapter(config,channel_key=channel_key)
        self.car_start_pending=False
        self.plan_sha256=hashlib.sha256(json.dumps(config,sort_keys=True,allow_nan=False).encode()).hexdigest()
        self.source_snapshot,self.car_publish=source_snapshot,car_publish
        self.origin_monotonic=origin_monotonic
        self.owner=get_ident();self.core=None;self.latest=None;self.last_at=None
        self.car_status=None;self.car_qualified=False;self.counter=0
        self.alignment=None;self.binding=None;self.start_pending=False;self.started=False
        self.target_ned=self.target_yaw=None;self.target_at=None;self.fault=None
        self.velocity_ned=(0.,0.,0.)
        self.last_status=self.sequence.status();self.car_publish_calls=0

    def bind(self,core):
        if (self.core is not None or core.cooperative is not None or core.motion is not None
                or core.phase!='PREPARE' or core.pilot_source!='native_rc'
                or abs(core.preview.height-self.sequence.config['clearance_m'])>1e-9):
            raise ValueError('unused fixed-point Pocket core and matching deck clearance required')
        self.core=core;core.cooperative=self

    def _owner(self):
        if get_ident()!=self.owner:raise RuntimeError('cooperative owner thread changed')

    def check_start(self,o):
        self._owner()
        if (self.started or self.fault or self.last_status['phase']!='WAIT_START'
                or self.last_at is None or not 0<=o.t-self.last_at<=.15):
            raise ValueError('current stopped car/platform state required before the existing start')

    def check_car_start(self,o):
        self._owner()
        if (self.car_task is None or self.car_task.phase!='WAIT_START' or self.car_start_pending
                or self.started or self.fault or self.core.phase!='PREPARE' or o is None
                or not o.navigation_ok() or not o.preflight_ready or not o.preparation_ground_ok()
                or not o.start_pilot_ok() or self.last_at is None or not 0<=o.t-self.last_at<=.15):
            raise ValueError('当前车机静止状态和小车起点计划尚未就绪')

    def request_car_start(self,o):
        self.check_car_start(o)
        if self.sequence.start_alignment:
            data=self.latest
            if o.armed or not o.landed or math.sqrt(sum(v*v for v in data['uav_velocity'])) > self.sequence.config['stopped_speed_mps']:
                raise ValueError('stationary_unarmed_ground_uav_required')
            # Check source ages at the command, not only at the previous tick.
            now=self.host_clock_ns() if self.host_clock_ns else round((self.origin_monotonic+o.t)*1e9)
            source_now=self.sequence.last_source_ns+max(0,now-self.sequence.last_ns)
            for name in ('car','uav'):
                self.sequence._pose(data[name],self.sequence.config[name+'_source'],now,source_now)
            resolved=self.sequence.resolve_start(data['car'],data['uav'],data['car_rigid'],data['uav_rigid'])
            # Preserve the existing handshake, signer and route owner.
            self.car_task.bind_resolved_start(resolved)
        else:
            if not self.sequence._inside_platform(self.latest['car'],self.latest['uav']):
                raise ValueError('initial_placement_outside_usable_platform')
            self.sequence.loading_pose=(self.latest['car'],self.latest['uav'])
        self.car_start_pending=True

    def preparation_allowed(self):
        # No frozen FC origin/selector before the loaded approach finishes.
        return self.car_task is None or bool(not self.fault
            and self.car_task.phase=='WAIT_TAKEOFF' and self.car_status
            and self.car_status['stop_confirmed'] and self.last_status['phase']=='WAIT_START')

    def _flight_progress(self,o,data,steady,ros,clearance):
        if o is None:return None
        # Repeated owner turns must not renew old FC or Vicon evidence. Use
        # the oldest contributing observation, not this function's call time.
        age=o.status_age_s+max(0.,steady-(self.origin_monotonic+o.t))
        age=max(age,steady-data['uav'].received_ns*1e-9,ros-data['uav'].stamp_ns*1e-9)
        if not math.isfinite(age) or age<0:return None
        return FlightProgress(self.car_task.task_id,self.counter,steady-age,ros-age,
            o.armed,o.landed,clearance,self.core.pilot_control or o.native_stop)

    def start(self,o):
        self.check_start(o)
        sample=self.latest['uav']
        self.alignment=make_alignment(sample.position,o.position_ned,_yaw(sample.quaternion_xyzw),o.yaw_ned)
        self.binding=self.core._binding(o)
        self.start_pending=True;self.started=True

    def _publish(self,wire):
        self.car_publish(wire)
        self.car_publish_calls+=1  # dispatch calls, not proof of wheel motion

    def _stop_car(self,steady,ros,reason):
        self.car.supervisor.request_stop(steady,reason,fault=True)
        self.car.gate.trip(reason)
        self._publish(self.car.signer.encode(ZERO,steady,ros,state='STOPPING',permit=False))

    def service_stopped(self, now, ros_ns):
        """Maintain zero output/feedback while the UAV restores telemetry.

        Same owner, signer and stop adapter; no sequence advance, prepare,
        route lease, UAV command or invented standstill confirmation.
        """
        self._owner()
        steady = (self.host_clock_ns()*1e-9 if self.host_clock_ns
                  else self.origin_monotonic+now)
        if type(ros_ns) is not int or ros_ns <= 0:
            self.car_status = None
            return self.describe()
        ros = ros_ns*1e-9
        try:
            data = self.source_snapshot()
            if data is None:
                self.car_status = None
                self._publish(self.car.signer.encode(ZERO,steady,ros,state='STOPPING',permit=False))
                return self.describe()
            self.counter = max(self.counter,self.car_task.counter if self.car_task else 0)+1
            index = self.car_task.index if self.car_task else self.last_status['route_index']
            intent = RouteIntent(self.counter,steady,ros,self.sequence.config['frame_id'],
                                self.sequence.config['frame_revision'],'stop',index)
            self.car_status = self.car.step(steady=steady,ros=ros,car=data['car'],
                frame_revision=data['frame_revision'],feedback=data['feedback'],
                lease=Lease('air_ground_existing_owner',self.counter,steady,ros,False),
                intent=intent,graph_ok=data['graph_ok'])
            self._publish(self.car_status['signed_authority'])
            self.fault = self.fault or self.car_status['fault']
        except (ValueError,TypeError,KeyError,OSError,RuntimeError) as exc:
            self.fault = self.fault or str(exc)
            self.car_status = None
            self._stop_car(steady,ros,self.fault)
        return self.describe()

    def service(self,runtime,now):
        """Called in the existing owner turn, after telemetry/RC processing."""
        self._owner()
        if runtime.core is not self.core:raise ValueError('cooperative UAV owner changed')
        if self.describe()['complete']:
            return self.service_stopped(now,runtime.cooperative_ros_ns)
        o,missing=runtime.inputs.build(now)
        state_reply_pending=False
        # Preserve the existing arm transaction's reply-pair grace. Commander
        # and actuator messages arrive separately; do not turn their temporary
        # disagreement into a new cooperative abort or advance ascent early.
        if (missing==['commander_and_actuator_armed_state_disagrees'] and not runtime.failure
                and (self.core.phase=='WAIT_ARM' and self.core.pending or self.core.phase=='LAND')):
            from dataclasses import replace
            o=runtime._degraded_observation(now)
            if self.core.phase=='WAIT_ARM':o=replace(o,armed=False,landed=True)
            missing=[]
            state_reply_pending=True
        # On landing, use the very same landed+disarmed proof as MissionCore.
        # It is not navigation readiness and cannot enable an airborne target.
        if o is None and self.core.phase in ('LAND','LAND_TIMEOUT','COMPLETE'):
            ground=runtime._degraded_observation(now)
            if self.core._landing_ground_confirmed(ground):
                o=ground;missing=[];state_reply_pending=True
        return self.update(o,now=now,ros_ns=runtime.cooperative_ros_ns,
            unavailable=runtime.failure or (';'.join(missing) if self.started or
                self.car_task and self.car_task.started_at is not None else None),
            pilot_control=self.core.pilot_control,stop_requested=runtime.cancel_requested or runtime.stop_latched,
            state_reply_pending=state_reply_pending,
            steady_ns=self.host_clock_ns() if self.host_clock_ns else None)

    def update(self,o,*,now,ros_ns,unavailable=None,pilot_control=False,stop_requested=False,state_reply_pending=False,
               steady_ns=None):
        """Same dispatch path for live owner and offline injected measurements."""
        self._owner()
        # The UAV turn's `now` is sampled BEFORE polling ROS. A real callback
        # can arrive milliseconds later. Evaluate car/pose receipt age against
        # the actual post-poll host clock, while retaining the mission's own
        # turn time for UAV target dispatch. Never restamp the measurements.
        if steady_ns is not None and (type(steady_ns) is not int or steady_ns<=0):
            raise ValueError('positive integer host monotonic timestamp required')
        steady=steady_ns*1e-9 if steady_ns is not None else self.origin_monotonic+now
        engaged=self.started or bool(self.car_task and self.car_task.started_at is not None)
        if type(ros_ns) is not int or ros_ns<=0:
            if not engaged:return self.describe()
            # Never restamp sensor data to repair missing ROS time.
            self.fault=self.fault or 'cooperative_source_clock_missing'
            # No usable clock for a fresh car envelope: stop renewing it;
            # the existing transport watchdog stops the car. Keep servicing
            # MissionCore so its original LAND/observation path can proceed.
            self.car_status=None
            return self.describe()
        ros=ros_ns*1e-9;self.counter+=1
        try:
            data=self.source_snapshot()
            if data is None:
                if not engaged:return self.describe()
                raise ValueError('cooperative_measurements_missing')
            self.latest=data;self.last_at=now
            frozen=self.sequence.resolved_start
            if frozen and not self.started:
                if (math.dist(data['uav'].position,frozen['uav_ground_position_map_m']) > self.sequence.config['car_arrival_m'] or
                        abs(_wrap(_yaw(data['uav'].quaternion_xyzw)-frozen['uav_heading_map_rad'])) > self.sequence.config['car_yaw_tolerance_rad']):
                    raise ValueError('stationary_uav_moved_after_start_alignment')
                if frozen.get('ground_approach') and not ground_approach_clear(data['car'],frozen['ground_approach']):
                    raise ValueError('approach_vehicle_clearance_lost')
            transport=data['transport']
            transport_ok=bool(fresh_envelope(transport['issued_steady'],transport['issued_ros'],steady,ros,.5)
                and transport['fresh'] is True and transport['local_stop'] is False
                and transport['native_fault_bits']==0 and not transport['fault'])
            # The real bridge's ready flag means DISABLED startup readiness.
            # It becomes False normally while wheels run; use current link/
            # fault/epoch evidence after activation, not that startup flag.
            # Use the SAME disabled-zero admission as CarMotionTask. A healthy
            # previous run leaves startup ready=False even in a new session;
            # requiring that old flag here deadlocks the flight handoff.
            # Actual motion still requires the existing signed epoch handshake.
            transport_ready=transport_ok and transport['ready'] is True
            transport_prepare_ready=transport_ok and zero_preparation_ready(transport)
            # The original bridge's ready also requires started_ms == 0.
            # After the loaded approach it stays false despite a valid stop;
            # hand off using current disabled output and measured standstill.
            approach_stopped=bool(self.car_task is not None
                and self.car_task.phase=='WAIT_TAKEOFF' and self.car_status
                and self.car_status['stop_confirmed']
                and transport.get('native_disabled_zero') is True)
            status=self.sequence.status()
            if o is not None:
                pilot_control=pilot_control or bool(self.started and o.armed and o.manual_control_active)
                if self.binding and self.core._binding(o)!=self.binding:
                    raise ValueError('cooperative_uav_epoch_changed')
                faulted=bool(self.fault or unavailable or stop_requested or self.core.fault)
                status=self.sequence.step(now_ns=round(steady*1e9),now_source_ns=ros_ns,
                    frame_revision=data['frame_revision'],car=data['car'],uav=data['uav'],
                    car_velocity=data['car_velocity'],uav_velocity=data['uav_velocity'],
                    car_ready=bool(self.car_qualified and transport_ok and not faulted
                        and (self.car_task is None or self.car_task.phase in ('WAIT_TAKEOFF','ROUTE','WAIT_LANDING','COMPLETE'))
                        and (transport_ready or approach_stopped
                             or self.sequence.phase not in ('WAIT_READY','WAIT_START','TAKEOFF'))),
                    # The sole missing predicate in this pending transaction
                    # is arm-state agreement (or final ground confirmation).
                    # This only holds a warmup target or observes LAND.
                    uav_ready=bool((o.navigation_ok() or state_reply_pending) and not faulted),
                    car_stopped=bool(self.car_status and self.car_status['stop_confirmed']),
                    armed=o.armed,landed=o.landed,start=self.start_pending,pilot_control=pilot_control,
                    car_route_index=max(0,self.car_task.index-self.car_task.plan.takeoff_index-1) if self.car_task else None,
                    car_route_finished=bool(self.car_task and self.car_task.phase in ('WAIT_LANDING','COMPLETE')))
                self.start_pending=False
            elif engaged:
                self.fault=self.fault or unavailable or 'cooperative_uav_observation_missing'
                self._stop_car(steady,ros,self.fault)
                # Missing UAV diagnostics prohibit mission progress, but must
                # not starve the independent car feedback/stop clock. Reuse
                # the same disabled owner used during telemetry restoration.
                return self.service_stopped(now,ros_ns)
            self.last_status=status
            if status['fault']:
                self.fault=self.fault or status['fault']
            if unavailable or stop_requested or self.core.fault:
                self.fault=self.fault or unavailable or self.core.fault or 'cooperative_cancelled'
            # WAIT_* and takeoff emit disabled zero. Motion starts only after
            # measured clearance AND the bridge's matching prepared epoch.
            if self.fault or pilot_control or self.core.phase in ('BLOCKED','STOP_LATCHED','PILOT_CONTROL','LAND_TIMEOUT'):
                status=dict(status,car_action='stop')
            if self.car_task is not None:
                task=self.car_task.step(steady=steady,ros=ros,car=data['car'],car_velocity=data['car_velocity'],
                    frame_revision=data['frame_revision'],feedback=data['feedback'],transport=transport,
                    graph_ok=data['graph_ok'],
                    flight=self._flight_progress(o,data,steady,ros,status['phase']=='FOLLOW' and not self.fault),
                    start=self.car_start_pending,
                    cancel=bool(self.fault or pilot_control or stop_requested or self.core.phase in
                        ('BLOCKED','STOP_LATCHED','PILOT_CONTROL','LAND_TIMEOUT')))
                self.car_start_pending=False
                self.car_status=self.car_task.last_car
                self._publish(self.car_status['signed_authority'])
                self.car_qualified |= self.car_status['ready'] and transport_prepare_ready
                self.fault=self.fault or task['fault']
                if self.started and status['uav_target_map_m'] is not None:
                    self._map_target(status,now)
                return self.describe()
            s,g=self.car.supervisor,self.car.gate
            held=bool(status['car_action']=='route' and transport_ok and s.epoch
                and g.primed_epoch==s.epoch and transport.get('armed_epoch')==s.epoch
                and (transport_ready if s.state=='ARMED' else
                     s.state=='ACTIVE' and transport.get('policy_state')=='ACTIVE'))
            lease=Lease('air_ground_existing_owner',self.counter,steady,ros,held)
            intent=RouteIntent.from_sequence(status,self.counter,steady,ros,self.car.plan)
            self.car_status=self.car.step(steady=steady,ros=ros,car=data['car'],
                frame_revision=data['frame_revision'],feedback=data['feedback'],lease=lease,
                intent=intent,graph_ok=data['graph_ok'] is True and transport_ok)
            self._publish(self.car_status['signed_authority'])
            self.car_qualified |= self.car_status['ready'] and transport_prepare_ready
            if self.car_status['fault']:
                self.fault=self.fault or self.car_status['fault']
            # STANDBY intentionally does not latch every missing prerequisite
            # in the original car supervisor. Once this flight has started,
            # losing one must also prevent the pending UAV arm transaction.
            if self.started and s.inputs.health(steady,ros):
                raise ValueError(s.inputs.health(steady,ros))
            if self.car_status['ready'] and status['car_action']=='route' and not self.fault:
                ok,reason=self.car.prepare(steady,ros)
                if not ok:raise ValueError(reason)
            if self.started and status['uav_target_map_m'] is not None:
                self._map_target(status,now)
        except (ValueError,TypeError,KeyError,AttributeError,OSError,RuntimeError) as exc:
            self.fault=self.fault or str(exc)
            # A failed route/publication never resumes. Only signed zero may
            # be attempted after uncertain delivery; original bridge timeout
            # remains the fallback if its owner/connection is unavailable.
            try:self._stop_car(steady,ros,self.fault)
            except (ValueError,TypeError,OSError,RuntimeError):pass
        return self.describe()

    def _map_target(self,status,now):
        self.target_ned=tuple(map_target_to_controller(self.alignment,status['uav_target_map_m']))
        self.target_yaw=yaw_target_controller(self.alignment,status['uav_yaw_map_rad'])
        rotation,_,_=checked_alignment(self.alignment)
        self.velocity_ned=tuple(float(v) for v in rotation@status['uav_velocity_map_mps'])
        self.target_at=now

    def velocity_feedforward(self,o):
        self.target(o)  # Same tick and controller epoch as the position.
        return (self.velocity_ned if self.sequence.config.get('controller_velocity_feedforward',False)
                else None)

    def target(self,o):
        if (not self.started or self.binding!=self.core._binding(o) or self.target_at!=o.t
                or self.target_ned is None or self.target_yaw is None):
            raise ValueError('current cooperative mapped target required')
        return self.target_ned,self.target_yaw

    def flight_tick(self,o):
        from uav_coord_bridge.mission_preview import State
        if self.fault:
            return dict(state=State.ABORT_LAND.value,reason=self.fault)
        try:self.target(o)
        except ValueError as exc:return dict(state=State.ABORT_LAND.value,reason=str(exc))
        status=self.last_status
        if status['uav_action']=='existing_failsafe':
            return dict(state=State.ABORT_LAND.value,reason=status['fault'] or 'cooperative_pose_missing')
        if status['phase']=='LAND':return dict(state=State.LAND.value)
        return dict(state='ASCEND' if status['phase']=='TAKEOFF' else 'HOVER')

    def candidate_allowed(self,msg,core):
        from pymavlink.dialects.v20 import common as m
        from disarmed_offboard_policy import POSITION_YAW_MASK,POSITION_VELOCITY_YAW_MASK
        if (core is not self.core or self.fault or self.target_at!=core.last_t
                or self.target_ned is None or core.binding!=self.binding):return False
        feedforward=self.sequence.config.get('controller_velocity_feedforward',False)
        mask=POSITION_VELOCITY_YAW_MASK if feedforward else POSITION_YAW_MASK
        velocity=self.velocity_ned if feedforward else (0.,0.,0.)
        return bool((msg.target_system,msg.target_component,msg.coordinate_frame,msg.type_mask)==
            (1,1,m.MAV_FRAME_LOCAL_NED,mask)
            and all(math.isfinite(v) for v in (msg.x,msg.y,msg.z,msg.yaw))
            and all(abs(a-b)<1e-6 for a,b in zip((msg.x,msg.y,msg.z),self.target_ned))
            and abs(math.atan2(math.sin(msg.yaw-self.target_yaw),math.cos(msg.yaw-self.target_yaw)))<1e-6
            and all(math.isfinite(v) and abs(v-e)<1e-6 for v,e in zip((msg.vx,msg.vy,msg.vz),velocity))
            and all(getattr(msg,k)==0 for k in ('afx','afy','afz','yaw_rate')))

    def describe(self):
        return dict(plan_sha256=self.plan_sha256,phase=self.last_status['phase'],fault=self.fault or self.last_status['fault'],
            car_task_phase=self.car_task.phase if self.car_task else None,
            car_waiting_for=self.car_task.waiting_for if self.car_task else None,
            car_route_index=max(0,self.car_task.index-self.car_task.plan.takeoff_index) if self.car_task else self.last_status['route_index'],
            car_path_index=self.car_task.index if self.car_task else None,
            takeoff_car_world_xyyaw=self.car_task.plan.route[self.car_task.plan.takeoff_index] if self.car_task and self.car_task.plan.route else None,
            resolved_start=self.sequence.resolved_start,
            landing_mode=self.last_status.get('landing_mode','platform'),
            follow_position=self.last_status.get('follow_position','initial_offset'),
            descent_paused=self.last_status.get('descent_paused',False),
            landing_waiting_for=self.last_status.get('landing_waiting_for'),
            complete=bool(self.last_status['complete'] and not self.fault and
                (self.car_task is None or self.car_task.phase=='COMPLETE')),
            car_state=None if self.car_status is None else self.car_status['state'],
            car_stopped=bool(self.car_status and self.car_status['stop_confirmed']),
            target_ned=self.target_ned,target_yaw_ned=self.target_yaw,
            velocity_ned_mps=self.velocity_ned,
            car_publish_calls=self.car_publish_calls,started=self.started,
            geometry_is_example=self.sequence.config.get('geometry_is_example',True),
            flight_authorized=False)
