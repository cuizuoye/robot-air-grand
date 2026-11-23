"""Opt-in real subscriptions/ownership for the existing cooperative controller.

Importing this module opens nothing. The sealed car's bridge process remains
the only MC-Link socket owner; neither its Sign source, supervisor nor browser
is started. Its original hardware envelope is reused as a transport profile,
NOT as authorization for the cooperative task. TAKEOFF still belongs to the
existing drone controller and its plan-bound execution review.
"""
from collections import deque
from contextlib import ExitStack
from dataclasses import replace
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import secrets
import sys
import time

from air_ground_execution import AirGroundExecution
from car_route_adapter import CAR_ENTRY, HOST, ZERO, RouteIntent, CarRoutePlan, CarMotionTask
from follow_target_generator import AirGroundSequence, TargetPose, _yaw, _wrap
from mecanum_wand_4wse_control.safety_core import Lease
from uav_coord_bridge.mavlink_vision import (checked_quaternion, finite_vector,
    quaternion_to_matrix, matrix_to_quaternion)

DOMAIN = 197
NAMESPACE = '/wand_revflow_stage6_long600'
CAR_TOPIC = '/vicon_b3_check/car01/car01'
OWNER = 'air_ground_owner'
BRIDGE = 'wand_hardware_bridge'


def configuration_gaps(plan):
    """Report unfilled site values in a draft; never invent geometric defaults."""
    paths=['platform_offset_car_m','platform_size_m','uav_landing_footprint_m',
           'edge_margin_m','landing_xy_tolerance_m','route_world_xyyaw',
           'live_inputs.car_ground_z_map_m',
           'live_inputs.car_body_origin_in_rigid_m','live_inputs.car_body_orientation_in_rigid_xyzw',
           'live_inputs.world_revision','live_inputs.car_rigid_revision','live_inputs.uav_rigid_revision']
    paths += ['follow_limits.'+k for k in ('xmin','xmax','ymin','ymax')]
    if plan.get('landing_mode') == 'initial_ground':
        paths=[p for p in paths if p not in ('platform_offset_car_m','platform_size_m',
            'uav_landing_footprint_m','edge_margin_m','route_world_xyyaw')]
        paths += ['start_alignment','route_local_forward_left_m']
    if 'takeoff_car_world_xyyaw' in plan:paths.append('takeoff_car_world_xyyaw')
    missing=[]
    for path in paths:
        value=plan
        for key in path.split('.'):
            value=value.get(key) if isinstance(value,dict) else None
        if value is None or isinstance(value,(list,tuple)) and any(x is None for x in value):
            missing.append(path)
    return missing


def validate_bindings(plan, uav_config=None, *, live=False, sources_only=False, car_only=False):
    """All calibration inputs are in the same reviewed plan, not hidden defaults."""
    if sources_only and live:
        raise ValueError('source-only coordinates cannot open cooperative control resources')
    alignment_only=plan.get('alignment_only') is True
    if alignment_only and (not car_only or not plan.get('start_alignment',{}).get('avoidance')):
        raise ValueError('alignment_only_requires_standalone_avoidance_plan')
    if car_only and plan.get('start_alignment') and not alignment_only:
        raise ValueError('automatic_start_requires_paired_uav_measurements')
    if not sources_only and plan.get('schema') not in (('air_ground_sequence_v1','car_cooperative_motion_v1') if car_only else ('air_ground_sequence_v1',)):
        raise ValueError('unknown_air_ground_plan')
    if Path(plan.get('car_entry','')).resolve()!=CAR_ENTRY:
        raise ValueError('confirmed car entry required before any resource open')
    if (plan['car_source'],plan['frame_id'])!=('car01','map') or not car_only and plan['uav_source']!='MircoAir':
        raise ValueError('existing car01 / MircoAir / map interfaces required')
    if not isinstance(plan.get('frame_revision'),str) or not plan['frame_revision']:
        raise ValueError('explicit common frame revision required')
    binding = plan.get('live_inputs', {})
    required = ('world_revision','car_rigid_revision','uav_rigid_revision',
                'car_body_origin_in_rigid_m','car_body_orientation_in_rigid_xyzw')
    if car_only:required=tuple(k for k in required if k!='uav_rigid_revision')
    missing = [k for k in required if k not in binding]
    if sources_only and missing:
        raise ValueError('source calibration bindings missing: '+', '.join(missing))
    for name in ('world_revision','car_rigid_revision')+(() if car_only else ('uav_rigid_revision',)):
        if name in binding and binding[name] is not None:
            if not isinstance(binding[name],str) or not binding[name]:
                raise ValueError('explicit calibration revision required: '+name)
    if binding.get('car_body_origin_in_rigid_m') is not None:
        finite_vector(binding['car_body_origin_in_rigid_m'],3)
    if binding.get('car_body_orientation_in_rigid_xyzw') is not None:
        checked_quaternion(binding['car_body_orientation_in_rigid_xyzw'])
    if uav_config and binding.get('uav_rigid_revision') != uav_config['revision']:
        raise ValueError('cooperative UAV body revision differs from flight profile')
    if sources_only and any(binding.get(k) is None for k in required):
        raise ValueError('source calibration bindings missing')
    gaps=[] if sources_only else configuration_gaps(plan)
    if car_only and not sources_only:
        gaps=[k for k in gaps if (k.startswith('live_inputs.') and k!='live_inputs.uav_rigid_revision')
              or k in ('takeoff_car_world_xyyaw','route_world_xyyaw')]
        if not alignment_only and plan.get('takeoff_car_world_xyyaw') is None and 'takeoff_car_world_xyyaw' not in gaps:gaps.append('takeoff_car_world_xyyaw')
    z=binding.get('car_ground_z_map_m')
    if z is not None and (type(z) not in (int,float) or not math.isfinite(z)):
        raise ValueError('car_ground_z_map_m must be finite world Z of the resting chassis reference')
    if binding.get('car_ground_plane_map_abc') is not None:
        finite_vector(binding['car_ground_plane_map_abc'],3)
    if gaps and live:
        raise ValueError('联合配置尚缺实测值：'+', '.join(gaps))
    if not sources_only and not gaps:
        if not car_only:AirGroundSequence(plan)
        if 'takeoff_car_world_xyyaw' in plan or plan.get('start_alignment'):
            CarRoutePlan(dict(plan,schema='car_cooperative_motion_v1'))
    evidence=plan.get('site_evidence',{})
    recorded_car=bool(car_only and evidence.get('car_motion_reference_definition',{}).get('applied_to_live_inputs') is True
        and evidence.get('takeoff_capture',{}).get('current_point_observed') is True
        and evidence.get('route_design',{}).get('physical_clearance_and_vicon_coverage_confirmed') is True)
    if live and (missing or not recorded_car and (plan.get('geometry_is_example') is not False
                 or binding.get('measured') is not True)):
        raise ValueError('实测平台/车体坐标尚未填入；示例不能打开联合硬件入口')
    return dict(schema='air_ground_resource_plan_v1',missing_bindings=missing,
        configuration_complete=not gaps,missing_configuration=gaps,
        configuration_status='待补齐现场数值；仅离线查看' if gaps else '字段已齐；不代表实机验证或运动授权',
        car_ground_z_map_m=z,car_ground_plane_map_abc=binding.get('car_ground_plane_map_abc'),
        geometry_is_example=plan.get('geometry_is_example',True),
        takeoff_car_world_xyyaw=plan.get('takeoff_car_world_xyyaw'),
        route_world_xyyaw=plan.get('route_world_xyyaw'),
        start_alignment=plan.get('start_alignment'),route_local_forward_left_m=plan.get('route_local_forward_left_m'),
        route_direction_mode='fixed_task_axes' if 'route_heading_map_rad' in plan else 'initial_uav_heading',
        route_heading_map_rad=plan.get('route_heading_map_rad'),
        world_route_status='resolved_once_at_CAR_START' if plan.get('start_alignment') else 'configured',
        landing_mode=plan.get('landing_mode','platform'),
        follow_position=plan.get('follow_position','initial_offset'),
        controller_velocity_feedforward=plan.get('controller_velocity_feedforward',False),
        airborne_offset_car_forward_left_m=plan.get('airborne_offset_car_forward_left_m'),
        takeoff_height_m=plan.get('clearance_m'),descent_speed_mps=plan.get('descent_speed_mps'),
        measured=binding.get('measured') is True,recorded_car_configuration=recorded_car,
        car_only=car_only,car_entry=str(CAR_ENTRY),sources_only=sources_only,
        car_role=('none; coordinate callbacks only' if sources_only else
                  'bridge only; no Sign/source/supervisor/gate/browser process'),
        car_domain=DOMAIN,car_namespace=NAMESPACE,car_pose_topic=CAR_TOPIC,
        vicon_connections=0 if sources_only else 1,shared_domain0_modified=False,
        sources='same owned pinned Vicon receiver; original source/receipt timestamps',
        standalone_car_entry_unchanged=True,automatic_motion=False,devices_opened=0,
        live_interfaces_implemented=not sources_only,hardware_validated=False)


def body_pose(position, quaternion, origin, orientation):
    """Same rigid-body extrinsics as UAV VPE, kept in map/FLU for coordination."""
    rotation = quaternion_to_matrix(checked_quaternion(quaternion))
    p = finite_vector(position,3) + rotation @ finite_vector(origin,3)
    q = matrix_to_quaternion(rotation @ quaternion_to_matrix(checked_quaternion(orientation)))
    return tuple(map(float,p)),tuple(map(float,q))


class JointMeasurements:
    """Callbacks only. Never restamp cached samples or substitute target velocity."""
    def __init__(self, plan, uav_config, *, car_only=False):
        # Coordinate callbacks do not require a route or a measured landing
        # footprint. JointResources still validates the FULL plan before I/O.
        validate_bindings(plan,uav_config,sources_only=True,car_only=car_only)
        self.car_only=car_only
        self.plan,self.uav_config = plan,uav_config
        self.poses={};self.rigid_poses={};self.history={name:deque(maxlen=128) for name in ('car','uav')}
        self.velocity={};self.feedback=self.transport=None
        self.graph_ok=False;self.fault=None;self.discarded_status=0

    def _pose(self, kind, stamp, received, frame, position, quaternion, valid=True):
        if frame != self.plan['frame_id'] or type(stamp) is not int or stamp <= 0:
            raise ValueError('joint source frame/timestamp invalid: '+kind)
        previous=self.poses.get(kind)
        if kind=='uav':
            origin,orientation=self.uav_config['origin'],self.uav_config['orientation']
        else:
            c=self.plan['live_inputs']
            origin,orientation=c['car_body_origin_in_rigid_m'],c['car_body_orientation_in_rigid_xyzw']
        p,q=body_pose(position,quaternion,origin,orientation)
        pose=TargetPose(self.plan[kind+'_source'],frame,stamp,received,p,q,valid,not valid)
        if previous and stamp <= previous.stamp_ns:
            if stamp==previous.stamp_ns and (p,q,valid)==(
                    previous.position,previous.quaternion_xyzw,previous.tracked):return
            raise ValueError('joint source clock regressed or duplicate changed: '+kind)
        if previous and received < previous.received_ns:raise ValueError('joint receipt clock regressed')
        self.poses[kind]=pose
        self.rigid_poses[kind]=replace(pose,position=tuple(map(float,position)),
                                      quaternion_xyzw=tuple(map(float,quaternion)))
        history=self.history[kind]
        if not valid:
            history.clear();self.velocity.pop(kind,None);return
        history.append(pose)
        # A short measured interval reduces frame-to-frame derivative noise.
        # Both ends retain real source stamps; one sample is never zero speed.
        while len(history)>2 and stamp-history[1].stamp_ns>=100_000_000:history.popleft()
        if len(history)>1:
            old=history[0];dt=(stamp-old.stamp_ns)*1e-9
            v=tuple((b-a)/dt for a,b in zip(old.position,p))
            self.velocity[kind]=v if kind=='uav' else (v[0],v[1],_wrap(_yaw(q)-_yaw(old.quaternion_xyzw))/dt)

    def receive_uav(self,row):
        try:
            if (row['subject'],row['segment'])!=('MircoAir','MircoAir'):
                raise ValueError('UAV FrameSample identity changed')
            valid=bool(row['sdk_ok'] and not row['occluded'] and row['pose_published'] and not row['queue_dropped'])
            if not valid:
                # Invalid FrameSample can have zero/default pose and timestamp.
                # Invalidate the last pose immediately without inventing a frame.
                if 'uav' in self.poses:self.poses['uav']=replace(self.poses['uav'],tracked=False,occluded=True)
                if 'uav' in self.rigid_poses:self.rigid_poses['uav']=replace(self.rigid_poses['uav'],tracked=False,occluded=True)
                self.history['uav'].clear();self.velocity.pop('uav',None)
                return
            p=row['world_pose']
            self._pose('uav',row['ros_stamp_ns'],row['callback_ns'],row['frame_id'],p[:3],p[3:])
        except (ValueError,TypeError,KeyError) as exc:self.fault=self.fault or str(exc)

    def receive_car(self,msg,received_ns):
        try:
            h=msg.header;p=msg.pose.position;q=msg.pose.orientation
            self._pose('car',h.stamp.sec*1_000_000_000+h.stamp.nanosec,received_ns,
                       h.frame_id,(p.x,p.y,p.z),(q.x,q.y,q.z,q.w))
        except (ValueError,TypeError,KeyError,AttributeError) as exc:self.fault=self.fault or str(exc)

    def receive_status(self,kind,msg):
        try:
            row=json.loads(msg.data)
            if not isinstance(row,dict):raise ValueError('joint bridge status must be an object')
            for key in ('issued_steady','issued_ros'):
                if type(row.get(key)) not in (int,float) or not math.isfinite(row[key]):
                    raise ValueError('joint bridge status missing source timestamp')
            old=getattr(self,kind)
            if old and row['issued_steady']<old['issued_steady']:
                self.discarded_status+=1
                return  # A delayed old packet cannot replace or refresh current state.
            if old and row['issued_steady']==old['issued_steady']:
                if row==old:return
                raise ValueError('joint bridge status regression: '+kind)
            setattr(self,kind,row)
        except (ValueError,TypeError,KeyError) as exc:self.fault=self.fault or str(exc)

    def snapshot(self):
        if self.fault:raise ValueError(self.fault)
        required={'car'} if self.car_only else {'car','uav'}
        if (not required<=set(self.poses) or not required<=set(self.velocity)
                or self.feedback is None or self.transport is None):return None
        result=dict(**self.poses,frame_revision=self.plan['frame_revision'],
            car_velocity=self.velocity['car'],feedback=self.feedback,transport=self.transport,graph_ok=self.graph_ok)
        if not self.car_only:result['uav_velocity']=self.velocity['uav']
        result.update({name+'_rigid':pose for name,pose in self.rigid_poses.items()})
        return result


def original_car_entry():
    """Load exact existing launcher helpers; never invoke its launch/main."""
    # Match running the original entry.py: its sibling receiver_manager must
    # be importable as well as the helpers under host/.
    sys.path.insert(0,str(CAR_ENTRY.parent))
    sys.path.insert(0,str(HOST))
    spec=importlib.util.spec_from_file_location('_air_ground_original_car_entry',CAR_ENTRY)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def bridge_arguments():
    """Original bridge and exact existing transport limits; no copied transport."""
    from mecanum_wand_4wse_control.feedback_policy import COMBINED_FEEDBACK_TIMEOUT_S
    profile=json.loads((HOST/'profiles/stage6.LONG600_NOT_APPROVED.json').read_text())
    hardware,c=profile['hardware'],profile['safety']
    params=dict(esp_ip=hardware['esp_ip'],network_interface=hardware['network_interface'],
        command_port=46000,telemetry_port=46001,bounded_motion_mode=True,
        required_motion_tag='4WSE',wire_timeout_ms=600,max_vx_mps=c['max_speed_mps'],
        max_vy_mps=c['max_speed_mps'],max_wz_radps=c['max_yaw_rate_radps'],
        command_timeout_s=.10,link_receive_timeout_s=COMBINED_FEEDBACK_TIMEOUT_S,
        invert_body_xy=False,udp_low_latency_qos=False)
    args=['/usr/bin/python3','-B',str(HOST/'node.py'),'--role','bridge','--ros-args',
          '-r','__node:='+BRIDGE,'-r','__ns:='+NAMESPACE]
    for key,value in params.items():
        args+=['-p',key+':='+(str(value).lower() if type(value) is bool else str(value))]
    return args


class JointResources:
    """One parent reactor, original car bridge child, shared UAV Vicon owner.

    Construction is explicit and does not enable the car or arm the drone.
    The parent owns the original car-launch lock; no global environment edits,
    process scanning/killing, shared Vicon changes or network reconfiguration.
    """
    def __init__(self, *, plan, uav_config, source, output, origin_monotonic,
                 open_devices=False, site_ready=False, car_only=False):
        if open_devices is not True or site_ready is not True:
            raise PermissionError('explicit joint hardware open and current site declaration required')
        validate_bindings(plan,uav_config,live=True,car_only=car_only)
        self.car_only=car_only
        self.alignment_only=car_only and plan.get('alignment_only') is True
        self.alignment_sequence=AirGroundSequence(dict(plan,alignment_only=False)) if self.alignment_only else None
        self.output=Path(output);self.source=source
        measurement_plan=dict(plan,alignment_only=False) if self.alignment_only else plan
        self.cache=JointMeasurements(measurement_plan,uav_config,car_only=car_only and not self.alignment_only)
        if self.alignment_only:self.source.on_frame=lambda row,at:self.cache.receive_uav(row)
        self.resources=ExitStack();self.proc=self.node=self.executor=None
        self.closing=False;self.closed=False;self.graph_seen=False;self.last_audit=-math.inf
        self.last_log=-math.inf;self.last_log_key=None;self.trace=None;self.execution=None
        self.errors=[];self.stop_result=None
        self.last_car_status=None;self.dispatch_calls=0
        try:
            self._open(plan,origin_monotonic)
        except BaseException as exc:
            try:self.resources.close()
            except Exception as cleanup:exc.add_note('Joint resource cleanup: '+str(cleanup))
            raise

    def _open(self,plan,origin):
        import fcntl,subprocess
        # Same flock as the known working car demo. Fail before bridge launch.
        lock=self.resources.enter_context(open('/tmp/wand_4wse_browser_operator.lock','a'))
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        entry=original_car_entry();entry.verify_release();entry.route_check()
        folder=self.output/'car_bridge';folder.mkdir()
        env=entry.prepare_session(False,600,True,folder)
        key=secrets.token_hex(32)
        env.update(WAND_4WSE_CHANNEL_KEY=key,WAND_AUTOPREP_FLOW='AUTO_PREPARE_SINGLE_SPACE_V1',
                   WAND_4WSE_RAW_RECEIVE_TRACE=str(folder/'NATIVE_JOURNAL.jsonl'))
        (folder/'USE.json').write_text(json.dumps(dict(
            task='standalone_car_route' if self.car_only else 'air_ground_cooperation',legacy_profile_scope='unchanged hardware transport envelope only',
            sign_controller_started=False,flight_authorized=False,
            plan_sha256=hashlib.sha256(json.dumps(plan,sort_keys=True,allow_nan=False).encode()).hexdigest()),indent=2)+'\n')
        self._subscribe()
        log=self.resources.enter_context((folder/'bridge.log').open('x'))
        self.proc=subprocess.Popen(bridge_arguments(),env=env,cwd=folder,stdout=log,stderr=log)
        self.resources.callback(self._stop_child)
        self.execution=(CarMotionTask(dict(plan,schema='car_cooperative_motion_v1'),channel_key=key,standalone=True)
            if self.car_only else AirGroundExecution(plan,source_snapshot=self.snapshot,
                car_publish=self.publish,channel_key=key,origin_monotonic=origin,host_clock_ns=time.monotonic_ns))
        from readiness_console import LatestStatusPrinter
        self.trace=LatestStatusPrinter((self.output/('car_motion.jsonl' if self.car_only else 'cooperative.jsonl')).open('x'),capacity=128,close_stream=True)
        self.resources.callback(self.trace.close,drain=True)

    def _subscribe(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_msgs.msg import String
        from geometry_msgs.msg import PoseStamped
        from vicon_frame_sample_capture import TOPIC
        self.string_type=String;self.uav_topic=TOPIC
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
        context=Context();rclpy.init(args=[],context=context,domain_id=DOMAIN,signal_handler_options=SignalHandlerOptions.NO)
        self.resources.callback(context.try_shutdown)
        self.node=Node(OWNER,namespace=NAMESPACE,context=context,use_global_arguments=False,enable_rosout=False)
        self.resources.callback(self.node.destroy_node)
        self.executor=SingleThreadedExecutor(context=context);self.executor.add_node(self.node)
        self.resources.callback(self.executor.shutdown,timeout_sec=0.)
        self.publisher=self.node.create_publisher(String,'actuator_command',qos)
        for kind in ('feedback','transport'):
            topic='plant_feedback' if kind=='feedback' else 'transport_status'
            self.node.create_subscription(String,topic,lambda msg,k=kind:self.cache.receive_status(k,msg),qos)
        # The pinned receiver publishes all subjects on this same connection.
        # communicator.cpp skips SDK-failed/occluded segment poses. Missing car
        # samples expire on their ORIGINAL timestamps; no generic PoseStamped
        # visibility assumption is made for any other publisher.
        sensor=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
        sub=self.source.node.create_subscription(PoseStamped,CAR_TOPIC,
            lambda msg:self.cache.receive_car(msg,time.monotonic_ns()),sensor)
        self.resources.callback(self.source.node.destroy_subscription,sub)
        # Only graph inspection on domain 0; no shared receiver lifecycle.
        from mecanum_wand_4wse_hardware.nodes import RawSource
        raw_context=Context();rclpy.init(args=[],context=raw_context,domain_id=0,signal_handler_options=SignalHandlerOptions.NO)
        self.resources.callback(raw_context.try_shutdown)
        self.raw_graph=Node('air_ground_conflict_observer',context=raw_context,use_global_arguments=False,enable_rosout=False)
        self.resources.callback(self.raw_graph.destroy_node)
        self.legacy_audit=lambda:RawSource.audit(self.raw_graph)

    @staticmethod
    def _owners(node,topic):
        return [(i.node_name,i.node_namespace) for i in node.get_publishers_info_by_topic(topic)]

    def audit_graph(self):
        expected={'actuator_command':(OWNER,NAMESPACE),'plant_feedback':(BRIDGE,NAMESPACE),
                  'transport_status':(BRIDGE,NAMESPACE)}
        ready=True;conflicts=list(self.legacy_audit())
        for topic,owner in expected.items():
            peers=self._owners(self.node,NAMESPACE+'/'+topic)
            if peers!=[owner]:
                ready=False
                # DDS can discover the endpoint before ROS attaches its node
                # identity. Before first readiness, a single unnamed endpoint
                # is incomplete discovery: wait at zero, never grant authority.
                pending_identity=(not self.graph_seen and
                    peers==[('_NODE_NAME_UNKNOWN_','_NODE_NAMESPACE_UNKNOWN_')])
                if peers and not pending_identity:conflicts.append('unexpected/duplicate publisher: '+topic+
                    '; expected='+repr([owner])+'; observed='+repr(peers))
        for topic in ('candidate','cmd_vel','operator_lease','mode_claim'):
            if self.node.count_publishers(NAMESPACE+'/'+topic):conflicts.append('competing car control: '+topic)
        for topic in ((CAR_TOPIC,) if getattr(self,'car_only',False) and not getattr(self,'alignment_only',False) else (CAR_TOPIC,self.uav_topic)):
            peers=self._owners(self.source.node,topic)
            if peers!=[('b3_vicon_isolated_receiver','/')]:
                ready=False
                if peers:conflicts.append('unexpected Vicon source: '+topic+'; observed='+repr(peers))
        if conflicts:raise ValueError('; '.join(conflicts))
        # Initial DDS discovery is not a fault. After discovery, persistent
        # disappearance is handled through the existing task failure.
        now=time.monotonic()
        if ready:self.graph_missing_since=None
        elif getattr(self,'graph_missing_since',None) is None:self.graph_missing_since=now
        # Brief DDS discovery gaps do not invalidate still-fresh measurements.
        # Feedback/pose timestamps and duplicate-owner checks stay independent.
        self.cache.graph_ok=ready or bool(self.graph_seen and now-self.graph_missing_since<2.)
        self.graph_seen |= ready
        return ready

    def poll(self):
        """Bounded car callbacks; errors must not stop the UAV's Vicon service."""
        if self.closed:return
        try:
            if self.proc.poll() is not None:raise RuntimeError('owned car bridge exited; see car_bridge/bridge.log')
            deadline=time.monotonic()+.002
            for _ in range(32):
                self.executor.spin_once(timeout_sec=0.)
                if time.monotonic()>=deadline:break
            now=time.monotonic()
            if now-self.last_audit>=.1:
                self.last_audit=now;self.audit_graph()
        except Exception as exc:
            self.cache.graph_ok=False;self.cache.fault=self.cache.fault or str(exc)

    def snapshot(self):
        if self.cache.fault:raise ValueError(self.cache.fault)
        if not self.graph_seen:return None
        return self.cache.snapshot()

    def publish(self,wire):
        if self.proc.poll() is not None:raise RuntimeError('car bridge no longer running')
        self.publisher.publish(self.string_type(data=wire))

    def step_car(self, *, start=False):
        """Same bridge owner, empty-car route; never creates flight observations."""
        if not self.car_only:raise ValueError('standalone car resources required')
        self.source.poll();self.poll()
        data=self.snapshot()
        steady=time.monotonic();ros=self.source.now_ros_ns()*1e-9
        task=self.execution
        if data is None:
            transport=self.cache.transport
            if transport and (transport.get('local_stop') or transport.get('fault')):
                (self.output/'car_connection_failure.json').write_text(json.dumps(transport,indent=2)+'\n')
                raise RuntimeError('小车连接准备失败：'+str(transport.get('fault') or 'local_stop')+
                                   '；见 car_connection_failure.json')
            if task.started_at is not None:raise RuntimeError('小车运行数据中断')
            if start:raise ValueError('小车尚未就绪；不缓存 START')
            self.publish(task.car.signer.encode(ZERO,steady,ros,state='STANDBY',permit=False))
            status=task.status()
        else:
            if start and task.phase!='WAIT_START':raise ValueError('小车尚未就绪或本轮已开始；不缓存 START')
            if getattr(self,'alignment_only',False):
                self.check_ground_alignment(data,steady,ros,start=start)
            status=task.step(steady=steady,ros=ros,car=data['car'],car_velocity=data['car_velocity'],
                frame_revision=data['frame_revision'],feedback=data['feedback'],transport=data['transport'],
                graph_ok=data['graph_ok'],start=start)
            self.publish(status['car_control']['signed_authority'])
        self.dispatch_calls+=1;self.last_car_status=status
        key=(status['phase'],status['fault'],status['route_index'],status['waiting_for'])
        if self.trace and 'car' in self.cache.poses and (steady-self.last_log>=.1 or key!=self.last_log_key):
            self.last_log=steady;self.last_log_key=key;car=self.cache.poses['car']
            public=dict(status,car_control={k:v for k,v in (status['car_control'] or {}).items() if k!='signed_authority'})
            self.trace.publish(json.dumps(dict(at_ns=round(steady*1e9),synthetic=False,
                frame_id=car.frame_id,frame_revision=self.cache.plan['frame_revision'],
                car_position_map_m=car.position,car_yaw_rad=_yaw(car.quaternion_xyzw),
                car_stamp_ns=car.stamp_ns,car_received_ns=car.received_ns,status=public,
                signed_dispatch_calls=self.dispatch_calls,actual_sent_to_fc=0,
                discarded_old_status=self.cache.discarded_status),ensure_ascii=False,allow_nan=False))
        return status

    def check_ground_alignment(self,data,steady,ros,*,start=False):
        """Paired Vicon only: the unpowered UAV is a stationary obstacle."""
        from follow_target_generator import ground_approach_clear, _wrap
        seq=self.alignment_sequence
        for kind in ('car','uav'):
            seq._pose(data[kind],seq.config[kind+'_source'],round(steady*1e9),round(ros*1e9))
        if start:
            resolved=seq.resolve_start(data['car'],data['uav'],data['car_rigid'],data['uav_rigid'])
            self.execution.bind_resolved_start(resolved)
            (self.output/'alignment_resolved.json').write_text(json.dumps(resolved,indent=2)+'\n')
        frozen=seq.resolved_start
        if frozen:
            if (math.dist(data['uav'].position,frozen['uav_ground_position_map_m'])>seq.config['car_arrival_m'] or
                abs(_wrap(_yaw(data['uav'].quaternion_xyzw)-frozen['uav_heading_map_rad']))>seq.config['car_yaw_tolerance_rad']):
                raise ValueError('stationary_uav_moved_after_start_alignment')
            if not ground_approach_clear(data['car'],frozen['ground_approach']):
                raise ValueError('approach_vehicle_clearance_lost')

    def record(self,runtime,status):
        now=time.monotonic()
        key=(status.get('phase'),self.execution.last_status['phase'],
             self.execution.describe().get('car_task_phase'),
             self.execution.last_status.get('landing_waiting_for'),status.get('failure'),status.get('done'))
        if not self.trace or (now-self.last_log<.05 and key==self.last_log_key):return
        poses=self.cache.poses
        if set(poses)!={'car','uav'}:return
        self.last_log=now;self.last_log_key=key
        p=self.execution.sequence.config
        row=dict(at_ns=time.monotonic_ns(),synthetic=False,frame_id=p['frame_id'],frame_revision=p['frame_revision'],
            car_position_map_m=poses['car'].position,uav_position_map_m=poses['uav'].position,
            car_yaw_rad=_yaw(poses['car'].quaternion_xyzw),uav_yaw_rad=_yaw(poses['uav'].quaternion_xyzw),
            car_stamp_ns=poses['car'].stamp_ns,uav_stamp_ns=poses['uav'].stamp_ns,
            car_received_ns=poses['car'].received_ns,uav_received_ns=poses['uav'].received_ns,
            observed_armed=(runtime.inputs.hb['armed'] if runtime.inputs.hb and
                0<=runtime.now-runtime.inputs.hb['at']<2 else None),
            observed_landed=(runtime.inputs.land['landed'] if runtime.inputs.land and
                0<=runtime.now-runtime.inputs.land['at']<.5 else None),
            observed_car_stopped=self.execution.describe()['car_stopped'],
            status=dict(self.execution.last_status,
                phase=(self.execution.car_task.phase if self.execution.car_task and not self.execution.started
                       else self.execution.last_status['phase']),
                uav_phase=self.execution.last_status['phase'],
                car_task_phase=self.execution.describe()['car_task_phase'],
                car_route_index=self.execution.describe()['car_route_index'],
                fault=status.get('failure') or self.execution.fault or self.cache.fault),
            source_fault=self.cache.fault,
            # Observability only: expose the SAME frozen body-map -> local-NED
            # transform used by the controller, never estimate a new alignment.
            position_comparison=dict(
                reference='uav_body',alignment=self.execution.alignment,
                boot_session=runtime.inputs.boot_session,
                valid=bool(self.execution.alignment and self.execution.binding
                    and self.execution.binding[0]==runtime.inputs.boot_session
                    and not (runtime.failure or runtime.inputs.fault or self.execution.fault or self.cache.fault)),
                reason=(runtime.failure or runtime.inputs.fault or self.execution.fault or self.cache.fault
                        or (None if self.execution.alignment else 'waiting_for_flight_alignment'))),
            car_control={k:v for k,v in (self.execution.car_status or {}).items()
                if k not in ('signed_authority','actual_sent_to_car','live_transport_connected')},
            signed_dispatch_calls=self.execution.car_publish_calls,
            bridge_reported_transport=self.cache.transport)
        self.trace.publish(json.dumps(row,ensure_ascii=False,allow_nan=False))

    def _stop_child(self):
        import subprocess
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill();self.proc.wait(timeout=2)
                self.errors.append('owned car bridge required forced shutdown')

    def close(self):
        """Measured car stop; joint flight calls this after UAV close."""
        if self.closed:return self.stop_result
        self.closing=True;started=time.monotonic();confirmed=False
        try:
            self.execution.car.supervisor.request_stop(started,'JOINT_CLOSE')
            while time.monotonic()-started<3.:
                steady=time.monotonic();ros=self.node.get_clock().now().nanoseconds*1e-9
                self.publish(self.execution.car.signer.encode(ZERO,steady,ros,state='STOPPING',permit=False))
                self.poll()
                try:self.source.poll()
                except Exception as exc:
                    if str(exc) not in self.errors:self.errors.append(str(exc))
                f=self.cache.feedback;car=self.cache.poses.get('car')
                if f and car:
                    # Reuse the existing full standstill window, not just one
                    # disabled packet and not the old cached startup result.
                    task=self.execution if self.car_only else self.execution.car_task
                    self.execution.counter=max(self.execution.counter,task.counter if task else 0)+1
                    i=self.execution.counter
                    steady=time.monotonic();ros=self.node.get_clock().now().nanoseconds*1e-9
                    plan=self.cache.plan
                    intent=RouteIntent(i,steady,ros,plan['frame_id'],plan['frame_revision'],'stop',
                        task.index if task else self.execution.last_status['route_index'])
                    state=self.execution.car.step(steady=steady,ros=ros,car=car,
                        frame_revision=plan['frame_revision'],feedback=f,
                        lease=Lease('air_ground_existing_owner',i,steady,ros,False),intent=intent,
                        graph_ok=self.cache.graph_ok)
                    if f['issued_steady']>=started and state['stop_confirmed']:
                        confirmed=True;break
                time.sleep(.003)
        except Exception as exc:self.errors.append(str(exc))
        finally:
            try:self.resources.close()
            except Exception as exc:self.errors.append(str(exc))
            self.closed=True
            self.stop_result=dict(car_stop_confirmed=confirmed,errors=self.errors,
                cooperative_complete=(self.execution.status()['complete'] if self.car_only else self.execution.describe()['complete']),
                last_transport=getattr(self.cache,'transport',None),
                last_feedback=getattr(self.cache,'feedback',None),
                hardware_motion_success_claimed=False)
            (self.output/('car_stop.json' if self.car_only else 'joint_stop.json')).write_text(json.dumps(self.stop_result,ensure_ascii=False,indent=2)+'\n')
        if not confirmed or self.errors:raise RuntimeError('小车停车/资源收尾未完全确认，见 '+('car_stop.json' if self.car_only else 'joint_stop.json'))
        return self.stop_result
