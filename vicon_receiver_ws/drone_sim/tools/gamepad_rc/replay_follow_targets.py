#!/usr/bin/env python3
"""Offline JSON/rosbag2 reader -> bounded target references, never ROS playback.

No ROS nodes, publishers, serial ports, sockets, MAVLink or flight grants.
rosbag2_py (optional) opens local MCAP files only. Bag receive time is a recorded
receive clock, NOT exposure time; virtual monotonic time is explicitly replay.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'ros2_ws/src/uav_coord_bridge'))
from follow_target_generator import FollowTargetGenerator, Limits, Mode, TargetPose

MAX_ROWS = 100_000


def car_motion_demo(config, *, initial_car=(-.25,-.15,0.), duration_s=180.,
                    period_s=.02, hook=None, auto_start=True, standalone=False):
    """Car program + real signed verifier, synthetic car/flight observations.

    Flight handoffs below are deliberately test inputs, never real clearance
    evidence. No drone mission, autopilot or hardware resources are started.
    """
    from car_route_adapter import CarMotionTask, FlightProgress, ZERO
    from mecanum_wand_4wse_hardware.commissioning import TransportPolicy
    from follow_target_generator import _rotate_xy, _wrap
    task=CarMotionTask(config,channel_key='29'*32,standalone=standalone)
    policy=TransportPolicy('29'*32,task.car.config.stage)
    car=[initial_car[0],initial_car[1],task.car.car_ground_z_map_m];yaw=initial_car[2]
    command=ZERO;phases=[];rows=[];since=0.;previous=None
    for i in range(int(duration_s/period_s)+1):
        t=i*period_s;steady=10.+t;ros=1_800_000_000.+steady
        vx,vy=_rotate_xy((command.vx,command.vy),yaw)
        car[0]+=vx*period_s;car[1]+=vy*period_s;yaw=_wrap(yaw+command.wz*period_s)
        car[2]=task.car.ground_z_at(*car[:2])  # Synthetic ground, never an altered live observation.
        if task.phase!=previous:since=t;previous=task.phase
        # The tests keep distinct waiting intervals; production must supply
        # the real flight-owner message with this task id and source times.
        departed=task.phase in ('ROUTE','WAIT_LANDING','COMPLETE') or (
            task.phase=='WAIT_TAKEOFF' and task.last_car and task.last_car['stop_confirmed'] and t-since>=2.)
        landed=task.phase=='COMPLETE' or (task.phase=='WAIT_LANDING'
            and task.last_car and task.last_car['stop_confirmed'] and t-since>=2.)
        f=None if standalone else FlightProgress(task.task_id,i,steady,ros,bool(departed and not landed),
            bool(not departed or landed),bool(departed and not landed))
        inp=dict(steady=steady,ros=ros,car=TargetPose(config['car_source'],config['frame_id'],
            round(ros*1e9),round(steady*1e9),tuple(car),(0.,0.,math.sin(yaw/2),math.cos(yaw/2)),True,False),
            car_velocity=(vx,vy,command.wz),frame_revision=config['frame_revision'],
            feedback=dict(sequence=i,issued_steady=steady,issued_ros=ros,output_zero=command==ZERO,
                enabled=policy.active(steady),speed=math.hypot(command.vx,command.vy),yaw_rate=abs(command.wz),fault_bits=0),
            transport=dict(issued_steady=steady,issued_ros=ros,ready=not policy.active(steady),fresh=True,
                local_stop=False,native_fault_bits=0,fault='',armed_epoch=policy.epoch,
                policy_state=(policy.envelope or {}).get('state')),
            graph_ok=True,flight=f,start=auto_start and task.phase=='WAIT_START')
        if hook:hook(t,task,inp)
        status=task.step(**inp)
        accepted=policy.receive(status['car_control']['signed_authority'],steady,ros)
        command=policy.command if policy.active(steady) else ZERO
        if not phases or phases[-1]['phase']!=status['phase']:
            phases.append(dict(at_s=t,phase=status['phase']))
        public=dict(status,car_control={k:v for k,v in status['car_control'].items() if k!='signed_authority'})
        rows.append(dict(at_s=t,at_ns=round(steady*1e9),synthetic=True,
            car_position_map_m=list(car),car_yaw_rad=yaw,frame_id=config['frame_id'],
            frame_revision=config['frame_revision'],status=public,transport_accepted=accepted,
            measured_feedback=inp['feedback'],observed_flight=None if inp['flight'] is None else vars(inp['flight']),
            applied_body_mps_rps=[command.vx,command.vy,command.wz]))
        if not accepted or task.phase in task.terminal:break
    report=dict(schema='car_motion_offline_replay_v1',synthetic=True,complete=task.phase=='COMPLETE',
        failure=task.first_fault or (None if accepted else 'TRANSPORT_REJECTED'),phase=task.phase,
        phases=phases,duration_s=t,initial_car_xyyaw=initial_car,final_car_xyyaw=[*car[:2],yaw],
        fixed_takeoff_point=config['takeoff_car_world_xyyaw'],fixed_route=config['route_world_xyyaw'],
        all_transport_accepted=all(r['transport_accepted'] for r in rows),
        devices_opened=0,actual_sent_to_car=0,actual_sent_to_fc=0,physical_validated=False,
        flight_handoffs='none; standalone car' if standalone else 'synthetic test inputs only',effective_offline_config=config)
    return report,rows


def save_car_motion_demo(config, output, *, initial_car=(-.25,-.15,0.)):
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    report,rows=car_motion_demo(config,initial_car=initial_car)
    (output/'analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    with (output/'car_motion.jsonl').open('x') as stream:
        for row in rows:stream.write(json.dumps(row,ensure_ascii=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='effective_offline_config'},ensure_ascii=False,indent=2))
    return 0 if report['complete'] else 2


def cooperative_demo(config, *, initial_car=(0.,0.,.15,0.), relative_uav=(.03,.02,.15,.2),
                     duration_s=180., period_s=.02, hook=None, car_control=False):
    """Two explicit kinematic stand-ins, never two hardware processes.

    Only validates sequencing, target continuity and rendezvous geometry.
    Actual airframe dynamics, car firmware, shared-radio load and live adapters
    are deliberately not simulated by a claim of 'hardware ready'.
    """
    from follow_target_generator import AirGroundSequence, _rotate_xy, _wrap
    seq = AirGroundSequence(config)
    car = list(initial_car[:3]); cyaw = float(initial_car[3])
    rel = _rotate_xy(relative_uav[:2],cyaw)
    uav = [car[0]+rel[0],car[1]+rel[1],car[2]+relative_uav[2]]
    uyaw = _wrap(cyaw+relative_uav[3])
    cv=[0.,0.,0.]; uv=[0.,0.,0.]
    armed,landed=False,True
    desired = None
    rows=[]; phases=[]
    car_limits = dict(config['follow_limits'])
    # The same existing reference governor is used for the software car plant.
    car_limits.update(speed_mps=config['car_speed_mps'],acceleration_mps2=config['car_acceleration_mps2'])
    car_plant = FollowTargetGenerator(initial_xy=car[:2],ground_z_map_m=0.,height_m=1.,
        yaw_map_rad=cyaw,limits=Limits(**car_limits),wand_source='unused',car_source='car01')
    adapter=policy=car_status=None
    car_command=(0.,0.,0.); car_was_ready=False; car_control_fault=''
    if car_control:
        from car_route_adapter import CarRouteAdapter, RouteIntent
        from mecanum_wand_4wse_control.safety_core import Lease
        from mecanum_wand_4wse_hardware.commissioning import TransportPolicy
        # Deterministic key is ONLY for this in-memory test. No live owner or
        # socket exists; a live runtime must generate its own session secret.
        adapter=CarRouteAdapter(config,channel_key='17'*32)
        policy=TransportPolicy('17'*32,adapter.config.stage)
    def sample(source,p,yaw,now,ros):
        return TargetPose(source,config['frame_id'],ros,now,tuple(p),
            (0.,0.,math.sin(yaw/2),math.cos(yaw/2)),True,False)
    for i in range(int(duration_s/period_s)+1):
        elapsed=i*period_s; now=10**9+round(elapsed*1e9); ros=1_800_000_000_000_000_000+round(elapsed*1e9)
        if desired:
            goal=desired['car_goal_map_xyyaw']
            if adapter:
                # Only the existing transport verifier's accepted body command
                # drives the kinematic plant. Reference positions never do.
                c,s=math.cos(cyaw),math.sin(cyaw)
                vx,vy,wz=car_command
                cv=[c*vx-s*vy,s*vx+c*vy,wz]
                car[:2]=[car[k]+cv[k]*period_s for k in (0,1)]
            else:
                car_plant.set_mode(Mode.HOVER,hold_xy=tuple(goal[:2]) if goal else car_plant.xy)
                car_output=car_plant.step(now_ns=now,now_source_ns=ros)
                car[:2]=car_output['position_map_m'][:2]
                cv[:2]=car_output['velocity_xy_mps']
                delta=0. if not goal else _wrap(goal[2]-cyaw)
                cv[2]=max(-.20,min(.20,delta/period_s))
            cyaw=_wrap(cyaw+cv[2]*period_s)
            target=desired['uav_target_map_m']
            if desired['uav_action']=='takeoff': armed=True
            if target and armed:
                target=list(target)
                if desired['uav_action']=='land': target[2]=car[2]+relative_uav[2]
                for axis in range(3):
                    uv[axis]=max(-.25,min(.25,4*(target[axis]-uav[axis])))
                    uav[axis]+=uv[axis]*period_s
                delta=_wrap(desired['uav_yaw_map_rad']-uyaw)
                uyaw=_wrap(uyaw+max(-.3*period_s,min(.3*period_s,delta)))
                landed=uav[2]<=car[2]+relative_uav[2]+.005
                if desired['uav_action']=='land' and landed:
                    armed=False; uv=[0.,0.,0.]
        inputs=dict(now_ns=now,now_source_ns=ros,frame_revision=config['frame_revision'],
            car=sample(config['car_source'],car,cyaw,now,ros),
            uav=sample(config['uav_source'],uav,uyaw,now,ros),car_velocity=tuple(cv),uav_velocity=tuple(uv),
            car_ready=car_was_ready and not car_control_fault if adapter else True,uav_ready=True,
            car_stopped=bool(car_status and car_status['stop_confirmed']) if adapter else math.hypot(*cv[:2])<.005 and abs(cv[2])<.005,
            armed=armed,landed=landed,start=seq.phase=='WAIT_START' if adapter else i==1)
        if hook: hook(elapsed,seq,inputs)
        desired=seq.step(**inputs)
        if adapter:
            steady,source_now=now*1e-9,ros*1e-9
            active=bool(car_status and car_status['state']=='ACTIVE')
            feedback=dict(sequence=i,issued_steady=steady,issued_ros=source_now,
                output_zero=not any(car_command),enabled=active,
                speed=math.hypot(*car_command[:2]),yaw_rate=abs(car_command[2]),fault_bits=0)
            held=(adapter.supervisor.state in ('ARMED','ACTIVE') and bool(adapter.supervisor.epoch)
                  and adapter.gate.primed_epoch==adapter.supervisor.epoch
                  and policy.epoch==adapter.supervisor.epoch)
            lease=Lease('offline_cooperative_operator',i,steady,source_now,held)
            intent=RouteIntent.from_sequence(desired,i,steady,source_now,adapter.plan)
            car_status=adapter.step(steady=steady,ros=source_now,car=inputs['car'],
                frame_revision=inputs['frame_revision'],feedback=feedback,lease=lease,intent=intent,graph_ok=True)
            car_was_ready |= car_status['ready']
            accepted=policy.receive(car_status['signed_authority'],steady,source_now)
            car_command=(0.,0.,0.)
            if policy.active(steady):
                v=policy.command; car_command=(v.vx,v.vy,v.wz)
            if not car_control_fault and (car_status['fault'] or policy.fault or not accepted):
                car_control_fault=car_status['fault'] or policy.fault or 'AUTHORITY_REJECTED'
            if car_status['ready'] and desired['car_action']=='route':
                ok,why=adapter.prepare(steady,source_now)
                if not ok: car_control_fault=why
        if not phases or phases[-1]['phase']!=desired['phase']:
            phases.append(dict(at_s=elapsed,phase=desired['phase']))
        rows.append(dict(at_ns=now,synthetic=True,frame_id=config['frame_id'],
            frame_revision=config['frame_revision'],car_position_map_m=list(inputs['car'].position),
            uav_position_map_m=list(inputs['uav'].position),car_yaw_rad=cyaw,uav_yaw_rad=uyaw,
            observed_armed=inputs['armed'],observed_landed=inputs['landed'],
            observed_car_stopped=inputs['car_stopped'],status=desired))
        if adapter:
            rows[-1]['car_control']={k:v for k,v in car_status.items() if k!='signed_authority'}
            rows[-1]['car_control']['transport_accepted']=accepted
        if seq.phase in seq.terminal: break
    report=dict(schema='air_ground_software_rehearsal_v1',synthetic=True,
        result='离线车机流程完成；不代表实机联动已完成' if seq.complete and not car_control_fault else '离线流程未完成',
        complete=seq.complete and not car_control_fault,failure=car_control_fault or seq.fault,phases=phases,duration_s=elapsed,
        final_relative_yaw_rad=_wrap(uyaw-cyaw),initial_relative_yaw_rad=relative_uav[3],
        final_relative_xy_m=_rotate_xy((uav[0]-car[0],uav[1]-car[1]),-cyaw),
        initial_relative_xy_m=relative_uav[:2],actual_sent_to_fc=0,actual_sent_to_car=0,
        devices_opened=0,ros_publishers_created=0,live_adapters_connected=False,
        known_car_entry=config.get('car_entry'),effective_offline_config=config,
        car_control_chain='4WSE Supervisor + route input + FinalGate + AuthoritySigner + TransportPolicy' if adapter else 'kinematic reference only',
        car_profile_sha256=adapter.profile_sha256 if adapter else None,
        car_runtime_wired=False,car_hardware_bridge_constructed=False)
    return report,rows


def save_cooperative_demo(config, output, *, car_control=False):
    output=Path(output)
    output.mkdir(parents=True,exist_ok=False)
    report,rows=cooperative_demo(config,car_control=car_control)
    (output/'analysis.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    with (output/'cooperative.jsonl').open('x') as stream:
        for row in rows: stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='effective_offline_config'},ensure_ascii=False,indent=2))
    return 0 if report['complete'] else 2


def bag_records(path, *, topic, source, max_rows=MAX_ROWS):
    # This is NOT ros2 bag play: other topics (e.g. stored car commands) never
    # leave the local file reader. No rclpy initialization/executor exists here.
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if types.get(topic) not in ('geometry_msgs/msg/PoseStamped', 'nav_msgs/msg/Odometry'):
        raise ValueError('explicit_PoseStamped_or_Odometry_topic_required')
    cls = get_message(types[topic])
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    rows = []
    while reader.has_next():
        _, wire, received = reader.read_next()
        msg = deserialize_message(wire, cls)
        pose = msg.pose.pose if types[topic] == 'nav_msgs/msg/Odometry' else msg.pose
        p, q = pose.position, pose.orientation
        if len(rows) >= max_rows:
            raise ValueError('bounded_bag_capacity_exceeded_no_truncation')
        rows.append(dict(source=source, frame_id=msg.header.frame_id,
            stamp_ns=int(msg.header.stamp.sec)*10**9+int(msg.header.stamp.nanosec),
            received_source_ns=int(received), position=[p.x, p.y, p.z],
            quaternion_xyzw=[q.x, q.y, q.z, q.w], tracked=True, occluded=False))
    return rows, {'topic': topic, 'type': types[topic],
                  'visibility_evidence': 'not_in_PoseStamped_or_Odometry_assumed_for_offline_replay_only',
                  'receive_time': 'bag_record_timestamp_not_exposure',
                  'contains_live_authorization': False}


def replay(rows, config):
    if not rows or len(rows) > MAX_ROWS:
        raise ValueError('nonempty_bounded_replay_required')
    mode = Mode(config['mode'])
    limits = Limits(**config['limits'])
    generator = FollowTargetGenerator(initial_xy=config['initial_xy'],
        ground_z_map_m=config['ground_z_map_m'], height_m=config['height_m'],
        yaw_map_rad=config['yaw_map_rad'], limits=limits,
        wand_source=config['wand_source'], car_source=config['car_source'],
        self_source=config.get('self_source', 'MircoAir'), frame_id=config.get('frame_id', 'map'))
    generator.set_mode(mode, offset_xy=config.get('offset_xy', [0., 0.]))
    period = config.get('period_ns', 20_000_000)
    if type(period) is not int or period <= 0 or period*1e-9 > limits.max_step_s:
        raise ValueError('valid_replay_period_required')
    first, last = rows[0]['received_source_ns'], rows[-1]['received_source_ns']
    if type(first) is not int or type(last) is not int or last < first or (last-first)//period > MAX_ROWS:
        raise ValueError('invalid_or_unbounded_replay_window')
    previous = -1
    previous_stamp = None
    duplicate_stamps = 0
    for r in rows:
        if type(r['received_source_ns']) is not int or r['received_source_ns'] < previous:
            raise ValueError('recorded_receive_clock_regression_not_sorted_away')
        previous = r['received_source_ns']
        stamp = r['stamp_ns']
        if type(stamp) is not int or stamp <= 0 or previous_stamp is not None and stamp < previous_stamp:
            raise ValueError('recorded_source_clock_regression_not_skipped_by_resampling')
        duplicate_stamps += int(stamp == previous_stamp)
        previous_stamp = stamp
    outputs, index, sample, fault = [], 0, None, None
    ticks = list(range(first, last+1, period))
    if ticks[-1] != last:
        ticks.append(last)  # Include the actual final observations, no tail silently discarded.
    for source_now in ticks:
        now = 1_000_000_000+source_now-first  # explicitly virtual replay clock
        while index < len(rows) and rows[index]['received_source_ns'] <= source_now:
            r = rows[index]
            sample = TargetPose(source=r['source'], frame_id=r['frame_id'], stamp_ns=r['stamp_ns'],
                received_ns=1_000_000_000+r['received_source_ns']-first,
                position=tuple(r['position']), quaternion_xyzw=tuple(r['quaternion_xyzw']),
                tracked=r.get('tracked', False), occluded=r.get('occluded', True))
            index += 1
        try:
            output = generator.step(now_ns=now, now_source_ns=source_now, target=sample)
            outputs.append(output)
            if generator.fault:
                fault = generator.fault
                break
        except ValueError as exc:
            fault = str(exc)
            break
    speed = max((math.hypot(*o['velocity_xy_mps']) for o in outputs), default=0.)
    accel = max((math.hypot(*o['acceleration_xy_mps2']) for o in outputs), default=0.)
    inside = all(limits.contains(o['position_map_m'][:2]) for o in outputs)
    z = config['ground_z_map_m']+config['height_m']
    fixed_height = all(o['position_map_m'][2] == z for o in outputs)
    return {'schema': 'offline_follow_replay_v1', 'mode': mode.value, 'input_rows': len(rows),
            'consumed_rows': index, 'output_rows': len(outputs),
            'recorded_duration_s': (last-first)*1e-9, 'fault': fault,
            'input_duplicate_source_stamps': duplicate_stamps,
            'max_reference_speed_mps': speed, 'max_reference_acceleration_mps2': accel,
            'all_references_inside_area': inside, 'fixed_world_height': fixed_height,
            'target_issue_outputs': sum(o['reason'] is not None for o in outputs),
            'reference_bounds_passed': bool(outputs and inside and fixed_height
                and speed <= limits.speed_mps+1e-9 and accel <= limits.acceleration_mps2+1e-9),
            'replay_completed': fault is None,
            'flight_authorized': False, 'live_dispatch_permitted': False,
            'actual_sent_to_fc': 0, 'devices_opened': 0, 'ros_publishers_created': 0,
            'clock': 'virtual_monotonic_from_recorded_receive_deltas_not_live_or_exposure',
            'scope': 'offline_reference_bounds_not_real_aircraft_tracking_or_sensor_validation',
            'outputs': outputs}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    inputs = ap.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--input-json', type=Path)
    inputs.add_argument('--bag', type=Path)
    inputs.add_argument('--cooperative-demo', action='store_true',help='Offline paired car/UAV state simulation; --output is a NEW directory; never launches devices')
    inputs.add_argument('--car-motion-demo',action='store_true',help='Offline car approach/wait/route/landing-wait; no drone program or devices')
    ap.add_argument('--initial-car',nargs=3,type=float,metavar=('X','Y','YAW'),help='Synthetic car starting pose; does not relocate fixed world goals')
    ap.add_argument('--car-control',action='store_true',help='Use real 4WSE control and signed transport policy IN MEMORY with the cooperative demo')
    ap.add_argument('--topic')
    ap.add_argument('--source')
    ap.add_argument('--config', type=Path, required=True, help='Explicit OFFLINE example, not flight approval')
    ap.add_argument('--mode', choices=[m.value for m in Mode], help='Override offline example mode only')
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args(argv)
    if args.output.exists():
        ap.error('New output required; never overwrite historical evidence')
    config = json.loads(args.config.read_text())
    if args.car_motion_demo:
        if args.mode or args.topic or args.source or args.car_control:
            ap.error('Car motion demo already uses real car control; cannot mix other replay options')
        return save_car_motion_demo(config,args.output,initial_car=tuple(args.initial_car or (-.25,-.15,0.)))
    if args.initial_car:ap.error('--initial-car requires --car-motion-demo')
    if args.cooperative_demo:
        if args.mode or args.topic or args.source:
            ap.error('Cooperative demo cannot mix bag/mode options')
        return save_cooperative_demo(config,args.output,car_control=args.car_control)
    if args.car_control:
        ap.error('--car-control requires --cooperative-demo')
    if args.mode:
        config['mode'] = args.mode
    if args.bag:
        if not args.topic or not args.source:
            ap.error('--bag requires explicit --topic and --source')
        rows, metadata = bag_records(args.bag, topic=args.topic, source=args.source)
        source_files = sorted(args.bag.glob('*.mcap'))+[args.bag/'metadata.yaml']
    else:
        rows = json.loads(args.input_json.read_text())
        metadata, source_files = {'kind': 'explicit_offline_json'}, [args.input_json]
    report = replay(rows, config)
    report['effective_offline_config'] = config
    report['source_metadata'] = metadata
    report['source_sha256'] = {str(p.resolve()): hashlib.sha256(p.read_bytes()).hexdigest()
                               for p in source_files+[args.config]}
    with args.output.open('x') as f:
        json.dump(report, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.write('\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'outputs'}, ensure_ascii=False, indent=2))
    return 0 if report['replay_completed'] and report['reference_bounds_passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
