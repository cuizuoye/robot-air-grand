"""New resource interfaces only. Real controller/geometry, fake ROS/OS peers."""
from contextlib import ExitStack
from copy import deepcopy
import json
import math
from pathlib import Path
import socket
import subprocess
import sys
from types import SimpleNamespace as NS

import pytest

import air_ground_resources as a
import vicon_fixed_point_entry as entry
from uav_coord_bridge.mavlink_vision import transform_rigid_pose, quaternion_to_matrix

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(autouse=True)
def no_io(monkeypatch):
    def deny(*args,**kw):raise AssertionError('offline test attempted live network/process I/O')
    monkeypatch.setattr(socket,'socket',deny)
    monkeypatch.setattr(subprocess,'Popen',deny)
    monkeypatch.setattr(subprocess,'run',deny)


def config():
    plan=json.loads((ROOT/'plan_b/follow/air_ground_example.json').read_text())
    plan['geometry_is_example']=False
    plan['live_inputs'].update(measured=True,world_revision='SYNTHETIC_WORLD',
        car_rigid_revision='SYNTHETIC_CAR',uav_rigid_revision='SYNTHETIC_UAV')
    return plan


def uav():return dict(revision='SYNTHETIC_UAV',origin=[.03,-.02,.01],orientation=[0.,0.,.6,.8])


def test_original_car_launcher_imports_sibling_without_starting_hardware(monkeypatch):
    monkeypatch.setattr(sys,'path',[p for p in sys.path if p!=str(a.CAR_ENTRY.parent)])
    monkeypatch.delitem(sys.modules,'receiver_manager',raising=False)
    original=a.original_car_entry()
    assert Path(original.__file__).resolve()==a.CAR_ENTRY
    assert Path(sys.modules['receiver_manager'].__file__).resolve()==a.CAR_ENTRY.parent/'receiver_manager.py'
    assert callable(original.prepare_session) and callable(original.route_check)


def message(t=100.,p=(1.,2.,.15),q=(0.,0.,0.,1.),frame='map'):
    stamp=round(t*1e9)
    return NS(header=NS(frame_id=frame,stamp=NS(sec=stamp//10**9,nanosec=stamp%10**9)),
        pose=NS(position=NS(**dict(zip('xyz',p))),orientation=NS(**dict(zip('xyzw',q)))))


def frame(t=100.,at=10.,p=(1.,2.,.3),q=(0.,0.,0.,1.),**kw):
    return dict(subject='MircoAir',segment='MircoAir',frame_id='map',ros_stamp_ns=round(t*1e9),
        callback_ns=round(at*1e9),world_pose=[*p,*q],**dict(
            dict(sdk_ok=True,occluded=False,pose_published=True,queue_dropped=False),**kw))


def test_geometry_matches_existing_vpe_without_double_axis_conversion():
    p,q=(.2,-.5,1.),(.2,-.1,.3,math.sqrt(.86))
    cfg=uav();body,rotation=a.body_pose(p,q,cfg['origin'],cfg['orientation'])
    ned,wxyz=transform_rigid_pose(p,q,cfg['origin'],cfg['orientation'])
    assert ned==pytest.approx([body[0],-body[1],-body[2]])
    import numpy as np
    f=np.diag([1.,-1.,-1.]);nq=[*wxyz[1:],wxyz[0]]
    assert quaternion_to_matrix(nq)==pytest.approx(f@quaternion_to_matrix(rotation)@f)


def test_car_heading_and_origin_are_explicit_and_applied_once():
    plan=config();plan['live_inputs'].update(car_body_origin_in_rigid_m=[.1,0.,0.],
        car_body_orientation_in_rigid_xyzw=[0.,0.,math.sin(.3/2),math.cos(.3/2)])
    cache=a.JointMeasurements(plan,uav())
    cache.receive_car(message(q=(0.,0.,math.sqrt(.5),math.sqrt(.5))),10**10)
    p=cache.poses['car']
    assert p.position==pytest.approx([1.,2.1,.15])
    assert a._yaw(p.quaternion_xyzw)==pytest.approx(math.pi/2+.3)
    assert 'car' not in cache.velocity  # one position is not standstill evidence


def test_joint_snapshot_retains_matching_raw_rigid_centers():
    plan=config();plan['live_inputs']['car_body_origin_in_rigid_m']=[.015,0.,0.]
    cache=a.JointMeasurements(plan,uav())
    for t in (100.,100.1):
        cache.receive_car(message(t),round((t-90)*1e9))
        cache.receive_uav(frame(t,t-90))
    cache.feedback=cache.transport={}
    data=cache.snapshot()
    for kind,p in (('car',(1.,2.,.15)),('uav',(1.,2.,.3))):
        body,rigid=data[kind],data[kind+'_rigid']
        assert rigid.position==p and rigid.position!=body.position
        assert (rigid.stamp_ns,rigid.received_ns)==(body.stamp_ns,body.received_ns)
    cache.receive_uav(frame(0.,10.2,occluded=True,pose_published=False))
    assert not cache.rigid_poses['uav'].tracked and cache.snapshot() is None


def test_current_side_follow_plan_has_no_old_absolute_route_or_platform_requirement():
    plan=json.loads((ROOT/'plan_b/follow/air_ground_side_follow.current.json').read_text())
    result=a.validate_bindings(plan)
    assert result['configuration_complete'] and result['devices_opened']==0
    assert result['world_route_status']=='resolved_once_at_CAR_START'
    assert result['takeoff_car_world_xyyaw'] is None and result['route_world_xyyaw'] is None
    assert result['route_local_forward_left_m']==[[1.,0.],[0.,0.]]
    reference=json.loads((ROOT/'plan_b/follow/task_direction_reference.current.json').read_text())
    assert result['route_direction_mode']=='fixed_task_axes'
    assert result['route_heading_map_rad']==pytest.approx(reference['fixed_forward_yaw_map_rad'])
    assert plan['start_alignment']['uav_rigid_offset_car_forward_left_m']==[0.,-.639]
    assert plan['live_inputs']['car_body_origin_in_rigid_m']==[0.,0.,0.]
    with pytest.raises(ValueError,match='paired_uav'):a.validate_bindings(plan,car_only=True)


def test_source_coordinates_do_not_require_guessed_landing_dimensions(tmp_path):
    plan=config()
    plan={k:plan[k] for k in ('car_entry','car_source','uav_source','frame_id','frame_revision','live_inputs')}
    plan['schema']='air_ground_sources_v1'
    plan['live_inputs']['measured']=False
    cache=a.JointMeasurements(plan,uav())
    cache.receive_car(message(),10**10)
    cache.receive_uav(frame())
    assert set(cache.poses)=={'car','uav'} and cache.fault is None
    with pytest.raises(ValueError,match='source-only'):
        a.validate_bindings(plan,uav(),live=True,sources_only=True)
    with pytest.raises(ValueError,match='unknown_air_ground_plan'):
        a.JointResources(plan=plan,uav_config=uav(),source=None,output=tmp_path,
            origin_monotonic=10.,open_devices=True,site_ready=True)
    assert not list(tmp_path.iterdir())


def test_duplicate_retains_receipt_and_velocity_uses_source_time():
    cache=a.JointMeasurements(config(),uav());cache.receive_car(message(),10**10)
    cache.receive_car(message(100.1,(1.01,2.,.15)),10_110_000_000)
    original=cache.poses['car']
    cache.receive_car(message(100.1,(1.01,2.,.15)),19_000_000_000)
    assert cache.poses['car'] is original
    assert cache.velocity['car']==pytest.approx([.1,0.,0.])
    assert original.stamp_ns==100_100_000_000 and original.received_ns==10_110_000_000
    cache.receive_car(message(100.1,(1.02,2.,.15)),20_000_000_000)
    assert 'duplicate changed' in cache.fault


@pytest.mark.parametrize('mode',['regression','frame','nan','quaternion'])
def test_bad_car_sample_latches_first_reason(mode):
    cache=a.JointMeasurements(config(),uav());cache.receive_car(message(),10**10)
    msg=message(100.1)
    if mode=='regression':msg=message(99.)
    if mode=='frame':msg.header.frame_id='odom'
    if mode=='nan':msg.pose.position.x=float('nan')
    if mode=='quaternion':msg.pose.orientation.w=0.
    cache.receive_car(msg,11*10**9)
    fault=cache.fault;assert fault
    cache.receive_car(message(100.2),12*10**9)
    with pytest.raises(ValueError,match=fault):cache.snapshot()


def test_invalid_uav_frame_replaces_valid_without_fake_pose_or_stamp():
    cache=a.JointMeasurements(config(),uav())
    cache.receive_uav(frame());cache.receive_uav(frame(100.1,10.1))
    p=cache.poses['uav'];assert 'uav' in cache.velocity
    bad=frame(0.,10.2,occluded=True,pose_published=False,p=(0.,0.,0.),q=(0.,0.,0.,0.))
    cache.receive_uav(bad)
    assert not cache.poses['uav'].tracked and cache.poses['uav'].occluded
    assert cache.poses['uav'].stamp_ns==p.stamp_ns and cache.poses['uav'].position==p.position
    assert 'uav' not in cache.velocity and not cache.fault
    cache.receive_uav(frame(100.3,10.3));assert 'uav' not in cache.velocity
    cache.receive_uav(frame(100.4,10.4));assert 'uav' in cache.velocity


def test_status_payload_keeps_bridge_clocks_and_rejects_replay():
    cache=a.JointMeasurements(config(),uav())
    value=dict(issued_steady=10.,issued_ros=100.,sequence=3,output_zero=True)
    cache.receive_status('feedback',NS(data=json.dumps(value)))
    original=cache.feedback
    cache.receive_status('feedback',NS(data=json.dumps(value)))
    assert cache.feedback is original
    cache.receive_status('feedback',NS(data=json.dumps(dict(value,sequence=4))))
    assert 'status regression' in cache.fault


def test_live_open_requires_real_binding_before_any_io(tmp_path):
    plan=config();plan['live_inputs']['measured']=False
    assert not a.validate_bindings(plan)['measured']
    with pytest.raises(ValueError,match='实测'):
        a.JointResources(plan=plan,uav_config=uav(),source=None,output=tmp_path,
            origin_monotonic=10.,open_devices=True,site_ready=True)
    with pytest.raises(PermissionError):
        a.JointResources(plan=config(),uav_config=uav(),source=None,output=tmp_path,
            origin_monotonic=10.)
    with pytest.raises(ValueError,match='revision'):
        a.validate_bindings(config(),dict(uav(),revision='other'),live=True)
    with pytest.raises(ValueError,match='confirmed car entry'):
        a.validate_bindings(dict(config(),car_entry='/tmp/other-car.py'),live=True)
    assert not list(tmp_path.iterdir())


def test_original_bridge_command_has_no_second_controller_or_unbound_sender():
    argv=a.bridge_arguments()
    assert argv[2]==str(a.HOST/'node.py') and argv[3:5]==['--role','bridge']
    assert 'command_timeout_s:=0.1' in argv and 'wire_timeout_ms:=600' in argv
    assert 'network_interface:=wlx502b73a508c9' in argv
    assert not any(s in argv for s in ('supervisor','gate','--launch','--synthetic'))


class Graph:
    def __init__(self,owners=None):self.owners=owners or {}
    def get_publishers_info_by_topic(self,t):
        return [NS(node_name=n,node_namespace=ns) for n,ns in self.owners.get(t,[])]
    def count_publishers(self,t):return len(self.owners.get(t,[]))


def graph_rig():
    r=object.__new__(a.JointResources);r.cache=a.JointMeasurements(config(),uav())
    r.node=Graph();r.source=NS(node=Graph());r.uav_topic='/vicon_b3_check/frame_sample'
    r.legacy_audit=lambda:[];r.graph_seen=False
    return r


def ready_graph(r):
    for t,n in [('actuator_command',a.OWNER),('plant_feedback',a.BRIDGE),('transport_status',a.BRIDGE)]:
        r.node.owners[a.NAMESPACE+'/'+t]=[(n,a.NAMESPACE)]
    for t in (a.CAR_TOPIC,r.uav_topic):r.source.node.owners[t]=[('b3_vicon_isolated_receiver','/')]


def test_graph_waits_for_discovery_then_detects_missing_or_duplicate_owners():
    r=graph_rig();assert not r.audit_graph() and r.snapshot() is None
    ready_graph(r);assert r.audit_graph() and r.graph_seen
    r.node.owners.pop(a.NAMESPACE+'/plant_feedback')
    assert not r.audit_graph() and r.graph_seen
    ready_graph(r);r.source.node.owners[a.CAR_TOPIC]*=2
    with pytest.raises(ValueError,match='unexpected Vicon'):r.audit_graph()


@pytest.mark.parametrize('topic',['actuator_command','plant_feedback','transport_status'])
def test_startup_unnamed_endpoint_waits_without_authority_until_identity(topic):
    r=graph_rig();ready_graph(r)
    r.node.owners[a.NAMESPACE+'/'+topic]=[('_NODE_NAME_UNKNOWN_','_NODE_NAMESPACE_UNKNOWN_')]
    assert not r.audit_graph()
    assert not r.graph_seen and not r.cache.graph_ok and r.snapshot() is None
    ready_graph(r)
    assert r.audit_graph() and r.graph_seen and r.cache.graph_ok


@pytest.mark.parametrize('topic',['actuator_command','plant_feedback','transport_status'])
def test_unnamed_duplicate_or_identity_loss_is_still_conflict(topic):
    r=graph_rig();ready_graph(r)
    unknown=('_NODE_NAME_UNKNOWN_','_NODE_NAMESPACE_UNKNOWN_')
    r.node.owners[a.NAMESPACE+'/'+topic]=[unknown,unknown]
    with pytest.raises(ValueError,match='duplicate publisher'):r.audit_graph()
    ready_graph(r);assert r.audit_graph()
    r.node.owners[a.NAMESPACE+'/'+topic]=[unknown]
    with pytest.raises(ValueError,match='duplicate publisher'):r.audit_graph()


@pytest.mark.parametrize('topic',['candidate','cmd_vel','operator_lease','mode_claim'])
def test_existing_sign_controller_cannot_share_joint_authority(topic):
    r=graph_rig();ready_graph(r);r.node.owners[a.NAMESPACE+'/'+topic]=[('old',a.NAMESPACE)]
    with pytest.raises(ValueError,match='competing car control'):r.audit_graph()


def test_dead_car_child_records_fault_without_throwing_through_uav_pose_poll():
    r=graph_rig();r.closed=False;r.proc=NS(poll=lambda:1)
    r.poll()
    assert not r.cache.graph_ok and 'bridge exited' in r.cache.fault
    with pytest.raises(ValueError,match='bridge exited'):r.snapshot()


def fake_ros(monkeypatch,events):
    class Context:
        def try_shutdown(self):events.append(('shutdown',self.domain))
    def init(*,context,domain_id,**kw):context.domain=domain_id;events.append(('init',domain_id))
    class Node(Graph):
        def __init__(self,name,*,context,namespace='/',**kw):
            super().__init__();self.name=name;self.context=context;self.namespace=namespace
            self.subs={};self.pubs={};events.append(('node',name,context.domain))
        def destroy_node(self):events.append(('destroy',self.name))
        def create_publisher(self,typ,topic,qos):
            pub=NS(publish=lambda msg:events.append(('publish',topic,msg.data)))
            self.pubs[topic]=pub;return pub
        def create_subscription(self,typ,topic,cb,qos):
            self.subs[topic]=cb;return topic
        def destroy_subscription(self,sub):events.append(('unsubscribe',sub))
        def get_clock(self):return NS(now=lambda:NS(nanoseconds=100_000_000_000))
    class Executor:
        def __init__(self,**kw):pass
        def add_node(self,node):self.node=node
        def shutdown(self,**kw):events.append(('executor_shutdown',self.node.name))
        def spin_once(self,**kw):pass
    modules={'rclpy':NS(init=init),'rclpy.context':NS(Context=Context),'rclpy.node':NS(Node=Node),
        'rclpy.executors':NS(SingleThreadedExecutor=Executor),
        'rclpy.signals':NS(SignalHandlerOptions=NS(NO=0)),
        'rclpy.qos':NS(QoSProfile=lambda **kw:kw,ReliabilityPolicy=NS(RELIABLE=1,BEST_EFFORT=0),DurabilityPolicy=NS(VOLATILE=0)),
        'std_msgs.msg':NS(String=lambda **kw:NS(**kw)), 'geometry_msgs.msg':NS(PoseStamped=object),
        'mecanum_wand_4wse_hardware.nodes':NS(RawSource=NS(audit=lambda node:[]))}
    for name,mod in modules.items():monkeypatch.setitem(sys.modules,name,mod)
    return Node


def test_real_subscription_method_routes_both_poses_and_bridge_feedback(monkeypatch):
    events=[];Node=fake_ros(monkeypatch,events)
    r=graph_rig();r.resources=ExitStack()
    r.source.node=Node('existing_uav_owner',context=NS(domain=83))
    r._subscribe()
    assert r.source.node.subs.keys()=={a.CAR_TOPIC}
    assert r.node.subs.keys()=={'plant_feedback','transport_status'}
    assert r.node.pubs.keys()=={'actuator_command'}
    assert [e[1] for e in events if e[0]=='init']==[197,0]  # no second Vicon context/SDK
    r.source.node.subs[a.CAR_TOPIC](message())
    assert r.cache.poses['car'].source=='car01'
    for t in r.node.subs:r.node.subs[t](NS(data=json.dumps(dict(issued_steady=10.,issued_ros=100.))))
    assert r.cache.feedback and r.cache.transport
    r.resources.close()
    assert ('unsubscribe',a.CAR_TOPIC) in events
    assert ('destroy','existing_uav_owner') not in events


def test_existing_factory_binds_single_owner_and_keeps_vpe_when_car_fails(tmp_path,monkeypatch):
    import ekf_static_profile,preparation_authorization,ev_delay_vicon_source,mavlink_link
    import mavlink_readonly_probe,vicon_receiver_provenance
    events=[];created={}
    profile=json.loads((ROOT/'plan_b/fixed_point/vicon_profile.initial_trial.json').read_text())
    plan=config();plan['live_inputs']['uav_rigid_revision']=profile['initial_trial']['revision']
    profile_path=tmp_path/'profile.json';profile_path.write_text(json.dumps(profile))
    monkeypatch.setattr(ekf_static_profile,'require_mission_profile',lambda *a,**kw:None)
    monkeypatch.setattr(ekf_static_profile,'inspect_initial_trial_evidence',lambda *a,**kw:None)
    monkeypatch.setattr(preparation_authorization,'require_no_pending_recovery',lambda *a:None)
    monkeypatch.setattr(mavlink_readonly_probe,'FC_USB',NS(exists=lambda:False))
    monkeypatch.setattr(vicon_receiver_provenance,'verify_current',lambda:{'synthetic':True})
    class Source:
        def __init__(self,*args,**kw):self.latest=frame();self.on_frame=None;events.append('source_open')
        def poll(self):events.append('source_poll')
        def now_ros_ns(self):return 100_000_000_000
        def close(self):events.append('source_close')
    class Port:
        def __init__(self,*args,**kw):events.append('port_open')
        def check(self):pass
        def close(self):events.append('port_close')
    class Joint:
        def __init__(self,**kw):
            created['joint']=self;created['joint_args']=kw
            self.cache=a.JointMeasurements(kw['plan'],kw['uav_config']);self.execution=object()
        def poll(self):self.cache.fault='car offline';events.append('car_poll')
        def close(self):events.append('car_close')
    monkeypatch.setattr(ev_delay_vicon_source,'ViconSource',Source)
    monkeypatch.setattr(mavlink_link,'FixedPeerUDP',Port)
    monkeypatch.setattr(a,'JointResources',Joint)
    def build(**kw):created['build']=kw;return NS(start=NS(),require_native_start=lambda:None)
    monkeypatch.setattr(entry,'build_guarded_session',build)
    c=entry.open_pocket_session(profile_path=profile_path,output=tmp_path/'run',peer=('a',1),bind=('b',1),
        open_devices=True,cooperative_plan=plan,site_ready=True)
    b=created['build'];j=created['joint']
    assert b['cooperative_execution'] is j.execution
    assert b['origin_monotonic']==created['joint_args']['origin_monotonic']==c.origin_monotonic
    assert b['poll_pose']()==(100_000_000_000,'map',[1.,2.,.3],[0.,0.,0.,1.])
    assert j.cache.fault=='car offline' and events[-2:]==['car_poll','source_poll']
    c.resources.close();assert events[-3:]==['car_close','port_close','source_close']


def test_cli_joint_plan_is_offline_and_does_not_fall_back_to_independent_pair(capsys):
    assert entry.main(['--cooperative-plan',str(ROOT/'plan_b/follow/air_ground_example.json')])==0
    result=json.loads(capsys.readouterr().out)['cooperative']
    assert result['devices_opened']==0 and result['live_interfaces_implemented'] and not result['measured']
    with pytest.raises(SystemExit):
        entry.main(['--cooperative-plan',str(ROOT/'plan_b/follow/air_ground_example.json'),'--run'])


def test_cooperative_review_hashes_resource_wiring():
    from flight_execution_review import required_code_paths
    assert str(ROOT/'air_ground_resources.py') in required_code_paths('native_rc',cooperative=True)


def test_post_poll_clock_does_not_misclassify_new_callback_as_future():
    from test_air_ground_execution import Plant
    h=Plant();h.tick()
    before=h.joint.fault;assert not before
    data=h.snapshot()
    from dataclasses import replace
    for name in ('car','uav'):
        data[name]=replace(data[name],stamp_ns=data[name].stamp_ns+8_000_000,
                           received_ns=data[name].received_ns+8_000_000)
    data['feedback']=dict(data['feedback'],sequence=2,issued_steady=h.steady+.008,issued_ros=h.ros+.008)
    data['transport']=dict(data['transport'],issued_steady=h.steady+.008,issued_ros=h.ros+.008)
    h.joint.source_snapshot=lambda:data
    h.joint.car_publish=lambda wire:None
    # Real owner `now` precedes the callbacks. Its post-poll host clock is
    # explicit; neither source nor receipt clocks are rewritten to fit now.
    h.joint.update(h.o,now=h.now,ros_ns=round((h.ros+.009)*1e9),steady_ns=round((h.steady+.009)*1e9))
    assert not h.joint.fault and h.joint.latest['car'].received_ns==data['car'].received_ns


def resource_rig(tmp_path,monkeypatch,*,spawn_fail=False,loaded=False):
    import builtins
    events=[];Node=fake_ros(monkeypatch,events)
    original_open=builtins.open
    def local_open(path,*args,**kwargs):
        if str(path)=='/tmp/wand_4wse_browser_operator.lock':path=tmp_path/'owner.lock'
        return original_open(path,*args,**kwargs)
    monkeypatch.setattr(builtins,'open',local_open)
    def prepare(software,seconds,site,folder):
        assert (software,seconds,site)==(False,600,True)
        events.append('prepare_transport_profile');return {'SYNTHETIC_CHILD_ONLY':'1'}
    helper=NS(verify_release=lambda:events.append('verify_sealed_car'),
              route_check=lambda:events.append('read_only_route'),prepare_session=prepare)
    monkeypatch.setattr(a,'original_car_entry',lambda:helper)
    class Child:
        dead=False
        def poll(self):return 0 if self.dead else None
        def terminate(self):self.dead=True;events.append('terminate_owned_bridge')
        def wait(self,**kw):events.append('wait_owned_bridge');return 0
    def spawn(argv,**kw):
        events.append(('spawn',argv,kw['env']))
        if spawn_fail:raise OSError('injected spawn failure')
        return Child()
    monkeypatch.setattr(subprocess,'Popen',spawn)
    source=NS(node=Node('existing_uav_owner',context=NS(domain=83)),poll=lambda:None)
    folder=tmp_path/'session';folder.mkdir()
    if spawn_fail:
        with pytest.raises(OSError,match='injected spawn failure'):
            a.JointResources(plan=config(),uav_config=uav(),source=source,output=folder,
                origin_monotonic=10.,open_devices=True,site_ready=True)
        return None,events
    plan=config()
    if loaded:plan.update(takeoff_car_world_xyyaw=[0.,0.,0.],flight_status_timeout_s=3.,maximum_task_seconds=600)
    r=a.JointResources(plan=plan,uav_config=uav(),source=source,output=folder,
        origin_monotonic=10.,open_devices=True,site_ready=True)
    return r,events


def test_launch_only_original_bridge_and_cleanup_partial_open(tmp_path,monkeypatch):
    r,events=resource_rig(tmp_path,monkeypatch,spawn_fail=True)
    assert len([e for e in events if isinstance(e,tuple) and e[0]=='spawn'])==1
    assert ('unsubscribe',a.CAR_TOPIC) in events
    assert ('destroy','air_ground_owner') in events and ('destroy','existing_uav_owner') not in events
    assert ('shutdown',197) in events and ('shutdown',0) in events
    # The parent lock is released even if Popen failed; no stale ownership.
    import fcntl
    with (tmp_path/'owner.lock').open('a') as handle:
        fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)


@pytest.mark.parametrize('loaded',[False,True])
def test_resource_close_uses_actual_full_standstill_and_then_releases_child(tmp_path,monkeypatch,loaded):
    r,events=resource_rig(tmp_path,monkeypatch,loaded=loaded)
    if loaded:
        r.execution.car_task.counter=500
        r.execution.car.supervisor.inputs.intent=a.RouteIntent(500,10.,100.,
            r.execution.sequence.config['frame_id'],r.execution.sequence.config['frame_revision'],'stop',0)
    now=[10.]
    monkeypatch.setattr(a.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(a.time,'monotonic_ns',lambda:round(now[0]*1e9))
    monkeypatch.setattr(a.time,'sleep',lambda dt:now.__setitem__(0,now[0]+dt))
    r.node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=round((90.+now[0])*1e9)))
    count=[0]
    def poll():
        now[0]+=.001;count[0]+=1
        steady=now[0];ros=90.+steady
        r.cache.receive_car(message(ros,p=(1.,2.,r.execution.car.config.ground_z_m)),round(steady*1e9))
        r.cache.receive_status('feedback',NS(data=json.dumps(dict(sequence=count[0],
            issued_steady=steady,issued_ros=ros,enabled=False,output_zero=True,
            speed=0.,yaw_rate=0.,fault_bits=0))))
        r.cache.graph_ok=True
    r.poll=poll
    result=r.close()
    if loaded:assert r.execution.car.supervisor.inputs.intent.sequence>500
    assert result['car_stop_confirmed'] and not result['cooperative_complete']
    assert .5<=now[0]-10.<1.5  # one packet / elapsed lease isn't called stopped
    assert events.index('terminate_owned_bridge')<events.index(('destroy',a.OWNER))
    assert r.closed and r.close() is result
    published=[json.loads(e[2])['payload'] for e in events if isinstance(e,tuple) and e[0]=='publish']
    assert published and all(not p['permit'] and not any(p['command'].values()) for p in published)
    assert all(p['state']=='STOPPING' for p in published)
    saved=json.loads((r.output/'joint_stop.json').read_text());assert saved==result


def test_close_missing_feedback_never_claims_stop_and_still_cleans_owned_resources(tmp_path,monkeypatch):
    r,events=resource_rig(tmp_path,monkeypatch)
    now=[10.]
    monkeypatch.setattr(a.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(a.time,'sleep',lambda dt:now.__setitem__(0,now[0]+.1))
    r.poll=lambda:None
    with pytest.raises(RuntimeError,match='停车'):
        r.close()
    assert r.closed and not r.stop_result['car_stop_confirmed']
    assert 'terminate_owned_bridge' in events and ('destroy','existing_uav_owner') not in events


def test_live_record_matches_existing_console_and_marks_stale_state_unknown(tmp_path,monkeypatch):
    from drone_console import ConsoleModel
    r,events=resource_rig(tmp_path,monkeypatch)
    r.cache.receive_car(message(),10**10);r.cache.receive_uav(frame())
    runtime=NS(now=10.,failure=None,inputs=NS(boot_session='test',fault=None,hb=dict(at=9.9,armed=False),land=dict(at=8.,landed=True)))
    r.record(runtime,{'failure':None});r.resources.close()
    rows=[json.loads(line) for line in (r.output/'cooperative.jsonl').read_text().splitlines()]
    assert len(rows)==1 and rows[0]['synthetic'] is False
    assert rows[0]['observed_armed'] is False and rows[0]['observed_landed'] is None
    model=ConsoleModel();model.ingest('cooperative.jsonl',rows[0])
    assert model.cooperative_synthetic is False


def test_phase_change_is_logged_even_inside_trace_throttle(tmp_path,monkeypatch):
    r,_=resource_rig(tmp_path,monkeypatch)
    r.cache.receive_car(message(),10**10);r.cache.receive_uav(frame())
    monkeypatch.setattr(a.time,'monotonic',lambda:10.)
    runtime=NS(now=10.,failure=None,inputs=NS(boot_session='test',fault=None,hb=None,land=None))
    r.record(runtime,{'phase':'LAND','done':False})
    r.execution.last_status=dict(r.execution.last_status,phase='COMPLETE',complete=True)
    r.record(runtime,{'phase':'COMPLETE','done':True})
    r.resources.close()
    rows=[json.loads(line) for line in (r.output/'cooperative.jsonl').read_text().splitlines()]
    assert [r['status']['phase'] for r in rows]==['WAIT_READY','COMPLETE']


def test_record_exposes_same_control_alignment_and_invalidates_on_failure(tmp_path,monkeypatch):
    from uav_coord_bridge.target_mapping import make_alignment
    r,_=resource_rig(tmp_path,monkeypatch)
    r.cache.receive_car(message(),10**10);r.cache.receive_uav(frame())
    alignment=make_alignment([1,2,3],[4,5,6],.2,-.3)
    r.execution.alignment=alignment;r.execution.binding=('test','rigid',(0,0,0))
    runtime=NS(now=10.,failure=None,inputs=NS(boot_session='test',fault=None,hb=None,land=None))
    r.record(runtime,{'phase':'HOVER'})
    runtime.failure='ekf_reset'
    r.record(runtime,{'phase':'LAND','failure':'ekf_reset'})
    r.resources.close()
    rows=[json.loads(s) for s in (r.output/'cooperative.jsonl').read_text().splitlines()]
    assert rows[0]['position_comparison']['alignment']==alignment
    assert rows[0]['position_comparison']['valid']
    assert rows[0]['position_comparison']['reference']=='uav_body'
    assert not rows[1]['position_comparison']['valid']
    assert rows[1]['position_comparison']['reason']=='ekf_reset'
    assert r.execution.alignment==alignment  # recording never re-aligns control


def test_loaded_plan_is_validated_before_resources_and_recorded_in_same_console(tmp_path,monkeypatch):
    from drone_console import ConsoleModel
    plan=config();plan.update(takeoff_car_world_xyyaw=[0.,0.,0.],flight_status_timeout_s=3.,maximum_task_seconds=600)
    assert a.validate_bindings(plan)['live_interfaces_implemented']
    bad=dict(plan,takeoff_car_world_xyyaw=[999.,0.,0.])
    with pytest.raises(ValueError):a.validate_bindings(bad)
    del bad['takeoff_car_world_xyyaw'];bad['takeoff_car_world_xyyaw']=[0.,0.,0.];del bad['flight_status_timeout_s']
    with pytest.raises((ValueError,KeyError)):a.validate_bindings(bad)
    r,_=resource_rig(tmp_path,monkeypatch)
    r.execution=a.AirGroundExecution(plan,source_snapshot=lambda:None,car_publish=lambda x:None,
        channel_key='39'*32,origin_monotonic=10.)
    r.cache.receive_car(message(),10**10);r.cache.receive_uav(frame())
    r.execution.car_task.phase='TO_TAKEOFF'
    runtime=NS(now=10.,failure=None,inputs=NS(boot_session='test',fault=None,hb=dict(at=9.9,armed=False),land=dict(at=9.9,landed=True)))
    r.record(runtime,{'failure':None});r.resources.close()
    row=json.loads((r.output/'cooperative.jsonl').read_text())
    assert row['status']['phase']=='TO_TAKEOFF' and row['status']['uav_phase']=='WAIT_READY'
    model=ConsoleModel();model.ingest('cooperative.jsonl',row)
    assert model.status['phase']=='TO_TAKEOFF'


def test_site_configuration_draft_reports_missing_values_without_opening_resources(tmp_path,capsys):
    draft=json.loads((ROOT/'plan_b/follow/air_ground_loaded.current.json').read_text())
    # Missing-geometry fixture remains independent of the populated site plan.
    for key in ('platform_offset_car_m','uav_landing_footprint_m','edge_margin_m','landing_xy_tolerance_m'):
        draft[key]=None
    path=tmp_path/'missing_geometry.json'
    path.write_text(json.dumps(draft))
    assert entry.main(['--cooperative-plan',str(path)])==0
    result=json.loads(capsys.readouterr().out)
    assert result['devices_opened']==result['physical_tx_bytes']==0
    report=result['cooperative']
    assert not report['configuration_complete']
    assert report['car_ground_z_map_m']==draft['live_inputs']['car_ground_z_map_m']
    assert {'platform_offset_car_m','uav_landing_footprint_m','edge_margin_m',
            'landing_xy_tolerance_m'}==set(report['missing_configuration'])
    assert not {'live_inputs.car_ground_z_map_m','live_inputs.car_body_origin_in_rigid_m',
                'takeoff_car_world_xyyaw','route_world_xyyaw'} & set(report['missing_configuration'])
    assert draft['platform_size_m']==[.23,.23]
    assert draft['site_evidence']['platform_surface_height_above_floor_estimate_m']==.07
    assert draft['platform_offset_car_m'] is None
    assert draft['live_inputs']['car_body_orientation_in_rigid_xyzw']==json.loads(
        (ROOT/'plan_b/follow/car01_sources.current.json').read_text())['live_inputs']['car_body_orientation_in_rigid_xyzw']
    # Merely flipping the old two measured flags cannot activate null geometry.
    draft['geometry_is_example']=False;draft['live_inputs']['measured']=True
    output=tmp_path/'resources'
    with pytest.raises(ValueError,match='联合配置尚缺实测值'):
        a.JointResources(plan=draft,uav_config={'revision':draft['live_inputs']['uav_rigid_revision']},
            source=None,output=output,origin_monotonic=10.,open_devices=True,site_ready=True)
    assert not output.exists()


def test_current_platform_geometry_contains_saved_placement_and_landing_window(capsys):
    from dataclasses import replace
    import numpy as np
    from follow_target_generator import AirGroundSequence, TargetPose, _rotate_xy
    path=ROOT/'plan_b/follow/air_ground_loaded.current.json'
    plan=json.loads(path.read_text())
    assert entry.main(['--cooperative-plan',str(path)])==0
    result=json.loads(capsys.readouterr().out)
    assert result['devices_opened']==result['physical_tx_bytes']==0
    assert result['cooperative']['configuration_complete']
    assert result['cooperative']['missing_configuration']==[]
    uav=json.loads((ROOT/'plan_b/fixed_point/vicon_profile.initial_trial.json').read_text())['initial_trial']
    a.validate_bindings(plan,uav,live=True)  # Validation only; no resource constructor.
    geometry=plan['site_evidence']['platform_geometry']
    assert geometry['physical_precision_proven'] is False
    assert geometry['landing_xy_tolerance_m'] < geometry['conservative_rotated_edge_room_after_margin_m']
    layout=json.loads(Path(plan['site_evidence']['layout_evidence']).read_text())
    snapshot=json.loads(Path(layout['source']).read_text())
    pairs={}
    for row in snapshot['rows']:pairs.setdefault(row['frame'],{})[row['subject']]=row
    seq=AirGroundSequence(plan)
    poses=[]
    for pair in pairs.values():
        converted=[]
        for name,origin,orientation in (
                ('car01',plan['live_inputs']['car_body_origin_in_rigid_m'],plan['live_inputs']['car_body_orientation_in_rigid_xyzw']),
                ('MircoAir',uav['origin'],uav['orientation'])):
            row=pair[name]
            p,q=a.body_pose(np.array(row['translation_mm'])/1000,row['quaternion_xyzw'],origin,orientation)
            converted.append(TargetPose(name,plan['frame_id'],10**9,10**9,p,q,True,False))
        car,drone=converted
        assert seq._inside_platform(car,drone)
        poses.append(converted)
    car,drone=poses[0]
    seq._capture(car,drone)
    # Platform surface does not replace the 5 cm higher seated board target.
    assert seq.relative_z-plan['platform_offset_car_m'][2]==pytest.approx(.05,abs=.001)
    assert seq._point(car)[2]==pytest.approx(drone.position[2])
    cyaw=a._yaw(car.quaternion_xyzw)
    for dyaw in (-plan['landing_yaw_tolerance_rad'],0.,plan['landing_yaw_tolerance_rad']):
        yaw=a._yaw(drone.quaternion_xyzw)+dyaw
        q=(0.,0.,math.sin(yaw/2),math.cos(yaw/2))
        for axis in (0,1):
            for sign in (-1,1):
                for distance,inside in ((plan['landing_xy_tolerance_m'],True),(.06,False)):
                    offset=[0.,0.];offset[axis]=sign*distance
                    dx,dy=_rotate_xy(offset,cyaw)
                    displaced=replace(drone,position=(drone.position[0]+dx,drone.position[1]+dy,drone.position[2]),quaternion_xyzw=q)
                    assert seq._inside_platform(car,displaced) is inside


def test_current_car_heading_uses_manual_forward_once_and_preserves_up_axis():
    import numpy as np
    plan=json.loads((ROOT/'plan_b/follow/air_ground_loaded.current.json').read_text())
    evidence=plan['site_evidence']['heading_calibration']
    candidate=json.loads(Path(evidence['source']).read_text())
    original=quaternion_to_matrix(evidence['original_orientation_in_rigid_xyzw'])
    applied=quaternion_to_matrix(plan['live_inputs']['car_body_orientation_in_rigid_xyzw'])
    measured=quaternion_to_matrix(candidate['body_orientation_in_rigid_xyzw_candidate'])
    projected=measured[:,0]-original[:,2]*np.dot(measured[:,0],original[:,2])
    projected/=np.linalg.norm(projected)
    assert applied[:,0]==pytest.approx(projected,abs=1e-10)
    assert applied[:,2]==pytest.approx(original[:,2],abs=1e-10)
    assert plan['live_inputs']['car_body_origin_in_rigid_m']==pytest.approx(.015*applied[:,0])
    origin=np.array(plan['live_inputs']['car_body_origin_in_rigid_m'])
    saved=np.array(plan['site_evidence']['captured_landing_reference_in_car_rigid_m'])
    relative=np.array(plan['site_evidence']['saved_landing_reference_in_defined_car_body_m'])
    assert origin+applied@relative==pytest.approx(saved,abs=1e-10)


def test_current_car_anchor_and_rectangle_reuse_raw_pose_in_corrected_body_frame():
    import numpy as np
    plan=json.loads((ROOT/'plan_b/follow/air_ground_loaded.current.json').read_text())
    snapshot=json.loads(Path(plan['site_evidence']['takeoff_capture']['source']).read_text())
    binding=plan['live_inputs']
    poses=[a.body_pose(np.array(r['translation_mm'])/1000,r['quaternion_xyzw'],
                      binding['car_body_origin_in_rigid_m'],binding['car_body_orientation_in_rigid_xyzw'])
           for r in snapshot['rows'] if r['subject']=='car01']
    positions=np.array([p for p,q in poses]);yaws=np.array([a._yaw(q) for p,q in poses])
    anchor=plan['takeoff_car_world_xyyaw']
    expected_yaw=math.atan2(np.sin(yaws).mean(),np.cos(yaws).mean())
    assert anchor==pytest.approx([*positions.mean(axis=0)[:2],expected_yaw],abs=1e-10)
    c,s=math.cos(expected_yaw),math.sin(expected_yaw)
    for waypoint,local in zip(plan['route_world_xyyaw'],[[3,0],[3,2],[0,2],[0,0]]):
        dx,dy=waypoint[0]-anchor[0],waypoint[1]-anchor[1]
        assert [c*dx+s*dy,-s*dx+c*dy]==pytest.approx(local,abs=1e-10)
        assert waypoint[2]==pytest.approx(anchor[2])
        landing=plan['site_evidence']['saved_landing_reference_in_defined_car_body_m']
        lx,ly=waypoint[0]+c*landing[0]-s*landing[1],waypoint[1]+s*landing[0]+c*landing[1]
        limits=plan['follow_limits']
        assert limits['xmin']<lx<limits['xmax'] and limits['ymin']<ly<limits['ymax']


@pytest.mark.parametrize('ground_plane',[None,[.01,.02,-.332]],ids=['constant','plane'])
def test_new_car_body_transform_and_control_share_world_height(ground_plane):
    from car_route_adapter import CarRouteAdapter,RouteIntent
    from mecanum_wand_4wse_control.safety_core import Lease
    plan=config();plan['live_inputs'].update(car_ground_z_map_m=-.28,
        car_body_origin_in_rigid_m=[.1,0.,-.03],
        car_body_orientation_in_rigid_xyzw=[0.,0.,math.sin(.3/2),math.cos(.3/2)])
    if ground_plane is not None:plan['live_inputs']['car_ground_plane_map_abc']=ground_plane
    cache=a.JointMeasurements(plan,uav())
    cache.receive_car(message(p=(1.,2.,-.25),q=(0.,0.,math.sqrt(.5),math.sqrt(.5))),10**10)
    pose=cache.poses['car']
    assert pose.position==pytest.approx([1.,2.1,-.28])
    adapter=CarRouteAdapter(plan,channel_key='39'*32)
    state=adapter.step(steady=10.,ros=100.,car=pose,frame_revision=plan['frame_revision'],
        feedback=dict(sequence=1,issued_steady=10.,issued_ros=100.,enabled=False,output_zero=True,
                      speed=0.,yaw_rate=0.,fault_bits=0),
        lease=Lease('test',1,10.,100.,False),intent=RouteIntent(1,10.,100.,'map',plan['frame_revision'],'stop',0),
        graph_ok=True)
    assert not state['fault'] and 'CAR_LIFT' not in state['reason']
    for core in (adapter.supervisor,adapter.gate):
        transformed=core.inputs.poses['car']['pose']
        assert transformed.z==pytest.approx(adapter.config.ground_z_m)
        assert transformed.yaw==pytest.approx(math.pi/2+.3)  # no old 135-degree correction a second time
    assert a.validate_bindings(plan,live=True)['car_ground_z_map_m']==-.28
    assert cache.poses['car'].position[2]==pytest.approx(-.28)  # shared world/deck never rebased


@pytest.mark.parametrize('plane',[[0.,0.],[0.,float('nan'),0.]])
def test_invalid_ground_plane_rejected_before_opening_resources(tmp_path,plane):
    plan=config();plan['live_inputs']['car_ground_plane_map_abc']=plane
    with pytest.raises(ValueError):
        a.JointResources(plan=plan,uav_config=uav(),source=None,output=tmp_path,
            origin_monotonic=10.,open_devices=True,site_ready=True)
    assert not list(tmp_path.iterdir())


def test_loaded_task_keeps_platform_world_z_with_new_car_height():
    from test_air_ground_execution import Plant,loaded_config
    plan=loaded_config();plan['live_inputs']['car_ground_z_map_m']=-.28
    plan['route_world_xyyaw']=[[.1,0.,0.],[.1,.1,0.]]
    h=Plant(plan=plan)
    shift=-.28-h.car[2]
    h.car[2]+=shift;h.uav[2]+=shift;h.goal[2]+=shift
    h.until(lambda:h.core.phase=='COMPLETE')
    assert not h.joint.fault and h.joint.describe()['complete']
    assert h.joint.sequence.initial_car.position[2]==pytest.approx(-.28)
    assert h.joint.sequence.relative_z==pytest.approx(.15)
    assert h.uav[2]==pytest.approx(-.13)
    assert h.joint.car.supervisor.inputs.poses['car']['pose'].z==pytest.approx(h.joint.car.config.ground_z_m)


def test_car_only_plan_does_not_require_uav_or_landing_geometry(capsys):
    path=ROOT/'plan_b/follow/air_ground_loaded.current.json'
    plan=json.loads(path.read_text())
    plan.pop('uav_source');plan['live_inputs'].pop('uav_rigid_revision')
    result=a.validate_bindings(plan,live=True,car_only=True)
    assert result['configuration_complete'] and result['car_only']
    cache=a.JointMeasurements(plan,None,car_only=True)
    cache.receive_car(message(),10**10)
    cache.receive_car(message(100.1,(1.01,2.,.15)),10_100_000_000)
    status=NS(data=json.dumps(dict(issued_steady=10.1,issued_ros=100.1)))
    cache.receive_status('feedback',status);cache.receive_status('transport',status)
    assert 'uav' not in cache.snapshot() and 'uav_velocity' not in cache.snapshot()
    assert entry.main(['--car-route-plan',str(path)])==0
    result=json.loads(capsys.readouterr().out)
    assert result['car_only'] and result['devices_opened']==0 and not result['automatic_motion']
    with pytest.raises(SystemExit):entry.main(['--car-route-plan',str(path),'--run'])
    with pytest.raises(SystemExit):entry.main(['--car-route-plan',str(path),'--flight-console'])


def test_old_bridge_packets_do_not_replace_or_refresh_current_status():
    cache=a.JointMeasurements(config(),uav())
    current=dict(issued_steady=10.,issued_ros=100.,sequence=4)
    cache.receive_status('feedback',NS(data=json.dumps(current)))
    original=cache.feedback
    cache.receive_status('feedback',NS(data=json.dumps(dict(issued_steady=9.,issued_ros=99.,sequence=3))))
    assert cache.feedback is original and cache.discarded_status==1 and not cache.fault
    # A same-time contradictory packet is different from an old delayed one.
    cache.receive_status('feedback',NS(data=json.dumps(dict(current,sequence=5))))
    assert cache.fault


def test_car_graph_tolerates_brief_discovery_gap_but_not_duplicate_owner(monkeypatch):
    r=graph_rig();r.car_only=True;ready_graph(r)
    r.source.node.owners.pop(r.uav_topic)
    clock=[10.];monkeypatch.setattr(a.time,'monotonic',lambda:clock[0])
    assert r.audit_graph()
    r.node.owners.pop(a.NAMESPACE+'/plant_feedback')
    clock[0]=10.1
    assert not r.audit_graph() and r.cache.graph_ok
    clock[0]=12.2
    assert not r.audit_graph() and not r.cache.graph_ok
    ready_graph(r);r.node.owners[a.NAMESPACE+'/actuator_command']*=2
    with pytest.raises(ValueError,match='duplicate publisher'):r.audit_graph()


@pytest.mark.parametrize('alignment',[False,True])
def test_car_factory_only_opens_original_vicon_and_bridge(tmp_path,monkeypatch,alignment):
    import ev_delay_vicon_source,vicon_receiver_provenance,mavlink_link
    plan=json.loads((ROOT/'plan_b/follow/air_ground_loaded.current.json').read_text())
    if alignment:
        plan=json.loads((ROOT/'plan_b/follow/air_ground_side_follow.current.json').read_text())
        trial=json.loads((ROOT/'plan_b/fixed_point/vicon_profile.initial_trial.json').read_text())['initial_trial']
        plan.update(alignment_only=True,alignment_uav_config={k:trial[k] for k in ('revision','origin','orientation')})
    events=[];seen={}
    monkeypatch.setattr(vicon_receiver_provenance,'verify_current',lambda:{})
    def forbidden(*args,**kw):raise AssertionError('car-only opened drone owner')
    monkeypatch.setattr(mavlink_link,'FixedPeerUDP',forbidden)
    monkeypatch.setattr(entry,'build_guarded_session',forbidden)
    class Source:
        def __init__(self,*args,**kw):seen['source']=kw;events.append('source_open')
        def close(self):events.append('source_close')
    class Owner:
        def __init__(self,**kw):seen['owner']=kw;events.append('bridge_open')
        def close(self):events.append('bridge_close')
    monkeypatch.setattr(ev_delay_vicon_source,'ViconSource',Source)
    monkeypatch.setattr(a,'JointResources',Owner)
    with pytest.raises(PermissionError):entry.open_car_route_session(plan=plan,output=tmp_path/'denied')
    assert not events
    owner=entry.open_car_route_session(plan=plan,output=tmp_path/'run',open_devices=True,site_ready=True)
    assert seen['source']==dict(latest_only=True,subject='MircoAir' if alignment else 'car01')
    assert seen['owner']['car_only']
    assert seen['owner']['uav_config']==(plan['alignment_uav_config'] if alignment else None)
    owner.session_resources.close()
    assert events==['source_open','bridge_open','bridge_close','source_close']


def test_car_console_requires_fresh_explicit_start_and_closes_on_stop():
    calls=[];closed=[];states=iter(['WAIT_START','ROUTE'])
    def step(**kw):
        calls.append(kw['start']);return dict(phase=next(states),fault=None,route_index=0,waiting_for=None)
    owner=NS(step_car=step,session_resources=NS(close=lambda:closed.append(True)),stop_result={'car_stop_confirmed':True})
    lines=iter(['START\n','START\n','STOP\n'])
    entry.serve_car_route_console(owner,lambda:next(lines),emit=lambda s:None,idle=lambda:None)
    assert calls==[False,True] and closed==[True]


@pytest.mark.parametrize('command',['',KeyboardInterrupt()])
def test_car_console_eof_or_interrupt_never_starts_and_always_closes(command):
    closed=[]
    def read():
        if isinstance(command,BaseException):raise command
        return command
    def step(**kw):raise AssertionError('no car turn expected before stop')
    owner=NS(step_car=step,session_resources=NS(close=lambda:closed.append(True)),stop_result={})
    entry.serve_car_route_console(owner,read,emit=lambda s:None,idle=lambda:None)
    assert closed==[True]


def test_car_resource_reactor_and_close_use_real_signing_with_fake_ros(tmp_path,monkeypatch):
    """Exercise actual resource wiring, tick and shutdown; no real bridge/socket."""
    from mecanum_wand_4wse_hardware.commissioning import TransportPolicy
    from follow_target_generator import _rotate_xy
    import builtins
    events=[];Node=fake_ros(monkeypatch,events)
    clock=[10.];env_seen={}
    monkeypatch.setattr(a.time,'monotonic',lambda:clock[0])
    monkeypatch.setattr(a.time,'monotonic_ns',lambda:round(clock[0]*1e9))
    monkeypatch.setattr(a.time,'sleep',lambda t:clock.__setitem__(0,clock[0]+t))
    monkeypatch.setattr(a,'open',lambda path,*args,**kw:builtins.open(
        tmp_path/'owner.lock' if path=='/tmp/wand_4wse_browser_operator.lock' else path,*args,**kw),raising=False)
    monkeypatch.setattr(a,'original_car_entry',lambda:NS(verify_release=lambda:None,route_check=lambda:None,
        prepare_session=lambda *args:{}))
    class Process:
        def __init__(self,*args,env,**kw):env_seen.update(env);self.ended=False
        def poll(self):return 0 if self.ended else None
        def terminate(self):self.ended=True
        def wait(self,**kw):return 0
    monkeypatch.setattr(subprocess,'Popen',Process)
    plan=config();plan.update(takeoff_car_world_xyyaw=[0.,0.,0.],route_world_xyyaw=[[.15,0.,0.],[0.,0.,0.]],
        flight_status_timeout_s=3.,maximum_task_seconds=600)
    plan['live_inputs'].update(car_body_origin_in_rigid_m=[0.,0.,0.],car_body_orientation_in_rigid_xyzw=[0.,0.,0.,1.])
    source=NS(node=Node('owned_source',context=NS(domain=83)),poll=lambda:None,
        now_ros_ns=lambda:round((100.+clock[0])*1e9))
    output=tmp_path/'run';output.mkdir()
    r=a.JointResources(plan=plan,uav_config=None,source=source,output=output,origin_monotonic=clock[0],
        open_devices=True,site_ready=True,car_only=True)
    policy=TransportPolicy(env_seen['WAND_4WSE_CHANNEL_KEY'],r.execution.car.config.stage)
    raw=[0.,0.,r.execution.car.car_ground_z_map_m];seq=[0];commands=[]
    def receive(wire):
        assert policy.receive(wire.data,clock[0],100.+clock[0])
        commands.append(json.loads(wire.data))
    r.publisher.publish=receive
    r.node.get_clock=lambda:NS(now=lambda:NS(nanoseconds=source.now_ros_ns()))
    ready_graph(r)
    source.node.owners.pop(r.uav_topic)
    def tick():
        clock[0]+=.02;seq[0]+=1
        command=policy.command if policy.active(clock[0]) else a.ZERO
        raw[0]+=command.vx*.02;raw[1]+=command.vy*.02
        source.node.subs[a.CAR_TOPIC](message(100.+clock[0],p=tuple(raw)))
        feedback=dict(sequence=seq[0],issued_steady=clock[0],issued_ros=100.+clock[0],
            output_zero=command==a.ZERO,enabled=policy.active(clock[0]),speed=math.hypot(command.vx,command.vy),
            yaw_rate=abs(command.wz),fault_bits=0)
        transport=dict(issued_steady=clock[0],issued_ros=100.+clock[0],ready=not policy.active(clock[0]),
            fresh=True,local_stop=False,native_fault_bits=0,fault='',armed_epoch=policy.epoch,
            policy_state=(policy.envelope or {}).get('state'))
        for topic,value in [('plant_feedback',feedback),('transport_status',transport)]:
            r.node.subs[topic](NS(data=json.dumps(value)))
    source.poll=tick
    state=None
    try:
        for _ in range(3000):
            state=r.step_car(start=bool(state and state['phase']=='WAIT_START'))
            if state['phase'] in ('COMPLETE','ABORTED'):break
        assert state['complete'] and state['car_stopped'],state
        assert r.cache.poses.keys()=={'car'} and r.execution.flight is None
        assert math.dist(raw[:2],(0.,0.))<=plan['car_arrival_m']
    finally:r.close()
    assert r.closed and r.proc.ended and r.stop_result['car_stop_confirmed']
    assert not policy.active(clock[0])
    assert (output/'car_motion.jsonl').stat().st_size>0 and (output/'car_stop.json').exists()
    assert not (output/'wifi_raw.jsonl').exists()

@pytest.mark.parametrize('failure',[None,'stale','moved','overlap'])
def test_unpowered_alignment_uses_live_uav_pose_and_frozen_obstacle(tmp_path,failure):
    from dataclasses import replace
    from follow_target_generator import AirGroundSequence,TargetPose
    from car_route_adapter import CarMotionTask
    plan=json.loads((ROOT/'plan_b/follow/air_ground_side_follow.current.json').read_text())
    plan['alignment_only']=True
    a.validate_bindings(plan,car_only=True)
    r=a.JointResources.__new__(a.JointResources);r.output=tmp_path
    r.alignment_sequence=AirGroundSequence(plan)
    r.execution=CarMotionTask(dict(plan,schema='car_cooperative_motion_v1'),channel_key='37'*32,standalone=True)
    car=TargetPose('car01','map',100_000_000_000,10_000_000_000,(-1.,0.,.1),(0.,0.,0.,1.),True,False)
    uav=replace(car,source='MircoAir',position=(0.,0.,.1))
    data=dict(car=car,uav=uav,car_rigid=car,uav_rigid=uav)
    r.check_ground_alignment(data,10.,100.,start=True)
    assert (tmp_path/'alignment_resolved.json').exists()
    if failure=='stale':data['uav']=replace(uav,stamp_ns=90_000_000_000)
    if failure=='moved':data['uav']=replace(uav,position=(.3,0.,.1),stamp_ns=100_010_000_000,received_ns=10_010_000_000)
    if failure=='overlap':data['car']=replace(car,position=(0.,0.,.1),stamp_ns=100_010_000_000,received_ns=10_010_000_000)
    if failure:
        with pytest.raises(ValueError):r.check_ground_alignment(data,10.01,100.01)
    else:r.check_ground_alignment(data,10.01,100.01)


def test_car_startup_failure_saved_without_waiting_for_missing_wheel_feedback(tmp_path):
    transport=dict(local_stop=True,fault='STARTUP_HELLO_RTT_OR_LINK_NOT_QUALIFIED',
                   startup_clock=dict(hello_rtt_s=None),startup_hello_attempts=3)
    r=a.JointResources.__new__(a.JointResources)
    r.car_only=True;r.output=tmp_path;r.source=NS(poll=lambda:None,now_ros_ns=lambda:100000000000)
    r.poll=lambda:None;r.snapshot=lambda:None;r.cache=NS(transport=transport)
    r.execution=NS(started_at=None)
    with pytest.raises(RuntimeError,match='STARTUP_HELLO_RTT'):
        r.step_car()
    assert json.loads((tmp_path/'car_connection_failure.json').read_text())==transport
