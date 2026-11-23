"""Host/binding verification with synthetic input, ROS and serial only."""
from pathlib import Path
import json
import sys
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import prepare_drone as host
import preparation_host_bindings as bindings
from gamepad_estop_monitor import EVENT,HOME
from test_mission_preparation_runtime import Peer,prepared_peer
from test_trs_vision_reference import baseline
from test_mission_preparation_io import MemoryPort,attach


def test_default_cli_is_offline_and_cannot_open_live_ports(monkeypatch,capsys):
    monkeypatch.setattr(host,'configuration',lambda p:{'revision':'test','heading':Path('h'),'board':Path('b')})
    def forbidden(*a,**k):raise AssertionError('No live entry in default/offline mode')
    monkeypatch.setattr(host,'run_live',forbidden)
    monkeypatch.setattr('builtins.input',forbidden)
    assert host.main([])==0
    report=json.loads(capsys.readouterr().out)
    assert report['hardware_opened']==0 and not report['live_authorization_installed']
    assert not report['flight_authorized']


def test_live_modes_require_flags_and_typed_confirmation_before_open(monkeypatch):
    monkeypatch.setattr(host,'configuration',lambda p:{})
    monkeypatch.setattr(host,'run_live',lambda *a:pytest.fail('Live resources must stay closed'))
    assert host.main(['--monitor-ground'])==2
    monkeypatch.setattr('builtins.input',lambda prompt:'cancel')
    assert host.main(['--monitor-ground','--confirm-battery-disconnected','--confirm-usb-only-stationary'])==2


def test_only_read_home_monitor_can_wait_five_minutes(monkeypatch):
    monkeypatch.setattr(host,'configuration',lambda p:{})
    monkeypatch.setattr('builtins.input',lambda prompt:'MONITOR')
    calls=[]
    monkeypatch.setattr(host,'run_live',lambda args,config:calls.append(args.seconds) or 0)
    flags=['--confirm-battery-disconnected','--confirm-usb-only-stationary','--seconds','300']
    assert host.main(['--monitor-ground',*flags])==0
    assert calls==[300]
    assert host.main(['--prepare-ground',*flags])==2
    assert calls==[300]


def test_run_live_cannot_be_accidentally_called_without_session_confirmation():
    args=NS(confirm_battery_disconnected=True,confirm_usb_only_stationary=True)
    with pytest.raises(ValueError,match='confirmation'):host.run_live(args,{})


class DrivenIO:
    failure=None
    def __init__(self,peer):self.peer=peer
    def step(self,now,ros_ns):
        assert now==pytest.approx(self.peer.now+.032)
        return self.peer.turn()
    def status(self):return dict(failure=self.failure,physical_tx_bytes=0)


def test_host_preparation_auto_closes_after_observation_but_never_starts_flight():
    p=prepared_peer();session=host.HostSession(p.runtime,DrivenIO(p),prepare=True,observe_seconds=.096)
    for _ in range(40):
        status=session.turn(round(p.now+.032,9),0)
        if status['done']:break
    assert status['done'] and status['prepared']
    assert status['reason']=='preparation_observation_complete'
    assert p.runtime.closed and p.params==baseline()
    assert not status['flight_authorized']


def test_host_monitor_does_not_report_success_from_timeout_alone():
    p=Peer();session=host.HostSession(p.runtime,DrivenIO(p),seconds=.1)
    for _ in range(8):
        status=session.turn(round(p.now+.032,9),0)
        if status['done']:break
    assert status['done'] and not status['monitor_conditions_observed']
    assert not status['prepared']


def test_host_monitor_reuses_current_home_proof_without_repressing_home():
    p=Peer();p.home_check();before=p.runtime.home_attempts
    session=host.HostSession(p.runtime,DrivenIO(p))
    for _ in range(8):
        status=session.turn(round(p.now+.032,9),0)
        if status['done']:break
    assert status['done'] and status['monitor_conditions_observed']
    assert p.runtime.home_attempts==before


def test_monitor_missing_reports_only_read_home_scope_not_disabled_fusion():
    p=Peer();p.home_check();session=host.HostSession(p.runtime,DrivenIO(p))
    assert session.monitor_missing(p.now)==[]
    p.runtime.inputs.home_proven=False
    assert session.monitor_missing(p.now)==['本会话HOME响应']


@pytest.fixture
def fake_pad(monkeypatch):
    queue=[];closed=[];bits=bytearray(96)
    def read(fd,size):
        if queue:return queue.pop(0)
        raise BlockingIOError
    monkeypatch.setattr(bindings,'find_pad',lambda:dict(path='/fake/event',name='fake'))
    monkeypatch.setattr(bindings,'gamepad_identity',lambda path:dict(generation=1))
    monkeypatch.setattr(bindings,'require_gamepad_identity',lambda *args:None)
    monkeypatch.setattr(bindings,'os',NS(O_RDONLY=0,O_NONBLOCK=1,open=lambda *a:8,read=read,close=closed.append))
    monkeypatch.setattr(bindings,'fcntl',NS(ioctl=lambda *a:bytes(bits)))
    return queue,closed,bits


def test_gamepad_partial_input_initial_held_home_and_release(fake_pad):
    queue,closed,bits=fake_pad;bits[HOME//8]|=1<<(HOME%8)
    pad=bindings.LinuxGamepad()
    assert pad.poll()['home_events']==(True,)
    packet=EVENT.pack(0,0,1,HOME,0);queue.append(packet[:7])
    assert pad.poll()['home_events']==()
    queue.append(packet[7:]);assert pad.poll()['home_events']==(False,)
    pad.close();assert closed==[8]


def test_gamepad_overflow_and_disconnect_are_not_fake_awake_inputs(fake_pad):
    queue,_,_=fake_pad;pad=bindings.LinuxGamepad()
    queue.append(EVENT.pack(0,0,0,3,0))
    with pytest.raises(OSError,match='SYN_DROPPED'):pad.poll()
    queue.append(b'')
    with pytest.raises(OSError,match='disconnected'):pad.poll()


def test_ros_binding_coalesces_latest_pose_and_uses_zero_timeout_poll():
    ros=object.__new__(bindings.RosLatestPose);ros.latest=None;ros.node=object();calls=[]
    ros.executor=NS(spin_once=lambda timeout_sec:calls.append(timeout_sec))
    def msg(x):
        return NS(header=NS(stamp=NS(sec=1,nanosec=2),frame_id='map'),
            pose=NS(position=NS(x=x,y=0,z=0),orientation=NS(x=0,y=0,z=0,w=1)))
    ros.receive(msg(1));ros.receive(msg(2))
    assert ros.poll()==(1_000_000_002,'map',[2,0,0],[0,0,0,1])
    assert ros.poll() is None and calls==[0.]*8


def test_ros_binding_owns_executor_with_its_own_context(monkeypatch):
    calls=[];context=NS(shutdown=lambda:calls.append('context_close'))
    class Executor:
        def __init__(self,*,context):
            assert context is ctx;calls.append('executor_open')
        def add_node(self,node):calls.append('add_node')
        def spin_once(self,*,timeout_sec):assert timeout_sec==0.
        def shutdown(self,*,timeout_sec):calls.append('executor_close')
    ctx=context
    class Node:
        def __init__(self,name,*,context):assert context is ctx
        def create_subscription(self,*args):return object()
        def destroy_node(self):calls.append('node_close')
    monkeypatch.setitem(sys.modules,'rclpy',NS(context=NS(Context=lambda:ctx),
        init=lambda **kwargs:None))
    monkeypatch.setitem(sys.modules,'rclpy.node',NS(Node=Node))
    monkeypatch.setitem(sys.modules,'rclpy.executors',NS(SingleThreadedExecutor=Executor))
    monkeypatch.setitem(sys.modules,'rclpy.qos',NS(QoSProfile=lambda **kwargs:None,
        ReliabilityPolicy=NS(BEST_EFFORT=1),DurabilityPolicy=NS(VOLATILE=1)))
    monkeypatch.setitem(sys.modules,'geometry_msgs.msg',NS(PoseStamped=object))
    ros=bindings.RosLatestPose();assert ros.poll() is None;ros.close()
    assert calls==['executor_open','add_node','executor_close','node_close','context_close']


def test_ros_clock_is_taken_after_polling_not_before():
    p=Peer();p.home_check();io,_,t,ns=attach(p)
    order=[]
    io.poll_pose=lambda:(order.append('pose') or (ns,'map',[1,-2,.1],[0,0,0,1]))
    io.step(t,lambda:order.append('clock') or ns)
    assert order==['pose','clock']
    assert p.runtime.inputs.pose_stamp==ns


@pytest.mark.parametrize('fault',[False,True])
def test_live_host_resource_cleanup_with_only_mock_devices(tmp_path,monkeypatch,fault):
    resources=[];clock=NS(value=0.)
    class Port(MemoryPort):
        def __enter__(self):resources.append('port_open');return self
        def __exit__(self,*args):resources.append('port_close')
    class Pad:
        def __init__(self):resources.append('pad_open')
        def poll(self):return dict(pad_generation='fake',pad_present=True)
        def close(self):resources.append('pad_close')
    class Pose:
        def __init__(self):resources.append('ros_open')
        def now_ns(self):return int((clock.value+100)*1e9)
        def poll(self):return None
        def close(self):resources.append('ros_close')
    class Console:
        def publish(self,text):pass
        def close(self):resources.append('console_close')
    monkeypatch.setitem(sys.modules,'serial',NS(Serial=lambda *a,**k:Port()))
    monkeypatch.setattr(bindings,'LinuxGamepad',Pad);monkeypatch.setattr(bindings,'RosLatestPose',Pose)
    monkeypatch.setattr(host,'FC_USB',NS(exists=lambda:True))
    monkeypatch.setattr(host,'require_port_free',lambda path:None)
    monkeypatch.setattr(host,'RUNS',tmp_path/'runs')
    monkeypatch.setattr(host,'LatestStatusPrinter',Console)
    monkeypatch.setattr(host,'time',NS(monotonic=lambda:clock.value,sleep=lambda delay:setattr(clock,'value',clock.value+.032)))
    if fault:
        def failed_turn(*args):raise RuntimeError('synthetic unexpected host error')
        monkeypatch.setattr(host.HostSession,'turn',failed_turn)
    args=NS(operator_session_confirmed=True,confirm_battery_disconnected=True,confirm_usb_only_stationary=True,
        baud=115200,prepare_ground=False,trs_port='/fake/TRS',seconds=.1,observe_seconds=1)
    result=host.run_live(args,dict(revision='fake',origin=[0,0,0],orientation=[0,0,0,1]))
    assert result==2  # No UID/HOME proof from the silent synthetic FC.
    assert resources==['port_open','pad_open','ros_open','console_close','ros_close','pad_close','port_close']
    report=json.loads(next((tmp_path/'runs').glob('*/report.json')).read_text())
    assert not report['flight_authorized'] and not report['resident_monitor_running_after_exit']
    if fault:assert report['reason']=='host_exception_recovery_may_be_required'
