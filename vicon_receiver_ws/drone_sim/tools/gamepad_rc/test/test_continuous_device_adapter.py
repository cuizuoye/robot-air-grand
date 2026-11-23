"""Fake resources only: device ownership, readonly boundary, failures/cleanup."""
from pathlib import Path
import json
import stat
import sys
from types import SimpleNamespace as NS
from unittest.mock import patch

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from continuous_device_io import ReceiveShadowRuntime,ContinuousDeviceIO
from continuous_mission_runtime import ContinuousMissionRuntime
from mavlink_crc_stream import CrcResyncParser
from trs_uart_readonly_check import UID2
import continuous_device_host as host


class Port:
    timeout=write_timeout=0
    def __init__(self):self.rx=b'';self.writes=[];self.write_result=None;self.closed=False
    @property
    def in_waiting(self):return len(self.rx)
    def read(self,n):
        value,self.rx=self.rx[:n],self.rx[n:]
        return value
    def write(self,value):
        self.writes.append(value)
        if isinstance(self.write_result,BaseException):raise self.write_result
        return len(value) if self.write_result is None else self.write_result
    def close(self):self.closed=True


def attach(*,queries=False):
    r=ReceiveShadowRuntime('fake-revision',body_origin_in_rigid=[0,0,0],body_orientation_in_rigid=[0,0,0,1])
    p=Port();state=dict(events=(),pose=None,now=0.,order=[])
    def pad():
        state['order'].append('pad')
        return dict(pad_generation='fake-pad',pad_present=True,pad_event=bool(state['events']),home_events=state['events'])
    def pose():state['order'].append('pose');return state['pose']
    def clock():state['order'].append('clock');return int((100+state['now'])*1e9)
    io=ContinuousDeviceIO(r,p,poll_gamepad=pad,poll_pose=pose,now_ros_ns=clock,
        check_port_identity=lambda:state['order'].append('identity'),queries=queries)
    return r,p,io,state


def frame(msg):
    e=m.MAVLink(None,srcSystem=1,srcComponent=1)
    return msg.pack(e)


def heartbeat(*,armed=False):
    return frame(m.MAVLink(None).heartbeat_encode(2,m.MAV_AUTOPILOT_PX4,128 if armed else 0,4<<16,0))


def version():
    e=m.MAVLink(None)
    return frame(e.autopilot_version_encode(0,(1<<24)+(14<<16)+(3<<8),0,0,54,
        bytes(8),bytes(8),bytes(8),0,0,1,bytes.fromhex(UID2)))


def identified():
    r,p,io,state=attach(queries=True)
    p.rx=heartbeat();io.step(0.)
    state['now']=.01;p.rx=heartbeat()+version();io.step(.01)
    assert r.inputs.identity==(UID2,'1.14.3')
    return r,p,io,state


def decoded(port):
    parser=CrcResyncParser()
    return [parser.parse_buffer(wire)[0] for wire in port.writes]


def test_default_plan_never_opens_real_apis_or_configuration(monkeypatch,capsys):
    monkeypatch.setattr(host,'configuration',lambda:pytest.fail('No configuration needed for offline plan'))
    monkeypatch.setattr(host,'run_live',lambda *a:pytest.fail('No real device entry by default'))
    with patch('serial.Serial') as serial,patch('os.open') as fd,patch('socket.socket') as socket:
        assert host.main([])==0
        serial.assert_not_called();fd.assert_not_called();socket.assert_not_called()
    result=json.loads(capsys.readouterr().out)
    assert result['devices_opened']==result['physical_tx_bytes']==0
    assert result['home_stop_sender_active'] is False


@pytest.mark.parametrize('flags',[
    ['--observe'],['--observe','--confirm-stationary'],['--queries'],
    ['--observe','--confirm-disarmed'],
    ['--observe','--confirm-stationary','--confirm-disarmed','--seconds','nan'],
    ['--observe','--confirm-stationary','--confirm-disarmed','--seconds','301'],
    ['--execute'],['--arm'],['--unlock']])
def test_cli_missing_scope_or_flight_options_refused_before_devices(monkeypatch,flags):
    monkeypatch.setattr(host,'run_live',lambda *a:pytest.fail('No resources may open'))
    monkeypatch.setattr(host,'configuration',lambda:pytest.fail('No configuration work before flags validated'))
    with pytest.raises(SystemExit) as exc:host.main(flags)
    assert exc.value.code==2


def test_passive_zero_tx_no_fake_identity_request_or_home_proof():
    r,p,io,state=attach()
    p.rx=heartbeat();io.step(0.)
    state.update(now=.01,events=(True,False));p.rx=heartbeat()+version()
    io.step(.01)
    assert p.writes==[] and io.actual_tx_bytes==0
    assert io.home_events_observed==1 and not r.inputs.home_proven
    assert r.inputs.identity is None and r.inputs.version_request is None
    assert not r.prepare_requested and r.coordinator.fusion.phase=='NEW'
    with pytest.raises(PermissionError):r.request_start(operator_confirmed=True)
    with pytest.raises(PermissionError):r.request_prepare(operator_confirmed=True)


def test_home_autorepeat_is_counted_as_one_physical_press():
    r,p,io,state=attach();state['events']=(True,True,False)
    io.step(0.)
    assert io.home_events_observed==1 and not io.home_down
    assert not p.writes


def test_explicit_readonly_queries_share_writer_and_do_not_send_home():
    r,p,io,state=identified()
    state.update(now=.02,events=(True,False));p.rx=heartbeat();io.step(.02)
    messages=decoded(p)
    assert any(msg.get_type()=='COMMAND_LONG' and msg.command==512 for msg in messages)
    assert any(msg.get_type()=='SERIAL_CONTROL' for msg in messages)
    assert all(msg.get_type() in ('HEARTBEAT','COMMAND_LONG','PARAM_REQUEST_READ','SERIAL_CONTROL') for msg in messages)
    assert all(msg.command==512 for msg in messages if msg.get_type()=='COMMAND_LONG')
    assert r.writer is r.core.sink is r.coordinator.fusion.sink is r.coordinator.writer is io.writer
    assert not r.prepare_requested and r.coordinator.fusion.phase=='NEW'


@pytest.mark.parametrize('command,p1',[(185,2),(185,0),(400,1),(400,0),(176,1),(21,0),(22,0),(246,1),(511,1)])
def test_privileged_commands_cannot_be_dispatched_even_if_fresh(command,p1):
    r,p,io,state=identified();before=len(p.writes)
    msg=r.encoder.command_long_encode(1,1,command,0,p1,0,0,0,0,0,0)
    with pytest.raises((ValueError,PermissionError)):io.send(msg)
    assert len(p.writes)==before


def test_poses_targets_and_param_writes_are_always_denied():
    r,p,io,state=identified();before=len(p.writes)
    e=r.encoder
    candidates=(e.param_set_encode(1,1,b'EKF2_EV_CTRL',11.,6),
        e.vision_position_estimate_encode(123,1,2,3,0,0,0,[0.]*21,0),
        e.set_position_target_local_ned_encode(123,1,1,1,3576,1,2,-.3,0,0,0,0,0,0,0,0))
    for msg in candidates:
        with pytest.raises((PermissionError,ValueError)):io.send(msg)
    assert len(p.writes)==before


def test_unrelated_ack_cannot_become_current_home_response():
    r,p,io,state=identified();state.update(now=.02,events=(True,False))
    p.rx=frame(m.MAVLink(None).command_ack_encode(185,0,target_system=250,target_component=220))
    io.step(.02)
    assert r.inputs.home_ack is None and not r.inputs.home_proven


@pytest.mark.parametrize('result',[0,1,OSError('synthetic write failed')])
def test_failed_or_partial_write_latches_and_never_retries(result):
    r,p,io,state=attach(queries=True);p.write_result=result
    status=io.step(0.)
    count=len(p.writes)
    assert status['failure'] and count==1
    io.step(.1)
    assert len(p.writes)==count
    assert io.complete_messages==0


def test_home_event_retained_when_following_serial_read_fails():
    r,p,io,state=attach();state['events']=(True,)
    def fail():raise OSError('port re-enumerated')
    io.check_port_identity=fail
    status=io.step(0.)
    assert io.home_events_observed==1 and r.inputs.home_down
    assert status['failure'] and not p.writes


def test_read_order_latest_pose_and_ros_time_after_callbacks():
    r,p,io,state=attach();state['pose']=(100_000_000_000,'map',[1,2,3],[0,0,0,1])
    io.step(0.)
    assert state['order'][:4]==['pad','identity','pose','clock']
    assert r.inputs.pose_stamp==100_000_000_000


def test_serial_backlog_aborts_without_refreshing_old_state():
    r,p,io,state=attach();p.rx=b'x'*2049
    status=io.step(0.)
    assert 'backlog' in status['failure'] and r.inputs.hb is None and not p.writes


def test_no_second_reader_and_no_old_capture_replay():
    r,p,io,state=attach()
    with pytest.raises(ValueError,match='second device'):
        ContinuousDeviceIO(r,p,poll_gamepad=lambda:{},poll_pose=lambda:None,now_ros_ns=lambda:0,
            check_port_identity=lambda:None)
    assert p.writes==[]
    active=ContinuousMissionRuntime('fake',body_origin_in_rigid=[0,0,0],body_orientation_in_rigid=[0,0,0,1])
    with pytest.raises(ValueError,match='sealed'):
        ContinuousDeviceIO(active,p,poll_gamepad=lambda:{},poll_pose=lambda:None,now_ros_ns=lambda:0,
            check_port_identity=lambda:None)


def test_preexisting_offline_frames_are_not_sent_when_adapter_attaches():
    r=ReceiveShadowRuntime('fake',body_origin_in_rigid=[0,0,0],body_orientation_in_rigid=[0,0,0,1])
    r.turn(0.)
    assert len(r.frames)>0
    p=Port()
    io=ContinuousDeviceIO(r,p,poll_gamepad=lambda:dict(pad_generation='fake',pad_present=True),
        poll_pose=lambda:None,now_ros_ns=lambda:100_010_000_000,
        check_port_identity=lambda:None,queries=True)
    io.step(.01)
    assert p.writes==[] and io.actual_tx_bytes==0


def test_blocking_serial_and_other_owner_thread_are_refused():
    from concurrent.futures import ThreadPoolExecutor
    r,p,io,state=attach()
    with ThreadPoolExecutor(max_workers=1) as worker:
        with pytest.raises(RuntimeError,match='immutable'):
            worker.submit(io.step,0.).result()
    p.timeout=1
    r2=ReceiveShadowRuntime('fake',body_origin_in_rigid=[0,0,0],body_orientation_in_rigid=[0,0,0,1])
    with pytest.raises(ValueError,match='nonblocking'):
        ContinuousDeviceIO(r2,p,poll_gamepad=lambda:{},poll_pose=lambda:None,
            now_ros_ns=lambda:0,check_port_identity=lambda:None)


def test_armed_aircraft_exits_instead_of_becoming_a_flight_monitor():
    r,p,io,state=attach();p.rx=heartbeat(armed=True)
    status=io.step(0.)
    assert status['failure']=='armed_aircraft_outside_observation_scope' and not p.writes


def test_bad_crc_does_not_refresh_heartbeat_and_clean_frame_can_parse():
    r,p,io,state=attach();bad=bytearray(heartbeat());bad[-1]^=1;p.rx=bytes(bad)
    io.step(0.)
    assert r.inputs.hb is None and r.inputs.bad_frames>0
    state['now']=.01;p.rx=heartbeat();io.step(.01)
    assert r.inputs.hb['at']==.01


def test_adapter_close_is_idempotent_and_cannot_reconnect():
    r,p,io,state=attach();io.close();io.close()
    with pytest.raises(RuntimeError,match='closed'):io.step(0.)
    assert io.closed and not p.writes


@pytest.mark.parametrize('fault',['none','pad_open','ros_open','poll','close'])
def test_native_host_partial_acquisition_and_cleanup(tmp_path,monkeypatch,fault):
    events=[];clock=NS(t=0.)
    p=Port()
    class Owner:
        def __init__(self,path):self.port=p;events.append('serial_open')
        def check(self):pass
        def close(self):p.close();events.append('serial_close')
    class Pad:
        def __init__(self):
            if fault=='pad_open':raise OSError('pad opening failed')
            events.append('pad_open')
        def poll(self):
            if fault=='poll':raise OSError('unplugged')
            return dict(pad_generation='fake',pad_present=True)
        def close(self):
            events.append('pad_close')
            if fault=='close':raise OSError('close failed')
    class Pose:
        def __init__(self):
            if fault=='ros_open':raise OSError('ROS opening failed')
            events.append('ros_open')
        def poll(self):return None
        def now_ns(self):return int((100+clock.t)*1e9)
        def close(self):events.append('ros_close')
    class Console:
        def publish(self,text):pass
        def close(self):events.append('console_close')
    monkeypatch.setattr(host,'SerialOwner',Owner);monkeypatch.setattr(host,'LinuxGamepad',Pad)
    monkeypatch.setattr(host,'RosLatestPose',Pose);monkeypatch.setattr(host,'LatestStatusPrinter',Console)
    monkeypatch.setattr(host,'time',NS(monotonic=lambda:clock.t,sleep=lambda _:setattr(clock,'t',clock.t+.05)))
    args=NS(session_confirmed=True,confirm_stationary=True,confirm_disarmed=True,
        output_dir=tmp_path/'new',trs_port='/fake',queries=False,seconds=.1)
    assert host.run_live(args,dict(revision='fake',origin=[0,0,0],orientation=[0,0,0,1]))==2
    report=json.loads((tmp_path/'new/report.json').read_text())
    assert events[-1]=='serial_close' and p.closed and p.writes==[]
    assert not report['flight_authorized'] and not report['resident_monitor_running_after_exit']
    assert report['resources_closed'] is (fault!='close')
    if fault=='close':assert report['cleanup_errors']
    if fault=='pad_open':assert events==['serial_open','serial_close']
    if fault=='ros_open':assert events==['serial_open','pad_open','pad_close','serial_close']


@pytest.mark.parametrize('bad_opened_fd',[False,True])
def test_native_open_is_delayed_exclusive_and_control_lines_set_before_open(monkeypatch,bad_opened_fd):
    events=[];identity=NS(st_ino=1,st_rdev=2,st_mode=stat.S_IFCHR|0o600)
    class FakePath:
        def __init__(self,value):self.value=str(value)
        def __str__(self):return self.value
        def __eq__(self,other):return str(other)==self.value
        @property
        def parent(self):return FakePath('/'.join(self.value.split('/')[:-1]))
        @property
        def name(self):return self.value.rsplit('/',1)[-1]
        def resolve(self,strict=True):return FakePath('/dev/ttyUSB777')
    class NativePort:
        def __setattr__(self,name,value):events.append((name,value));object.__setattr__(self,name,value)
        def open(self):events.append('open')
        def close(self):events.append('close')
        def fileno(self):return 7
    def serial(**kwargs):
        assert kwargs==dict(port=None,baudrate=115200,timeout=0,write_timeout=0,exclusive=True)
        return NativePort()
    monkeypatch.setattr(host,'Path',FakePath)
    opened=NS(st_ino=9,st_rdev=2) if bad_opened_fd else identity
    monkeypatch.setattr(host,'os',NS(stat=lambda _:identity,fstat=lambda _:opened))
    monkeypatch.setattr(host,'require_port_free',lambda _:events.append('port_free_checked'))
    monkeypatch.setitem(sys.modules,'serial',NS(Serial=serial))
    if bad_opened_fd:
        with pytest.raises(OSError,match='re-enumerated'):
            host.SerialOwner('/dev/serial/by-id/usb-Silicon_Labs_CP2102_FAKE')
        assert events.count('close')==1
        return
    owner=host.SerialOwner('/dev/serial/by-id/usb-Silicon_Labs_CP2102_FAKE')
    assert events.index(('dtr',False))<events.index('open')
    assert events.index(('rts',False))<events.index('open')
    identity.st_ino=3
    # Snapshot itself is immutable in reality; replace the current stat only.
    monkeypatch.setattr(host,'os',NS(stat=lambda _:NS(st_ino=9,st_rdev=2),fstat=lambda _:identity))
    with pytest.raises(OSError,match='re-enumerated'):owner.check()
    owner.close();owner.close();assert events.count('close')==1
