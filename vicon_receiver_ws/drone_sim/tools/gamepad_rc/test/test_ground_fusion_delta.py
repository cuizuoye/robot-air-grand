"""New isolated fusion route only: synthetic frames/FC, no physical I/O."""
import json
import socket
from types import SimpleNamespace

import pytest
from pymavlink.dialects.v20 import common as m

from test_pocket_wifi_delta import Peer
import native_rc_ground as ground
import vicon_fixed_point_entry as entry
from synthetic_mission_peer import SyntheticPeer


@pytest.fixture(autouse=True)
def no_network(monkeypatch):
    monkeypatch.setattr(socket, 'socket', lambda *a, **kw: pytest.fail('Real network attempted'))


class FusionPeer(Peer):
    def __init__(self):
        super().__init__(); self.next_attitude = 0.; self.silent_shell = False
        self.target_test=False
        self.mode=3; self.reject_entry=False; self.drop_mode_ack=False; self.ignore_entry=False

    @property
    def in_waiting(self):
        now = ground.time.monotonic()
        if now >= self.next_attitude:
            self.emit(self.enc.attitude_quaternion_encode(int(now*1000),1,0,0,0,0,0,0))
            self.next_attitude = now+.1
        return len(self.rx)

    def write(self, wire):
        msg = m.MAVLink(None).parse_buffer(wire)[0]
        if msg.get_type()=='COMMAND_LONG' and msg.command==176:
            self.calls.append(msg)
            rejected=self.reject_entry and msg.param2==6
            if not rejected and not (self.ignore_entry and msg.param2==6):self.mode=int(msg.param2)
            if not self.drop_mode_ack:
                self.emit(self.enc.command_ack_encode(176,2 if rejected else 0,0,0,250,220))
            self.emit(self.enc.heartbeat_encode(2,12,0,self.mode<<16,3))
            return len(wire)
        if msg.get_type()=='HEARTBEAT':
            self.calls.append(msg);self.emit(self.enc.heartbeat_encode(2,12,0,self.mode<<16,3))
            return len(wire)
        if msg.get_type() != 'SERIAL_CONTROL' or not msg.count:
            return super().write(wire)
        self.calls.append(msg)
        if self.silent_shell: return len(wire)
        payload = bytes(msg.data[:msg.count])
        key = next((k for k,v in ground.FUSION_COMMANDS.items() if v == payload), None)
        if key == 'offboard_control_mode':
            reply=payload.decode()+f'TOPIC: offboard_control_mode\n timestamp: {int(ground.time.monotonic()*1e6)}\n position: True\n velocity: False\n acceleration: False\n attitude: False\n body_rate: False\nnsh> '
        elif key == 'visual':
            stamp = int(ground.time.monotonic()*1e6)
            body = (f'TOPIC: vehicle_visual_odometry\n timestamp: {stamp}\n timestamp_sample: {stamp-100}\n'
                    ' pose_frame: 1\n velocity_frame: 1\n reset_counter: 0\n quality: 0\n'
                    ' position: [1, -2, -3]\n q: [1, 0, 0, 0]\n')
            reply = payload.decode()+body+'nsh> '
        elif key or payload == ground.COMMANDS['vehicle_status']:
            key = key or 'commander'
            peer = SimpleNamespace(now=ground.time.monotonic(),boot_offset=0,armed=False,locked=False,
                injected=False,x=1.,y=-2.,z=-3.,vx=0.,vy=0.,velocity=0.,offboard=False,ground_z=-3.)
            reply = SyntheticPeer._reply_text(peer,key)
            if key=='local_position' and self.target_test:
                reply=reply.replace('heading_good_for_control: False','heading_good_for_control: True')
            if key == 'commander':
                if self.target_test:reply=reply.replace('nav_state: 4',f'nav_state: {14 if self.mode==6 else 2}')
                else:reply = reply.replace('pre_flight_checks_pass: True','pre_flight_checks_pass: False')
        elif payload == ground.COMMANDS['actuator_armed']:
            peer = SimpleNamespace(now=ground.time.monotonic(),boot_offset=0,armed=False,locked=False)
            reply = SyntheticPeer._reply_text(peer,'actuator')
        else:
            reply = payload.decode()+'Preflight Fail: No manual control input\nnsh> '
        data = reply.encode()
        for i in range(0,len(data),70):
            chunk = data[i:i+70]
            self.emit(self.enc.serial_control_encode(10,1,0,0,len(chunk),chunk.ljust(70,b'\0')))
        return len(wire)


def setup_capture(monkeypatch, *, invalid_after=None):
    import rigid_body_revision, ev_delay_vicon_source, vicon_receiver_provenance
    now=[100.]; peer=FusionPeer()
    monkeypatch.setattr(ground.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(ground.time,'sleep',lambda dt:now.__setitem__(0,now[0]+dt))
    monkeypatch.setattr(ground,'FixedPeerUDP',lambda *a,**kw:peer)
    monkeypatch.setattr(rigid_body_revision,'measured_capture_inputs',lambda p:dict(
        origin=[0,0,0],orientation=[0,0,0,1],revision='synthetic',calibration_approved=False))
    monkeypatch.setattr(vicon_receiver_provenance,'verify_current',lambda:{'synthetic':True})
    class Source:
        def __init__(self,*args,latest_only=False):
            assert latest_only is True
            self.report={}; self.on_frame=lambda *a:None
        def now_ros_ns(self): return int(now[0]*1e9)
        def poll(self):
            self.on_frame(dict(sdk_ok=True,occluded=invalid_after is not None and now[0]>=invalid_after,
                pose_published=True,queue_dropped=0,subject='MircoAir',segment='MircoAir',frame_id='map',
                ros_stamp_ns=self.now_ros_ns(),world_pose=[1,2,3,0,0,0,1]),self.now_ros_ns())
        def close(self): self.report['closed']=True
    monkeypatch.setattr(ev_delay_vicon_source,'ViconSource',Source)
    return peer,now


def setup_timesync_capture(monkeypatch, *, fail_probe=False, stable_after=None):
    peer,now=setup_capture(monkeypatch)
    monkeypatch.setattr(ground.time,'monotonic_ns',lambda:int(now[0]*1e9))
    monkeypatch.setattr(ground.time,'sleep',lambda dt:now.__setitem__(0,now[0]+max(dt,.02)))
    base_write=peer.write
    state=dict(interval=0,remote=0,pose_stamp=None)
    def write(wire):
        msg=m.MAVLink(None).parse_buffer(wire)[0]
        kind=msg.get_type()
        if kind=='VISION_POSITION_ESTIMATE':state['pose_stamp']=msg.usec
        if kind=='COMMAND_LONG' and msg.command in (510,511):
            peer.calls.append(msg)
            assert msg.param1==111
            if msg.command==511:
                state['interval']=int(max(0,msg.param2))
                peer.emit(peer.enc.command_ack_encode(511,0,0,0,250,220))
            else:peer.emit(peer.enc.message_interval_encode(111,state['interval']))
            return len(wire)
        if kind=='TIMESYNC' and msg.tc1>0:
            peer.calls.append(msg);state['remote']=msg.tc1//1000
            return len(wire)
        if kind=='TIMESYNC' and msg.tc1==0:
            result=base_write(wire)
            if state['interval']:peer.emit(peer.enc.timesync_encode(0,int(now[0]*1e9)))
            return result
        if kind=='SERIAL_CONTROL' and msg.count:
            payload=bytes(msg.data[:msg.count])
            if payload==ground.FUSION_COMMANDS['visual'] and fail_probe:
                raise OSError('synthetic visual probe failure')
            if payload==ground.FUSION_COMMANDS['visual'] and stable_after is not None:
                peer.calls.append(msg)
                data=(payload.decode()+'TOPIC: vehicle_visual_odometry\n'+
                    f' timestamp: {int(now[0]*1e6)}\n timestamp_sample: {state["pose_stamp"]}\n'+
                    ' pose_frame: 1\n velocity_frame: 0\n reset_counter: 0\n quality: 0\n'+
                    ' position: [1, -2, -3]\n q: [1, 0, 0, 0]\nnsh> ').encode()
                for i in range(0,len(data),70):
                    chunk=data[i:i+70];peer.emit(peer.enc.serial_control_encode(10,1,0,0,len(chunk),chunk.ljust(70,b'\0')))
                return len(wire)
            if payload in ground.SYNC_COMMANDS.values():
                peer.calls.append(msg)
                stable=stable_after is not None and now[0]>=100+stable_after
                data=(payload.decode()+'TOPIC: timesync_status\n'+
                    f' timestamp: {int(now[0]*1e6)}\n source_protocol: 0\n remote_timestamp: {state["remote"]}\n'+
                    f' observed_offset: {0 if stable else 31530}\n estimated_offset: 0\n'+
                    f' round_trip_time: {4000 if stable else 15000}\nnsh> ').encode()
                for i in range(0,len(data),70):
                    chunk=data[i:i+70];peer.emit(peer.enc.serial_control_encode(10,1,0,0,len(chunk),chunk.ljust(70,b'\0')))
                return len(wire)
        return base_write(wire)
    peer.write=write
    return peer,now,state


@pytest.mark.parametrize('fail_probe,lagged_mapping',[(False,False),(True,False),(False,True)])
def test_wifi_timesync_negative_probe_and_failure_restore_original(tmp_path,monkeypatch,fail_probe,lagged_mapping):
    peer,now,state=setup_timesync_capture(monkeypatch,fail_probe=fail_probe)
    if lagged_mapping:
        # Conversion can work while the FC offset estimate is still inaccurate.
        monkeypatch.setattr(ground,'check_clock_probe',lambda *a,**kw:dict(
            verified=True,clock_accuracy_candidate_pass=False,fc_sample_age_us=94947,
            observed_minus_estimated_offset_us=31530))
    result=ground.run(output=tmp_path/'sync',peer=('192.168.4.1',14550),bind=('192.168.4.2',14550),
        motor_power_isolated=True,fusion_observe=True,timesync_check=True,layout_evidence='synthetic',observe_seconds=30)
    assert state['interval']==0 and result['timesync_rate']['restoration_verified']
    assert peer.closed and result['localization_messages']==1
    assert result['timestamp_mapping_verified'] is lagged_mapping
    assert not result['flight_authorized']
    assert not result['clock_accuracy_candidate_pass']
    assert 'wifi_clock_accuracy_not_verified' in result['diagnostic_gaps']
    if lagged_mapping:assert 'wifi_fc_sample_time_mapping_not_verified' not in result['diagnostic_gaps']
    assert result['parameter_writes']==result['control_messages']==0
    assert not any(x.get_type() in ('PARAM_REQUEST_READ','PARAM_SET','SET_POSITION_TARGET_LOCAL_NED') for x in peer.calls)
    assert all(x.command in (510,511,512) for x in peer.calls if x.get_type()=='COMMAND_LONG')
    journal=json.loads((tmp_path/'sync'/'rates_recovery.json').read_text())
    assert not journal['needs_restore'] and journal['scope']=='wifi-motor-isolated-timesync-no-flight'
    if fail_probe:assert 'synthetic visual probe failure' in result['error']
    else:
        assert result['error'] is None and result['observation_window_complete']
        assert result['clock_probe']['fc_sample_age_us']==(94947 if lagged_mapping else 100)


def test_continuous_sync_can_settle_after_old_60s_deadline_without_restart(tmp_path,monkeypatch):
    peer,now,state=setup_timesync_capture(monkeypatch,stable_after=65)
    result=ground.run(output=tmp_path/'late_sync',peer=('192.168.4.1',14550),bind=('192.168.4.2',14550),
        motor_power_isolated=True,fusion_observe=True,timesync_check=True,layout_evidence='synthetic',observe_seconds=180)
    assert result['error'] is None
    assert 65 < result['window_completed_at']-result['window_started_at'] < 100
    assert result['sync_stop_reason']=='stable_mapping' and result['clock_stability_candidate_pass']
    assert result['timestamp_mapping_verified'] and result['clock_accuracy_candidate_pass']
    assert result['fc_accepted_sample_count'] is None  # Never infer this from host reply count.
    assert result['timesync_response_count'] < 500  # Synthetic peer already models a converged FC.
    assert not result['diagnostic_gaps'] and not result['flight_authorized']
    assert result['localization_messages']==1
    assert [x['interval_us'] for x in result['timesync_rate']['writes']]==[100000,-1]
    assert state['interval']==0 and result['timesync_rate']['restoration_verified'] and peer.closed


def test_missing_vicon_stops_before_rate_change_and_keeps_primary_error(tmp_path,monkeypatch):
    import ev_delay_vicon_source
    peer,now,state=setup_timesync_capture(monkeypatch)
    source=ev_delay_vicon_source.ViconSource
    monkeypatch.setattr(source,'poll',lambda self:None)
    def close(self):
        self.report['closed']=True
        raise RuntimeError('synthetic cleanup failure')
    monkeypatch.setattr(source,'close',close)
    result=ground.run(output=tmp_path/'missing',peer=('192.168.4.1',14550),bind=('192.168.4.2',14550),
        motor_power_isolated=True,fusion_observe=True,timesync_check=True,layout_evidence='synthetic',observe_seconds=180)
    assert 'No fresh Vicon frame' in result['error']
    assert 'synthetic cleanup failure' in result['source_cleanup_error']
    assert result['localization_messages']==0 and peer.closed
    assert not any(x.get_type()=='COMMAND_LONG' and x.command==511 for x in peer.calls)


def test_source_failure_does_not_prevent_rate_restoration(tmp_path,monkeypatch):
    import ev_delay_vicon_source
    peer,now,state=setup_timesync_capture(monkeypatch)
    source=ev_delay_vicon_source.ViconSource;poll=source.poll
    def failing_poll(self):
        if now[0]>105:raise RuntimeError('synthetic Vicon disconnect')
        poll(self)
    monkeypatch.setattr(source,'poll',failing_poll)
    result=ground.run(output=tmp_path/'disconnect',peer=('192.168.4.1',14550),bind=('192.168.4.2',14550),
        motor_power_isolated=True,fusion_observe=True,timesync_check=True,layout_evidence='synthetic',observe_seconds=180)
    assert 'synthetic Vicon disconnect' in result['error']
    assert result['timesync_rate']['restoration_verified'] and state['interval']==0
    assert not result.get('timesync_restore_error') and peer.closed


@pytest.mark.parametrize('seconds,timesync',[(181,True),(61,False)])
def test_longer_deadline_only_for_bounded_timesync_before_opening_devices(tmp_path,seconds,timesync):
    out=tmp_path/'not_opened'
    with pytest.raises(ValueError,match='observation seconds'):
        ground.run(output=out,peer=('192.168.4.1',14550),bind=('192.168.4.2',14550),
            motor_power_isolated=True,fusion_observe=True,timesync_check=timesync,
            layout_evidence='synthetic',observe_seconds=seconds)
    assert not out.exists()


def test_fusion_and_sync_share_owner_and_restore_without_control(tmp_path,monkeypatch):
    peer,now,state=setup_timesync_capture(monkeypatch,stable_after=0)
    result=ground.run(output=tmp_path/'fusion_sync',peer=('192.168.4.1',14550),bind=('192.168.4.2',14550),
        motor_power_isolated=True,fusion_observe=True,fusion_clock_check=True,layout_evidence='synthetic',observe_seconds=60)
    assert result['error'] is None and result['observation_window_complete']
    assert len(result['fusion_rounds'])==3 and len(result['timesync_status'])==3
    assert result['localization_messages']>1000 and result['timesync_response_count']>50
    assert all('timesync' in row for row in result['fusion_rounds'])
    assert not result['diagnostic_gaps'] and not result['flight_control_effect_verified']
    assert result['parameter_writes']==result['control_messages']==0
    assert not any(x.get_type() in ('PARAM_SET','PARAM_REQUEST_READ','SET_POSITION_TARGET_LOCAL_NED','MANUAL_CONTROL','RC_CHANNELS_OVERRIDE') for x in peer.calls)
    assert all(x.command in (510,511,512) for x in peer.calls if x.get_type()=='COMMAND_LONG')
    assert state['interval']==0 and result['timesync_rate']['restoration_verified'] and peer.closed


@pytest.mark.parametrize('failure',[None,'source','armed','landed_stale','usb'])
def test_battery_static_reuses_rates_without_control(tmp_path,monkeypatch,failure):
    peer,now,state=setup_timesync_capture(monkeypatch,stable_after=0)
    monkeypatch.setattr(ground,'FC_USB',SimpleNamespace(exists=lambda:failure=='usb' and now[0]>110))
    original=peer.write
    def write(wire):
        msg=m.MAVLink(None).parse_buffer(wire)[0]
        result=original(wire)
        if msg.get_type()=='HEARTBEAT':
            if failure=='armed' and now[0]>110:
                peer.emit(peer.enc.heartbeat_encode(2,12,128,3<<16,3))
            if not (failure=='landed_stale' and now[0]>110):
                peer.emit(peer.enc.extended_sys_state_encode(0,1))
        return result
    peer.write=write
    if failure=='source':
        import ev_delay_vicon_source
        old=ev_delay_vicon_source.ViconSource.poll
        def poll(source):
            if now[0]>110:raise RuntimeError('synthetic source lost')
            old(source)
        monkeypatch.setattr(ev_delay_vicon_source.ViconSource,'poll',poll)
    result=ground.run(output=tmp_path/'battery',peer=('192.168.4.1',14550),bind=('192.168.4.2',14550),
        battery_static=True,fusion_observe=True,fusion_clock_check=True,layout_evidence='synthetic',observe_seconds=60)
    assert result['battery_static'] and not result['motor_power_isolated']
    assert not result['formal_flight_runtime_validated']
    assert result['parameter_writes']==result['control_messages']==0 and peer.closed
    assert not any(x.get_type() in ('PARAM_SET','PARAM_REQUEST_READ','SET_POSITION_TARGET_LOCAL_NED','MANUAL_CONTROL','RC_CHANNELS_OVERRIDE') for x in peer.calls)
    assert all(x.command in (510,511,512) for x in peer.calls if x.get_type()=='COMMAND_LONG')
    if failure is None:
        assert result['error'] is None and result['observation_window_complete'],result['error']
        assert len(result['fusion_rounds'])==3 and result['timesync_response_count']>50
    else:assert result['error']
    if failure in (None,'source'):
        assert state['interval']==0 and result['timesync_rate']['restoration_verified']
    else:assert result['timesync_restore_error'] and not result['timesync_rate']['restoration_verified']


@pytest.mark.parametrize('override',[dict(motor_power_isolated=True),dict(target_load=True),
    dict(offboard_check=True),dict(observe_seconds=180),dict(fusion_clock_check=False)])
def test_battery_static_scope_rejected_before_resources(tmp_path,override):
    options=dict(output=tmp_path/'never_opened',peer=('192.168.4.1',14550),bind=('192.168.4.2',14550),
        battery_static=True,fusion_observe=True,fusion_clock_check=True,layout_evidence='synthetic',observe_seconds=60)
    options.update(override)
    with pytest.raises(ValueError):ground.run(**options)
    assert not options['output'].exists()


@pytest.mark.parametrize('extra',[dict(timesync_check=True),dict(target_load=True),dict(offboard_check=True)])
def test_fusion_clock_check_cannot_expand_into_other_live_routes(tmp_path,extra):
    with pytest.raises(ValueError):
        ground.run(output=tmp_path/'never_opened',peer=('192.168.4.1',14550),bind=('192.168.4.2',14550),
            motor_power_isolated=True,fusion_observe=True,fusion_clock_check=True,
            layout_evidence='synthetic',observe_seconds=60,**extra)
    assert not (tmp_path/'never_opened').exists()


def test_same_entry_complete_fusion_window_without_rc_or_controls(tmp_path,monkeypatch):
    peer,now=setup_capture(monkeypatch)
    out=tmp_path/'fusion'
    code=entry.main(['--run','--fusion-observe','--motor-power-isolated','--output',str(out),
        '--layout-evidence',str(tmp_path/'synthetic.json'),'--observe-seconds','15'])
    report=json.loads((out/'report.json').read_text())
    assert code == 0, report['error']
    assert report['observation_window_complete'] and len(report['fusion_rounds'])==2
    assert report['localization_messages']>200 and report['max_pose_send_gap_s']<.06
    assert report['parameter_writes']==report['control_messages']==0
    assert not report['diagnostic_gaps']
    assert not report['flight_authorized'] and not report['calibration_approved']
    for row in report['fusion_rounds']:
        assert row['selector_stable'] and row['primary']==1
        assert row['aids']['aid']['parsed']['estimator_instance']==1
        assert row['aids']['aid']['topic']=='aid_0'  # Slot is NOT estimator index.
        assert row['flags']['parsed']['cs_ev_pos']
        assert row['vehicle_status']['parsed']['pre_flight_checks_pass'] is False
        assert 'No manual control input' in row['commander']['text']
    assert peer.closed and report['vicon_receiver']['closed']
    assert all(x.get_type() not in ('PARAM_SET','RC_CHANNELS_OVERRIDE','MANUAL_CONTROL','SET_POSITION_TARGET_LOCAL_NED') for x in peer.calls)
    assert all(x.command==512 for x in peer.calls if x.get_type()=='COMMAND_LONG')
    commands=[bytes(x.data[:x.count]) for x in peer.calls if x.get_type()=='SERIAL_CONTROL']
    assert not any(b'input_rc' in x or b'manual_control' in x for x in commands)
    assert (out/'traffic.jsonl').stat().st_size and (out/'vicon_frames.jsonl').stat().st_size


def test_isolated_target_load_reuses_writer_without_mode_arm_or_parameter_scan(tmp_path,monkeypatch):
    peer,now=setup_capture(monkeypatch);peer.target_test=True
    result=ground.run(output=tmp_path/'load',peer=('192.168.4.1',14550),bind=('192.168.4.2',14550),
        motor_power_isolated=True,fusion_observe=True,target_load=True,layout_evidence='synthetic',observe_seconds=15)
    assert result['error'] is None,result['error']
    targets=[x for x in peer.calls if x.get_type()=='SET_POSITION_TARGET_LOCAL_NED']
    assert 40<=len(targets)<=60 and result['observation_window_complete']
    assert len({(x.x,x.y,x.z,x.yaw) for x in targets})==1
    assert all((x.x,x.y,x.z)==(1.,-2.,-3.) and x.type_mask==2552 for x in targets)
    assert not any(x.get_type() in ('PARAM_SET','PARAM_REQUEST_READ','RC_CHANNELS_OVERRIDE','MANUAL_CONTROL') for x in peer.calls)
    assert all(x.command==512 for x in peer.calls if x.get_type()=='COMMAND_LONG')
    assert result['target_receipt']['parsed']['position'] and peer.closed


@pytest.mark.parametrize('fault',[None,'rejected','no_ack','no_state','source_loss'])
def test_disarmed_offboard_entry_exit_and_failure_return(tmp_path,monkeypatch,fault):
    peer,now=setup_capture(monkeypatch,invalid_after=108 if fault=='source_loss' else None)
    peer.target_test=True;peer.reject_entry=fault=='rejected'
    peer.drop_mode_ack=fault=='no_ack';peer.ignore_entry=fault=='no_state'
    result=ground.run(output=tmp_path/'mode',peer=('192.168.4.1',14550),bind=('192.168.4.2',14550),
        motor_power_isolated=True,fusion_observe=True,target_load=True,offboard_check=True,
        layout_evidence='synthetic',observe_seconds=30)
    modes=[x for x in peer.calls if x.get_type()=='COMMAND_LONG' and x.command==176]
    assert [x.param2 for x in modes]==[6,3]
    assert all(x.command in (176,512) for x in peer.calls if x.get_type()=='COMMAND_LONG')
    assert not any(x.get_type() in ('PARAM_SET','PARAM_REQUEST_READ','RC_CHANNELS_OVERRIDE','MANUAL_CONTROL') for x in peer.calls)
    targets=[x for x in peer.calls if x.get_type()=='SET_POSITION_TARGET_LOCAL_NED']
    assert len({(x.x,x.y,x.z,x.yaw) for x in targets})==1
    assert peer.closed and peer.mode==3
    assert result['observation_window_complete'] is (fault is None)
    if fault is None:
        assert result['error'] is None and all(r['confirmed'] for r in result['mode_requests'])
        assert result['offboard_status']['parsed']['nav_state']==14
    else:assert result['error']
    if fault=='no_ack':assert result['mode_restore_error']


def test_offboard_scope_and_one_shot_mode_permissions():
    now=[100.];peer=Peer()
    ob=ground.GroundObserver(peer,lambda r:None,clock=lambda:now[0],offboard_check=True)
    ob.identity={'synthetic':True};ob.rc.heartbeat=dict(armed=False,received_at=100.,custom_mode=3<<16)
    for main in (3,6,4):
        with pytest.raises(PermissionError):ob.request_ground_mode(main)
    for command in (176,400):
        with pytest.raises(PermissionError):ob.send(ob.encoder.command_long_encode(1,1,command,0,1,6,0,0,0,0,0))
    ob.mode_requests=[dict(main=6,at=99.)]
    ob.request_ground_mode(3)
    with pytest.raises(PermissionError):ob.request_ground_mode(3)
    with pytest.raises(PermissionError):ob.request_ground_mode(6)


def test_rejected_slow_clock_sample_schedules_prompt_refresh_without_relaxing_age():
    now=[100.];peer=Peer();rows=[]
    ob=ground.GroundObserver(peer,rows.append,clock=lambda:now[0],fusion_observe=True)
    ob.identity={'synthetic':True};ob.rc.heartbeat=dict(armed=False,received_at=100.,custom_mode=3<<16)
    ob.rc.observe_clock(99_000_000,99.)
    ob.pending_sync={42:100.};ob.next_sync=101.
    now[0]=100.102
    ob.ingest(peer.enc.timesync_encode(100_050_000_000,42).pack(peer.enc))
    assert ob.rc.fc_clock==(99_000_000,99.) and ob.next_sync==pytest.approx(100.202)
    assert rows[-1]['accepted'] is False
    now[0]=101.01
    with pytest.raises(RuntimeError,match='clock missing'):
        ob.pose(dict(source_stamp_ns=101_010_000_000),101_010_000_000)


@pytest.mark.parametrize('fault',['mode','armed','stale_pose','stale_local','reset','moved'])
def test_fixed_ground_targets_stop_on_changed_live_state(fault):
    now=[100.];peer=Peer()
    ob=ground.GroundObserver(peer,lambda row:None,clock=lambda:now[0],fusion_observe=True,ground_target_load=True)
    ob.identity={'synthetic':True};ob.rc.heartbeat=dict(armed=False,received_at=100.,custom_mode=3<<16)
    ob.rc.observe_clock(100_000_000,100.);ob.last_pose_at=100.
    local=dict(at=100.,parsed=dict(timestamp=100_000_000,x=1.,y=2.,z=-.1,heading=0.,
        xy_valid=True,z_valid=True,heading_good_for_control=True,xy_reset_counter=0,z_reset_counter=0,heading_reset_counter=0))
    status=dict(at=100.,parsed=dict(timestamp=100_000_000,nav_state=2,pre_flight_checks_pass=True))
    actuator=dict(at=100.,parsed=dict(timestamp=100_000_000,armed=False,lockdown=False,manual_lockdown=False,force_failsafe=False))
    ob.freeze_ground_target(local,status,actuator)
    # Even after explicit setup, callers cannot directly inject arbitrary targets.
    with pytest.raises(PermissionError):
        ob.send(ob.encoder.set_position_target_local_ned_encode(1,1,1,1,2552,1,2,-5,0,0,0,0,0,0,0,0))
    if fault=='mode':ob.rc.heartbeat['custom_mode']=6<<16
    elif fault=='armed':ob.rc.heartbeat['armed']=True
    elif fault=='stale_pose':ob.last_pose_at=99.
    elif fault=='stale_local':local['at']=98.
    elif fault=='reset':local['parsed']['xy_reset_counter']=1
    else:local['parsed']['x']=1.2
    with pytest.raises(RuntimeError,match='state changed/stale'):ob.ground_target_tick()
    assert not peer.calls


def test_occlusion_stops_pose_sending_and_preserves_partial_evidence(tmp_path,monkeypatch):
    peer,now=setup_capture(monkeypatch,invalid_after=102.)
    result=ground.run(output=tmp_path/'lost',peer=('192.168.4.1',14550),bind=('192.168.4.2',14550),
        motor_power_isolated=True,fusion_observe=True,layout_evidence='synthetic',observe_seconds=15)
    assert 'Fresh Vicon frame missing' in result['error']
    assert not result['observation_window_complete'] and result['localization_messages']>0
    assert peer.closed and (tmp_path/'lost'/'report.json').exists()


def test_fusion_observer_cannot_send_arbitrary_pose_or_controls():
    peer=Peer(); ob=ground.GroundObserver(peer,lambda row:None,clock=lambda:100.,fusion_observe=True)
    ob.identity={'synthetic':True}; ob.rc.heartbeat={'armed':False,'received_at':100.}
    enc=m.MAVLink(None)
    for msg in [enc.vision_position_estimate_encode(1,0,0,0,0,0,0),
                enc.command_long_encode(1,1,400,0,1,0,0,0,0,0,0),
                enc.param_set_encode(1,1,b'EKF2_EV_CTRL',11,6)]:
        with pytest.raises(PermissionError): ob.send(msg)
    assert not peer.calls


@pytest.mark.parametrize('extra', [[],['--layout-evidence','fake'],['--motor-power-isolated']])
def test_missing_fusion_conditions_never_open_resources(tmp_path,monkeypatch,extra):
    monkeypatch.setattr(ground,'run',lambda **kw:pytest.fail('Incomplete scope opened resources'))
    with pytest.raises(SystemExit):
        entry.main(['--run','--fusion-observe','--output',str(tmp_path/'out'),*extra])


def test_fusion_requires_current_clock_and_fresh_pose():
    peer=Peer(); ob=ground.GroundObserver(peer,lambda row:None,clock=lambda:100.,fusion_observe=True)
    ob.identity={'synthetic':True}; ob.rc.heartbeat={'armed':False,'received_at':100.}
    obs=dict(source_stamp_ns=100_000_000_000,position=[0,0,0],quaternion_wxyz=[1,0,0,0])
    with pytest.raises(RuntimeError,match='clock missing'): ob.pose(obs,100_000_000_000)
    ob.rc.observe_clock(100_000_000,100.)
    with pytest.raises(RuntimeError,match='Vicon frame missing'): ob.pose(obs,100_200_000_000)
    assert not peer.calls


def test_armed_during_fusion_stops_without_auto_disarm(tmp_path,monkeypatch):
    peer,now=setup_capture(monkeypatch)
    original_read=peer.read
    def read(n):
        if now[0]>=102.:
            peer.emit(peer.enc.heartbeat_encode(2,12,128,3<<16,3))
            n=len(peer.rx)
        return original_read(n)
    monkeypatch.setattr(peer,'read',read)
    report=ground.run(output=tmp_path/'armed',peer=('192.168.4.1',14550),bind=('192.168.4.2',14550),
        motor_power_isolated=True,fusion_observe=True,layout_evidence='synthetic',observe_seconds=15)
    assert 'ARMED' in report['error'] and not report['observation_window_complete']
    assert all(msg.command==512 for msg in peer.calls if msg.get_type()=='COMMAND_LONG')


def test_incomplete_diagnostic_preserves_evidence_and_does_not_retry(tmp_path,monkeypatch):
    peer,now=setup_capture(monkeypatch)
    original_write=peer.write
    def write(wire):
        msg=m.MAVLink(None).parse_buffer(wire)[0]
        if msg.get_type()=='SERIAL_CONTROL' and bytes(msg.data[:msg.count])==ground.FUSION_COMMANDS['selector']:
            peer.silent_shell=True
        return original_write(wire)
    monkeypatch.setattr(peer,'write',write)
    report=ground.run(output=tmp_path/'partial',peer=('192.168.4.1',14550),bind=('192.168.4.2',14550),
        motor_power_isolated=True,fusion_observe=True,layout_evidence='synthetic',observe_seconds=15)
    assert 'Incomplete reply: selector' in report['error']
    assert not report['observation_window_complete'] and report['localization_messages']>0
    commands=[bytes(x.data[:x.count]) for x in peer.calls if x.get_type()=='SERIAL_CONTROL']
    assert commands.count(ground.FUSION_COMMANDS['selector'])==1 and peer.closed
