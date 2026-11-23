"""New Pocket/WiFi boundaries only. No physical socket, serial, RC or Vicon I/O."""
import json
from pathlib import Path
import socket
import sys

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pymavlink.dialects.v20 import common as m
from mavlink_link import FixedPeerUDP
import native_rc_ground as ground
from rc_watch import NativeRCMonitor
import vicon_fixed_point_entry as entry


@pytest.fixture(autouse=True)
def no_network(monkeypatch):
    def deny(*a, **kw): raise AssertionError('Offline test attempted a real socket')
    monkeypatch.setattr(socket, 'socket', deny)


class Socket:
    def __init__(self, *args): self.queue=[]; self.sent=[]; self.closed=False
    def setblocking(self, value): assert value is False
    def bind(self, addr): self.bound=addr
    def recvfrom(self, size):
        if not self.queue: raise BlockingIOError()
        return self.queue.pop(0)
    def sendto(self, data, addr): self.sent.append((data, addr)); return len(data)
    def close(self): self.closed=True


def test_fixed_peer_never_learns_foreign_peer_and_records_every_datagram():
    sock=Socket(); evidence=[]
    port=FixedPeerUDP(('192.168.4.1',14550),('192.168.4.2',14550),
        socket_factory=lambda *a:sock,on_datagram=lambda *row:evidence.append(row))
    sock.queue=[(b'foreign',('192.168.50.1',14550)),(b'ab',port.peer),(b'cd',port.peer)]
    assert port.in_waiting==4 and port.read(3)==b'abc' and port.read(8)==b'd'
    assert evidence[0][2] is False and port.foreign_datagrams==1
    assert port.write(b'first-heartbeat')==15
    assert sock.sent==[(b'first-heartbeat',('192.168.4.1',14550))]
    port.close(); assert sock.closed


def test_udp_overflow_is_recorded_then_stops_without_reconnect():
    sock=Socket(); evidence=[]
    port=FixedPeerUDP(('192.168.4.1',14550),('192.168.4.2',14550),
        socket_factory=lambda *a:sock,on_datagram=lambda *r:evidence.append(r))
    sock.queue=[(b'a'*40000,port.peer),(b'b'*40000,port.peer)]
    with pytest.raises(BufferError): _=port.in_waiting
    assert len(evidence)==2 and not sock.sent


def topic_text(topic, now=100_000_000, **changes):
    fields={name:now if name.startswith('timestamp') else 1
            for name in NativeRCMonitor.TOPICS[topic][0]}
    fields.update({name:False for name in NativeRCMonitor.TOPICS[topic][1]})
    if topic=='input_rc': fields.update(input_source=9,channel_count=8,rssi=-1)
    if topic=='manual_control_setpoint': fields.update(data_source=1,valid=True)
    if topic=='manual_control_switches': fields.update(kill_switch=3,arm_switch=3,offboard_switch=3)
    fields.update(changes)
    return 'TOPIC: '+topic+'\n'+''.join(f' {k}: {v}\n' for k,v in fields.items())


def ready_monitor():
    rc=NativeRCMonitor()
    rc.observe_clock(100_010_000,5.)
    for topic in rc.TOPICS: rc.observe_topic(topic,topic_text(topic),5.)
    return rc


def test_native_rc_unknown_rssi_and_centered_channels_are_not_empty():
    rc=ready_monitor(); enc=m.MAVLink(None,srcSystem=1,srcComponent=1)
    wire=enc.rc_channels_encode(100000,8,*([1500]*18),255).pack(enc)
    msg=m.MAVLink(None).parse_buffer(wire)[0]
    rc.observe_message(msg,5.)
    status=rc.status(5.1)
    assert status['native_input_observed']
    assert status['channels']['values']==[1500]*8 and status['channels']['rssi'] is None
    assert not status['channels']['source_identity_proven']
    assert not status['flight_authorized']


@pytest.mark.parametrize('topic,change',[
    ('input_rc',{'input_source':6}), ('input_rc',{'input_source':0}),
    ('input_rc',{'rc_lost':True}), ('input_rc',{'rc_failsafe':True}),
    ('input_rc',{'timestamp_last_signal':1}), ('input_rc',{'channel_count':0}),
    ('manual_control_setpoint',{'data_source':2}), ('manual_control_setpoint',{'valid':False}),
    ('manual_control_setpoint',{'timestamp_sample':1}),
    ('failsafe_flags',{'manual_control_signal_lost':True}),
])
def test_missing_native_rc_evidence_does_not_reuse_gamepad_success(topic,change):
    rc=ready_monitor(); rc.observe_topic(topic,topic_text(topic,**change),5.)
    assert not rc.status(5.1)['native_input_observed']


def test_stale_uorb_and_stale_host_receipts_never_become_ready():
    rc=ready_monitor()
    assert not rc.status(7.1)['native_input_observed']
    rc.observe_clock(200_000_000,8.)
    for topic in rc.TOPICS: rc.observe_topic(topic,topic_text(topic),8.)
    assert not rc.status(8.)['native_input_observed']
    with pytest.raises(RuntimeError,match='regressed'): rc.observe_clock(10,8.1)


def test_recorded_failsafe_reply_no_longer_expires_at_501ms():
    from copy import deepcopy
    evidence=json.loads((Path(__file__).parent/'fixtures/native_shell_timing_20261109.json').read_text())
    failed=evidence['failure_native'];now=evidence['failure_context']['now']
    rc=NativeRCMonitor();rc.topics=deepcopy(failed['raw_snapshots'])
    upper=failed['input_rc']['timestamp']+failed['topic_timing']['input_rc']['source_age_upper_s']*1e6
    rc.observe_clock(upper,now)
    status=rc.status(now,max_age=.5)
    timing=status['topic_timing']['failsafe_flags']
    assert timing['host_age_s']==pytest.approx(.501266929)
    assert timing['reply_limit_s']==1. and timing['source_limit_s']==1.5
    assert not timing['problems'] and status['native_input_observed']
    # A received loss flag is never delayed by the longer missing-reply limit.
    rc.observe_topic('failsafe_flags',topic_text('failsafe_flags',now=int(upper),manual_control_signal_lost=True),now)
    assert 'fc_manual_control_loss_or_unknown' in rc.status(now,max_age=.5)['reasons']


@pytest.mark.parametrize('host_age,source_age,valid',[(.501,.667,True),(1.,1.5,True),
    (1.001,.9,False),(.1,1.501,False),(-.001,.1,False)])
def test_failsafe_one_second_boundary_keeps_source_age(host_age,source_age,valid):
    rc=NativeRCMonitor();rc.observe_clock(100_000_000,5.)
    rc.observe_topic('failsafe_flags',topic_text('failsafe_flags',now=100_000_000-round(source_age*1e6)),5.-host_age)
    timing=rc.topic_timing('failsafe_flags',5.,max_age=.5)
    assert (not timing['problems']) is valid
    if valid:assert timing['query_headroom_s']==pytest.approx(min(1-host_age,1.5-source_age))
    rc.observe_topic('manual_control_setpoint',topic_text('manual_control_setpoint'),4.499)
    assert 'reply_stale_or_future' in rc.topic_timing('manual_control_setpoint',5.)['problems']


def test_kill_intent_is_not_output_confirmation_and_failed_parse_clears_old_value():
    rc=ready_monitor()
    rc.observe_topic('manual_control_switches',topic_text('manual_control_switches',kill_switch=1),5.)
    state=rc.status(5.1)
    assert state['kill_requested'] is True and state['output_lock_confirmed'] is False
    rc.observe_topic('actuator_armed','truncated',5.1)
    assert rc.status(5.1)['output_lock_confirmed'] is None


def test_switch_intent_does_not_manufacture_actual_mode_change():
    rc=ready_monitor(); enc=m.MAVLink(None,srcSystem=1,srcComponent=1)
    decoder=m.MAVLink(None)
    def hb(mode): return decoder.parse_buffer(enc.heartbeat_encode(2,12,0,mode,3).pack(enc))[0]
    rc.observe_message(hb(6<<16),5.)
    rc.observe_topic('manual_control_switches',topic_text('manual_control_switches',mode_slot=2),5.)
    assert rc.status(5.1)['actual_flight_mode']==6<<16
    assert not rc.status(5.1)['mode_changes']
    rc.observe_message(hb(3<<16),5.2)
    assert rc.status(5.2)['actual_flight_mode']==3<<16
    assert len(rc.status(5.2)['mode_changes'])==1


@pytest.mark.parametrize('command,params',[(185,[2,0,0,0,0,0,0]),(400,[1,0,0,0,0,0,0]),
                                        (176,[1,6,0,0,0,0,0]),(511,[65,100000,0,0,0,0,0])])
def test_ground_writer_rejects_lock_arm_mode_and_rate_commands(command,params):
    enc=m.MAVLink(None)
    assert not ground.allowed(enc.command_long_encode(1,1,command,0,*params),True)


def test_ground_writer_rejects_override_parameter_and_pose_writes():
    enc=m.MAVLink(None)
    msgs=[enc.rc_channels_override_encode(1,1,*([1500]*8)),
          enc.param_set_encode(1,1,b'COM_RC_IN_MODE',0,m.MAV_PARAM_TYPE_INT32),
          enc.vision_position_estimate_encode(1,0,0,0,0,0,0)]
    assert all(not ground.allowed(msg,True) for msg in msgs)
    assert not ground.allowed(enc.serial_control_encode(10,2,0,0,7,b'reboot\n'.ljust(70,b'\0')),True)


class Peer:
    """Synthetic flight controller behind an in-memory byte transport."""
    def __init__(self, *a, **kw):
        self.enc=m.MAVLink(None,srcSystem=1,srcComponent=1)
        self.dec=m.MAVLink(None); self.rx=bytearray(); self.calls=[]; self.closed=False
    @property
    def in_waiting(self): return len(self.rx)
    def read(self,n): data=bytes(self.rx[:n]); del self.rx[:n]; return data
    def emit(self,msg):
        self.rx.extend(msg.pack(self.enc)); self.enc.seq=(self.enc.seq+1)%256
    def write(self,wire):
        msg=self.dec.parse_buffer(wire)[0]; self.calls.append(msg)
        kind=msg.get_type(); now=ground.time.monotonic()
        if kind=='HEARTBEAT': self.emit(self.enc.heartbeat_encode(2,12,0,3<<16,3))
        elif kind=='COMMAND_LONG':
            self.emit(self.enc.autopilot_version_encode(0,(1<<24)+(14<<16)+(3<<8),0,0,54,
                bytes(8),bytes(8),bytes(8),0,0,1,bytes.fromhex(ground.UID2)))
        elif kind=='TIMESYNC': self.emit(self.enc.timesync_encode(int(now*1e9),msg.ts1))
        elif kind=='PARAM_REQUEST_READ':
            self.emit(self.enc.param_value_encode(msg.param_id.encode(),1.,m.MAV_PARAM_TYPE_REAL32,31,0))
        elif kind=='SERIAL_CONTROL' and msg.count:
            command=bytes(msg.data[:msg.count]); topic=command.decode().split()[1]
            body=topic_text(topic,int(now*1e6)) if topic in NativeRCMonitor.TOPICS else 'Preflight check: FAILED\n'
            reply=command+body.encode()+b'nsh> '
            for i in range(0,len(reply),70):
                chunk=reply[i:i+70]
                self.emit(self.enc.serial_control_encode(10,1,0,0,len(chunk),chunk.ljust(70,b'\0')))
        return len(wire)
    def close(self): self.closed=True


def test_wifi_replies_only_to_valid_same_channel_requests_in_source_clock():
    peer=Peer();rows=[];stamp=1_800_000_000_123_456_789
    ob=ground.GroundObserver(peer,rows.append,clock=lambda:100.,timesync_check=True,ros_clock_ns=lambda:stamp)
    ob.identity=dict(uid2=ground.UID2,firmware='1.14.3')
    ob.ingest(peer.enc.heartbeat_encode(2,12,0,3<<16,3).pack(peer.enc))
    request=peer.enc.timesync_encode(0,90_000_000_000).pack(peer.enc)
    ob.ingest(request);ob.ingest(request)
    replies=[v for v in peer.calls if v.get_type()=='TIMESYNC']
    assert len(replies)==1 and (replies[0].tc1,replies[0].ts1)==(stamp,90_000_000_000)
    assert len(ob.sync_replies)==1
    foreign=m.MAVLink(None,srcSystem=2,srcComponent=1)
    ob.ingest(foreign.timesync_encode(0,91_000_000_000).pack(foreign))
    assert len(ob.sync_replies)==1
    with pytest.raises(PermissionError): ob.send(ob.encoder.timesync_encode(stamp,92_000_000_000))
    ob.timesync_check=False
    ob.ingest(peer.enc.timesync_encode(0,93_000_000_000).pack(peer.enc))
    assert len(ob.sync_replies)==1


def test_timesync_diagnostic_cannot_be_combined_with_mode_or_flight_routes(monkeypatch):
    monkeypatch.setattr(ground,'run',lambda **kw:pytest.fail('Invalid combination opened hardware'))
    for extra in (['--target-load'],['--flight-console'],['--observe-seconds','15']):
        with pytest.raises(SystemExit):
            entry.main(['--run','--fusion-observe','--wifi-timesync-check',*extra])


def test_current_entry_dispatches_one_native_observer_without_gamepad(tmp_path,monkeypatch):
    now=[100.]; peer=Peer()
    monkeypatch.setattr(ground.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(ground.time,'sleep',lambda dt:now.__setitem__(0,now[0]+dt))
    monkeypatch.setattr(ground,'FixedPeerUDP',lambda *a,**kw:peer)
    monkeypatch.setattr(entry,'open_session',lambda **kw:pytest.fail('Legacy gamepad route selected'))
    out=tmp_path/'native'
    code=entry.main(['--run','--handset','pocket','--ground-observe','--motor-power-isolated',
                     '--output',str(out),'--observe-seconds','0'])
    report=json.loads((out/'report.json').read_text())
    assert code==0,report['error']
    assert report['native_rc']['native_input_observed'] and peer.closed
    assert report['parameter_writes']==report['control_messages']==report['localization_messages']==0
    assert not report['missing_parameters'] and not report['flight_authorized']
    assert {msg.get_type() for msg in peer.calls}=={'HEARTBEAT','COMMAND_LONG','TIMESYNC','PARAM_REQUEST_READ','SERIAL_CONTROL'}
    assert all(msg.command==512 for msg in peer.calls if msg.get_type()=='COMMAND_LONG')


def test_crc_corruption_does_not_refresh_heartbeat_and_valid_frame_survives():
    now=[100.]; peer=Peer(); rows=[]
    ob=ground.GroundObserver(peer,rows.append,clock=lambda:now[0])
    good=peer.enc.heartbeat_encode(2,12,0,3<<16,3).pack(peer.enc)
    bad=bytearray(good); bad[-1]^=1
    ob.ingest(bytes(bad)+good)
    assert ob.rc.heartbeat['received_at']==100.
    assert any(row['direction']=='bad_frame' for row in rows)
    now[0]=102.1; ob.ingest(bytes(bad))
    with pytest.raises(RuntimeError,match='heartbeat lost'): ob.pump()


def test_default_and_incomplete_cli_never_open_devices(monkeypatch,capsys):
    monkeypatch.setattr(ground,'run',lambda **kw:pytest.fail('No explicit run scope'))
    assert entry.main([])==0
    report=json.loads(capsys.readouterr().out)
    assert report['physical_tx_bytes']==0 and report['pocket_flight_runtime_integrated']
    with pytest.raises(SystemExit): entry.main(['--run'])


def test_failure_saves_report_and_closes_socket(tmp_path,monkeypatch):
    now=[100.]; peer=Peer()
    def fail_write(wire): raise OSError('synthetic send failure')
    peer.write=fail_write
    monkeypatch.setattr(ground,'FixedPeerUDP',lambda *a,**kw:peer)
    report=ground.run(output=tmp_path/'failure',peer=('192.168.4.1',14550),
        bind=('192.168.4.2',14550),motor_power_isolated=True,observe_seconds=0)
    assert 'synthetic send failure' in report['error'] and peer.closed
    assert (tmp_path/'failure'/'report.json').exists()


def test_only_correlated_low_rtt_clock_samples_are_used():
    now=[100.]; peer=Peer()
    ob=ground.GroundObserver(peer,lambda row:None,clock=lambda:now[0])
    ob.pending_sync={42:100.}
    ob.ingest(peer.enc.timesync_encode(100_000_000_000,43).pack(peer.enc))
    assert ob.rc.fc_clock is None
    now[0]=100.2
    ob.ingest(peer.enc.timesync_encode(100_000_000_000,42).pack(peer.enc))
    assert ob.rc.fc_clock is None


def test_recorded_rtt_drop_is_not_a_fc_clock_regression():
    # 2026-11-10 flight: raw clock +39.769 ms, receipt +43.243370 ms,
    # request +100.795284 ms. RTT shrank, so the upper bound fell 17.782914 ms.
    # Translate the unknown host origin; only these recorded deltas matter.
    now=[1.072];peer=Peer();ob=ground.GroundObserver(peer,lambda row:None,clock=lambda:now[0])
    samples=((1.,1.072,432535127000),
             (1.100795284,1.115243370,432574896000))
    bounds=[]
    for sent,received,remote in samples:
        now[0]=received;token=round(sent*1e9);ob.pending_sync[token]=sent
        ob.ingest(peer.enc.timesync_encode(remote,token).pack(peer.enc))
        bounds.append(ob.rc.fc_clock[0])
    assert bounds[1]-bounds[0]==pytest.approx(-17782.914)
    assert ob.rc.fc_raw_boot_us==432574896
    assert ob.rc.fc_clock==(bounds[1],now[0])  # no max/clamp of the bound
    topic='input_rc'
    ob.rc.observe_topic(topic,topic_text(topic,now=432574896),now[0])
    assert not ob.rc.topic_timing(topic,now[0])['problems']
    assert 'fc_clock_missing_or_stale' in ob.rc.topic_timing(topic,now[0]+.501)['problems']


def test_actual_raw_clock_regression_still_rejected_when_upper_bound_rises():
    rc=NativeRCMonitor()
    rc.observe_clock(100_010_000,1.,remote_boot_us=100_000_000)
    with pytest.raises(RuntimeError,match='FC clock regressed'):
        rc.observe_clock(100_090_000,1.1,remote_boot_us=99_999_000)
    assert rc.fc_clock==(100_010_000,1.) and rc.fc_raw_boot_us==100_000_000


def test_armed_observation_stops_without_auto_disarm_or_lock():
    peer=Peer(); ob=ground.GroundObserver(peer,lambda row:None)
    armed=peer.enc.heartbeat_encode(2,12,128,3<<16,3).pack(peer.enc)
    with pytest.raises(RuntimeError,match='ARMED'): ob.ingest(armed)
    assert not peer.calls


def test_measured_layout_is_used_only_for_local_observation(tmp_path,monkeypatch):
    import rigid_body_revision
    import ev_delay_vicon_source
    import vicon_receiver_provenance
    now=[100.]; peer=Peer()
    monkeypatch.setattr(ground.time,'monotonic',lambda:now[0])
    monkeypatch.setattr(ground.time,'sleep',lambda dt:now.__setitem__(0,now[0]+dt))
    monkeypatch.setattr(ground,'FixedPeerUDP',lambda *a,**kw:peer)
    layout=dict(origin=[.1,.2,.3],orientation=[0,0,0,1],revision='SYNTHETIC_NEW',
                calibration_approved=False,flight_authorized=False)
    monkeypatch.setattr(rigid_body_revision,'measured_capture_inputs',lambda p:layout)
    monkeypatch.setattr(vicon_receiver_provenance,'verify_current',lambda:{'synthetic':True})
    class Source:
        def __init__(self,*args,latest_only=False):
            assert latest_only is False
            self.report={}; self.on_frame=lambda *a:None
        def poll(self):
            self.on_frame(dict(sdk_ok=True,occluded=False,pose_published=True,queue_dropped=0,
                subject='MircoAir',segment='MircoAir',frame_id='map',ros_stamp_ns=self.now_ros_ns(),
                world_pose=[1,2,3,0,0,0,1]),int(now[0]*1e9))
        def now_ros_ns(self): return int(now[0]*1e9)
        def close(self): self.report['closed']=True
    monkeypatch.setattr(ev_delay_vicon_source,'ViconSource',Source)
    result=ground.run(output=tmp_path/'layout',peer=('192.168.4.1',14550),
        bind=('192.168.4.2',14550),motor_power_isolated=True,observe_seconds=0,
        layout_evidence=tmp_path/'synthetic.json')
    assert result['error'] is None
    assert result['last_pose_observation']['position']==[1.1,-2.2,-3.3]
    assert result['layout']['revision']=='SYNTHETIC_NEW' and result['vicon_receiver']['closed']
    assert not result['last_pose_observation']['sent_to_fc']
    assert not any(msg.get_type()=='VISION_POSITION_ESTIMATE' for msg in peer.calls)
