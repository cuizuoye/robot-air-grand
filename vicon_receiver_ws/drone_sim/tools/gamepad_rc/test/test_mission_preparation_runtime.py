"""Full offline reactor: real MAVLink bytes/parsers, no device or ROS opens."""
from collections import deque
from pathlib import Path
import struct
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mission_preparation_runtime import PreparationRuntime
from mission_audit_collector import COMMANDS, MissionAuditCollector
from mavlink_crc_stream import CrcResyncParser
from trs_uart_readonly_check import UID2
from test_mission_audit_collector import reply_text, deliver
from test_trs_vision_reference import baseline


class Peer:
    """Synthetic FC: encoded streams + echo/prompt-separated fixed topic replies."""
    def __init__(self, *, fusion=False, silent_diagnostics=False):
        self.runtime=PreparationRuntime('offline-revision',body_origin_in_rigid=[0,0,0],
            body_orientation_in_rigid=[0,0,0,1],enable_offline_fusion=fusion)
        self.encoder=m.MAVLink(None,srcSystem=1,srcComponent=1)
        self.decoder=CrcResyncParser()
        self.pending=deque();self.shell=deque();self.cursor=0;self.now=0.
        self.params=baseline();self.locked=False;self.last_pose=None
        self.silent=silent_diagnostics;self.calls=[];self.visual_wait=None

    def pack(self,msg):
        wire=msg.pack(self.encoder);self.encoder.seq=(self.encoder.seq+1)%256
        return wire

    def shell_text(self,text):
        data=text.encode()
        for i in range(0,len(data),70):
            part=data[i:i+70]
            self.shell.append(self.pack(self.encoder.serial_control_encode(10,0,0,0,len(part),part.ljust(70,b'\0'))))

    def inspect_output(self):
        frames=list(self.runtime.frames)
        for row in frames[self.cursor:]:
            msg=self.decoder.parse_buffer(bytes.fromhex(row['wire_hex']))[0]
            self.calls.append((self.now,msg))
            kind=msg.get_type()
            if kind=='COMMAND_LONG' and msg.command==512:
                self.pending.append(self.pack(self.encoder.autopilot_version_encode(0,(1<<24)+(14<<16)+(3<<8),
                    0,0,54,bytes(8),bytes(8),bytes(8),0,0,1,bytes.fromhex(UID2))))
            elif kind=='COMMAND_LONG' and msg.command==185:
                self.locked=True
                self.pending.append(self.pack(self.encoder.command_ack_encode(185,0,target_system=250,target_component=220)))
            elif kind=='PARAM_REQUEST_READ':
                name=msg.param_id.rstrip('\0');value=self.params[name]
                floating=name=='EKF2_EV_DELAY'
                value=float(value) if floating else struct.unpack('<f',struct.pack('<i',int(value)))[0]
                self.pending.append(self.pack(self.encoder.param_value_encode(name.encode(),value,9 if floating else 6,926,0)))
            elif kind=='PARAM_SET':
                name=msg.param_id.rstrip('\0')
                value=msg.param_value if msg.param_type==9 else struct.unpack('<i',struct.pack('<f',msg.param_value))[0]
                self.params[name]=value
                self.pending.append(self.pack(self.encoder.param_value_encode(name.encode(),msg.param_value,msg.param_type,926,0)))
            elif kind=='VISION_POSITION_ESTIMATE':self.last_pose=msg
            elif kind=='SERIAL_CONTROL' and msg.count and not self.silent:
                payload=bytes(msg.data[:msg.count]);key=next(k for k,v in COMMANDS.items() if v==payload)
                assert not self.shell and self.visual_wait is None  # Never overlap NSH commands.
                if key.startswith('visual_series'):self.visual_wait=self.now;self.visual_key=key
                else:
                    text=reply_text(key,self.now)
                    if key=='actuator':text=text.replace('lockdown: True',f'lockdown: {self.locked}')
                    self.shell_text(text)
        self.cursor=len(frames)

    def turn(self, *, home=(), with_pose=True, extra_rx=b'', pad=True):
        self.now=round(self.now+.032,9);boot=int((self.now+10)*1000)
        stream=b''.join(self.pack(msg) for msg in (
            self.encoder.heartbeat_encode(2,m.MAV_AUTOPILOT_PX4,0,4<<16,0),
            self.encoder.attitude_quaternion_encode(boot,1,0,0,0,0,0,0),
            self.encoder.local_position_ned_encode(boot,1,2,-.1,0,0,0),
            self.encoder.extended_sys_state_encode(0,m.MAV_LANDED_STATE_ON_GROUND)))
        if self.visual_wait is not None and self.now-self.visual_wait>=(.384 if self.visual_key=='visual_series_ground' else .352):
            msg=self.last_pose;assert msg
            text='nsh> '+COMMANDS[self.visual_key].decode()
            count=13 if self.visual_key=='visual_series_ground' else 12
            for i in range(count):
                stamp=10_000_000 if count==13 and i==0 else round((self.now+10-(count-1-i)*.032)*1e6)
                text+=('TOPIC: vehicle_visual_odometry\n'
                    f' timestamp: {stamp}\n timestamp_sample: {stamp}\n pose_frame: 1\n'
                    ' velocity_frame: 0\n reset_counter: 0\n quality: 0\n'
                    f' position: [{msg.x}, {msg.y}, {msg.z}]\n q: [1, 0, 0, 0]\n')
            self.shell_text(text+'nsh> ');self.visual_wait=None
        for queue in (self.pending,self.shell):
            while queue and len(stream)+len(queue[0])+len(extra_rx)<=self.runtime.max_rx_bytes:
                stream+=queue.popleft()
        ns=int((100+self.now)*1e9)
        status=self.runtime.turn(self.now,rx=stream+extra_rx,now_ros_ns=ns,
            pose=(ns,'map',[1,-2,.1],[0,0,0,1]) if with_pose else None,
            pad_generation='synthetic-pad',pad_present=pad,pad_event=bool(home),home_events=home)
        self.inspect_output()
        return status

    def home_check(self):
        for _ in range(6):self.turn()
        self.turn(home=(True,));self.turn(home=(False,))
        for _ in range(65):self.turn()
        assert self.runtime.inputs.home_proven


def test_default_is_capture_only_and_has_no_flight_or_hardware_entry():
    p=Peer();p.home_check()
    for _ in range(100):p.turn()
    r=p.runtime
    assert r.status()['physical_tx_bytes']==r.status()['devices_opened']==0
    assert r.coordinator.fusion.phase=='NEW' and not r.prepare_requested
    assert all(msg.get_type() not in ('VISION_POSITION_ESTIMATE','PARAM_SET','SET_POSITION_TARGET_LOCAL_NED') for _,msg in p.calls)
    with pytest.raises(PermissionError):r.request_prepare(operator_confirmed=True)
    assert not hasattr(r,'start') and not hasattr(r,'port')


def test_home_runs_before_invalid_rx_and_latches_during_preparation():
    p=Peer(fusion=True);p.home_check();r=p.runtime
    r.request_prepare(operator_confirmed=True)
    previous=len(r.frames)
    result=r.turn(p.now+.032,rx=b'x'*2049,pad_generation='synthetic-pad',pad_present=True,home_events=(True,))
    first=list(r.frames)[previous]
    msg=CrcResyncParser().parse_buffer(bytes.fromhex(first['wire_hex']))[0]
    assert msg.command==185 and msg.param1==2
    assert result['stop_latched'] and result['failure']
    assert r.coordinator.core.phase=='STOP_LATCHED'
    assert result['captured_counts'].get('VISION_POSITION_ESTIMATE',0)==0


def test_unpublished_aid_reads_are_not_scheduled_before_fusion():
    p=Peer();p.home_check()
    assert not any(msg.get_type()=='SERIAL_CONTROL' and msg.count and
        bytes(msg.data[:msg.count])!=COMMANDS['actuator'] for _,msg in p.calls)


def test_single_writer_sequence_and_identity_are_shared_by_all_producers():
    p=Peer();p.home_check()
    wires=[bytes.fromhex(row['wire_hex']) for row in p.runtime.frames]
    msgs=[CrcResyncParser().parse_buffer(wire)[0] for wire in wires]
    assert all((msg.get_srcSystem(),msg.get_srcComponent())==(250,220) for msg in msgs)
    assert [msg.get_seq() for msg in msgs]==[i%256 for i in range(len(msgs))]
    assert p.runtime.coordinator.core.sink is p.runtime.coordinator.fusion.sink is p.runtime.writer


def test_default_boundary_rejects_every_flight_or_unlock_command():
    p=Peer();p.home_check();r=p.runtime
    for command,first in ((176,1),(400,1),(400,0),(21,0),(22,0),(185,0),(246,1),(511,1)):
        with pytest.raises(PermissionError):
            r.writer(r.encoder.command_long_encode(1,1,command,0,first,0,0,0,0,0,0))


def test_silent_diagnostic_wait_does_not_block_home_and_has_bounded_failure():
    p=Peer(silent_diagnostics=True)
    for _ in range(12):p.turn()
    before=len(p.runtime.frames);p.turn(home=(True,))
    new=list(p.runtime.frames)[before:]
    assert new[0]['type']=='COMMAND_LONG'
    for _ in range(200):p.turn()
    assert p.runtime.coordinator.audit.disabled
    assert p.runtime.status()['failure'] and p.runtime.home_attempts==1


def test_duplicate_and_wrong_frame_poses_do_not_refresh_freshness():
    p=Peer();p.home_check();r=p.runtime
    stamp=r.inputs.pose_stamp;old=r.inputs.pose_at
    r.turn(p.now+.01,now_ros_ns=stamp+10_000_000,pose=(stamp,'map',[1,-2,.1],[0,0,0,1]))
    assert r.inputs.pose_at==old and r.pose_gate.rejected==1
    r.turn(p.now+.02,now_ros_ns=stamp+20_000_000,pose=(stamp+20_000_000,'wrong',[1,-2,.1],[0,0,0,1]))
    assert r.inputs.pose_at==old and r.pose_gate.rejected==2


def test_explicit_preparation_requires_current_home_and_never_unlocks():
    p=Peer(fusion=True)
    with pytest.raises(PermissionError):p.runtime.request_prepare(operator_confirmed=True)
    p.home_check()
    with pytest.raises(PermissionError):p.runtime.request_prepare()
    p.runtime.request_prepare(operator_confirmed=True)
    p.turn()
    assert p.locked and p.runtime.coordinator.fusion.phase=='RECEIVE_WARMUP'
    assert not any(msg.get_type()=='COMMAND_LONG' and msg.command in (176,400) for _,msg in p.calls)


def test_preparation_pipeline_reaches_handoff_and_closes_without_any_flight_commands():
    p=Peer(fusion=True);p.home_check();r=p.runtime;r.request_prepare(operator_confirmed=True)
    for _ in range(400):
        status=p.turn()
        if status['failure'] or r.coordinator.fusion.phase=='HANDED_OFF':break
    assert not status['failure'],status
    assert r.coordinator.fusion.phase=='HANDED_OFF',status
    assert r.coordinator.core.phase=='WAIT_EXPLICIT_START'
    assert len(r.inputs.audit_rows)==3 and p.params['EKF2_EV_CTRL']==11
    poses=r.sent_counts['VISION_POSITION_ESTIMATE']
    for _ in range(8):p.turn()
    assert r.sent_counts['VISION_POSITION_ESTIMATE']>poses
    r.request_close()
    for _ in range(50):
        status=p.turn()
        if status['closed']:break
    assert status['closed'] and not status['fusion_configuration_may_need_restore']
    assert p.params==baseline()
    assert not any(msg.get_type()=='COMMAND_LONG' and msg.command in (176,400,21,22) for _,msg in p.calls)


def test_fast_diagnostic_mode_does_not_abandon_an_existing_reply():
    c=MissionAuditCollector();c.action(0,True);deliver(c,'actuator',.01)
    assert c.action(.02,True)=='selector'
    assert c.action(.03,True,full_audit=False) is None
    assert c.key=='selector' and c.pending_at==.02
    deliver(c,'selector',.04)
    assert c.action(.05,True,full_audit=False)=='actuator'
    assert c.row=={} and not c.take_audits()


def prepared_peer():
    p=Peer(fusion=True);p.home_check();p.runtime.request_prepare(operator_confirmed=True)
    for _ in range(400):
        status=p.turn()
        if status['failure'] or p.runtime.coordinator.fusion.phase=='HANDED_OFF':break
    assert not status['failure'] and p.runtime.coordinator.fusion.phase=='HANDED_OFF',status
    return p


def test_home_after_handoff_stops_pose_stream_and_cannot_reuse_old_home_ack():
    p=prepared_peer();r=p.runtime
    old_poses=r.sent_counts['VISION_POSITION_ESTIMATE'];before=len(p.calls)
    p.turn(home=(True,));p.turn(home=(False,))
    for _ in range(8):p.turn()
    stops=[msg for _,msg in p.calls[before:] if msg.get_type()=='COMMAND_LONG' and msg.command==185]
    assert len(stops)==1  # New stop despite the earlier accepted preflight HOME.
    assert r.stop_latched and r.coordinator.core.stop_latched
    assert r.coordinator.core.phase=='STOP_LATCHED'
    assert r.sent_counts['VISION_POSITION_ESTIMATE']==old_poses
    with pytest.raises(PermissionError):r.request_prepare(operator_confirmed=True)
    r.request_close()
    for _ in range(12):p.turn()
    assert r.closed and r.stop_latched and r.coordinator.core.phase=='STOP_LATCHED'
    assert p.params==baseline()


def test_pose_loss_blocks_preparation_and_source_recovery_never_auto_resumes():
    p=prepared_peer();r=p.runtime
    for _ in range(8):p.turn(with_pose=False)
    assert r.failure and r.coordinator.core.phase=='BLOCKED'
    count=r.sent_counts['VISION_POSITION_ESTIMATE']
    for _ in range(8):p.turn()
    assert r.sent_counts['VISION_POSITION_ESTIMATE']==count
    # Explicit ground restoration may proceed despite absent Vicon.
    r.request_close()
    for _ in range(12):p.turn(with_pose=False)
    assert r.closed and p.params==baseline()


def test_owner_long_turn_gap_blocks_before_next_pose_and_can_only_restore():
    p=prepared_peer();r=p.runtime;count=r.sent_counts['VISION_POSITION_ESTIMATE']
    p.now+=.2;status=p.turn()
    assert status['failure']=='owner_turn_gap'
    assert r.sent_counts['VISION_POSITION_ESTIMATE']==count
    assert not status['flight_authorized']


def test_gamepad_disconnect_or_generation_change_never_reuses_old_home_proof():
    p=prepared_peer();status=p.turn(pad=False)
    assert status['failure']=='gamepad_disconnected'
    assert not status['mission_inputs']['observation_complete']
    with pytest.raises(PermissionError):p.runtime.request_prepare(operator_confirmed=True)


def test_crc_damage_does_not_update_local_sample_and_never_reconstructs_bytes():
    p=Peer();p.home_check();r=p.runtime
    old_stamp=r.inputs.last_boot_by_kind['LOCAL_POSITION_NED']
    msg=p.encoder.local_position_ned_encode(999999,99,0,0,0,0,0)
    bad=bytearray(p.pack(msg));bad[-1]^=1
    r.turn(p.now+.01,rx=bytes(bad))
    assert r.inputs.last_boot_by_kind['LOCAL_POSITION_NED']==old_stamp
    assert r.inputs.bad_frames>0 and r.parser.summary()['repaired_bytes']==0


def test_preflight_home_press_does_not_clear_physical_lock_or_auto_request_prepare():
    p=Peer(fusion=True);p.home_check()
    for _ in range(20):p.turn()
    assert p.locked and not p.runtime.prepare_requested
    assert p.runtime.coordinator.fusion.phase=='NEW'
    assert p.runtime.coordinator.core.phase=='PREPARE'


@pytest.mark.parametrize('field',['origin','orientation'])
def test_invalid_or_unknown_extrinsics_cannot_be_silently_adopted(field):
    args=dict(body_origin_in_rigid=[0,0,0],body_orientation_in_rigid=[0,0,0,1])
    args['body_origin_in_rigid' if field=='origin' else 'body_orientation_in_rigid']=None
    with pytest.raises((ValueError,TypeError)):
        PreparationRuntime('r',**args)


def test_unknown_state_after_reboot_cannot_trigger_parameter_rollback():
    p=prepared_peer();r=p.runtime
    msg=p.encoder.attitude_quaternion_encode(1,1,0,0,0,0,0,0)
    r.turn(p.now+.01,rx=p.pack(msg))
    assert r.inputs.fault=='fc_boot_clock_regressed_or_out_of_order'
    before=r.sent_counts['PARAM_SET']
    with pytest.raises(PermissionError):r.request_close()
    assert r.sent_counts['PARAM_SET']==before


def test_preparation_cannot_claim_landing_or_clear_global_holds_by_elapsed_time():
    p=Peer();p.home_check();r=p.runtime
    assert r.status()['coordinator']['core_phase']=='PREPARE'
    assert not r.status()['flight_authorized']
    assert r.status()['devices_opened']==0
    assert not hasattr(r,'clear_hold') and not hasattr(r,'unlock')


def test_bad_clock_does_not_forget_home_and_next_valid_turn_still_captures_stop():
    p=prepared_peer();r=p.runtime;before=len(r.frames)
    with pytest.raises(ValueError,match='clock'):
        r.turn(float('nan'),pad_generation='synthetic-pad',pad_present=True,home_events=(True,))
    assert r.stop_latched and r.home_check_requested and not r.home_acknowledged
    assert len(r.frames)==before  # No freshness proof from an invalid clock.
    p.turn(home=(False,))
    decoded=CrcResyncParser().parse_buffer(bytes.fromhex(list(r.frames)[before]['wire_hex']))[0]
    assert decoded.command==185 and decoded.param1==2
