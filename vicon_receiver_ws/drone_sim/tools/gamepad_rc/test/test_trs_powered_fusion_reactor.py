"""Real runner and MAVLink bytes; synthetic ROS/radio/input, NO real devices.

Internal field parsers are stubbed here and exercised by separate parser
tests. This harness checks runner wiring, send gates, phases and rollback.
"""
import json
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import trs_powered_fusion_bench as p
from test_trs_vision_reference import baseline


@pytest.fixture
def reactor(tmp_path,monkeypatch):
    def run(scenario='healthy',baud=57600,unified=False,receive_rate_only=False,compact_ground_fusion=False,home_wait_seconds=90.,short_status=False,binary_diagnostics=False,low_bandwidth_vision=False,frozen_target_stream=False,fixed_document=None,fixed_live=False):
        fixed=fixed_document is not None
        if fixed:
            assert baud==115200
            unified=compact_ground_fusion=short_status=binary_diagnostics=True
            # Only this synthetic harness removes the production release hold.
            if not fixed_live:monkeypatch.setattr(p,'FIXED_GROUND_LIVE_ENABLED',True)
            else:low_bandwidth_vision=True
        clock=NS(value=0.0);sent=[];epoch=1_800_000_000.
        fc=m.MAVLink(None,srcSystem=1,srcComponent=1);decoder=m.MAVLink(None)
        initial_values=baseline()
        if fixed:initial_values={n:s['value'] for n,s in fixed_document['parameters'].items()}
        if low_bandwidth_vision:
            from low_bandwidth_vision import f32
            initial_values.update(EKF2_EVP_NOISE=f32(.1),EKF2_EVA_NOISE=f32(.1))
        values=dict(initial_values);values['COM_OF_LOSS_T']=1
        if fixed and scenario=='fixed_ctrl_mismatch':values['EKF2_EV_CTRL']=2
        if fixed and scenario=='fixed_delay_mismatch':values['EKF2_EV_DELAY']=65.
        class Radio:
            def __init__(self):
                self.queue=bytearray();self.closed=False;self.last_pose=None
                self.fused_at=None;self.next_hb=self.next_attitude=0;self.lost=False
                self.corrupted=False
                self.rollback_started=False;self.pending_prompt=False;self.diagnostic_commands=[]
                self.pending_rate=None
                self.short_rate=0;self.next_short=0
                self.binary_rate=0;self.next_binary=0;self.binary_seq=0
                self.last_pose_received=None;self.drop_binary_restore=False
                self.clock_replies_dropped=0
                self.local_rate=0;self.next_local=0;self.first_target=None;self.drop_local_restore=False
                self.restore_queries=0
                self.fixed_frozen=False
            def __enter__(self):return self
            def __exit__(self,*args):self.closed=True
            def reset_input_buffer(self):self.queue.clear()
            @property
            def in_waiting(self):return max(len(self.queue),1)
            def append(self,msg):
                self.queue.extend(msg.pack(fc));fc.seq=(fc.seq+1)%256
            def read(self,size):
                clock.value+=.005;t=clock.value
                if self.pending_rate and t>=self.pending_rate[0]:
                    text=self.pending_rate[1];self.pending_rate=None
                    for index in range(0,len(text),70):
                        part=text[index:index+70]
                        self.append(fc.serial_control_encode(10,0,0,0,len(part),part.ljust(70,b'\0')))
                if self.last_pose is not None and scenario=='receive_radio_lost':self.lost=True
                if self.fused_at is not None and t-self.fused_at>.2 and scenario=='radio_lost':self.lost=True
                if self.lost:self.queue.clear();return b''
                boot=int((100+t)*1000)
                if self.local_rate and t>=self.next_local and scenario!='target_local_missing':
                    position=[.34,-.27,-.08]
                    if self.first_target is not None and t-self.first_target>.5:
                        if scenario=='target_local_lost':self.local_rate=0
                        if scenario=='target_moved':position[0]+=.1
                        if scenario=='target_position_nan':position[0]=float('nan')
                    local_boot=boot-500 if scenario=='target_local_stale' else boot-20
                    self.append(fc.local_position_ned_encode(local_boot,*position,0,0,0))
                    self.next_local=t+.05
                if self.short_rate and t>=self.next_short:
                    bits=0x4c530107 if scenario=='short_unsafe' and self.fused_at is not None else 0x4c530105
                    lock_stamp=int((100+t//.5*.5)*1000)-20 if frozen_target_stream else boot-50
                    self.append(fc.named_value_int_encode(lock_stamp,b'OUTLOCK1',bits))
                    self.next_short=t+.2
                if self.binary_rate and t>=self.next_binary and self.last_pose is not None:
                    from fusion_binary_protocol import encode_visual,encode_fusion,message
                    from fusion_binary_offline import synthetic_row
                    stamp=int((100+t-.02)*1e6)
                    if scenario!='binary_missing' and not (scenario=='binary_stream_lost' and self.fused_at is not None and t-self.fused_at>.5):
                        pose=self.last_pose
                        visual=dict(timestamp=self.last_pose_received,timestamp_sample=self.last_pose_received,
                            pose_frame=1,velocity_frame=0,position=[pose.x,pose.y,pose.z],q=[1.,0.,0.,0.],reset_counter=0,quality=0)
                        record=synthetic_row(stamp)
                        record['flags']['timestamp']=int((100+int(t))*1e6)
                        record['flags']['timestamp_sample']=record['flags']['timestamp']-100
                        record['selector']['instance_changed_count']=1
                        if self.fused_at is not None and t-self.fused_at>4:
                            if scenario=='binary_selector_change' and (not fixed or self.fixed_frozen):
                                record['selector']['instance_changed_count']+=2
                            if scenario=='binary_position_reset':record['local']['xy_reset_counter']=1
                        if self.first_target is not None and t-self.first_target>.3 and scenario=='target_reset':
                            record['local']['xy_reset_counter']=1
                        for key in ('aid','height_aid','yaw_aid'):
                            record[key]['fusion_enabled']=values['EKF2_EV_CTRL']==11
                            record[key]['fused']=values['EKF2_EV_CTRL']==11
                        inactive_inf=scenario=='binary_inactive_inf' and values['EKF2_EV_CTRL']!=11
                        if inactive_inf:
                            for key,flag in (('aid','cs_ev_pos'),('yaw_aid','cs_ev_yaw')):
                                record['flags'][flag]=False
                                record[key].update(fusion_enabled=False,fused=False,innovation_rejected=True)
                        fusion_payload=encode_fusion(record,self.binary_seq)
                        if inactive_inf or scenario=='binary_activated_inf' and self.fused_at is not None:
                            import fusion_binary_record as layout
                            fusion_payload=bytearray(fusion_payload)
                            offset=layout.HEAD.size
                            fields=list(layout.AID.unpack_from(fusion_payload,offset));fields[5]=float('inf')
                            layout.AID.pack_into(fusion_payload,offset,*fields)
                            fusion_payload=bytes(fusion_payload)
                        for payload in (encode_visual(visual,self.binary_seq),fusion_payload):
                            reply=message(fc,payload)
                            if scenario=='binary_bad_crc' or scenario=='binary_one_crc' and not self.corrupted:
                                broken=bytearray(reply.pack(fc));broken[-1]^=1
                                fc.seq=(fc.seq+1)%256;self.queue.extend(broken);self.corrupted=True
                            else:self.append(reply)
                    self.binary_seq+=1;self.next_binary=t+self.binary_rate/1e6
                if self.last_pose is not None and scenario=='receive_fc_reboot':boot=int(t*1000)
                if self.fused_at is not None and scenario=='fc_reboot':boot=int(t*1000)
                if t>=self.next_hb and not (scenario=='heartbeat_lost' and t>2):
                    mode=(4<<16)|(3<<24) if frozen_target_stream else 0
                    if self.first_target is not None and t-self.first_target>.3 and scenario=='target_mode_changed':mode=3<<16
                    self.append(fc.heartbeat_encode(m.MAV_TYPE_QUADROTOR,m.MAV_AUTOPILOT_PX4,0,mode,0));self.next_hb=t+1
                    if unified:self.append(fc.battery_status_encode(0,0,0,2500,[12000]+[65535]*9,-1,-1,-1,80))
                if t>=self.next_attitude:
                    self.append(fc.attitude_quaternion_encode(boot-20 if frozen_target_stream else boot,1,0,0,0,0,0,0));self.next_attitude=t+.05
                chunk=bytes(self.queue[:max(size,4096)]);del self.queue[:max(size,4096)];return chunk
            def param(self,name):
                value=values[name];is_float=name=='EKF2_EV_DELAY' or isinstance(value,float)
                if fixed:is_float=p.fixed_profile.PARAMETER_TYPES[name]==9
                wire=float(value) if is_float else struct.unpack('<f',struct.pack('<i',int(value)))[0]
                self.append(fc.param_value_encode(name.encode(),wire,9 if is_float else 6,926,0))
            def write(self,wire):
                for msg in decoder.parse_buffer(wire) or []:
                    sent.append((clock.value,msg))
                    if self.lost:continue
                    kind=msg.get_type()
                    if kind=='TIMESYNC':
                        if (scenario=='binary_two_clock_replies_lost' and self.last_pose is not None
                                and self.clock_replies_dropped<2):
                            self.clock_replies_dropped+=1
                            continue
                        if not (scenario=='short_clock_lost' and self.fused_at is not None):
                            self.append(fc.timesync_encode(int((100+clock.value)*1e9),msg.ts1))
                    elif kind=='COMMAND_LONG' and msg.command==510:
                        if not (msg.param1==248 and self.drop_binary_restore or msg.param1==32 and self.drop_local_restore):
                            self.append(fc.message_interval_encode(int(msg.param1),int(self.binary_rate if msg.param1==248 else self.local_rate if msg.param1==32 else self.short_rate)))
                    elif kind=='COMMAND_LONG' and msg.command==511:
                        if msg.param1==248:
                            self.binary_rate=max(0,int(msg.param2))
                            if scenario=='binary_set_partial' and msg.param2>0 and not self.corrupted:
                                self.corrupted=True;raise OSError('Partial write after rate may have applied')
                            if scenario=='binary_restore_echo_missing' and msg.param2<0:self.drop_binary_restore=True
                        elif msg.param1==32:
                            self.local_rate=max(0,int(msg.param2))
                            if scenario=='target_rate_partial' and msg.param2>0:raise OSError('Partial local rate write')
                            if scenario=='target_restore_missing' and msg.param2<0:self.drop_local_restore=True
                        else:self.short_rate=max(0,int(msg.param2))
                        self.append(fc.command_ack_encode(511,0,target_system=250,target_component=213))
                    elif kind=='COMMAND_LONG' and msg.command==512:
                        self.append(fc.autopilot_version_encode(0,(1<<24)+(14<<16)+(3<<8),
                            0,0,54,bytes(8),bytes(8),bytes(8),0,0,1,bytes.fromhex(p.UID2)))
                    elif kind=='PARAM_REQUEST_READ':
                        name=str(msg.param_id)
                        if fixed and scenario=='fixed_parameter_timeout' and name=='EKF2_EV_DELAY':continue
                        if scenario=='restore_first_read_lost' and self.rollback_started and name=='EKF2_EV_CTRL':
                            self.restore_queries+=1
                            if self.restore_queries==1:continue
                        self.param(name)
                    elif kind=='PARAM_SET':
                        assert not fixed,'Fixed-mode normal/error/finally path emitted PARAM_SET'
                        name=str(msg.param_id);value=p.decode_parameter(msg)
                        if self.fused_at is not None and value in (0,2):self.rollback_started=True
                        if scenario=='restore_echo_missing' and self.fused_at is not None and value in (0,2):
                            continue
                        values[name]=value
                        if not (scenario=='restore_first_read_lost' and self.rollback_started and name=='EKF2_EV_CTRL'):
                            self.param(name)
                        if name=='EKF2_EV_CTRL' and value==11:self.fused_at=clock.value
                    elif kind=='VISION_POSITION_ESTIMATE':
                        self.last_pose=msg;self.last_pose_received=int((100+clock.value)*1e6)
                        if fixed and self.fused_at is None:self.fused_at=clock.value
                    elif kind=='SET_POSITION_TARGET_LOCAL_NED':
                        if self.first_target is None:self.first_target=clock.value
                        if scenario=='target_write_partial':raise OSError('Partial target write')
                    elif kind=='COMMAND_LONG' and msg.command==185:
                        self.append(fc.command_ack_encode(185,0,target_system=250,target_component=213))
                    elif kind=='SERIAL_CONTROL' and msg.count:
                        command=bytes(msg.data[:msg.count])
                        self.diagnostic_commands.append((clock.value,command))
                        assert command in p.policy.COMMANDS.values() or unified and command in (
                            b'listener vehicle_visual_odometry -n 13\n',p.compact.RATE_COMMAND)
                        text=command+b'SYNTHETIC\n\nnsh> '
                        if command==p.compact.RATE_COMMAND:
                            text=command+b'TOPIC NAME INST #SUB RATE #Q SIZE\nvehicle_visual_odometry 0 1 32 1 196\n\nnsh> '
                            if receive_rate_only:
                                self.pending_rate=(clock.value+2.,text)
                                continue
                        for index in range(0,len(text),70):
                            part=text[index:index+70]
                            reply=fc.serial_control_encode(10,0,0,0,len(part),part.ljust(70,b'\0'))
                            if scenario=='truncated_diagnostic' and not self.corrupted:
                                # A truncated real SERIAL_CONTROL frame whose declared length overlaps
                                # a complete following heartbeat. Never fabricate a replacement reply.
                                damaged=reply.pack(fc);fc.seq=(fc.seq+1)%256
                                self.queue.extend(damaged[:15]);self.corrupted=True
                                self.append(fc.heartbeat_encode(m.MAV_TYPE_QUADROTOR,
                                    m.MAV_AUTOPILOT_PX4,0,0x31415926,0))
                                self.pending_prompt=True
                            elif self.fused_at is not None and scenario=='damaged_diagnostic' and not self.corrupted:
                                damaged=bytearray(reply.pack(fc));damaged[-1]^=1
                                fc.seq=(fc.seq+1)%256;self.queue.extend(damaged);self.corrupted=True
                            elif scenario=='visual_clean_retry' and command==b'listener vehicle_visual_odometry -n 1\n' and not self.corrupted:
                                damaged=bytearray(reply.pack(fc));damaged[-1]^=1
                                fc.seq=(fc.seq+1)%256;self.queue.extend(damaged);self.corrupted=True
                                self.pending_prompt=True
                            elif scenario in ('diagnostic_clean_retry','home_and_bad_packet','diagnostic_repeated_bad') and (
                                    not self.corrupted or scenario=='diagnostic_repeated_bad'):
                                damaged=bytearray(reply.pack(fc));damaged[-1]^=1
                                fc.seq=(fc.seq+1)%256;self.queue.extend(damaged);self.corrupted=True
                                self.pending_prompt=True
                            else:self.append(reply)
                    elif kind=='SERIAL_CONTROL' and not msg.count and self.pending_prompt:
                        part=b'late discarded fields\n\nnsh> '
                        self.append(fc.serial_control_encode(10,0,0,0,len(part),part.ljust(70,b'\0')))
                        self.pending_prompt=False
                return len(wire)
        port=Radio();home_sent=False;home_released=False;abort_sent=False
        if fixed:
            original_freeze=p.FusionBinaryReceiver.freeze
            def freeze(receiver,*args,**kwargs):
                result=original_freeze(receiver,*args,**kwargs)
                port.fixed_frozen=True
                return result
            monkeypatch.setattr(p.FusionBinaryReceiver,'freeze',freeze)
            def forbid_old_encoder(*args,**kwargs):
                pytest.fail('Fixed mode called legacy EKF PARAM_SET encoder')
            monkeypatch.setattr(p.vision,'parameter_message',forbid_old_encoder)
            if scenario=='fixed_rogue_cleanup':
                original_restore=p.ShortStatusTransport.restore
                def restore(transport,send,pump,clock_fn):
                    if transport.message_id==248:
                        send(m.MAVLink(None).param_set_encode(1,1,b'EKF2_EV_DELAY',0.,9))
                    return original_restore(transport,send,pump,clock_fn)
                monkeypatch.setattr(p.ShortStatusTransport,'restore',restore)
        def read_input(*args):
            nonlocal home_sent,home_released,abort_sent
            if scenario=='one_home_pad_lost_before_press' and clock.value>1:
                raise OSError(2,'Input device disappeared before HOME')
            if unified:
                threshold=.005 if scenario=='one_home_early' else 30 if scenario=='one_home_delayed' else 1000 if scenario=='no_home' else 2
                if not home_sent and clock.value>threshold:
                    home_sent=True;return p.EVENT.pack(0,0,1,p.HOME,1)
                if home_sent and not home_released and clock.value>threshold+.2:
                    home_released=True;return p.EVENT.pack(0,0,1,p.HOME,0)
                if scenario=='one_home_abort' and port.fused_at is not None and not abort_sent:
                    abort_sent=True;return p.EVENT.pack(0,0,1,p.HOME,1)
                if port.first_target is not None and clock.value-port.first_target>.3:
                    if scenario=='target_home' and not abort_sent:
                        abort_sent=True;return p.EVENT.pack(0,0,1,p.HOME,1)
                    if scenario=='target_pad_lost':raise OSError(19,'No such device')
            if scenario=='home_and_bad_packet' and port.corrupted and not home_sent:
                home_sent=True;return p.EVENT.pack(0,0,1,p.HOME,1)
            if scenario=='home_during_restore' and port.rollback_started and not home_sent:
                home_sent=True;return p.EVENT.pack(0,0,1,p.HOME,1)
            if scenario=='pad_lost_during_restore' and port.rollback_started:raise OSError(19,'No such device')
            if scenario=='receive_pad_lost' and port.last_pose is not None:raise OSError(19,'No such device')
            if scenario=='home_watch' and port.rollback_started and clock.value>15 and not home_sent:
                home_sent=True;return p.EVENT.pack(0,0,1,p.HOME,1)
            if port.fused_at is not None:
                if scenario=='pad_lost':raise OSError(19,'No such device')
                if scenario=='home' and not home_sent:
                    home_sent=True;return p.EVENT.pack(0,0,1,p.HOME,1)
            raise BlockingIOError
        class Node:
            def __init__(self,*args):pass
            def create_subscription(self,_type,_topic,callback,_qos):self.callback=callback;return object()
            def get_clock(self):return NS(now=lambda:NS(nanoseconds=int((epoch+clock.value)*1e9)))
            def destroy_node(self):pass
        def spin(node,**kwargs):
            if scenario=='source_lost' and port.fused_at is not None:return
            if scenario=='receive_source_lost' and port.last_pose is not None:return
            stamp=int((epoch+clock.value)*1e9)
            node.callback(NS(header=NS(frame_id='map',stamp=NS(sec=stamp//10**9,nanosec=stamp%10**9)),
                pose=NS(position=NS(x=1.,y=2.,z=.05),orientation=NS(x=0.,y=0.,z=0.,w=1.))))
        ros=NS(init=lambda **kwargs:None,ok=lambda:True,shutdown=lambda:None,spin_once=spin)
        for name,module in {'rclpy':ros,'rclpy.node':NS(Node=Node),
            'rclpy.qos':NS(QoSProfile=lambda **kw:None,ReliabilityPolicy=NS(BEST_EFFORT=1),DurabilityPolicy=NS(VOLATILE=1)),
            'geometry_msgs.msg':NS(PoseStamped=object)}.items():monkeypatch.setitem(sys.modules,name,module)
        monkeypatch.setattr(p,'time',NS(monotonic=lambda:clock.value,monotonic_ns=lambda:int((epoch+clock.value)*1e9),
            time=lambda:epoch+clock.value,sleep=lambda duration:setattr(clock,'value',clock.value+duration)))
        def serial_open(path,speed,**kwargs):
            assert path=='/fake/TRS' and speed==baud
            return port
        monkeypatch.setattr(p,'serial',NS(Serial=serial_open))
        monkeypatch.setattr(p,'os',NS(O_RDONLY=0,O_NONBLOCK=0,open=lambda *args:999,close=lambda fd:None,
            read=read_input,stat=lambda path:NS(st_rdev=1),fstat=lambda fd:NS(st_rdev=1)))
        monkeypatch.setattr(p,'fcntl',NS(ioctl=lambda *args:bytes(96)))
        monkeypatch.setattr(p,'subprocess',NS(run=lambda *args,**kw:NS(returncode=1)))
        monkeypatch.setattr(p,'FC_USB',tmp_path/'no-usb')
        monkeypatch.setattr(p,'find_pad',lambda:{'path':'/fake/pad'})
        monkeypatch.setattr(p,'gamepad_identity',lambda path:{'synthetic':1})
        monkeypatch.setattr(p,'require_gamepad_identity',lambda *args:None)
        monkeypatch.setattr(p,'load_backup',lambda *args,**kwargs:initial_values|{'COM_OF_LOSS_T':1})
        if fixed:
            monkeypatch.setattr(p,'load_fixed_backup',lambda *args,**kwargs:dict(initial_values))
        def consume(*args,**kwargs):
            assert kwargs['baud']==baud
            return {'reference_boot_s':100,'reference_utc_unix':epoch,'trs_baud':baud,
                'receive_rate_only':receive_rate_only,'compact_ground_fusion':compact_ground_fusion,
                'short_status':short_status,
                'binary_diagnostics':binary_diagnostics,
                'low_bandwidth_vision':low_bandwidth_vision,
                'frozen_target_stream':frozen_target_stream,
                'scope':p.one_home_policy.FIXED_SCOPE if fixed and scenario!='fixed_legacy_permit' else p.one_home_policy.FROZEN_TARGET_SCOPE if frozen_target_stream else p.one_home_policy.SCOPE,
                'profile_sha256':('f'*64 if scenario=='fixed_wrong_profile_permit' else p.fixed_profile.fingerprint(fixed_document)) if fixed else None,
                '_seal':NS(fresh=lambda:True,start=lambda:None,close=lambda:None) if short_status else None}
        monkeypatch.setattr(p.permit_policy,'consume',consume)
        monkeypatch.setattr(p.one_home_policy,'consume',consume)
        ready=lambda text:'SYNTHETIC' in text
        stamp=lambda:int((100+clock.value)*1e6)
        def arm(text):
            return dict(timestamp_us=stamp()-(800_000 if scenario=='aged_lock_reply' else 400_000 if scenario=='moderately_aged_lock_reply' else 0),armed=False,
                lockdown=True,manual_lockdown=False,force_failsafe=False) if ready(text) else None
        def selector(text):
            return dict(timestamp=stamp(),primary_instance=0,instance_changed_count=1,instances_available=1,
                healthy=[True]+[False]*8,gyro_fault_detected=False,accel_fault_detected=False) if ready(text) else None
        def visual(text,samples=12):
            if not ready(text) or not port.last_pose:return None
            pose=port.last_pose
            period=70000 if scenario=='low_visual_rate' else 31250
            rows=[dict(timestamp=stamp()-(11-i)*period,timestamp_sample=stamp()-(11-i)*period-1,
                pose_frame=1,reset_counter=0,position=[pose.x,pose.y,pose.z],
                q=port.last_pose and [1,0,0,0]) for i in range(12)]
            return [dict(rows[0],timestamp=1_000_000),*rows] if samples==13 else rows
        def aid(text):
            return dict(timestamp=stamp(),timestamp_sample=stamp()-1,estimator_instance=0,time_last_fuse=stamp()-1,
                fusion_enabled=values['EKF2_EV_CTRL']==11,fused=True,innovation_rejected=False,
                observation_variance=.1,test_ratio=.1) if ready(text) else None
        def flags(text):
            if not ready(text):return None
            result=dict.fromkeys(('cs_gps','cs_mag_hdg','cs_mag_3d','cs_gps_yaw','cs_ev_vel','cs_in_air',
                'fs_bad_pos_n','fs_bad_pos_e','fs_bad_pos_d','fs_bad_hdg','reject_hor_pos','reject_ver_pos','reject_yaw'),False)
            result.update(timestamp=stamp(),cs_tilt_align=True,cs_baro_hgt=True,cs_ev_pos=True,cs_ev_hgt=True,cs_ev_yaw=True)
            return result
        monkeypatch.setattr(p,'parse_actuator_snapshot',arm);monkeypatch.setattr(p,'parse_selector',selector)
        monkeypatch.setattr(p,'parse_visual_snapshot',lambda text:visual(text)[-1] if ready(text) and port.last_pose else None)
        monkeypatch.setattr(p.compact,'visual_series',visual)
        for name in ('parse_aid','parse_height_aid','parse_yaw_aid'):monkeypatch.setattr(p.combined,name,aid)
        monkeypatch.setattr(p.combined,'parse_flags',flags)
        monkeypatch.setattr(p.combined,'parse_local_position',lambda text:dict(timestamp=stamp(),xy_valid=True,z_valid=True) if ready(text) else None)
        heading,board=tmp_path/'heading.json',tmp_path/'board.json'
        heading.write_text(json.dumps({'body_orientation_in_rigid_xyzw_candidate':[0,0,0,1]}))
        board.write_text(json.dumps({'board_center_in_rigid_m_candidate':[0,0,0]}))
        args=NS(one_home_preflight=unified,receive_rate_only=receive_rate_only,compact_ground_fusion=compact_ground_fusion,short_status=short_status,binary_diagnostics=binary_diagnostics,low_bandwidth_vision=low_bandwidth_vision,frozen_target_stream=frozen_target_stream,home_wait_seconds=home_wait_seconds,powered_locked_static_confirmed=True,trs_port='/fake/TRS',baud=baud,output=tmp_path/'result.json',
            permit=tmp_path/'fake-permit',backup=tmp_path/'fake-backup',heading=heading,board=board,
            home_watch_seconds=20 if scenario in ('home_watch','watch_healthy','pad_lost_during_restore') else 0)
        if fixed:
            path=tmp_path/'SYNTHETIC-fixed-profile.json'
            path.write_text(json.dumps(fixed_document));args.fixed_ekf_profile=path
        if fixed_live:
            # Real permit consumer/dispatch, fake evidence and hardware only.
            import fixed_ground_entry as entry
            monkeypatch.setattr(entry,'time',p.time)
            monkeypatch.setattr(entry,'inspect',lambda **kw:(fixed_document,initial_values,{},'SYNTHETIC'))
            monkeypatch.setattr(entry,'evidence_paths',lambda **kw:{})
            monkeypatch.setattr(entry,'require_no_pending_recovery',lambda *args:None)
            args.output=tmp_path/'report.json'
            record=entry.draft(run_dir=args.output.parent,trs_port=args.trs_port,
                profile=path,backup=args.backup,heading=heading,board=board,
                proof=tmp_path/'proof.json',usb_proof=tmp_path/'usb.json')
            record.update(user_authorized=True,physical_confirmed=True,
                physical_confirmations=dict(entry.PHYSICAL),issued_unix=epoch,
                expires_unix=epoch+(6 if scenario=='permit_expires' else 600))
            args.permit=tmp_path/'SYNTHETIC-live-permit.json'
            args.permit.write_text(json.dumps(record));args.fixed_ground_live=True
        code=p.run(args);report=json.loads(args.output.read_text())
        assert report['trs_baud']==baud
        assert report['rx_parser']['implementation']=='strict_crc_bytewise_resynchronization'
        assert port.closed
        assert report['arming_output_unlock_mode_or_motion_commands_sent'] is bool(report['frozen_target_write_attempts'])
        assert report['arming_output_unlock_mode_or_moving_targets_sent'] is False
        assert not report['flight_authorized']
        assert all(msg.command in ((512,185,510,511) if short_status else (512,185)) for _,msg in sent if msg.get_type()=='COMMAND_LONG')
        assert all(msg.param1==2 for _,msg in sent if msg.get_type()=='COMMAND_LONG' and msg.command==185)
        return code,report,sent
    return run


def test_short_status_real_runner_synthetic_transport_and_restoration(reactor):
    code,report,sent=reactor(unified=True,compact_ground_fusion=True,short_status=True)
    assert code==0,report['error'] or report['restore_errors']
    assert report['short_status_transport']['restoration_verified']
    assert report['telemetry_rate_writes_sent']==2
    assert report['tx_messages']['TIMESYNC']>0
    assert report['tx_messages']['VISION_POSITION_ESTIMATE']>0
    assert report['startup_home']['short_post_home_remote_barrier_ns'] is not None
    snapshot=report['configuration_snapshot']
    assert snapshot['started_at'] is not None and snapshot['maximum_seconds']==20
    # Once streaming starts only explicit EV write/readback transactions remain.
    assert all(str(msg.param_id) in ('EKF2_EV_CTRL','EKF2_EV_DELAY')
               for at,msg in sent if at>=snapshot['started_at']
               and msg.get_type()=='PARAM_REQUEST_READ')
    assert not any(msg.get_type()=='SERIAL_CONTROL' and b'actuator_armed' in bytes(msg.data)
                   for _,msg in sent)


@pytest.mark.parametrize('scenario',['short_unsafe','short_clock_lost','one_home_abort'])
def test_short_status_runner_failure_stops_and_restores_rate(reactor,scenario):
    code,report,sent=reactor(scenario=scenario,unified=True,compact_ground_fusion=True,short_status=True)
    assert code==2
    assert report['error']
    assert report['short_status_transport']['restoration_verified']
    assert report['telemetry_rate_writes_sent']==2
    assert report['flight_authorized'] is False


@pytest.mark.parametrize('scenario',['healthy','binary_one_crc','binary_inactive_inf','binary_two_clock_replies_lost'])
def test_binary_runner_no_text_and_both_rates_restored(reactor,scenario):
    code,r,sent=reactor(scenario,baud=115200,unified=True,compact_ground_fusion=True,short_status=True,binary_diagnostics=True)
    assert code==0,r['error'] or r['restore_errors']
    assert r['compact_ground_fusion_verified'] and len(r['fusion_audits'])==3
    assert r['all_rate_transactions']['restoration_verified']
    assert set(r['all_rate_transactions']['original_intervals_us'])=={'248','252'}
    assert r['telemetry_rate_writes_sent']==4
    assert not any(msg.get_type()=='SERIAL_CONTROL' for _,msg in sent)
    assert r['binary_receiver_fault'] is None
    if scenario=='binary_inactive_inf':assert r['binary_not_ready_records']>0
    if scenario=='binary_two_clock_replies_lost':
        assert r['short_clock_probe']['counts']['expired_unanswered_request']==2


@pytest.mark.parametrize('scenario',['binary_missing','binary_bad_crc','binary_stream_lost',
    'binary_selector_change','binary_position_reset','binary_set_partial','short_clock_lost','one_home_abort','binary_activated_inf'])
def test_binary_runner_fault_stops_without_text_fallback(reactor,scenario):
    code,r,sent=reactor(scenario,baud=115200,unified=True,compact_ground_fusion=True,short_status=True,binary_diagnostics=True)
    assert code==2 and not r['compact_ground_fusion_verified']
    assert r['all_rate_transactions']['restoration_verified'],r['restore_errors']
    assert not any(msg.get_type()=='SERIAL_CONTROL' for _,msg in sent)
    if scenario=='binary_selector_change':assert 'Primary estimator' in r['error']
    if scenario=='binary_position_reset':assert 'local_position_reset' in r['error']
    if scenario=='binary_set_partial':assert 'Partial write' in r['error']
    if scenario=='binary_activated_inf':assert 'Invalid aid evidence' in r['error']
    if scenario in ('binary_missing','binary_bad_crc','binary_set_partial'):
        assert not any(msg.get_type()=='PARAM_SET' for _,msg in sent)


def test_binary_restore_missing_echo_keeps_durable_obligation(reactor,tmp_path):
    code,r,sent=reactor('binary_restore_echo_missing',baud=115200,unified=True,compact_ground_fusion=True,short_status=True,binary_diagnostics=True)
    assert code==2 and not r['all_rate_transactions']['restoration_verified']
    assert r['short_status_transport']['restoration_verified']
    journal=json.loads((tmp_path/'rates_recovery.json').read_text())
    assert journal['needs_restore'] and not journal['verified_closed']
    assert set(journal['rates']['original_intervals_us'])=={'248','252'}


def test_frozen_target_actual_runner_sends_only_one_fixed_fc_xyz_yaw(reactor):
    code,r,sent=reactor(baud=115200,unified=True,compact_ground_fusion=True,
        short_status=True,binary_diagnostics=True,low_bandwidth_vision=True,frozen_target_stream=True)
    assert code==0,r['error'] or r['restore_errors']
    assert r['frozen_target_stream_verified'] and r['compact_ground_fusion_verified']
    targets=[(at,msg) for at,msg in sent if msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED']
    assert len(targets)>=32 and targets[-1][0]-targets[0][0]>=2
    assert len({(msg.x,msg.y,msg.z,msg.yaw) for _,msg in targets})==1
    assert r['frozen_target_summary']['frozen_position']==pytest.approx([.34,-.27,-.08])
    assert r['telemetry_rate_writes_sent']==6
    assert set(r['all_rate_transactions']['original_intervals_us'])=={'32','248','252'}
    assert r['all_rate_transactions']['restoration_verified']
    assert all(not (msg.get_type()=='COMMAND_LONG' and msg.command==176) for _,msg in sent)


@pytest.mark.parametrize('scenario',['target_local_missing','target_local_stale','target_local_lost',
    'target_moved','target_position_nan','target_reset','target_mode_changed','target_home',
    'target_pad_lost','target_rate_partial','target_write_partial'])
def test_frozen_target_fault_stops_and_restores_without_resume(reactor,scenario):
    code,r,sent=reactor(scenario,baud=115200,unified=True,compact_ground_fusion=True,
        short_status=True,binary_diagnostics=True,low_bandwidth_vision=True,frozen_target_stream=True)
    assert code==2 and not r['frozen_target_stream_verified'] and r['error']
    assert r['all_rate_transactions']['restoration_verified'],r['restore_errors']
    rollback=next(index for index,(_,msg) in enumerate(sent) if msg.get_type()=='PARAM_SET'
        and str(msg.param_id)=='EKF2_EV_CTRL' and p.decode_parameter(msg)==2)
    assert not any(msg.get_type() in ('VISION_POSITION_ESTIMATE','SET_POSITION_TARGET_LOCAL_NED')
                   for _,msg in sent[rollback:])
    if scenario in ('target_local_missing','target_local_stale','target_rate_partial'):
        assert not r['frozen_target_write_attempts']


def test_frozen_target_third_rate_restore_failure_remains_pending(reactor,tmp_path):
    code,r,_=reactor('target_restore_missing',baud=115200,unified=True,compact_ground_fusion=True,
        short_status=True,binary_diagnostics=True,low_bandwidth_vision=True,frozen_target_stream=True)
    assert code==2 and not r['frozen_target_stream_verified']
    journal=json.loads((tmp_path/'rates_recovery.json').read_text())
    assert journal['needs_restore'] and not journal['verified_closed']
    assert set(journal['rates']['original_intervals_us'])=={'32','248','252'}


def test_lost_baseline_echo_and_first_read_retries_only_reads_within_same_deadline(reactor):
    code,r,sent=reactor('restore_first_read_lost',baud=115200,unified=True,compact_ground_fusion=True,
        short_status=True,binary_diagnostics=True,low_bandwidth_vision=True,frozen_target_stream=True)
    assert code==0,r['error'] or r['restore_errors']
    attempts=[a for a in r['parameter_read_attempts'] if a['name']=='EKF2_EV_CTRL' and a['restoring']]
    assert any(a['retry'] for a in attempts)
    writes=[msg for _,msg in sent if msg.get_type()=='PARAM_SET' and str(msg.param_id)=='EKF2_EV_CTRL'
            and p.decode_parameter(msg)==2]
    assert len(writes)==1  # No duplicate write and never restarting active flow.
    assert attempts[-1]['at']-attempts[0]['at']<2
    assert r['frozen_target_stream_verified'] and r['all_rate_transactions']['restoration_verified']


def test_short_startup_input_loss_without_effects_has_no_false_restore_failure(reactor):
    code,r,sent=reactor('one_home_pad_lost_before_press',baud=115200,unified=True,
        compact_ground_fusion=True,short_status=True,binary_diagnostics=True)
    assert code==2 and r['input_faults']
    assert r['home_lock_attempts']==0 and r['pose_sends']==[] and r['parameter_writes']==[]
    assert r['all_rate_transactions']['writes']==[] and r['restore_errors']==[]
    assert r['parameter_restoration_verified'] and r['actuator_after'] is None
    assert not r['compact_ground_fusion_verified'] and not r['startup_home']['verified']


def test_binary_low_bandwidth_actual_runner_has_48_byte_poses_and_same_gates(reactor):
    code,r,sent=reactor(baud=115200,unified=True,compact_ground_fusion=True,
        short_status=True,binary_diagnostics=True,low_bandwidth_vision=True)
    assert code==0,r
    assert r['compact_ground_fusion_verified'] and r['low_bandwidth_vision']
    assert r['vision_noise_equivalence']['float32_equivalent']
    assert all(s['wire_bytes']==48 for s in r['pose_sends'])
    assert r['tx_wire_bytes']['VISION_POSITION_ESTIMATE']==len(r['pose_sends'])*48
    assert r['compact_ground_receipts']['host_send_rate_hz']>=30
    assert r['all_rate_transactions']['restoration_verified']
    assert r['restored']=={'EKF2_EV_CTRL':True,'EKF2_EV_DELAY':True}
    assert r['tx_messages'].get('SERIAL_CONTROL',0)==0
    assert not r['flight_authorized']


@pytest.mark.parametrize('scenario',['healthy','one_home_early'])
def test_one_home_entire_pipeline_in_one_serial_session_and_exact_restore(reactor,scenario):
    code,r,sent=reactor(scenario,baud=115200,unified=True)
    assert code==0,r
    assert r['one_home_preflight_verified'] and r['startup_home']['verified']
    assert len([e for e in r['home_events'] if e['value']==1])==1
    assert r['parameter_restoration_verified'] and len(r['fusion_audits'])==3
    assert r['all_checks_in_one_process'] and r['telemetry_rate_writes_sent']==0
    assert r['restored']=={'EKF2_EV_CTRL':True,'EKF2_EV_DELAY':True}
    assert any(q['topic']=='initial_visual_cache_excluded' for q in r['internal_observations'])
    lock=next(at for at,msg in sent if msg.get_type()=='COMMAND_LONG' and msg.command==185)
    first_pose=next(at for at,msg in sent if msg.get_type()=='VISION_POSITION_ESTIMATE')
    assert first_pose>lock+1.5


def test_second_home_aborts_unified_flow_without_resume_and_restores(reactor):
    code,r,sent=reactor('one_home_abort',baud=115200,unified=True)
    assert code==2 and r['startup_home']['aborted'] and not r['one_home_preflight_verified']
    assert r['restored']=={'EKF2_EV_CTRL':True,'EKF2_EV_DELAY':True}
    second=r['home_events'][-1]['at']
    assert not any(at>=second for at,msg in sent if msg.get_type()=='VISION_POSITION_ESTIMATE')


def test_delayed_home_has_no_repeated_shell_traffic_before_press_or_settling(reactor):
    code,r,sent=reactor('one_home_delayed',baud=115200,unified=True)
    assert code==0,r
    assert r['startup_home']['verified'] and r['one_home_preflight_verified']
    boundary=max(r['startup_home']['release_at'],r['startup_home']['ack_at'])+1.5
    commands=[(at,msg) for at,msg in sent if msg.get_type()=='SERIAL_CONTROL' and msg.count]
    assert commands and all(at>=boundary for at,_ in commands)
    assert all(at>=boundary for at,msg in sent if msg.get_type() in ('PARAM_SET','VISION_POSITION_ESTIMATE'))
    assert r['timing']['after_home_seconds']<r['timing']['total_seconds']-29
    assert r['timing']['human_wait_after_inputs_s']>20


def test_missing_home_exits_short_wait_without_lock_pose_or_parameter_writes(reactor):
    code,r,sent=reactor('no_home',baud=115200,unified=True,home_wait_seconds=10.)
    assert code==2 and r['timing']['after_home_seconds'] is None
    assert r['duration_s']<12 and r['timing']['initial_home_wait_limit_s']==10
    assert not any(msg.get_type() in ('PARAM_SET','VISION_POSITION_ESTIMATE') or
        msg.get_type()=='COMMAND_LONG' and msg.command==185 for _,msg in sent)


def test_low_listener_rate_fails_without_following_incompatible_counter_or_fusion(reactor):
    code,r,sent=reactor('low_visual_rate',baud=115200,unified=True)
    assert code==2 and not r['one_home_preflight_verified']
    assert r['receive_timing']['rate_hz']<30
    assert not any(q['topic']=='visual_rate' for q in r['internal_observations'])
    assert not r['parameter_writes'] and r['parameter_restoration_verified']
    assert not any(msg.get_type()=='PARAM_SET' for _,msg in sent)


def test_separate_receive_diagnostic_never_runs_long_series_or_fusion(reactor):
    code,r,sent=reactor(baud=115200,unified=True,receive_rate_only=True)
    assert code==0,r
    assert r['receive_diagnostic_completed'] and r['receive_rate_only']
    assert not r['one_home_preflight_verified'] and not r['standalone_locked_static_fusion_verified']
    assert not r['parameter_writes'] and not r['fusion_audits']
    assert not r['receive_diagnostic']['maximum_gap_verified']
    assert r['receive_diagnostic']['counter']['rounded_publication_rate_hz']==32
    stopped=r['receive_diagnostic']['stream_stopped']
    assert stopped['reason']=='planned_receive_only_phase'
    assert r['receive_diagnostic']['counter_window_includes_stopped_source']
    assert r['stream_guard_failure'] is None
    assert not any(at>=stopped['at'] for at,msg in sent if msg.get_type()=='VISION_POSITION_ESTIMATE')
    assert any(at>stopped['at']+.5 for at,msg in sent if msg.get_type()=='SERIAL_CONTROL')
    commands=[bytes(msg.data[:msg.count]) for _,msg in sent if msg.get_type()=='SERIAL_CONTROL' and msg.count]
    assert p.compact.RATE_COMMAND in commands
    assert b'listener vehicle_visual_odometry -n 13\n' not in commands
    assert not any(msg.get_type()=='PARAM_SET' for _,msg in sent)


def test_compact_ground_mode_keeps_full_audits_but_cannot_be_strict_flight_proof(reactor):
    code,r,sent=reactor(baud=115200,unified=True,compact_ground_fusion=True)
    assert code==0,r
    assert r['compact_ground_fusion_verified'] and r['parameter_restoration_verified']
    assert len(r['fusion_audits'])==3 and r['startup_home']['verified']
    assert not r['one_home_preflight_verified'] and not r['standalone_locked_static_fusion_verified']
    assert not r['receive_diagnostic_completed'] and not r['flight_authorized']
    proof=r['compact_ground_receipts']
    assert proof['snapshot_count']==3 and not proof['maximum_fc_publication_gap_verified']
    commands=[bytes(msg.data[:msg.count]) for _,msg in sent if msg.get_type()=='SERIAL_CONTROL' and msg.count]
    assert p.compact.RATE_COMMAND not in commands
    assert b'listener vehicle_visual_odometry -n 13\n' not in commands
    assert commands.count(b'listener vehicle_visual_odometry -n 1\n')==3


@pytest.mark.parametrize('scenario',['aged_lock_reply','moderately_aged_lock_reply'])
def test_lock_refresh_uses_source_age_even_when_reply_just_arrived(reactor,scenario):
    code,r,sent=reactor(scenario,baud=115200,unified=True,compact_ground_fusion=True)
    assert code==0,r
    commands=[bytes(msg.data[:msg.count]) for _,msg in sent if msg.get_type()=='SERIAL_CONTROL' and msg.count]
    lock=b'listener actuator_armed -n 1\n'
    # Every non-lock query refreshes an already aged source snapshot, even
    # though its reply has only just arrived. Arrival-age-only logic skips it.
    for index,command in enumerate(commands):
        if command!=lock:assert index>0 and commands[index-1]==lock


def test_compact_visual_retry_is_paced_and_refreshes_lock(reactor):
    code,r,sent=reactor('visual_clean_retry',baud=115200,unified=True,compact_ground_fusion=True)
    assert code==0,r
    assert r['compact_ground_fusion_verified'] and not r['flight_authorized']
    assert r['diagnostic_failures']
    commands=[(at,bytes(msg.data[:msg.count])) for at,msg in sent if msg.get_type()=='SERIAL_CONTROL' and msg.count]
    visual=b'listener vehicle_visual_odometry -n 1\n'
    indices=[i for i,(_,cmd) in enumerate(commands) if cmd==visual]
    first,second=indices[:2]
    assert commands[second][0]-commands[first][0]>=.25
    assert commands[second-1][1]==b'listener actuator_armed -n 1\n'
    assert len(indices)==4  # failed reply + three new clean snapshots


def test_compact_human_wait_does_not_flood_parameters_and_refreshes_after_home(reactor):
    code,r,sent=reactor('one_home_delayed',baud=115200,unified=True,compact_ground_fusion=True)
    assert code==0,r
    press=r['startup_home']['press_at']
    reads=[(at,str(msg.param_id)) for at,msg in sent if msg.get_type()=='PARAM_REQUEST_READ']
    assert len([at for at,_ in reads if at<press])<=len(p.policy.PARAMS)+2
    first_pose=min(at for at,msg in sent if msg.get_type()=='VISION_POSITION_ESTIMATE')
    assert set(n for at,n in reads if press<=at<first_pose)==set(p.policy.PARAMS)
    assert not r['flight_authorized']


@pytest.mark.parametrize('scenario',['one_home_abort','source_lost','radio_lost','fc_reboot','pad_lost','restore_echo_missing'])
def test_compact_ground_fault_never_passes_or_resumes(reactor,scenario):
    code,r,sent=reactor(scenario,baud=115200,unified=True,compact_ground_fusion=True)
    assert code==2 and not r['compact_ground_fusion_verified']
    assert not r['one_home_preflight_verified'] and not r['flight_authorized']


@pytest.mark.parametrize('scenario',['receive_pad_lost','receive_source_lost','receive_radio_lost','receive_fc_reboot'])
def test_receive_diagnostic_fault_is_never_a_success(reactor,scenario):
    code,r,sent=reactor(scenario,baud=115200,unified=True,receive_rate_only=True)
    assert code==2 and not r['receive_diagnostic_completed']
    assert not r['one_home_preflight_verified'] and not r['fusion_audits']
    assert not any(msg.get_type()=='PARAM_SET' for _,msg in sent)


@pytest.mark.parametrize('scenario',['source_lost','radio_lost','fc_reboot','restore_echo_missing','pad_lost'])
def test_unified_fault_never_passes_and_restore_uncertainty_is_durable(reactor,scenario,tmp_path):
    code,r,sent=reactor(scenario,baud=115200,unified=True)
    assert code==2 and not r['one_home_preflight_verified'] and (r['error'] or r['restore_errors'])
    journal=json.loads((tmp_path/'recovery.json').read_text())
    if scenario in ('radio_lost','fc_reboot','restore_echo_missing'):
        assert not r['parameter_restoration_verified'] and journal['needs_restore']
    else:
        assert r['parameter_restoration_verified'] and not journal['needs_restore']


@pytest.mark.parametrize('baud',[57600,115200])
def test_fusion_runner_uses_only_selected_baud_and_records_strict_parser(reactor,baud):
    code,r,_=reactor(baud=baud)
    assert code==0 and r['trs_baud']==baud
    assert r['rx_parser']['repaired_bytes']==0


def test_complete_runner_standalone_fusion_and_two_parameter_restore(reactor):
    code,r,_=reactor()
    assert code==0 and r['standalone_locked_static_fusion_verified']
    assert len(r['fusion_audits'])==2 and r['receive_timing']['rate_hz']==32
    assert r['restored']=={'EKF2_EV_CTRL':True,'EKF2_EV_DELAY':True}
    assert r['actuator_after']['lockdown']


@pytest.mark.parametrize('scenario',['home','pad_lost','source_lost','damaged_diagnostic'])
def test_live_abort_stops_poses_and_restores_without_restart(reactor,scenario):
    code,r,sent=reactor(scenario)
    assert code==2 and not r['standalone_locked_static_fusion_verified']
    assert r['restored']=={'EKF2_EV_CTRL':True,'EKF2_EV_DELAY':True}
    rollback=next(at for at,msg in sent if msg.get_type()=='PARAM_SET'
        and str(msg.param_id)=='EKF2_EV_CTRL' and p.decode_parameter(msg)==2)
    assert not any(at>=rollback for at,msg in sent if msg.get_type()=='VISION_POSITION_ESTIMATE')
    if scenario=='home':assert any(msg.get_type()=='COMMAND_LONG' and msg.command==185 for _,msg in sent)


@pytest.mark.parametrize('scenario',['radio_lost','fc_reboot','restore_echo_missing'])
def test_unconfirmed_same_link_restore_is_reported_not_invented(reactor,scenario):
    code,r,_=reactor(scenario)
    assert code==2 and not r['standalone_locked_static_fusion_verified']
    assert r['restore_errors']
    assert r['restored']!={'EKF2_EV_CTRL':True,'EKF2_EV_DELAY':True}
    assert not r['same_link_restore_guaranteed']


def test_damaged_transaction_must_drain_then_clean_retry_can_pass(reactor):
    code,r,sent=reactor('diagnostic_clean_retry')
    assert code==0 and r['standalone_locked_static_fusion_verified']
    assert r['diagnostic_failures']
    assert r['internal_observations'][0]['attempt']==2
    assert r['diagnostic_phase_at_exit']=='idle'
    commands=[(at,bytes(msg.data[:msg.count])) for at,msg in sent if msg.get_type()=='SERIAL_CONTROL' and msg.count]
    assert commands[0][1]==commands[1][1]==p.policy.COMMANDS['actuator_armed']
    requests=[i for i,(_,msg) in enumerate(sent) if msg.get_type()=='SERIAL_CONTROL' and msg.count]
    assert any(msg.get_type()=='SERIAL_CONTROL' and not msg.count
               for _,msg in sent[requests[0]+1:requests[1]])


def test_truncated_bad_frame_preserves_next_intact_heartbeat_without_certifying_reply(reactor):
    code,r,_=reactor('truncated_diagnostic')
    assert code==0
    assert r['rx_parser']['counts']['invalid MAVLink CRC']>=1
    assert r['rx_parser']['repaired_bytes']==0
    assert any(h['custom_mode']==0x31415926 for h in r['heartbeats'])
    assert r['diagnostic_failures']
    assert r['internal_observations'][0]['attempt']==2


def test_repeated_damaged_reply_never_becomes_state_or_starts_poses(reactor):
    code,r,sent=reactor('diagnostic_repeated_bad')
    assert code==2 and not r['pose_sends'] and not r['parameter_writes']
    assert not r['internal_observations']
    assert not r['standalone_locked_static_fusion_verified']


def test_home_preempts_bad_packet_in_same_reactor_iteration(reactor):
    code,r,sent=reactor('home_and_bad_packet')
    assert code==2 and r['home_events'] and r['home_lock_acknowledged']
    assert any(msg.get_type()=='COMMAND_LONG' and msg.command==185 for _,msg in sent)
    assert not r['pose_sends'] and not r['parameter_writes']
    home=next(i for i,(_,msg) in enumerate(sent) if msg.get_type()=='COMMAND_LONG' and msg.command==185)
    first_query=next(i for i,(_,msg) in enumerate(sent) if msg.get_type()=='SERIAL_CONTROL' and msg.count)
    assert home>first_query
    assert not any(msg.get_type()=='PARAM_REQUEST_READ' for _,msg in sent[first_query+1:home])


def test_home_is_still_serviced_during_parameter_restore(reactor):
    code,r,sent=reactor('home_during_restore')
    assert code==2 and r['home_lock_acknowledged']
    assert r['restored']=={'EKF2_EV_CTRL':True,'EKF2_EV_DELAY':True}
    rollback=next(at for at,msg in sent if msg.get_type()=='PARAM_SET' and p.decode_parameter(msg)==2)
    home=next(at for at,msg in sent if msg.get_type()=='COMMAND_LONG' and msg.command==185)
    assert rollback<=home
    assert not any(at>=home for at,msg in sent if msg.get_type()=='VISION_POSITION_ESTIMATE')


def test_stop_only_watch_preserves_owner_and_accepts_home_without_restart(reactor):
    code,r,sent=reactor('home_watch')
    assert code==2 and r['home_watch_completed'] and r['home_lock_acknowledged']
    assert r['home_watch_seconds']==20
    assert not r['home_monitor_running_after_exit']
    home=next(at for at,msg in sent if msg.get_type()=='COMMAND_LONG' and msg.command==185)
    assert not any(at>=home for at,msg in sent if msg.get_type()=='VISION_POSITION_ESTIMATE')


def test_input_loss_during_restore_does_not_skip_parameter_readbacks(reactor):
    code,r,_=reactor('pad_lost_during_restore')
    assert code==2 and r['input_faults']
    assert r['restored']=={'EKF2_EV_CTRL':True,'EKF2_EV_DELAY':True}


def test_parameter_requests_are_paced_not_bursted(reactor):
    _,_,sent=reactor()
    # Explicit PARAM_SET verification reads are allowed immediately; only
    # cyclic baseline reads of unchanged parameters are included here.
    names=set(p.policy.PARAMS)-{'EKF2_EV_CTRL','EKF2_EV_DELAY'}
    reads=[at for at,msg in sent if msg.get_type()=='PARAM_REQUEST_READ' and str(msg.param_id) in names]
    assert len(reads)>=len(names)
    assert all(b-a>=.049 for a,b in zip(reads,reads[1:]))


def test_stop_only_watch_cannot_send_any_pose_or_enable_parameter(reactor):
    code,r,sent=reactor('watch_healthy')
    assert code==0 and r['home_watch_completed'] and r['actuator_after']['lockdown']
    baseline_writes=[at for at,msg in sent if msg.get_type()=='PARAM_SET' and p.decode_parameter(msg) in (0,2)]
    assert baseline_writes
    assert not any(at>=max(baseline_writes) and (msg.get_type()=='VISION_POSITION_ESTIMATE'
        or (msg.get_type()=='PARAM_SET' and p.decode_parameter(msg) in (11,65))) for at,msg in sent)


def test_heartbeat_loss_is_not_reclassified_as_recoverable_diagnostic(reactor):
    code,r,_=reactor('heartbeat_lost')
    assert code==2 and not r['standalone_locked_static_fusion_verified']
    assert r['error']
