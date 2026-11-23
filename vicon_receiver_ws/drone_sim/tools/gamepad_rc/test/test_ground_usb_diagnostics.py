"""Two separate encoded receive streams and memory outputs; no real devices."""
from collections import deque
import struct

import pytest
from pymavlink.dialects.v20 import common as m

from test_ground_fusion_acceptance import SlowActorPeer
from test_mission_preparation_io import MemoryPort,decode
from mission_preparation_runtime import PreparationRuntime
from mission_preparation_io import PreparationIO
from mission_audit_collector import COMMANDS
from trs_uart_readonly_check import UID2
from test_trs_vision_reference import baseline
from test_preparation_authorization import files
import preparation_authorization as authorization
import json
import time


class DualPeer(SlowActorPeer):
    def __init__(self,*,usb_uid=UID2):
        super().__init__(ground=True)
        self.usb=deque();self.usb_uid=usb_uid;self.routes=[]
        self.runtime=PreparationRuntime('offline-revision',body_origin_in_rigid=[0,0,0],
            body_orientation_in_rigid=[0,0,0,1],enable_offline_fusion=True,
            enable_temporary_rates=True,rates_persist=lambda r:self.saved.append(r),
            ground_fusion_only=True,usb_observer=True)
        original=self.runtime.turn
        def receive(now,**kwargs):
            boot=int((now+10)*1000)
            data=self.pack(self.encoder.heartbeat_encode(2,m.MAV_AUTOPILOT_PX4,0,4<<16,0))
            data+=self.pack(self.encoder.attitude_quaternion_encode(boot,1,0,0,0,0,0,0))
            while self.usb and len(data)+len(self.usb[0])<=2048:data+=self.usb.popleft()
            return original(now,observer_rx=data,**kwargs)
        self.runtime.turn=receive

    def shell_text(self,text):
        super().shell_text(text)
        while self.shell:self.usb.append(self.shell.popleft())

    def inspect_output(self):
        for row in list(self.runtime.frames)[self.cursor:]:
            msg=self.decoder.parse_buffer(bytes.fromhex(row['wire_hex']))[0]
            self.calls.append((self.now,msg));self.routes.append((row['route'],msg))
            queue=self.usb if row['route']=='usb' else self.pending
            kind=msg.get_type()
            if kind=='COMMAND_LONG' and msg.command==512:
                uid=self.usb_uid if row['route']=='usb' else UID2
                queue.append(self.pack(self.encoder.autopilot_version_encode(0,(1<<24)+(14<<16)+(3<<8),
                    0,0,54,bytes(8),bytes(8),bytes(8),0,0,1,bytes.fromhex(uid))))
            elif kind=='COMMAND_LONG' and msg.command==185:
                assert row['route']=='trs';self.locked=True
                queue.append(self.pack(self.encoder.command_ack_encode(185,0,target_system=250,target_component=220)))
            elif kind in ('PARAM_REQUEST_READ','PARAM_SET'):
                assert row['route']=='usb';name=msg.param_id.rstrip('\0')
                if kind=='PARAM_SET':
                    self.params[name]=msg.param_value if msg.param_type==9 else struct.unpack('<i',struct.pack('<f',msg.param_value))[0]
                floating=name=='EKF2_EV_DELAY';value=self.params[name]
                wire=float(value) if floating else struct.unpack('<f',struct.pack('<i',int(value)))[0]
                queue.append(self.pack(self.encoder.param_value_encode(name.encode(),wire,9 if floating else 6,926,0)))
            elif kind=='VISION_POSITION_ESTIMATE':assert row['route']=='trs';self.last_pose=msg
            elif kind=='SERIAL_CONTROL' and msg.count:
                assert row['route']=='usb'
                payload=bytes(msg.data[:msg.count]);key=next(k for k,v in COMMANDS.items() if v==payload)
                if key.startswith('visual_series'):self.visual_wait=self.now;self.visual_key=key
                else:
                    from test_mission_audit_collector import reply_text
                    text=reply_text(key,self.now)
                    if key=='actuator':text=text.replace('lockdown: True',f'lockdown: {self.locked}')
                    self.shell_text(text)
            elif kind=='COMMAND_LONG' and msg.command in (510,511):
                assert row['route']=='trs';key=int(msg.param1)
                if msg.command==510:answer=self.encoder.message_interval_encode(interval_us=self.values[key],message_id=key)
                else:
                    self.values[key]=int(msg.param2)
                    answer=self.encoder.command_ack_encode(511,0,target_system=250,target_component=220)
                queue.append(self.pack(answer))
        self.cursor=len(self.runtime.frames)


def test_distinct_usb_identity_and_trs_pose_home_routes_reach_ground_proof():
    p=DualPeer();p.rates_ready();r=p.runtime;r.request_prepare(operator_confirmed=True)
    for _ in range(700):
        status=p.turn()
        if status['failure'] or r.coordinator.ground_verified(p.now):break
    assert not status['failure'],status
    assert r.observer_ok() and r.coordinator.ground_verified(p.now)
    trace=r.coordinator.receipt_query_trace
    assert trace['query_key']=='visual_series_ground' and len(trace['raw_timestamps_us'])==13
    assert trace['excluded_initial_timestamp_us']==10_000_000
    assert trace['selected_count']==12
    started=r.coordinator.fusion_started_at
    assert started is not None
    full_topics=('estimator_selector_status','estimator_aid_src','estimator_status','failsafe_flags','vehicle_local_position','vehicle_status')
    for at,msg in p.calls:
        if started<=at<started+1 and msg.get_type()=='SERIAL_CONTROL' and msg.count:
            payload=bytes(msg.data[:msg.count]).decode()
            assert not any(topic in payload for topic in full_topics)
    r.request_close()
    for _ in range(160):
        p.turn()
        if r.closed:break
    assert r.closed and p.params==baseline() and p.values==p.original
    assert any(route=='usb' and msg.get_type()=='SERIAL_CONTROL' for route,msg in p.routes)
    assert not any(route=='usb' and msg.get_type()=='VISION_POSITION_ESTIMATE' for route,msg in p.routes)


def test_wrong_usb_identity_blocks_before_any_configuration_or_pose():
    p=DualPeer(usb_uid='00'*18)
    for _ in range(20):p.turn()
    assert p.runtime.failure and not p.runtime.observer_ok()
    assert not any(msg.get_type() in ('PARAM_SET','VISION_POSITION_ESTIMATE','SERIAL_CONTROL') for _,msg in p.routes)


def test_rf_damaged_frame_does_not_destroy_usb_diagnostic_reply():
    p=DualPeer();p.home_check();r=p.runtime
    assert r.observer_ok()
    r.coordinator.audit.action(p.now,True,full_audit=False)
    phase=r.coordinator.audit.transaction.phase
    # A structurally invalid RF prefix is reduced only by the RF parser.
    p.turn(extra_rx=b'bad-radio-bytes')
    assert not any(f['reason']=='audit_reply_damaged_packet' for f in r.coordinator.audit.failures)
    assert r.observer_parser.summary()['counts'].get('invalid MAVLink CRC',0)==0


def test_transport_cannot_redirect_diagnostics_or_pose_to_other_port():
    p=DualPeer();p.rates_ready();r=p.runtime
    trs,usb=MemoryPort(),MemoryPort()
    io=PreparationIO(r,trs,observer_port=usb,poll_gamepad=lambda:{},poll_pose=lambda:None,
        usb_present=lambda:True,authorize=lambda *args:True)
    r.coordinator.note_parameter_request('EKF2_EV_CTRL',p.now)
    r._capture(r.encoder.param_request_read_encode(1,1,b'EKF2_EV_CTRL',-1))
    record=r.frames[-1];assert record['route']=='usb'
    io._dispatch(record);assert len(usb.writes)==1 and not trs.writes
    with pytest.raises(PermissionError):io._dispatch({**record,'route':'trs'})
    assert len(usb.writes)==1 and not trs.writes


def test_usb_observer_not_a_silent_fallback_for_strict_mission_mode():
    with pytest.raises(ValueError,match='ground-fusion-only'):
        PreparationRuntime('r',body_origin_in_rigid=[0,0,0],body_orientation_in_rigid=[0,0,0,1],usb_observer=True)


def test_physical_grant_binds_usb_observer_identity_and_diagnostic_route(files):
    p=DualPeer();p.rates_ready();r=p.runtime;r.request_prepare(operator_confirmed=True)
    for _ in range(700):
        p.turn()
        if r.coordinator.ground_verified(p.now):break
    assert r.coordinator.ground_verified(p.now)
    def token(observer,name):
        d=authorization.draft(files.run,files.heading,files.board,files.proof,files.backup,
            ground_fusion_only=True,usb_observer=observer)
        now=time.time();d.update(user_authorized=True,issued_unix=now-1,expires_unix=now+120,
            physical_confirmations=dict(battery_disconnected=True,usb_only=True,stationary=True,props_treated_as_installed=True))
        path=files.root/name;path.write_text(json.dumps(d));return path
    with pytest.raises(ValueError,match='diagnostic_route'):
        authorization.consume(token(False,'wrong-route'),r,files.run,files.heading,files.board,files.proof,files.backup)
    grant=authorization.consume(token(True,'usb-route'),r,files.run,files.heading,files.board,files.proof,
        files.backup,clock=lambda:p.now)
    frame=next(row for row in reversed(r.frames) if row['type']=='VISION_POSITION_ESTIMATE')
    assert frame['at']==r.now and grant(r,frame,decode(bytes.fromhex(frame['wire_hex'])))
    r.observer_inputs.hb['at']-=3
    assert not grant(r,frame,decode(bytes.fromhex(frame['wire_hex'])))
