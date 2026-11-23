"""Offline lifecycle using encoded MAVLink, not a live send permission."""
from dataclasses import replace
from pathlib import Path
import struct
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import mission_fusion_service as p
from mavlink_crc_stream import CrcResyncParser
from test_trs_vision_reference import baseline
from test_mission_live_inputs import current,telemetry,actuator,pose,audit
from test_mission_audit_collector import deliver
from mission_audit_collector import MissionAuditCollector
from flight_mission_core import MissionCore


def context(now=0,**changes):
    values=dict(now=now,boot_session='offline-boot',rigid_revision='offline-rigid',boot_ms=int((now+10)*1000),
                uid2=p.UID2,firmware='1.14.3',heartbeat_age=.01,actuator_age=.01,
                source_age=.01,attitude_age=.01,armed=False,landed=True,lockdown=True,
                gamepad_current=True,home_pressed=False,epoch_valid=True)
    return p.Context(**{**values,**changes})


class WireSink:
    def __init__(self):
        self.messages=[]
        self.encoder=m.MAVLink(None,srcSystem=250,srcComponent=220)
    def __call__(self,msg):
        parsed=CrcResyncParser().parse_buffer(msg.pack(self.encoder))
        assert len(parsed)==1 and parsed[0].get_type()!='BAD_DATA'
        self.encoder.seq=(self.encoder.seq+1)%256
        self.messages.append(parsed[0])
        return True


def begun(*,authorized=True,binding=None):
    sink=WireSink()
    service=p.ContinuousFusionService(sink,authorizer=(lambda *args:True) if authorized else None)
    c=context(**(binding or {}))
    params=baseline()
    service.begin(c,parameters=params,parameter_times=dict.fromkeys(params,0),
                  source_q=[1,0,0,0],fc_q=[1,0,0,0])
    return service,sink,c


def stream(service,until,*,binding=None):
    t=0 if service.last_send is None else round(service.last_send+.032,9)
    while t <= until+1e-8:
        service.pose(context(t,**(binding or {})),int((100+t)*1e6),[1,2,3],[1,0,0,0])
        t=round(t+.032,9)


def reception(service,now):
    return [dict(timestamp=int((10+now-(11-i)*.032)*1e6),
                 timestamp_sample=int((10+now-(11-i)*.032)*1e6),
                 position=[1,2,3],q=[1,0,0,0],pose_frame=1,reset_counter=0)
            for i in range(12)]


def echo(service,c,name,value,*,source=(1,1),type_override=None):
    integer=name=='EKF2_EV_CTRL'
    wire=struct.unpack('<f',struct.pack('<i',int(value)))[0] if integer else float(value)
    kind=type_override if type_override is not None else 6 if integer else 9
    e=m.MAVLink(None,srcSystem=source[0],srcComponent=source[1])
    msg=e.param_value_encode(name.encode(),wire,kind,926,0)
    decoded=m.MAVLink(None).parse_char(msg.pack(e))
    return service.ingest(decoded,c)


def fusing(binding=None):
    service,sink,c=begun(binding=binding)
    stream(service,.352,binding=binding)
    c=context(.352,**(binding or {}))
    service.confirm_reception(c,reception(service,.352),requested_at=0)
    service.tick(c);assert echo(service,c,'EKF2_EV_DELAY',65)
    service.tick(c);assert echo(service,c,'EKF2_EV_CTRL',11)
    assert service.phase=='FUSING'
    return service,sink,c


def test_default_authorizer_denies_before_any_external_sink_call():
    service,sink,c=begun(authorized=False)
    with pytest.raises(PermissionError):service.pose(c,1,[0,0,0],[1,0,0,0])
    assert not sink.messages and not service.changed


@pytest.mark.parametrize('change',[
    {'armed':True},{'landed':False},{'lockdown':False},{'source_age':.151},
    {'actuator_age':.501},{'heartbeat_age':2.},{'gamepad_current':False},
    {'home_pressed':True},{'uid2':'wrong'},{'armed':0}])
def test_initialization_requires_real_locked_ground_context(change):
    service=p.ContinuousFusionService(WireSink())
    params=baseline()
    with pytest.raises(ValueError):
        service.begin(context(**change),parameters=params,parameter_times=dict.fromkeys(params,0),
                      source_q=[1,0,0,0],fc_q=[1,0,0,0])


def test_exact_wire_parameter_sequence_and_current_echo_before_enable():
    service,sink,c=fusing()
    params=[msg for msg in sink.messages if msg.get_type()=='PARAM_SET']
    assert [str(msg.param_id) for msg in params]==['EKF2_EV_DELAY','EKF2_EV_CTRL']
    assert [p.decode_parameter(msg) for msg in params]==[65,11]
    assert all(msg.get_type() in ('PARAM_SET','VISION_POSITION_ESTIMATE') for msg in sink.messages)
    assert not service.status()['physical_flight_authorized']


def test_missing_incomplete_stale_baseline_is_not_archived_readiness():
    for changes,times in (({'EKF2_EV_CTRL':11},0),({},-10)):
        params={**baseline(),**changes}
        with pytest.raises(ValueError):
            p.ContinuousFusionService(WireSink()).begin(context(),parameters=params,
                parameter_times=dict.fromkeys(params,times),source_q=[1,0,0,0],fc_q=[1,0,0,0])


@pytest.mark.parametrize('case',['short','slow','old','mismatch'])
def test_received_pose_proof_must_be_complete_current_matching_and_fast(case):
    service,_,_=begun();stream(service,.352)
    rows=reception(service,.352)
    if case=='short':rows=rows[-2:]
    if case=='slow':rows[1]['timestamp']+=20000
    if case=='old':
        for r in rows:r['timestamp']-=1_000_000;r['timestamp_sample']-=1_000_000
    if case=='mismatch':rows[-1]['position']=[99,2,3]
    with pytest.raises(ValueError):service.confirm_reception(context(.352),rows,requested_at=0)
    assert not service.receive_verified


def test_waiting_for_echo_never_retries_enable_or_accepts_other_fc():
    service,sink,_=begun();stream(service,.352)
    c=context(.352);service.confirm_reception(c,reception(service,.352),requested_at=0);service.tick(c)
    assert not echo(service,c,'EKF2_EV_DELAY',65,source=(2,1))
    assert service.phase=='WAIT_DELAY_ECHO'
    stream(service,3.392)
    service.tick(context(3.392))
    assert service.phase=='BLOCKED' and service.fault=='fusion_parameter_echo_timeout'
    assert len([m for m in sink.messages if m.get_type()=='PARAM_SET'])==1
    assert not echo(service,context(3.4),'EKF2_EV_DELAY',65)


def test_wrong_encoding_does_not_count_as_successful_echo():
    service,sink,c=fusing()
    service.close(c);service.tick(c)
    assert not echo(service,c,'EKF2_EV_CTRL',2,type_override=9)
    assert service.phase=='BLOCKED'


def test_duplicate_source_stamp_and_expired_send_continuity_cannot_resume():
    service,sink,c=fusing();n=len(sink.messages)
    assert not service.pose(context(.36),service.last_stamp,[1,2,3],[1,0,0,0])
    assert len(sink.messages)==n
    assert not service.pose(context(.6),123456789,[1,2,3],[1,0,0,0])
    assert service.phase=='BLOCKED'
    assert not service.pose(context(.61),123456790,[1,2,3],[1,0,0,0])


def test_ground_restore_is_allowed_without_vicon_but_never_armed_or_unknown():
    service,sink,c=fusing()
    with pytest.raises(PermissionError):service.close(replace(c,armed=True))
    assert service.land_required
    with pytest.raises(PermissionError):service.close(replace(c,armed=None))
    c=replace(c,source_age=99.,gamepad_current=False,home_pressed=True)
    service.close(c);service.tick(c);assert echo(service,c,'EKF2_EV_CTRL',2)
    service.tick(c);assert echo(service,c,'EKF2_EV_DELAY',0)
    assert service.phase=='CLOSED'
    assert not service.pose(c,123,[1,2,3],[1,0,0,0])


def test_stream_keeps_running_while_nonblocking_audit_parses_all_fixed_topics():
    service,sink,c=begun();collector=MissionAuditCollector();completed=[]
    for index in range(201):
        t=index*.005
        service.pose(context(t),int((100+t)*1e6),[1,2,3],[1,0,0,0])
        key=collector.action(t,True)
        if key and key!='poll':
            deliver(collector,key,t+.001)
            completed.extend(collector.take_audits())
    times=[r['elapsed_s'] for r in service.sent]
    assert completed and len(times)>25
    assert max(b-a for a,b in zip(times,times[1:]))<=.04
    assert all(msg.get_type()=='VISION_POSITION_ESTIMATE' for msg in sink.messages)


def test_preparation_handoff_to_same_core_writer_never_starts_or_restores_during_land():
    inputs=current();binding=dict(boot_session=inputs.boot_session,rigid_revision=inputs.rigid_revision)
    service,sink,_=fusing(binding)
    stream(service,2.4,binding=binding)
    o,_=inputs.build(2.4)
    core=MissionCore(service.sink)
    assert not core.prepare(o,inputs.selector)
    service.mark_ready(context(2.4,**binding),inputs)
    for t in (3.4,4.4):
        stream(service,t,binding=binding)
        telemetry(inputs,t);pose(inputs,t);inputs.observe_actuator(actuator(t),t)
        row,commander=audit(t,2)
        inputs.observe_audit(row,commander,requested_at=t-.01,now=t)
        o,missing=inputs.build(t);assert o is not None and not missing
        core.prepare(o,inputs.selector)
    assert core.phase=='WAIT_EXPLICIT_START'
    c=context(4.4,**binding);service.handoff(c,core)
    assert service.phase=='HANDED_OFF' and not core.arm_attempted and not core.offboard_attempted
    assert not service.pose(c,999999999,[1,2,3],[1,0,0,0])  # Only core can send after handoff.
    core.phase='LAND'
    with pytest.raises(PermissionError):service.close(c)
    assert service.phase=='HANDED_OFF'
    assert not any(msg.get_type()=='COMMAND_LONG' for msg in sink.messages)


def test_readiness_does_not_use_a_simple_external_true_flag():
    service,_,c=fusing()
    inputs=current()
    with pytest.raises(ValueError):service.mark_ready(c,inputs)  # Different boot/revision.


def test_clock_fault_cannot_silently_resume_pose_stream():
    service,sink,c=fusing()
    with pytest.raises(ValueError):service.tick(context(.1))
    n=len(sink.messages)
    assert service.phase=='BLOCKED'
    assert not service.pose(context(.4),123456,[1,2,3],[1,0,0,0])
    assert len(sink.messages)==n


def test_new_query_cannot_reuse_an_old_same_boot_receive_series():
    service,_,_=begun();stream(service,.992)
    old=reception(service,.352)
    with pytest.raises(ValueError):
        service.confirm_reception(context(.992),old,requested_at=.8)


@pytest.mark.parametrize('source_rate',[100,300,330,400])
def test_pose_cadence_does_not_drift_to_source_quantized_lower_rate(source_rate):
    service,sink,_=begun()
    for n in range(source_rate*4+1):
        now=n/source_rate
        service.pose(context(now),int((100+now)*1e6),[1,2,3],[1,0,0,0])
    stamps=[row['elapsed_s'] for row in service.sent]
    measured=(len(stamps)-1)/(stamps[-1]-stamps[0])
    assert 31.8<=measured<=32.2
    assert max(b-a for a,b in zip(stamps,stamps[1:]))<=.05
    assert len(sink.messages)<=130


def test_pose_cadence_skips_missed_slots_and_never_bursts_or_replays_source():
    service,sink,_=begun()
    assert service.pose(context(0),100_000_000,[1,2,3],[1,0,0,0])
    assert service.pose(context(.1),100_100_000,[1,2,3],[1,0,0,0])
    assert service.next_send>.1
    for index in range(1,10):
        assert not service.pose(context(.1+index*.0001),100_100_000+index,[1,2,3],[1,0,0,0])
    assert len(sink.messages)==2


def test_receipt_report_names_failed_gate_without_granting_or_configuring():
    service,sink,_=begun();stream(service,.352)
    rows=reception(service,.352)
    for index,row in enumerate(rows):row['timestamp']=10_000_000+index*40_000;row['timestamp_sample']=row['timestamp']
    with pytest.raises(ValueError):service.confirm_reception(context(.45),rows,requested_at=0)
    assert 'rate_at_least_30hz' in service.status()['receipt_checks']['failed']
    assert service.status()['receipt_checks']['timing']['rate_hz']==25
    assert not service.receive_verified and not service.changed
    assert not any(msg.get_type()=='PARAM_SET' for msg in sink.messages)


def test_unknown_epoch_prohibits_ground_restore_even_with_matching_uid():
    service,sink,c=fusing()
    n=len(sink.messages)
    with pytest.raises(PermissionError):service.close(replace(c,epoch_valid=False))
    assert len(sink.messages)==n
