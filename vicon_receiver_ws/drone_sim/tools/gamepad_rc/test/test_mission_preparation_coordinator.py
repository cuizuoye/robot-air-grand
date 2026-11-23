"""Composition tests: one writer, real encoded frames, no hardware opens."""
from pathlib import Path
import sys
import struct

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from mission_preparation_coordinator import PreparationCoordinator
from mission_live_inputs import LiveMissionInputs
from mission_fusion_service import context_from_inputs
from test_mission_live_inputs import current,packet,telemetry,pose,actuator,identity
from test_mission_fusion_service import WireSink
from test_trs_vision_reference import baseline


def bound(*,authorized=False):
    inputs=current(component=220);sink=WireSink()
    runner=PreparationCoordinator(inputs,sink,fusion_authorizer=(lambda *a:True) if authorized else None)
    runner.observe_source_pose(inputs.pose_stamp,[1,2,3],[1,0,0,0])
    return runner,inputs,sink


def parameters(runner,now):
    values=baseline();fc=m.MAVLink(None)
    for name,value in values.items():
        runner.note_parameter_request(name,now)
        floating=name=='EKF2_EV_DELAY'
        wire=float(value) if floating else struct.unpack('<f',struct.pack('<i',int(value)))[0]
        msg=fc.param_value_encode(name.encode(),wire,9 if floating else 6,926,0)
        runner.ingest(packet(msg),now)


def test_all_real_context_fields_default_unknown_and_not_ready():
    inputs=LiveMissionInputs('r',component=220)
    c=context_from_inputs(inputs,0)
    assert c.armed is None and c.landed is None and c.lockdown is None
    assert not c.locked_ground() and c.boot_ms==-1


def test_component_mismatch_not_silently_adopted():
    with pytest.raises(ValueError,match='component 220'):
        PreparationCoordinator(current(),WireSink())


def test_default_composition_only_fixed_reads_even_with_enabled_flag():
    runner,inputs,sink=bound();parameters(runner,2.4)
    runner.step(2.4,enable_fusion_preparation=True)
    assert runner.waiting_reason=='separate_fusion_dispatch_authorizer_missing'
    assert all(msg.get_type()=='SERIAL_CONTROL' for msg in sink.messages)
    assert not runner.status()['flight_authorized']


def test_explicit_offline_authorizer_begins_pose_stream_but_not_fusion_before_receipt():
    runner,inputs,sink=bound(authorized=True);parameters(runner,2.4)
    runner.step(2.4,enable_fusion_preparation=True)
    assert runner.fusion.phase=='RECEIVE_WARMUP'
    assert 'visual_series' in runner.audit.queue
    assert [msg.get_type() for msg in sink.messages]==['SERIAL_CONTROL','VISION_POSITION_ESTIMATE']
    assert not any(msg.get_type()=='PARAM_SET' for msg in sink.messages)
    assert runner.core.phase=='PREPARE' and not runner.core.arm_attempted


def test_missing_baseline_or_unsolicited_parameter_does_not_start_fusion():
    runner,inputs,sink=bound(authorized=True)
    msg=m.MAVLink(None).param_value_encode(b'EKF2_EV_CTRL',0.,6,926,0)
    runner.ingest(packet(msg),2.4)
    assert not runner.parameters
    runner.step(2.4,enable_fusion_preparation=True)
    assert runner.fusion.phase=='NEW'
    assert all(msg.get_type()=='SERIAL_CONTROL' for msg in sink.messages)


def test_pose_must_match_exact_current_gate_stamp():
    runner,inputs,_=bound()
    with pytest.raises(ValueError):runner.observe_source_pose(inputs.pose_stamp-1,[1,2,3],[1,0,0,0])


def test_new_pose_still_stops_when_actual_source_expired():
    runner,inputs,sink=bound(authorized=True);parameters(runner,2.4)
    runner.step(2.4,enable_fusion_preparation=True)
    telemetry(inputs,2.6);inputs.observe_actuator(actuator(2.6),2.6)
    runner.step(2.6,enable_fusion_preparation=True)
    assert runner.fusion.phase=='BLOCKED'
    assert runner.fusion.fault=='fusion_preparation_context_lost'
    assert not any(msg.get_type()=='PARAM_SET' for msg in sink.messages)


def test_writer_failure_does_not_silently_advance_fixed_read_owner():
    inputs=current(component=220)
    runner=PreparationCoordinator(inputs,lambda message:False)
    with pytest.raises(OSError):runner.step(2.4)
    assert runner.failed=='fixed_read_dispatch_uncertain'
    runner.step(2.5)  # Latched, no automatic retry or fusion start.
    assert runner.fusion.phase=='NEW'


def test_initial_actuator_request_does_not_depend_on_already_having_an_actuator_reply():
    inputs=LiveMissionInputs('r',component=220);identity(inputs);telemetry(inputs,0)
    sink=WireSink();runner=PreparationCoordinator(inputs,sink)
    assert context_from_inputs(inputs,0).armed is None
    runner.step(0)
    assert len(sink.messages)==1 and sink.messages[0].get_type()=='SERIAL_CONTROL'
    assert bytes(sink.messages[0].data[:sink.messages[0].count])==b'listener actuator_armed -n 1\n'


def test_usb_actor_just_ahead_of_rf_clock_does_not_abort_or_refresh_state():
    from test_mission_audit_collector import reply_text
    runner,inputs,_=bound()
    original=inputs.actuator.copy()
    assert runner.audit.action(2.4,True,full_audit=False)=='actuator'
    fc=m.MAVLink(None)
    text=reply_text('actuator',2.425).encode()
    def receive(now):
        for offset in range(0,len(text),70):
            part=text[offset:offset+70]
            runner.ingest(packet(fc.serial_control_encode(10,0,0,0,len(part),part.ljust(70,b'\0'))),now)
    receive(2.41)  # USB is 15ms ahead of the independent RF boot estimate.
    assert inputs.actuator==original and not inputs.fault
    assert runner.audit.last_snapshot is None and not runner.audit.failures
    assert runner.audit.noncurrent_actuator_reads[-1]['reason']=='ahead_of_owner_clock'
    telemetry(inputs,2.53)
    assert runner.audit.action(2.53,True,full_audit=False)=='actuator'
    receive(2.54)
    assert inputs.actuator['timestamp_us']==12_425_000
    assert inputs.actuator['at']==2.54 and not inputs.fault


def test_ground_receipt_waits_for_original_clock_without_accepting_future_proof():
    from test_mission_fusion_service import reception
    runner,inputs,sink=bound(authorized=True)
    runner.ground_fusion_only=True;runner.status_age=1.
    parameters(runner,2.4);runner.step(2.4,enable_fusion_preparation=True)
    for n in range(1,13):
        now=round(2.4+n*.032,6)
        telemetry(inputs,now);inputs.observe_actuator(actuator(now),now);pose(inputs,now)
        runner.observe_source_pose(inputs.pose_stamp,[1,2,3],[1,0,0,0])
        runner.step(now,enable_fusion_preparation=True,service_diagnostics=False)
    now=2.784
    # Service uses [1,2,3] for this synthetic source. All strict receipt gates
    # pass except a USB publication 20ms ahead of the RF clock estimate.
    rows=reception(runner.fusion,now+.020)
    group=dict(rows=rows,requested_at=2.4)
    runner._receive_group(group,now)
    assert runner.pending_receipt and not runner.fusion.receive_verified
    assert not runner.fusion.changed
    now=2.816
    telemetry(inputs,now);inputs.observe_actuator(actuator(now),now);pose(inputs,now)
    runner.observe_source_pose(inputs.pose_stamp,[1,2,3],[1,0,0,0])
    runner.step(now,enable_fusion_preparation=True,service_diagnostics=False)
    assert runner.fusion.receive_verified and runner.receipt_clock_waits==1
    assert runner.pending_receipt is None


def test_strict_receipt_does_not_get_ground_clock_wait_exception():
    from test_mission_fusion_service import reception
    runner,inputs,_=bound(authorized=True);parameters(runner,2.4)
    runner.step(2.4,enable_fusion_preparation=True)
    rows=reception(runner.fusion,2.42)
    with pytest.raises(ValueError):runner._receive_group(dict(rows=rows,requested_at=2.4),2.4)
    assert runner.pending_receipt is None and not runner.fusion.receive_verified


@pytest.mark.parametrize('case',['caught_up','future_sample','clock_stalled','too_far_ahead'])
def test_native_snapshot_waits_for_real_clock_without_rewriting_or_accepting_bad_pose(case):
    from copy import deepcopy
    from dataclasses import replace
    from fixed_profile_fixtures import approved_profile
    from test_mission_live_inputs import native_current_status
    from test_mission_fusion_service import context
    inputs=native_current_status();inputs.component=220
    inputs.audit_rows.clear()  # Receipt warmup has not collected EKF fusion audits yet.
    profile=approved_profile();profile['timestamp_mode']='local_ros_timesync'
    runner=PreparationCoordinator(inputs,WireSink(),fixed_configuration=True,
        fixed_profile=profile,fusion_authorizer=lambda *a:True)
    first=replace(context(0),pilot_source='native_rc',native_rc_current=True,
                  native_stop_clear=True,lockdown=False)
    service=runner.fusion
    service.binding=(first.boot_session,first.rigid_revision)
    service.phase='RECEIVE_WARMUP';service.start_at=0.;service.start_boot_ms=10000
    service.timesync_reply_count=1
    service.sent=[dict(elapsed_s=k/32,source_stamp_ns=int((100+k/32)*1e9),
        position=[1,2,3],q=[1,0,0,0]) for k in range(77)]
    for now in (2.,2.2):
        service.last_send=now
        c=replace(first,now=now,boot_ms=int((10+now)*1000))
        row=dict(timestamp=int((10+now-.02)*1e6),timestamp_sample=int((10+now-.02)*1e6)-2,
            position=[1,2,3],q=[1,0,0,0],pose_frame=1,reset_counter=0)
        service.confirm_snapshot_reception(c,row,requested_at=now-.03)
    c=replace(first,now=2.4,boot_ms=12400);runner.context_provider=lambda now:c
    service.last_send=2.4
    stamp=12_400_449 if case!='too_far_ahead' else 12_450_001
    row=dict(timestamp=stamp,timestamp_sample=stamp+1 if case=='future_sample' else stamp-2,
        position=[1,2,3],q=[1,0,0,0],pose_frame=1,reset_counter=0)
    group=dict(rows=[row],requested_at=2.37,query_key='visual_snapshot');original=deepcopy(group)
    if case=='too_far_ahead':
        with pytest.raises(ValueError):runner._receive_group(group,2.4)
        assert runner.pending_receipt is None
        return
    runner._receive_group(group,2.4)
    assert runner.pending_receipt and not service.receive_verified
    assert len(service.receipt_snapshots)==2 and group==original
    now=2.56 if case=='clock_stalled' else 2.402
    c=replace(c,now=now,boot_ms=12400 if case=='clock_stalled' else 12402)
    if case=='caught_up':
        runner.step(now,enable_fusion_preparation=True,service_diagnostics=False)
        assert service.receive_verified and runner.pending_receipt is None
        assert service.receipt_snapshots[-1]['snapshot']==original['rows'][0]
        assert runner.receipt_clock_waits==1
    else:
        with pytest.raises(ValueError):runner.step(now,enable_fusion_preparation=True,service_diagnostics=False)
        assert not service.receive_verified


@pytest.mark.parametrize('bad_new_sample',[False,True])
def test_ground_query_excludes_only_initial_cache_not_bad_current_samples(bad_new_sample):
    from copy import deepcopy
    from test_mission_fusion_service import reception
    runner,inputs,_=bound(authorized=True)
    runner.ground_fusion_only=True;runner.status_age=1.
    parameters(runner,2.4);runner.step(2.4,enable_fusion_preparation=True)
    for n in range(1,13):
        now=round(2.4+n*.032,6)
        telemetry(inputs,now);inputs.observe_actuator(actuator(now),now);pose(inputs,now)
        runner.observe_source_pose(inputs.pose_stamp,[1,2,3],[1,0,0,0])
        runner.step(now,enable_fusion_preparation=True,service_diagnostics=False)
    rows=reception(runner.fusion,2.784)
    cached=deepcopy(rows[0]);cached['timestamp']=cached['timestamp_sample']=1_000_000
    if bad_new_sample:rows[4]['timestamp']=rows[4]['timestamp_sample']=2_000_000
    group=dict(rows=[cached,*rows],requested_at=2.4,query_key='visual_series_ground')
    if bad_new_sample:
        with pytest.raises(ValueError):runner._receive_group(group,2.784)
        assert not runner.fusion.receive_verified and not runner.fusion.changed
    else:
        runner._receive_group(group,2.784)
        assert runner.fusion.receive_verified
        assert runner.fusion.receipt_checks['timing']['samples']==12
        assert runner.receipt_query_trace['excluded_initial_timestamp_us']==1_000_000
    assert runner.receipt_query_trace['selected_count']==12


@pytest.mark.parametrize('ground,count',[(False,13),(True,12),(True,14)])
def test_extra_sample_query_cannot_change_strict_mode_or_accept_wrong_count(ground,count):
    from test_mission_fusion_service import reception
    runner,_,_=bound(authorized=True);runner.ground_fusion_only=ground
    rows=reception(runner.fusion,2.784)
    while len(rows)<count:rows.insert(0,rows[0])
    rows=rows[:count]
    with pytest.raises(ValueError,match='13-snapshot'):
        runner._receive_group(dict(rows=rows,requested_at=2.4,query_key='visual_series_ground'),2.4)
    assert not runner.fusion.receive_verified
