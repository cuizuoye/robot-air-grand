"""Plan B continuous fixed-point integration: synthetic peer, no real devices.

The imported approval/persistence fixture is SYNTHETIC ONLY. It never writes a
profile or issues a physical permit. conftest forbids serial/input opens.
"""
from copy import deepcopy
from pathlib import Path
import struct
import sys
from types import SimpleNamespace as NS

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import ekf_static_profile as fixed
from fixed_profile_fixtures import approved_profile
from synthetic_mission_peer import SyntheticPeer
from continuous_mission_runtime import ContinuousMissionRuntime
from fixed_mission_fusion import FixedMissionFusionService
from continuous_execution_io import ContinuousExecutionIO
from mission_preparation_io import PreparationIO
from gamepad_estop_monitor import HOME
from test_mission_fusion_service import context, WireSink


def peer(profile=None, **kwargs):
    return SyntheticPeer(fixed_configuration=True,
        fixed_profile=approved_profile() if profile is None else profile, **kwargs)


def prepared(**kwargs):
    p=peer(**kwargs); result=p.prepare()
    assert not result['failure'], result
    assert p.runtime.coordinator.fusion.phase=='HANDED_OFF', result
    return p


def finish(p, turns=5000):
    for _ in range(turns):
        result=p.turn()
        if result['closed'] or p.runtime.core.phase=='LAND_TIMEOUT':break
    return p.runtime.status()


def no_writes(p):
    assert not [msg for _,msg in p.calls if msg.get_type() in ('PARAM_SET','PARAM_EXT_SET')]
    assert not p.runtime.coordinator.fusion.changed


@pytest.mark.parametrize('case',['fallback','mapped','future','old','regressed','no_reply','mismatch'])
def test_local_ros_receipt_contract_handles_mapping_without_precision_tuning(case):
    from test_mission_fusion_service import reception
    profile=approved_profile();profile['timestamp_mode']='local_ros_timesync'
    profile['timestamp_evidence']='SYNTHETIC test only'
    service=FixedMissionFusionService(lambda msg:True,profile=profile)
    service.start_at=0.;service.start_boot_ms=10000
    service.sent=[dict(source_stamp_ns=100000000000,position=[1,2,3],q=[1,0,0,0])]
    service.timesync_reply_count=1
    rows=reception(service,.352)
    for row in rows:row['timestamp_sample']-=20_000 if case!='fallback' else 0
    if case=='future':rows[-1]['timestamp_sample']=rows[-1]['timestamp']+1
    if case=='old':rows[-1]['timestamp_sample']=rows[-1]['timestamp']-150001
    if case=='regressed':rows[-1]['timestamp_sample']=rows[-2]['timestamp_sample']
    if case=='no_reply':service.timesync_reply_count=0
    if case=='mismatch':rows[-1]['position']=[9,9,9]
    result=service.reception_checks(context(.352),rows,requested_at=0)
    assert (not result['failed']) == (case in ('fallback','mapped'))
    assert result['absolute_clock_accuracy_verified'] is False
    # Existing legacy contract still rejects a mapped sample.
    if case=='mapped':
        service.profile['timestamp_mode']='fc_receive_time'
        assert 'receive_time_mode' in service.reception_checks(context(.352),rows,requested_at=0)['failed']


def test_local_ros_profile_needs_evidence_and_new_host_review_not_fc_reboot():
    profile=approved_profile();old_host=fixed.fingerprint(profile);old_fc=fixed.configuration_digest(profile)
    profile['timestamp_mode']='local_ros_timesync'
    assert 'timestamp_evidence_missing' in fixed.profile_errors(profile)
    profile['timestamp_evidence']='SYNTHETIC test only'
    assert not fixed.profile_errors(profile)
    assert fixed.compare(profile,profile['parameters'])['configuration_approved']
    assert fixed.configuration_digest(profile)==old_fc
    assert fixed.fingerprint(profile)!=old_host


def test_host_mode_edit_cannot_reuse_active_runtime_configuration():
    p=prepared();service=p.runtime.coordinator.fusion
    assert service.configuration_bound(p.runtime.context())
    service.profile['timestamp_mode']='local_ros_timesync'
    service.profile['timestamp_evidence']='SYNTHETIC changed host mode'
    assert not service.configuration_bound(p.runtime.context())


def test_local_ros_timesync_continues_through_fixed_point_sequence():
    profile=approved_profile();profile['timestamp_mode']='local_ros_timesync'
    profile['timestamp_evidence']='SYNTHETIC test only'
    profile['persistence']['configuration_sha256']=fixed.configuration_digest(profile)
    p=peer(profile);p.runtime.sync_rates_enabled=True  # Synthetic only.
    assert not p.prepare()['failure']
    before=p.runtime.fixed_sync_reply_count
    p.runtime.request_start(operator_confirmed=True)
    result=finish(p)
    assert result['closed'] and p.runtime.core.phase=='COMPLETE',result
    assert p.runtime.fixed_sync_reply_count>before
    assert any(row['armed'] for row in p.state_rows)
    assert p.intervals[111]==0 and p.runtime.sync_rates.summary()['restoration_verified']
    no_writes(p)


def test_nominal_fixed_point_complete_zero_writes_and_actual_state_confirmation():
    p=prepared(); params=deepcopy(p.params); r=p.runtime
    original_proof=deepcopy(r.coordinator.fusion.configuration)
    assert not r.enable_sent and not r.core.arm_attempted
    assert not any(msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED' for _,msg in p.calls)
    r.request_start(operator_confirmed=True)
    result=finish(p)
    assert result['closed'] and r.core.phase=='COMPLETE' and r.core.fault is None, result
    assert p.params==params and r.coordinator.fusion.configuration==original_proof
    assert not p.armed and p.locked
    targets=[msg for _,msg in p.calls if msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED']
    assert targets and all((msg.x,msg.y)==(1.,2.) for msg in targets)
    assert min(msg.z for msg in targets)==pytest.approx(p.ground_z-.3)
    pose_times=[t for t,msg in p.calls if msg.get_type()=='VISION_POSITION_ESTIMATE']
    target_rows=[(t,msg) for t,msg in p.calls if msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED']
    assert max(b-a for a,b in zip(pose_times,pose_times[1:]))<=.05
    # Synthetic peer timestamps are decimal-rounded seconds. Compare integer
    # nanoseconds instead of treating 0.11000000000000032 as a physical overrun.
    assert max(round((b[0]-a[0])*1e9) for a,b in zip(target_rows,target_rows[1:]))<=110_000_000
    assert all(abs(b[1].z-a[1].z)<=.15*(b[0]-a[0])+1e-6
               for a,b in zip(target_rows,target_rows[1:]))
    assert r.core.preview.height==.3 and r.core.preview.hover_seconds==3.
    assert set(fixed.READ_PARAMS)<={msg.param_id for _,msg in p.calls if msg.get_type()=='PARAM_REQUEST_READ'}
    no_writes(p)
    assert result['hardware_execution_available'] is result['flight_authorized'] is False


def test_no_fixed_profile_fallback_when_profile_missing():
    p=SyntheticPeer(fixed_configuration=True)
    # The synthetic telemetry can supply all typed values, but no approved profile.
    p.params={n:s['value'] for n,s in approved_profile()['parameters'].items()}
    result=p.prepare()
    assert result['failure'] and p.runtime.core.phase=='BLOCKED'
    no_writes(p)


@pytest.mark.parametrize('change', ['delay_unapproved','ctrl_unapproved','persistence_missing','profile_identity',
                                  'delay_mismatch','ctrl_mismatch','profile_mutated'])
def test_configuration_rejects_unapproved_mismatched_or_changed_inputs(change):
    profile=approved_profile()
    if change=='delay_unapproved':profile['parameters']['EKF2_EV_DELAY']['status']='pending_calibration'
    if change=='ctrl_unapproved':profile['parameters']['EKF2_EV_CTRL']['status']='unreviewed'
    if change=='persistence_missing':profile['persistence']['status']='unverified'
    if change=='profile_identity':profile['firmware']['uid2']='0'*36
    p=peer(profile)
    if change=='delay_mismatch':p.params['EKF2_EV_DELAY']=0.
    if change=='ctrl_mismatch':p.params['EKF2_EV_CTRL']=2
    if change=='profile_mutated':p.runtime.coordinator.fusion.profile['profile_id']='changed'
    result=p.prepare()
    assert result['failure'] and not p.runtime._handed_off(), result
    no_writes(p)


def test_typed_current_post_home_snapshot_required_no_cached_time_refresh():
    p=prepared(); f=p.runtime.coordinator.fusion
    proof=f.configuration
    assert proof['post_home_requests'] and proof['current_configuration_verified']
    assert not proof['physical_authorization'] and not proof['flight_authorized']
    first=deepcopy(proof)
    for _ in range(100):p.turn()
    assert f.configuration==first
    assert f.configuration_at<p.now


@pytest.mark.parametrize('name', ['EKF2_EV_CTRL','EKF2_EV_DELAY','COM_OBL_RC_ACT'])
def test_unsolicited_parameter_drift_rejects_before_output_enable(name):
    p=prepared(); r=p.runtime
    kind=fixed.PARAMETER_TYPES[name]; value=p.params[name]+1
    wire=float(value) if kind==9 else struct.unpack('<f',struct.pack('<i',value))[0]
    p.turn(extra_rx=p.pack(p.encoder.param_value_encode(name.encode(),wire,kind,926,0)))
    assert r.failure and not r.enable_sent
    with pytest.raises(PermissionError):r.request_start(operator_confirmed=True)
    no_writes(p)


def test_unknown_landing_prevents_cleanup_no_parameter_restoration():
    p=prepared(); r=p.runtime
    r.request_start(operator_confirmed=True)
    for _ in range(1000):
        p.turn()
        if r.core.phase=='ASCEND':break
    with pytest.raises(PermissionError):r.request_close()
    assert not r.closed and p.armed
    no_writes(p)


def test_existing_switch_pro_home_code_and_press_release_meaning_reused():
    from gamepad_estop_monitor import StopLatch
    assert HOME==0x13c
    latch=StopLatch(); latch.event(1,HOME,1); latch.event(1,HOME,0)
    assert latch.latched and not latch.home_pressed
    p=prepared(); r=p.runtime
    assert r.inputs.home_proven and not r.enable_sent
    p.turn(home=(True,));p.turn(home=(False,))
    assert r.stop_latched and r.core.stop_latched
    with pytest.raises(PermissionError):r.request_start(operator_confirmed=True)
    no_writes(p)


def test_real_io_entry_stays_closed_before_any_resource_access():
    r=peer().runtime
    class ForbiddenPort:
        def __getattr__(self,name):raise AssertionError('No device or port property may be accessed')
    port=ForbiddenPort()
    with pytest.raises(PermissionError,match='OFFLINE ONLY'):
        ContinuousExecutionIO(r,port,poll_gamepad=lambda:None,poll_pose=lambda:None,
            now_ros_ns=lambda:0,check_port_identity=lambda:None,usb_present=lambda:False)
    with pytest.raises(PermissionError,match='OFFLINE ONLY'):
        PreparationIO(r,port,poll_gamepad=lambda:None,poll_pose=lambda:None,usb_present=lambda:True)


def test_attempted_legacy_parameter_transactions_cannot_escape_fixed_service_or_writer():
    p=prepared();r=p.runtime; f=r.coordinator.fusion
    f.pending=dict(name='EKF2_EV_CTRL',value=2,at=p.now)
    f.phase='RESTORE_CTRL'
    msg=r.encoder.param_set_encode(1,1,b'EKF2_EV_CTRL',0.,6)
    with pytest.raises(PermissionError):r.writer(msg)
    with pytest.raises(PermissionError):f._dispatch(msg,r.context())
    with pytest.raises(PermissionError):f._parameter(r.context(),'EKF2_EV_CTRL',2,'WAIT_RESTORE_CTRL')
    f.phase='WAIT_CTRL_ECHO';f.pending=dict(name='EKF2_EV_CTRL',value=11,at=p.now)
    wire=struct.unpack('<f',struct.pack('<i',11))[0]
    echo=p.encoder.param_value_encode(b'EKF2_EV_CTRL',wire,6,926,0)
    assert not f.ingest(echo,r.context()) and f.phase=='WAIT_CTRL_ECHO'
    f.tick(r.context())
    assert f.phase=='BLOCKED' and f.fault=='legacy_transaction_in_fixed_mode'
    no_writes(p)


def test_higher_envelope_not_unlocked_by_fixed_configuration():
    with pytest.raises(ValueError,match='0.2..0.5'):
        peer(height=2.)


@pytest.mark.parametrize('scenario', ['offboard_rejected','arm_rejected','enable_ack_lost',
                                     'climb_timeout','landing_timeout','start_withheld'])
def test_existing_command_and_landing_failures_never_restore_parameters(scenario):
    p=prepared(scenario=scenario);r=p.runtime
    if scenario!='start_withheld':r.request_start(operator_confirmed=True)
    result=finish(p,turns=7500)
    if scenario=='landing_timeout':
        assert r.core.phase=='LAND_TIMEOUT' and not result['closed']
    elif scenario=='start_withheld':
        # Waiting for an operator no longer has an automatic start deadline.
        # It must stay disarmed and remain cancellable, not manufacture a fault.
        assert r.core.phase=='WAIT_EXPLICIT_START' and not result['closed']
        assert r.core.fault is None and r.start_requested_at is None
        assert not any(msg.get_type()=='COMMAND_LONG' and msg.command==400 for _,msg in p.calls)
        r.request_cancel()
        assert finish(p)['closed']
    else:
        assert result['closed'] and r.core.fault is not None, result
    assert sum(msg.get_type()=='COMMAND_LONG' and msg.command==400 for _,msg in p.calls)<=1
    no_writes(p)


def ascend(p):
    p.runtime.request_start(operator_confirmed=True)
    for _ in range(1000):
        p.turn()
        if p.runtime.core.phase=='ASCEND':return
    pytest.fail('No synthetic ASCEND')


def test_timesync_expiry_aborts_but_preserves_fresh_pose_during_land(monkeypatch):
    p=prepared();ascend(p);r=p.runtime
    monkeypatch.setattr(r.fixed_clock,'ingest',lambda *a:False)
    result=finish(p)
    assert result['closed'] and r.core.fault and 'timesync' in r.failure
    land_times=[t for t,msg in p.calls if msg.get_type()=='COMMAND_LONG' and msg.command==21]
    assert land_times
    assert any(t>=land_times[0] for t,msg in p.calls if msg.get_type()=='VISION_POSITION_ESTIMATE')
    assert not any(t>land_times[0] for t,msg in p.calls if msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED')
    no_writes(p)


@pytest.mark.parametrize('event', ['pose_loss','gamepad_loss','link_loss','fc_reboot','cancel','home_stop'])
def test_airborne_faults_preserve_existing_stop_or_land_semantics(event):
    p=prepared(scenario=event);ascend(p);r=p.runtime
    for _ in range(20):p.turn()
    p.injected=True
    if event=='fc_reboot':p.boot_offset-=5
    if event=='cancel':r.request_cancel()
    if event=='home_stop':p.turn(home=(True,));p.turn(home=(False,))
    result=finish(p,turns=4000 if event!='home_stop' else 100)
    if event in ('link_loss','fc_reboot'):
        assert not result['closed'] and r.core.phase=='LAND_TIMEOUT'
    elif event=='home_stop':
        assert r.core.stop_latched and r.core.phase=='STOP_LATCHED'
        assert not result['closed']  # Emergency stop is NOT observed touchdown.
    else:
        assert result['closed'] and r.failure
        assert any(msg.get_type()=='COMMAND_LONG' and msg.command==21 for _,msg in p.calls)
    no_writes(p)


@pytest.mark.parametrize('which', ['attitude','timesync'])
def test_original_500ms_proof_bounds_not_relaxed_before_start(which):
    p=prepared();r=p.runtime
    if which=='attitude':r.inputs.attitude['at']=p.now-.501
    else:r.fixed_clock.reference['received_ns']=int(p.now*1e9)-501_000_000
    with pytest.raises(PermissionError):r.request_start(operator_confirmed=True)
    assert not r.enable_sent and not r.core.arm_attempted
    no_writes(p)


def test_snapshot_correct_but_no_actual_fusion_never_hands_off():
    p=peer(); original=p._reply_text
    def not_fused(key):
        return original(key).replace('cs_ev_pos: True','cs_ev_pos: False').replace('fused: True','fused: False')
    p._reply_text=not_fused
    result=p.prepare()
    assert p.runtime.coordinator.fusion.receive_verified
    assert not p.runtime._handed_off() and not p.runtime.core.arm_attempted
    assert not result['flight_authorized']
    no_writes(p)


def test_wrong_parameter_type_rejects_even_if_decoded_value_looks_matching():
    p=peer(); original=p.pack
    def wrong_type(msg):
        if msg.get_type()=='PARAM_VALUE' and msg.param_id.rstrip('\0')=='EKF2_EV_DELAY':
            msg.param_type=6
        return original(msg)
    p.pack=wrong_type
    result=p.prepare()
    assert result['failure'] and not p.runtime._handed_off()
    no_writes(p)


def test_missing_parameter_reply_times_out_without_unlimited_retry():
    p=peer();original=p.inspect_output
    def missing_reply():
        original()
        packets=[]
        for wire in p.pending:
            msg=m.MAVLink(None).parse_char(wire)
            if msg.get_type()=='PARAM_VALUE' and msg.param_id=='EKF2_EV_DELAY':continue
            packets.append(wire)
        p.pending.clear();p.pending.extend(packets)
    p.inspect_output=missing_reply
    result=p.prepare()
    assert result['failure']=='fixed_configuration_read_timeout' and p.now<10
    no_writes(p)


def test_no_home_input_means_no_lock_pose_or_mission_start():
    p=peer()
    for _ in range(200):p.turn()
    assert not p.runtime.prepare_requested and not p.runtime.inputs.home_proven
    assert not any(msg.get_type() in ('VISION_POSITION_ESTIMATE','SET_POSITION_TARGET_LOCAL_NED')
        or msg.get_type()=='COMMAND_LONG' and msg.command!=512 for _,msg in p.calls)
    no_writes(p)


def test_fixed_profile_cannot_silently_select_legacy_experiment():
    with pytest.raises(ValueError,match='never fall back'):
        ContinuousMissionRuntime('test',body_origin_in_rigid=[0,0,0],
            body_orientation_in_rigid=[0,0,0,1],fixed_profile=approved_profile())


@pytest.mark.parametrize('command',[180,245,246])
def test_alternative_parameter_storage_or_reboot_commands_still_rejected(command):
    p=prepared();r=p.runtime
    with pytest.raises(PermissionError):r.writer(r.encoder.command_long_encode(1,1,command,0,1,0,0,0,0,0,0))
    no_writes(p)


@pytest.mark.parametrize('known_ground',[True,False])
def test_finally_after_failed_pose_dispatch_never_writes_parameters(known_ground):
    from dataclasses import replace
    profile=approved_profile();values={n:s['value'] for n,s in profile['parameters'].items()}
    seen=[]
    def failing_sink(msg):
        seen.append(msg.get_type())
        raise OSError('synthetic short/failed dispatch')
    service=FixedMissionFusionService(failing_sink,profile=profile,authorizer=lambda *a:True)
    c=context()
    service.begin_fixed(c,parameters=values,parameter_types=fixed.PARAMETER_TYPES,
        parameter_times=dict.fromkeys(values,0.),parameter_requests=dict.fromkeys(values,0.),
        home_barrier=0.,source_q=[1,0,0,0],fc_q=[1,0,0,0])
    try:
        with pytest.raises(OSError):service.pose(c,100,[1,2,3],[1,0,0,0])
    finally:
        if known_ground:service.close(c)
        else:
            with pytest.raises(PermissionError):service.close(replace(c,armed=None))
    assert seen==['VISION_POSITION_ESTIMATE'] and not service.changed
    assert service.fault=='fusion_dispatch_uncertain'
    assert (service.phase=='CLOSED') is known_ground


@pytest.mark.parametrize('rebased,native,difference,accepted',[
    (True,True,5.11,True),(True,True,6.01,False),
    (False,True,5.04,False),(True,False,5.04,False),
])
def test_rebased_trial_tilt_allowance_is_scoped_and_failure_is_atomic(
        monkeypatch,rebased,native,difference,accepted):
    import math
    from dataclasses import replace
    profile=approved_profile()
    if rebased:profile['initial_trial']={'rigid_frame_transfer':{'test_only':True}}
    sent=[];service=FixedMissionFusionService(sent.append,profile=profile)
    # Isolate the alignment policy. Production configuration validation still
    # validates the frame-transfer evidence before this branch is reachable.
    monkeypatch.setattr(service,'check_configuration',lambda *a,**kw:{'current_configuration_verified':True})
    c=replace(context(),pilot_source='native_rc' if native else 'gamepad',
              native_rc_current=native,native_stop_clear=native,lockdown=not native)
    rad=math.radians(difference)
    kwargs=dict(parameters={},parameter_types={},parameter_times={},parameter_requests={},
                home_barrier=0.,source_q=[math.cos(rad/2),math.sin(rad/2),0.,0.],fc_q=[1.,0.,0.,0.])
    if accepted:
        service.begin_fixed(c,**kwargs)
        assert service.phase=='RECEIVE_WARMUP'
        report=service.status()['initial_configuration']['startup_attitude_alignment']
        assert report['tilt_difference_deg']==pytest.approx(difference)
        assert report['effective_tolerance_deg']==6. and report['nominal_exceeded']
        assert report['tilt_calibrated'] is False
    else:
        with pytest.raises(ValueError,match='tilt disagreement'):service.begin_fixed(c,**kwargs)
        assert service.phase=='NEW' and service.configuration is None and service.configuration_epoch is None
    assert not sent


@pytest.mark.parametrize('fault',[None,'bounded_jitter','duplicate','future','old_sample','mismatch','wrong_boot','no_sync'])
def test_native_sparse_receipts_keep_matching_time_and_epoch_guards(fault):
    from dataclasses import replace
    profile=approved_profile();profile['timestamp_mode']='local_ros_timesync'
    service=FixedMissionFusionService(lambda msg:True,profile=profile)
    first=replace(context(0),pilot_source='native_rc',native_rc_current=True,native_stop_clear=True,lockdown=False)
    service.binding=(first.boot_session,first.rigid_revision)
    service.phase='RECEIVE_WARMUP';service.start_at=0.;service.start_boot_ms=10000
    service.timesync_reply_count=0 if fault=='no_sync' else 1
    def feed(i):
        now=i*.2;c=replace(first,now=now,boot_ms=int((10+now)*1000))
        service.last_send=now
        service.sent=[dict(elapsed_s=k/32,source_stamp_ns=int((100+k/32)*1e9),position=[1,2,3],q=[1,0,0,0])
                      for k in range(int(now*32)+1)]
        if fault=='bounded_jitter':
            service.sent=[s for s in service.sent if not .09<s['elapsed_s']<.16]
        row=dict(timestamp=int((10+now-.02)*1e6),timestamp_sample=int((10+now-.03)*1e6),
                 position=[1,2,3],q=[1,0,0,0],pose_frame=1,reset_counter=0)
        if i==2:
            if fault=='duplicate':row=service.receipt_snapshots[0]['snapshot']
            elif fault=='future':row['timestamp_sample']=row['timestamp']+1
            elif fault=='old_sample':row['timestamp_sample']=row['timestamp']-150001
            elif fault=='mismatch':row['position']=[99,2,3]
            elif fault=='wrong_boot':c=replace(c,boot_session='different-boot')
        service.confirm_snapshot_reception(c,row,requested_at=now-.03)
    if fault=='no_sync':
        with pytest.raises(ValueError):feed(1)
    else:
        feed(1)
        assert not service.receive_verified and service.phase=='RECEIVE_WARMUP'
        if fault and fault!='bounded_jitter':
            with pytest.raises(ValueError):feed(2)
        else:
            feed(2);feed(3)
            assert service.receive_verified and service.phase=='FUSING'
            assert not service.receipt_checks['maximum_fc_publication_gap_verified']
            assert not service.receipt_checks['all_packet_reception_rate_verified']
            assert service.receipt_checks['cadence_policy']=='native_rc_mission_continuity'
            if fault=='bounded_jitter':assert not service.receipt_checks['host_nominal_cadence_met']
    if fault and fault!='bounded_jitter':assert not service.receive_verified
