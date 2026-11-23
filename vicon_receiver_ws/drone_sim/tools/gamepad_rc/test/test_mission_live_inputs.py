"""Receive integration using real MAVLink bytes and synthetic offline inputs."""
from copy import deepcopy
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import mission_live_inputs as p
from mavlink_crc_stream import CrcResyncParser
from test_usb_combined_fusion_policy import rows as fusion_rows
from test_flight_mission_core import selector


def packet(msg, system=1, component=1):
    encoder = m.MAVLink(None, srcSystem=system, srcComponent=component)
    return CrcResyncParser().parse_buffer(msg.pack(encoder))[0]


def telemetry(state, now, *, armed=False, main_mode=4, boot=None):
    e = m.MAVLink(None)
    boot = int(now*1000)+10000 if boot is None else boot
    for msg in (
        e.heartbeat_encode(m.MAV_TYPE_QUADROTOR,m.MAV_AUTOPILOT_PX4,
                           128 if armed else 0,main_mode << 16,0),
        e.attitude_quaternion_encode(boot,1,0,0,0,0,0,0),
        e.local_position_ned_encode(boot,1,2,-.1,0,0,0),
        e.extended_sys_state_encode(0,m.MAV_LANDED_STATE_ON_GROUND),
    ):
        state.ingest(packet(msg),now)


def identity(state, now=0):
    state.request_identity(now)
    e=m.MAVLink(None)
    msg=e.autopilot_version_encode(0,(1 << 24)+(14 << 16)+(3 << 8),
        0,0,54,bytes(8),bytes(8),bytes(8),0,0,1,bytes.fromhex(p.UID2))
    state.ingest(packet(msg),now)
    return msg


def actuator(now, *, locked=True):
    return dict(timestamp_us=int((now+10)*1e6),at=now,requested_at=now-.01,
                armed=False,lockdown=locked,manual_lockdown=False,force_failsafe=False)


def pose(state, now):
    ns=int((now+100)*1e9)
    sample=NS(stamp_ns=ns,received_monotonic_ns=int(now*1e9))
    state.observe_pose(sample,ns,int(now*1e9),now)
    return sample


def home(state):
    state.observe_gamepad('opened-pad-generation',True,.1,event=True,home=True)
    state.observe_gamepad('opened-pad-generation',True,.2,event=True,home=False)
    ack=m.MAVLink(None).command_ack_encode(185,m.MAV_RESULT_ACCEPTED,
                                         target_system=250,target_component=state.component)
    state.ingest(packet(ack),.3)


def audit(now, index):
    stamp=int((now+10)*1e6)
    row=deepcopy(fusion_rows()[index])
    row['selector']={**selector(1,2),'timestamp':stamp}
    for key in ('flags','local','status','failsafe','aid','height_aid','yaw_aid'):
        if key not in row:
            row[key]={}
        row[key]['timestamp']=stamp
    row['local'].update(xy_reset_counter=0,z_reset_counter=0,heading_reset_counter=0)
    for key in ('aid','height_aid','yaw_aid'):
        row[key].update(timestamp_sample=stamp-100,time_last_fuse=stamp-10)
    row['status'].update(dict.fromkeys(p.control.STATUS_FLAGS,False))
    row['status']['timestamp_sample']=stamp-100
    row['failsafe'].update(dict.fromkeys(p.control.FAILSAFE_FLAGS,False))
    return row,dict(timestamp=stamp,pre_flight_checks_pass=True,arming_state=1,nav_state=4)


def current(*,component=202):
    state=p.LiveMissionInputs('offline-revision',component=component)
    identity(state);telemetry(state,0);home(state)
    for index,now in enumerate((2.,2.2,2.4)):
        telemetry(state,now)
        state.observe_actuator(actuator(now),now)
        pose(state,now)
        row,commander=audit(now,index)
        state.observe_audit(row,commander,requested_at=now-.01,now=now)
    return state


def native_current_status():
    s=current()
    status=dict(reasons=[],selected_manual_input=dict(roll=0.,pitch=0.,yaw=0.,throttle=-1.,sticks_moving=False),
        kill_requested=False,stop_active=False,native_input_observed=True,sticks_ready=True)
    s.native_rc=NS(status=lambda *a,**k:deepcopy(status),observe_message=lambda *a:None)
    refresh_native_status(s,2.4)
    return s


def refresh_native_status(s,now):
    telemetry(s,now);pose(s,now);s.observe_actuator(actuator(now),now)
    row,commander=audit(now,0)
    row['local'].update(v_xy_valid=True,v_z_valid=True)
    s.observe_status(dict(local=row['local'],commander=commander),requested_at=now-.01,now=now)


def test_native_sequential_aids_can_span_recorded_gap_but_pose_must_stay_live():
    s=native_current_status()
    for row in s.audit_rows:
        for key in ('aid','height_aid','yaw_aid'):
            for field in ('timestamp','timestamp_sample','time_last_fuse'):
                row[key][field]-=2_492_173
    original=deepcopy(s.audit_rows)
    assert s.build(2.4)[0] is not None
    assert s.audit_rows==original
    refresh_native_status(s,2.6)
    s.pose_at=2.4
    assert 'vicon_pose_missing_or_stale' in s.build(2.6)[1]


def test_native_fast_feedback_survives_full_audit_interval_without_restamping():
    s=native_current_status();original=deepcopy(s.audit_rows)
    refresh_native_status(s,3.1)
    o,missing=s.build(3.1)
    assert o is not None and not missing and o.status_age_s==0
    assert s.audit_rows==original and s.audit_at==2.4
    refresh_native_status(s,4.401)
    assert not s.build(4.401)[1]
    assert s.audit_rows==original and s.audit_at==2.4
    # Measured 5.575506057s gap, rounded up to this fixture's FC millisecond.
    now=7.976
    refresh_native_status(s,now)
    assert not s.build(now)[1]
    assert s.audit_rows==original and s.audit_at==2.4
    now=2.4+p.NATIVE_AUDIT_MAX_AGE_S+.001
    refresh_native_status(s,now)
    assert s.build(now)[1]==['current_ekf_full_audit_missing_or_stale']


@pytest.mark.parametrize('native',[False,True])
@pytest.mark.parametrize('duration',[5.361059464,8.001])
def test_recorded_slow_row_admission_preserves_source_times(native,duration):
    # Recorded receive-to-receive minimum assembly time; synthetic values.
    s=native_current_status() if native else current()
    now=12.;requested=now-duration
    row,_=audit(requested,2)
    recent,commander=audit(now,2)
    for topic in ('local','status','failsafe'):row[topic]=recent[topic]
    before=deepcopy(row)
    if native:refresh_native_status(s,now)
    else:telemetry(s,now)
    if native and duration<8.:
        s.observe_audit(row,commander,requested_at=requested,now=now)
        assert not s.build(now)[1]
        assert s.audit_rows[-1]['aid']==before['aid']
        assert s.audit_at==now and s.audit_request==requested
        s.pose_at=now-.151
        assert 'vicon_pose_missing_or_stale' in s.build(now)[1]
    else:
        with pytest.raises(ValueError,match='Bounded current-boot audit transaction'):
            s.observe_audit(row,commander,requested_at=requested,now=now)
        assert s.audit_at==2.4
    assert row==before


@pytest.mark.parametrize('topics',[('selector',),('flags',),('selector','flags')])
def test_native_current_periodic_ekf_status_can_repeat_without_restamping(topics):
    s=native_current_status()
    original=deepcopy(s.audit_rows[-1])
    refresh_native_status(s,2.6)
    row,commander=audit(2.6,2)
    for topic in topics:row[topic]=deepcopy(original[topic])
    s.observe_audit(row,commander,requested_at=2.5,now=2.6)
    observation,missing=s.build(2.6)
    assert observation is not None and not missing
    for topic in topics:assert s.audit_rows[-1][topic]==original[topic]
    assert s.audit_rows[-1]['aid']['timestamp']>original['aid']['timestamp']


@pytest.mark.parametrize('case',['regression','conflict','expired','frozen_aid','frozen_row'])
def test_native_periodic_status_exception_does_not_admit_stale_or_conflicting_data(case):
    s=native_current_status()
    original=deepcopy(s.audit_rows[-1])
    now=2.4+p.NATIVE_AUDIT_MAX_AGE_S+.001 if case=='expired' else 2.6
    refresh_native_status(s,now)
    row,commander=audit(now,2)
    row['selector']=deepcopy(original['selector'])
    if case=='regression':row['selector']['timestamp']-=1
    if case=='conflict':row['selector']['instance_changed_count']+=1
    if case=='frozen_aid':row['aid']=deepcopy(original['aid'])
    if case=='frozen_row':row=deepcopy(original)
    with pytest.raises(ValueError):
        s.observe_audit(row,commander,requested_at=now-.1,now=now)
    assert s.audit_at==2.4 and s.audit_rows[-1]==original


@pytest.mark.parametrize('topic,limit',[('local',1.),('commander',2.5)])
def test_frozen_fast_source_expires_despite_a_recent_reply(topic,limit):
    s=native_current_status()
    frozen=deepcopy(s.critical_status[topic]['value'])
    now=2.4+limit-.01
    refresh_native_status(s,now)
    # Simulate a recent reread of the original source in the reducer state.
    s.critical_status[topic]=dict(value=frozen,at=now)
    now+=.02
    telemetry(s,now);pose(s,now);s.observe_actuator(actuator(now),now)
    assert 'current_'+topic+'_status_missing_or_stale' in s.build(now)[1]


@pytest.mark.parametrize('change',['future','regressed','conflicting','bad_counter','bad_boolean'])
def test_invalid_fast_status_cannot_replace_current_evidence(change):
    s=native_current_status();values=deepcopy(s.critical_status['local']['value'])
    now=2.4
    if change=='future':values['timestamp']+=1
    if change=='regressed':values['timestamp']-=1
    if change=='conflicting':values['xy_reset_counter']+=1
    if change in ('bad_counter','bad_boolean'):
        now=2.41;telemetry(s,now);values['timestamp']=12_410_000
        if change=='bad_counter':values['xy_reset_counter']=256
        else:values['xy_valid']=1
    original=deepcopy(s.critical_status)
    with pytest.raises(ValueError):s.observe_status(dict(local=values),requested_at=now-.01,now=now)
    assert s.critical_status==original


def test_fast_reset_and_preflight_change_do_not_wait_for_full_audit():
    s=native_current_status();refresh_native_status(s,2.5)
    local=deepcopy(s.critical_status['local']['value']);local['timestamp']+=10000;local['xy_reset_counter']=1
    telemetry(s,2.51)
    s.observe_status(dict(local=local),requested_at=2.5,now=2.51)
    assert s.build(2.51)[0].reset_counters==(1,0,0)
    assert s.audit_rows[-1]['local']['xy_reset_counter']==0
    commander=deepcopy(s.critical_status['commander']['value'])
    commander.update(timestamp=12510000,pre_flight_checks_pass=False)
    s.observe_status(dict(commander=commander),requested_at=2.5,now=2.51)
    assert 'current_preflight_not_verified' in s.build(2.51)[1]


def test_fast_selector_change_is_seen_before_next_audit_completion():
    s=native_current_status();s.offline_flight_observation=True
    selected=deepcopy(s.selector);selected['instance_changed_count']+=1
    s.observe_status(dict(selector=selected),requested_at=2.39,now=2.4)
    assert 'current_ekf_selector_changed' in s.build(2.4)[1]


def test_nothing_is_ready_by_default_and_no_zero_position_substitute():
    s=p.LiveMissionInputs('r')
    o,reasons=s.build(0)
    assert o is None and 'current_fc_identity_missing' in reasons
    assert 'current_xyz_yaw_fusion_not_verified' in reasons
    assert not s.describe(0)['flight_authorized']


def test_current_wire_and_audit_producers_build_actual_core_observation():
    s=current();o,reasons=s.build(2.4)
    assert not reasons and o.navigation_ok()
    assert o.position_ned==(1,2,pytest.approx(-.1))
    assert o.fusion_ready and o.preflight_ready and o.home_response_verified
    assert o.reset_counters==(0,0,0) and o.lockdown and not o.home_pressed


@pytest.mark.parametrize('height_ref,baro_ctrl,accepted', [(3,0,True),(0,0,False),(3,1,False)])
def test_fixed_profile_selects_height_contract_without_false_baro_block(height_ref,baro_ctrl,accepted):
    from fixed_profile_fixtures import approved_profile
    from mission_preparation_coordinator import PreparationCoordinator
    state=current(component=220)
    profile=approved_profile()
    profile['parameters']['EKF2_HGT_REF']['value']=height_ref
    profile['parameters']['EKF2_BARO_CTRL']['value']=baro_ctrl
    # Synthetic construction only; no profile approval or device writer.
    PreparationCoordinator(state,lambda msg: None,
        fixed_configuration=True,fixed_profile=profile)
    for row in state.audit_rows:row['flags']['cs_baro_hgt']=False
    observation,missing=state.build(2.4)
    assert (observation is not None) == accepted
    assert ('current_xyz_yaw_fusion_not_verified' not in missing) == accepted


@pytest.mark.parametrize('failure',['height_disabled','height_rejected','stale_yaw','cached_aid'])
def test_ev_height_contract_preserves_actual_fusion_checks(failure):
    state=current();state.require_baro_height=False
    for row in state.audit_rows:row['flags']['cs_baro_hgt']=False
    if failure=='height_disabled':state.audit_rows[-1]['flags']['cs_ev_hgt']=False
    if failure=='height_rejected':state.audit_rows[-1]['height_aid']['innovation_rejected']=True
    if failure=='stale_yaw':state.audit_rows[-1]['yaw_comparison']['fresh']=False
    if failure=='cached_aid':state.audit_rows[-1]['height_aid']=deepcopy(state.audit_rows[-2]['height_aid'])
    assert 'current_xyz_yaw_fusion_not_verified' in state.build(2.4)[1]


def test_one_hz_heartbeat_is_not_misrepresented_as_half_second_commander_state():
    s=current();s.hb['at']=1.5
    o,reasons=s.build(2.4)
    assert o is not None and not reasons
    assert o.status_age_s < .5  # Comes from the fresh actuator/Commander, not heartbeat.
    s.hb['at']=.4
    assert 'heartbeat_missing_or_stale' in s.build(2.4)[1]


def test_mode_comes_from_current_commander_not_old_heartbeat():
    s=current();telemetry(s,2.5,main_mode=4);pose(s,2.5);s.observe_actuator(actuator(2.5),2.5)
    row,commander=audit(2.5,2);commander['nav_state']=14
    s.observe_audit(row,commander,requested_at=2.49,now=2.5)
    observation,reasons=s.build(2.5)
    assert not reasons and observation.offboard and not s.hb['offboard']


def test_fresh_telemetry_does_not_refresh_internal_commander_or_reset_proof():
    s=current();telemetry(s,3);pose(s,3);s.observe_actuator(actuator(3),3)
    _,missing=s.build(3)
    assert 'commander_state_missing_or_stale' in missing
    assert 'current_ekf_audit_reset_counters_and_commander_missing_or_stale' in missing


def test_commander_and_actuator_arm_disagreement_is_not_resolved_by_ack():
    s=current();telemetry(s,2.5);pose(s,2.5);s.observe_actuator(actuator(2.5),2.5)
    row,commander=audit(2.5,2);commander['arming_state']=2
    s.observe_audit(row,commander,requested_at=2.49,now=2.5)
    assert 'commander_and_actuator_armed_state_disagrees' in s.build(2.5)[1]


def test_reuses_existing_mission_core_without_start_or_any_send():
    from flight_mission_core import MissionCore
    state=current();messages=[]
    core=MissionCore(lambda msg: messages.append(msg))
    observation,_=state.build(2.4)
    assert not core.prepare(observation,state.selector)
    assert core.phase=='PREPARE' and not messages and not core.arm_attempted


@pytest.mark.parametrize('change', ['stale','bad_height','rejected_yaw','commander_fail',
                                  'velocity_invalid','missing_status','selector_unhealthy'])
def test_bad_or_incomplete_proof_never_readies(change):
    s=current()
    if change=='stale':
        now=3.
    else:
        now=2.5;telemetry(s,now);pose(s,now);s.observe_actuator(actuator(now),now)
        row,commander=audit(now,2)
        if change=='bad_height':row['height_aid']['fused']=False
        if change=='rejected_yaw':row['yaw_aid']['innovation_rejected']=True
        if change=='commander_fail':commander['pre_flight_checks_pass']=False
        if change=='velocity_invalid':row['local']['v_z_valid']=False
        if change=='missing_status':del row['status']['timestamp']
        if change=='selector_unhealthy':row['selector']['healthy'][1]=False
        try:s.observe_audit(row,commander,requested_at=now-.01,now=now)
        except (KeyError,ValueError):s.fail('rejected_audit')
    assert s.build(now)[0] is None


def test_identity_requires_same_session_request_and_expected_fc_source():
    s=p.LiveMissionInputs('r');e=m.MAVLink(None)
    msg=identity(p.LiveMissionInputs('other'))
    s.ingest(packet(msg),0)
    assert s.identity is None
    s.request_identity(0);s.ingest(packet(msg,component=88),0)
    assert s.identity is None
    s.ingest(packet(msg),.1)
    assert s.identity==(p.UID2,'1.14.3')


def test_wrong_identity_latches_no_recovery_on_later_correct_packet():
    s=current();s.request_identity(2.5)
    msg=identity(p.LiveMissionInputs('r'))
    msg.uid2=bytes(18);s.ingest(packet(msg),2.5)
    identity(s,2.6)
    assert s.fault=='fc_identity_changed_or_wrong' and s.build(2.6)[0] is None


def test_duplicate_fc_packet_and_crc_failure_do_not_refresh_local_state():
    s=current();last=s.local['at']
    msg=m.MAVLink(None).local_position_ned_encode(12400,1,2,3,0,0,0)
    s.ingest(packet(msg),2.8)
    bad=NS(get_type=lambda:'BAD_DATA')
    s.ingest(bad,2.9)
    assert s.local['at']==last and s.bad_frames==1
    assert 'local_position_missing_or_stale' in s.build(3.)[1]


def test_out_of_order_boot_latches_and_does_not_accept_new_identity_as_reset():
    s=current();telemetry(s,2.5,boot=1)
    assert s.fault=='fc_boot_clock_regressed_or_out_of_order'
    assert s.build(2.5)[0] is None


@pytest.mark.parametrize('case',['future','old','repeat'])
def test_pose_must_advance_and_pass_both_clocks(case):
    s=current();before=s.pose_at
    sample=NS(stamp_ns=int(102.4e9),received_monotonic_ns=int(2.4e9))
    ros=int((102.3 if case=='future' else 102.7 if case=='old' else 102.4)*1e9)
    s.observe_pose(sample,ros,int(2.5e9),2.5)
    assert s.pose_at==before


def test_fd_presence_does_not_prove_awake_and_reenumeration_is_not_auto_adopted():
    s=p.LiveMissionInputs('r')
    s.observe_gamepad('fd1',True,0)
    assert 'gamepad_current_input_not_observed' in s.build(0)[1]
    s.observe_gamepad('fd2',True,.1,event=True)
    assert s.fault=='gamepad_generation_changed'


def test_home_ack_and_early_snapshot_are_not_release_lock_confirmation():
    s=p.LiveMissionInputs('r');identity(s);telemetry(s,0);home(s)
    telemetry(s,.4);s.observe_actuator(actuator(.4),.4)
    assert not s.home_proven
    telemetry(s,2);s.observe_actuator(actuator(2),2)
    assert s.home_proven


def test_wrong_ack_target_cannot_supply_home_proof():
    s=p.LiveMissionInputs('r');identity(s);telemetry(s,0)
    s.observe_gamepad('fd1',True,.1,event=True,home=True)
    s.observe_gamepad('fd1',True,.2,event=True,home=False)
    msg=m.MAVLink(None).command_ack_encode(185,0,target_system=250,target_component=220)
    s.ingest(packet(msg),.3)
    telemetry(s,2);s.observe_actuator(actuator(2),2)
    assert not s.home_proven


@pytest.mark.parametrize('case',['repeat','old','future','missing_counter','fake_bool'])
def test_audit_cannot_relabel_cached_data_as_current(case):
    s=current();telemetry(s,2.5)
    row,commander=audit(2.5,2)
    if case=='repeat':row=deepcopy(s.audit_rows[-1])
    if case=='old':row['aid']['timestamp']=1
    if case=='future':row['selector']['timestamp']=99_000_000
    if case=='missing_counter':del row['local']['xy_reset_counter']
    if case=='fake_bool':commander['pre_flight_checks_pass']=1
    with pytest.raises((ValueError,KeyError)):
        s.observe_audit(row,commander,requested_at=2.49,now=2.5)


def test_malformed_input_time_and_nonfinite_data_fail_closed():
    s=current()
    with pytest.raises(ValueError):s.build(2.3)
    assert s.fault=='session_clock_invalid'
    s=p.LiveMissionInputs('r')
    msg=m.MAVLink(None).local_position_ned_encode(100,math_nan(),0,0,0,0,0)
    s.ingest(packet(msg),0)
    assert s.fault=='non_finite_local_telemetry'


def math_nan():
    return float('nan')


@pytest.mark.parametrize('delta,accepted',[(5.01,True),(9.99,True),(10.01,False),(90.,False)])
def test_native_demo_yaw_tolerance_keeps_large_misalignment_blocked(delta,accepted):
    s=native_current_status()
    s.audit_rows[-1]['yaw_comparison']['fc_minus_vicon_wrapped_deg']=delta
    assert (s.build(2.4)[0] is not None) is accepted
    legacy=current()
    legacy.audit_rows[-1]['yaw_comparison']['fc_minus_vicon_wrapped_deg']=delta
    assert legacy.build(2.4)[0] is None


@pytest.mark.parametrize('kind',['LOCAL_POSITION_NED','RC_CHANNELS'])
def test_isolated_reordered_native_frame_does_not_refresh_either_reducer(kind):
    from rc_watch import NativeRCMonitor
    s=native_current_status();s.native_rc=NativeRCMonitor();e=m.MAVLink(None)
    def frame(stamp):
        if kind=='LOCAL_POSITION_NED':return packet(e.local_position_ned_encode(stamp,9,8,7,0,0,0))
        return packet(e.rc_channels_encode(stamp,8,*([1500]*18),100))
    s.ingest(frame(12410),2.41)
    before=deepcopy(s.local if kind=='LOCAL_POSITION_NED' else s.native_rc.channels)
    s.ingest(frame(12310),2.42)
    assert s.fault is None and len(s.reordered_frames)==1
    assert (s.local if kind=='LOCAL_POSITION_NED' else s.native_rc.channels)==before
    s.ingest(frame(12500),2.5)
    assert s.fault is None and s.last_boot_by_kind[kind]==12500


@pytest.mark.parametrize('persistent',[False,True])
def test_large_or_persistent_boot_regression_is_not_treated_as_reordering(persistent):
    s=native_current_status();e=m.MAVLink(None)
    stamp=12300 if persistent else 100
    s.ingest(packet(e.local_position_ned_encode(stamp,0,0,0,0,0,0)),2.41)
    if persistent:
        assert s.fault is None
        s.ingest(packet(e.local_position_ned_encode(stamp,0,0,0,0,0,0)),2.671)
    assert s.fault=='fc_boot_clock_regressed_or_out_of_order'
