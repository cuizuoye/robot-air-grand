"""New capture interfaces: synthetic data only; serial open forbidden by conftest."""
from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from pymavlink.dialects.v20 import common as m

from ev_delay_capture import DelayRecorder, analyze, prepare
from ev_delay_sources import (AtomicViconAdapter, frame_sample_fields, ulog_events,
                             load_ulog_file, merge_ulog_capture)
from ev_delay_maintenance import MaintenanceCapture, MaintenanceObservation, LIVE_ENTRY_ENABLED
from test_ev_delay_capture import capture
from test_short_home_proof import prepared
from trs_powered_fusion_policy import Observation
import compact_vision


def sample(sequence=1,frame=100,stamp=1000,**changes):
    row=dict(sequence=sequence,frame=frame,sdk_hz=100.,sdk_ok=True,occluded=False,
        pose_published=True,epoch=0,missing_frames=0,queue_dropped=0,
        sdk_begin_ns=10,sdk_end_ns=20,ros_received_ns=999,ros_stamp_ns=stamp,
        frame_id='map',subject='MircoAir',segment='MircoAir',
        sdk_pose=[1.,2.,3.,0.,0.,0.,1.],world_pose=[4.,5.,6.,0.,0.,0.,1.],
        exposure_time_known=False)
    row.update(changes)
    return row


def vpe():
    return NS(get_type=lambda:'VISION_POSITION_ESTIMATE',roll=0.,pitch=0.,yaw=0.,
              x=4.,y=5.,z=6.,usec=987)


def test_atomic_frame_keeps_raw_and_transformed_pose_not_latest_frame_topic():
    rec=DelayRecorder('MOCK',{})
    adapter=AtomicViconAdapter(rec)
    assert adapter.observe(sample(),callback_ns=30)
    first=rec.events[0]
    assert (first['frame'],first['ros_stamp_ns'],first['px'],first['qw'])==(100,1000,4.,1.)
    assert rec.events[1]['px']==1. and not rec.events[1]['exposure_time_known']
    assert adapter.observe_sent(vpe(),source_stamp_ns=1000,source_frame_id='map',
        tx_start_ns=25,tx_end_ns=29,callback_ns=31)
    assert rec.events[-1]['frame']==100
    assert rec.events[-1]['wire_usec']==987
    assert adapter.finish()['complete']


def test_delayed_sidechannel_joins_exact_already_sent_pose_without_retimestamping():
    rec=DelayRecorder('MOCK',{})
    adapter=AtomicViconAdapter(rec)
    assert adapter.observe_sent(vpe(),source_stamp_ns=1000,source_frame_id='map',
        tx_start_ns=25,tx_end_ns=29,callback_ns=30)
    assert not rec.events
    assert adapter.observe(sample(),callback_ns=90)
    assert rec.events[-1]['sent_ns']==29 and rec.events[-1]['at_ns']==90
    assert rec.events[0]['source_observed_ns']==20
    assert adapter.finish()['matched_sends']==1


@pytest.mark.parametrize('change',[
    dict(occluded=True),dict(sdk_ok=False),dict(pose_published=False),dict(queue_dropped=1),
    dict(subject='car01'),dict(exposure_time_known=True),dict(sdk_end_ns=31),
    dict(sdk_hz=0),dict(world_pose=[0.]*7),dict(ros_stamp_ns=0),
])
def test_invalid_atomic_frame_fails_capture_instead_of_freshening_old_pose(change):
    rec=DelayRecorder('MOCK',{})
    adapter=AtomicViconAdapter(rec)
    assert not adapter.observe(sample(**change),callback_ns=30)
    assert rec.fault
    assert not adapter.observe(sample(),callback_ns=40)
    assert not adapter.finish()['complete']


def test_sdk_frame_loss_is_counted_and_sidechannel_loss_or_restart_fails():
    rec=DelayRecorder('MOCK',{})
    adapter=AtomicViconAdapter(rec)
    assert adapter.observe(sample(),callback_ns=30)
    assert adapter.observe(sample(sequence=2,frame=102,stamp=2000,missing_frames=1),callback_ns=40)
    assert adapter.finish()['missing_sdk_frames']==1
    assert not adapter.observe(sample(sequence=4,frame=103,stamp=3000,missing_frames=1),callback_ns=50)


def test_latest_only_retains_exact_join_and_reports_unrecorded_frames():
    adapter=AtomicViconAdapter(DelayRecorder('MOCK',{'vicon_delivery_policy':'latest_only'}))
    assert adapter.observe(sample(),callback_ns=30)
    assert adapter.observe(sample(sequence=4,frame=104,stamp=2000,missing_frames=1),callback_ns=40)
    assert adapter.observe_sent(vpe(),source_stamp_ns=2000,source_frame_id='map',
        tx_start_ns=41,tx_end_ns=42,callback_ns=43)
    report=adapter.finish()
    assert report['complete'] and report['unrecorded_source_samples']==2
    assert report['missing_sdk_frames']==1 and report['matched_sends']==1
    assert not adapter.observe(sample(sequence=5,frame=105,stamp=3000,missing_frames=1,occluded=True),callback_ns=50)


@pytest.mark.parametrize('change',[dict(sequence=1),dict(frame=100),dict(missing_frames=0),dict(epoch=1)])
def test_latest_only_does_not_accept_reset_or_inconsistent_sdk_gap(change):
    adapter=AtomicViconAdapter(DelayRecorder('MOCK',{'vicon_delivery_policy':'latest_only'}))
    assert adapter.observe(sample(),callback_ns=30)
    values=dict(sequence=4,frame=104,stamp=2000,missing_frames=1);values.update(change)
    assert not adapter.observe(sample(**values),callback_ns=40)


def test_missing_exact_header_or_different_frame_id_is_never_nearest_joined():
    adapter=AtomicViconAdapter(DelayRecorder('MOCK',{}))
    assert adapter.observe(sample(),callback_ns=30)
    assert adapter.observe_sent(vpe(),source_stamp_ns=1001,source_frame_id='other',
        tx_start_ns=31,tx_end_ns=32,callback_ns=33)
    assert not adapter.finish()['complete']


def test_observers_are_ram_only_no_disk_transport_or_analysis_in_callback():
    rec=DelayRecorder('MOCK',{},capacity=1)
    adapter=AtomicViconAdapter(rec)
    # Overflow cannot block or turn the one saved sample into a valid capture.
    assert not adapter.observe(sample(),callback_ns=30)
    assert rec.fault=='diagnostic_capacity_exceeded'
    assert not prepare(rec.export())['prepared']
    assert rec.export()['dropped_events']>0
    assert not adapter.finish()['complete']


def test_typed_ROS_fields_preserve_uint64_frame_and_two_distinct_stamps():
    stamp=lambda value:NS(sec=value//10**9,nanosec=value%10**9)
    pose=lambda:NS(position=NS(x=1.,y=2.,z=3.),orientation=NS(x=0.,y=0.,z=0.,w=1.))
    row=frame_sample_fields(NS(sequence=1,sdk_frame=2**53+123,sdk_hz=100.,sdk_ok=True,
        occluded=False,pose_published=True,sdk_epoch=0,missing_frames=0,queue_dropped=0,
        sdk_begin_monotonic_ns=10,sdk_end_monotonic_ns=20,
        ros_received_stamp=stamp(1_800_000_000_000_000_001),
        source_header=NS(stamp=stamp(1_800_000_000_000_000_010),frame_id='map'),
        subject='MircoAir',segment='MircoAir',sdk_pose=pose(),world_pose=pose(),exposure_time_known=False))
    assert row['frame']==2**53+123
    assert row['ros_stamp_ns']-row['ros_received_ns']==9


def test_clock_decision_records_original_request_reply_bounds_and_late_rejection():
    from test_lock_clock_probe import ready,reply
    probe=ready()
    rec=DelayRecorder('MOCK',{})
    assert rec.observe_clock_decision(probe,at_ns=1_050_000_000)
    accepted=rec.events[-1]
    assert accepted['event']=='accepted'
    assert accepted['original_event_ns']==1_040_000_000
    assert accepted['lower_ns']==5_000_000_000-1_040_000_000
    probe.request(1_100_000_000)
    assert not probe.ingest(reply(token=1_100_000_000),1_201_000_000)
    assert rec.observe_clock_decision(probe,at_ns=1_202_000_000)
    assert rec.events[-1]['event']=='rtt_rejected'
    assert 'received_ns' not in rec.events[-1] # cannot refresh the old clock


def dataset(name,rows):
    return NS(name=name,data={key:[r[key] for r in rows] for key in rows[0]})


@pytest.mark.parametrize('fault',[None,'armed','missing_confirmation'])
def test_wifi_ulog_import_accepts_real_isolation_without_inventing_output_lock(fault):
    _,u=fake_ulog();arm=next(x.data for x in u.data_list if x.name=='actuator_armed')
    arm['lockdown'][0]=False
    physical=dict(battery_disconnected=True,usb_only=True,motor_power_isolated=True,
        independent_cutoff_ready=True,scope_manually_approved=True,props_installed=True,props_removed=False)
    if fault=='armed':arm['armed'][0]=True
    if fault=='missing_confirmation':physical.pop('motor_power_isolated')
    kwargs=dict(boot_session='MOCK BOOT+window binding',expected_info=u.msg_info_dict,
        import_ns=10**15,physical_conditions=physical)
    if fault:
        with pytest.raises(ValueError):ulog_events(u,**kwargs)
    else:
        r=ulog_events(u,**kwargs)
        assert not r['output_lock_proven'] and r['motor_isolation_source']=='operator_statement_not_telemetry'


def fake_ulog(source='vehicle_imu'):
    doc=capture()
    imu=[];visual=[]
    for e in doc['events']:
        if e['kind']=='calibration_imu':
            row=dict(timestamp=e['sample_us']+200,gyro_calibration_count=0)
            if source=='vehicle_imu':
                row.update(timestamp_sample=e['sample_us'],gyro_device_id=7,
                    delta_angle_dt=e['integration_us'],delta_angle_clipping=0,
                    **{f'delta_angle[{i}]':e[k] for i,k in enumerate(('dx','dy','dz'))})
            else:
                row.update(timestamp=e['sample_us'],gyro_integral_dt=e['integration_us'],gyro_clipping=0,
                    **{f'gyro_rad[{i}]':e[k]/.002 for i,k in enumerate(('dx','dy','dz'))})
            imu.append(row)
        elif e['kind']=='calibration_visual':
            visual.append(dict(timestamp=e['published_us'],timestamp_sample=e['sample_us'],
                reset_counter=0,quality=0,pose_frame=1,
                **{f'position[{i}]':e[k] for i,k in enumerate(('px','py','pz'))},
                **{f'q[{i}]':e[k] for i,k in enumerate(('qw','qx','qy','qz'))}))
    # Stop poses before stopping the logger: retain real IMU lead-out coverage.
    visual=visual[:-5]
    health=[dict(timestamp=t,gyro_device_id=7,gyro_error_count=0,
                 **{f'gyro_clipping[{i}]':0 for i in range(3)}) for t in range(500000,16500000,1000000)]
    info=dict(sys_uuid='MOCKUID',ver_sw='MOCKBUILD',ver_hw='MOCKBOARD')
    ulog=NS(msg_info_dict=info,dropouts=[],data_list=[dataset(source,imu),
        dataset('vehicle_visual_odometry',visual),dataset('sensor_selection',[
            dict(timestamp=1,gyro_device_id=7,accel_device_id=8)]),
        dataset('vehicle_imu_status',health),dataset('actuator_armed',[
            dict(timestamp=1,armed=False,lockdown=True)])])
    return doc,ulog


def import_fake(ulog,source='vehicle_imu'):
    return ulog_events(ulog,boot_session='MOCK BOOT+window binding',
                       expected_info=ulog.msg_info_dict,import_ns=10**15,source=source)


@pytest.mark.parametrize('source',['vehicle_imu','sensor_combined'])
def test_ulog_raw_gyro_preserves_native_timestamps_and_reuses_existing_calibrator(source):
    doc,ulog=fake_ulog(source)
    report=import_fake(ulog,source)
    assert report['max_imu_gap_us']==10000
    assert not report['flight_authorized'] and not report['import_time_is_measurement_time']
    assert report['sensor_id']==7
    doc['events']=[e for e in doc['events'] if e['kind'] not in ('calibration_imu','calibration_visual')]
    doc['events'].extend(report['events'])
    doc['capture_metadata']['sensor_selection_evidence']=report['sensor_selection_evidence']
    fit=analyze(doc)
    assert fit['candidate_identified'],fit['reasons']
    assert fit['fit']['effective_delay_candidate_ms']==pytest.approx(37,abs=1)
    assert fit['ev_delay_parameter_candidate_ms'] is None
    assert not fit['control_ready']


def test_offline_merge_requires_independent_same_boot_binding_and_keeps_old_times():
    doc,ulog=fake_ulog('sensor_combined')
    doc['events']=[e for e in doc['events'] if e['kind'] not in ('calibration_imu','calibration_visual')]
    before=deepcopy(doc)
    with pytest.raises(ValueError,match='binding_required'):
        merge_ulog_capture(doc,ulog,expected_info=ulog.msg_info_dict,import_ns=10**15)
    doc['capture_metadata']['ulog_boot_binding_evidence']='MOCK independent identity/boot/window evidence'
    merged,summary=merge_ulog_capture(doc,ulog,expected_info=ulog.msg_info_dict,import_ns=10**15)
    assert merged['events'][:len(before['events'])]==before['events']
    assert summary['raw_imu_source']=='sensor_combined'
    assert not merged['physical_authorization']
    assert analyze(merged)['candidate_identified']
    with pytest.raises(ValueError,match='import_twice'):
        merge_ulog_capture(merged,ulog,expected_info=ulog.msg_info_dict,import_ns=10**15)


@pytest.mark.parametrize('fault',[
    'EKF_attitude_substitute','low_rate','sample_regression','publication_regression',
    'sensor_switch','missing_selection','calibration_change','clipping','dropout',
    'identity','health_error','no_health','no_lock','armed','no_visual',
])
def test_ulog_missing_raw_or_untrustworthy_data_is_refused(fault):
    _,ulog=fake_ulog()
    byname={d.name:d.data for d in ulog.data_list}
    imu=byname['vehicle_imu']
    if fault=='EKF_attitude_substitute':ulog.data_list[0].name='vehicle_attitude'
    elif fault=='low_rate':
        for key in imu:imu[key]=imu[key][::50]
    elif fault=='sample_regression':imu['timestamp_sample'][20]=imu['timestamp_sample'][19]
    elif fault=='publication_regression':imu['timestamp'][20]=imu['timestamp'][19]
    elif fault=='sensor_switch':
        data=byname['sensor_selection']
        for k,v in dict(timestamp=3000000,gyro_device_id=9,accel_device_id=8).items():data[k].append(v)
    elif fault=='missing_selection':ulog.data_list=[d for d in ulog.data_list if d.name!='sensor_selection']
    elif fault=='calibration_change':imu['gyro_calibration_count'][20]=1
    elif fault=='clipping':imu['delta_angle_clipping'][20]=1
    elif fault=='dropout':ulog.dropouts=[NS(duration=1)]
    elif fault=='identity':ulog.msg_info_dict['sys_uuid']='' # cannot match an empty identity
    elif fault=='health_error':byname['vehicle_imu_status']['gyro_error_count'][5]=1
    elif fault=='no_health':ulog.data_list=[d for d in ulog.data_list if d.name!='vehicle_imu_status']
    elif fault=='no_lock':byname['actuator_armed']['lockdown'][0]=False
    elif fault=='armed':byname['actuator_armed']['armed'][0]=True
    elif fault=='no_visual':ulog.data_list=[d for d in ulog.data_list if d.name!='vehicle_visual_odometry']
    with pytest.raises(ValueError):import_fake(ulog)


def maintenance():
    home,lock=prepared()
    assert home.observe_short(lock,now_ns=1_050_000_000,post_home_ns=950_000_000,identity_verified=True)
    gate=MaintenanceCapture(home=home,lock=lock,uid='MOCK',fc_boot='BOOT',
                            gamepad_generation='PAD',created_ns=700_000_000)
    obs=MaintenanceObservation(now_ns=1_050_000_000,uid='MOCK',firmware='1.14.3',fc_boot='BOOT',
        identity_received_ns=1_040_000_000,attitude_received_ns=1_040_000_000,
        vicon_received_ns=1_040_000_000,gamepad_generation='PAD',tilt_difference_deg=1.,
        props_removed=True,independent_cutoff_ready=True,scope_manually_approved=True,
        log_ready=True,source_valid=True,config_readonly_unchanged=True,recorder_healthy=True,
        attitude_source_fresh=True)
    return gate,obs


def test_motion_scope_is_not_static_HOME_acceptance_and_never_grants_flight():
    gate,obs=maintenance()
    assert gate.begin(obs)
    msg=compact_vision.message(m.MAVLink(None),123,[0,0,0],[1,0,0,0],0)[0]
    assert gate.permits(msg,obs)
    assert not Observation(now=1.,stationary=False).healthy() # original static guard remains
    report=gate.close()
    assert report['state']=='CLOSED' and not report['flight_permit_created']
    assert report['parameter_writes']==0 and not report['flight_authorized']
    assert not LIVE_ENTRY_ENABLED
    assert not gate.permits(msg,obs)


@pytest.mark.parametrize('change',[
    dict(fc_boot='NEW'),dict(uid='OTHER'),dict(gamepad_generation='NEW'),
    dict(attitude_received_ns=550_000_000),dict(vicon_received_ns=900_000_000),
    dict(attitude_source_fresh=False),dict(log_ready=False),dict(source_valid=False),
    dict(scope_manually_approved=False),dict(props_removed=False),
    dict(recorder_healthy=False),dict(config_readonly_unchanged=False),
])
def test_maintenance_safety_failure_latches_and_never_resumes(change):
    gate,obs=maintenance()
    assert not gate.begin(replace(obs,**change))
    assert gate.state=='FAILED'
    assert not gate.begin(obs)
    assert gate.close()['parameter_writes']==0


@pytest.mark.parametrize('fault',['home_unknown','home_abort','old_home','clock_stale','armed'])
def test_maintenance_reuses_real_existing_HOME_and_lock_checks(fault):
    gate,obs=maintenance()
    if fault=='home_unknown':gate.home.verified=False
    elif fault=='home_abort':gate.home.aborted=True
    elif fault=='old_home':gate.created_ns=900_000_000
    elif fault=='clock_stale':gate.lock.clock.reference['received_ns']=1
    elif fault=='armed':gate.lock.status.sample['bits']|=2
    assert not gate.begin(obs)
    assert not gate.close()['control_ready']


@pytest.mark.parametrize('kind',['PARAM_SET','PARAM_EXT_SET','COMMAND_LONG','SET_MODE',
    'SET_POSITION_TARGET_LOCAL_NED','SET_ATTITUDE_TARGET','SERIAL_CONTROL','ODOMETRY'])
def test_any_exit_and_forbidden_message_cannot_send_parameters_modes_or_targets(kind):
    gate,obs=maintenance()
    assert gate.begin(obs)
    assert not gate.permits(NS(get_type=lambda:kind),obs)
    report=gate.close()
    assert report['state']=='FAILED' and report['parameter_writes']==0
    assert not report['live_enabled'] and report['devices_opened']==0


def test_ulog_loader_rejects_device_paths_without_opening_them(tmp_path):
    with pytest.raises(ValueError):load_ulog_file('/dev/null')
    empty=tmp_path/'empty.ulg';empty.touch()
    with pytest.raises(ValueError):load_ulog_file(empty)


def test_source_uses_atomic_getters_and_original_normal_pose_without_recent_topic_join():
    root=Path('/home/czy/vicon_receiver_ws/src/ros2-vicon-receiver/vicon_receiver')
    code=(root/'src/communicator.cpp').read_text()
    body=code.split('void Communicator::get_frame()',1)[1].split('void Communicator::publish_frame_status',1)[0]
    assert body.count('vicon_client.GetFrame()')==1
    for getter in ('GetFrameNumber','GetFrameRate','GetSegmentGlobalTranslation','GetSegmentGlobalRotationQuaternion'):
        assert 'vicon_client.'+getter in body
    assert 'row.source_ros_ns=rclcpp::Time(tf_msg.header.stamp).nanoseconds()' in body
    assert 'row.world_pose[6]=p.orientation.w; row.pose_published=true' in body
    assert '"capture_frame_samples", false' in code
    assert 'capture_queue_.push(row)' in body
    assert 'pub.publish(global_pose_msg)' in body


def test_maintenance_modules_have_no_device_encoder_or_flight_grant_dependency():
    import ast
    for name in ('ev_delay_sources.py','ev_delay_maintenance.py'):
        code=Path(name).read_text()
        tree=ast.parse(code)
        imports=[n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
        assert not any(n in ('serial','mavutil','flight_mission_core','preparation_authorization') for n in imports)
        assert 'param_set_send(' not in code and 'command_long_encode(' not in code


def test_new_logger_spec_and_offline_import_cli_cannot_enable_live_entry(capsys):
    from phase2_acceptance import main
    spec=json.loads(Path('plan_b/phase2/ev_delay_interfaces.json').read_text())
    assert spec['physical_authorization'] is False and spec['live_entry_enabled'] is False
    assert spec['ulog_preferred']['no_imu_over_radio']
    for option in ('--port','--connect','--live','--arm','--write-params'):
        with pytest.raises(SystemExit):main(['delay-ulog',option,'/dev/ttyACM0'])


def test_original_500ms_guard_survives_dynamic_capture_and_does_not_auto_recover():
    gate,obs=maintenance()
    assert gate.begin(obs)
    late=replace(obs,now_ns=1_550_000_000,identity_received_ns=1_550_000_000,
                 vicon_received_ns=1_550_000_000,attitude_received_ns=1_050_000_000)
    assert not gate.check(late) and gate.fault=='attitude_received_ns_stale'
    assert not gate.check(replace(late,attitude_received_ns=1_550_000_000))
    assert gate.close()['parameter_writes']==0


def test_existing_GCS_heartbeat_and_requested_clock_do_not_add_new_telemetry_streams():
    gate,obs=maintenance()
    assert gate.begin(obs)
    enc=m.MAVLink(None)
    hb=enc.heartbeat_encode(m.MAV_TYPE_GCS,m.MAV_AUTOPILOT_INVALID,0,0,0)
    assert gate.permits(hb,obs)
    token=gate.lock.clock.request(obs.now_ns)
    assert gate.permits(enc.timesync_encode(0,token),obs)
    assert not gate.permits(hb,obs) # unbounded heartbeat flooding refused
    assert gate.close()['state']=='FAILED'
