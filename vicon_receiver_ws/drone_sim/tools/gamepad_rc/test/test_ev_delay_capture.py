"""Synthetic/offline adapters only. conftest forbids opening real devices."""
import ast
from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from ev_delay_capture import DelayRecorder, analyze, prepare, quaternion_rate
from ev_delay_calibration import calibrate
from ground_link_diagnostics import TimingRecorder
from phase2_acceptance import main


def capture(lag_ms=37, seconds=15):
    """SYNTHETIC FC-boot data, not historic or live hardware evidence."""
    host, boot = 1_000_000_000_000, 1_000_000_000
    metadata = dict(fc_boot_session='SYNTHETIC-BOOT',
        shared_fc_clock_evidence='SYNTHETIC same HRT',
        rigid_body_revision_sha256='a'*64,
        orientation_transform_evidence='SYNTHETIC FLU/FRD',
        sdk_frame_pose_evidence='SYNTHETIC atomic frame',
        sdk_rate_evidence='SYNTHETIC constant 50Hz',
        frame_pose_join='atomic_sdk_frame', timestamp_mode='fc_receive_time',
        clock_uncertainty_ns=1000, interval_uncertainty_ns=1000,
        body_orientation_in_rigid_xyzw=[0, 0, 0, 1], alignment_yaw_rad=0.)

    def angle(t):
        # Analytic multi-frequency SYNTHETIC excitation, not motion instructions.
        return .15*math.sin(7*t)+.07*math.sin(17*t)+.03*math.sin(23*t)

    def attitude(t):
        # Rz(.05*t) Rx(angle(t)); a second component makes source matches unique.
        c,s=math.cos(.025*t),math.sin(.025*t)
        x,y=math.cos(angle(t)/2),math.sin(angle(t)/2)
        return [c*x,c*y,s*y,s*x]

    events = []
    for i in range(seconds*50):
        t = i*.02; physical_ns = i*20_000_000
        theta = angle(t); frame = 100+i; stamp = 1_800_000_000_000_000_000+physical_ns
        w,x,y,z=attitude(t)
        # FLU/FRD conjugation flips the Y/Z components of the raw quaternion.
        events.append(dict(kind='calibration_vicon', at_ns=host+physical_ns+5_000_000,
            frame=frame, ros_stamp_ns=stamp, sdk_hz=50., occluded=False,
            qw=w, qx=x, qy=-y, qz=-z))
        q = dict(qw=w, qx=x, qy=y, qz=z)
        p = dict(px=-.3, py=.2, pz=-.1)
        events.append(dict(kind='calibration_send', at_ns=host+physical_ns+6_000_000,
            frame=frame, ros_stamp_ns=stamp, route='trs', **q, **p))
        sample_us = (boot+physical_ns+lag_ms*1_000_000)//1000
        events.append(dict(kind='calibration_visual',
            at_ns=host+physical_ns+lag_ms*1_000_000+10_000_000,
            sample_us=sample_us, published_us=sample_us+2, reset_counter=0,
            pose_frame=1, route='usb_diagnostic', **q, **p))
    for i in range(seconds*100):
        t=i*.01; physical_ns=i*10_000_000
        mid=t-.001
        theta=angle(mid)
        theta_dot=1.05*math.cos(7*mid)+1.19*math.cos(17*mid)+.69*math.cos(23*mid)
        events.append(dict(kind='calibration_imu', at_ns=host+physical_ns+30_000_000,
            source='vehicle_imu', sample_us=(boot+physical_ns)//1000,
            integration_us=2000, dx=theta_dot*.002,
            dy=.05*math.sin(theta)*.002, dz=.05*math.cos(theta)*.002,
            sensor_id=7, clipping=0))
    events.sort(key=lambda e:e['at_ns'])  # Test fixture only, not source timestamp repair.
    recorder=DelayRecorder('SYNTHETIC',metadata)
    for e in events:
        assert recorder.record(e['kind'], **{k:v for k,v in e.items() if k!='kind'})
    return recorder.export()


def test_interval_failure_reports_all_gaps_without_discarding_or_approving():
    doc=capture()
    visuals=[e for e in doc['events'] if e['kind']=='calibration_visual']
    omitted={id(visuals[i]) for i in (50,51,150,151)}
    doc['events']=[e for e in doc['events'] if id(e) not in omitted]
    report=prepare(doc)
    assert not report['prepared'] and not report['parameter_approved']
    assert report['reasons']==['visual_gap_over_50ms_insufficient_recording_cadence']
    diagnostic=report['timing_diagnostics']
    assert diagnostic['matched_visual_samples']==len(visuals)-4
    assert diagnostic['invalid_interval_count']==2
    assert all(r['fc_interval_ns']==60_000_000 for r in diagnostic['invalid_intervals'])
    assert report['ev_delay_parameter_candidate_ms'] is None


def no_grant(report):
    for key in ('physical_authorization','flight_authorized','control_ready','live_enabled'):
        assert report[key] is False
    assert report['devices_opened']==0
    assert not report['parameter_approved']
    assert report['ev_delay_parameter_candidate_ms'] is None


def test_quaternion_sign_and_body_rate_not_euler_wrap():
    assert quaternion_rate([1,0,0,0],[-1,0,0,0],20_000_000)==[0,0,0]
    a=math.radians(179);b=math.radians(181)
    rate=quaternion_rate([math.cos(a/2),0,0,math.sin(a/2)],
        [math.cos(b/2),0,0,math.sin(b/2)],20_000_000)
    assert rate[2]==pytest.approx(math.radians(2)/.02)


def test_positive_lag_recovered_and_holdout_never_approves_parameter():
    original=capture();before=deepcopy(original)
    report=analyze(original)
    assert report['candidate_identified'],report['reasons']
    assert report['fit']['effective_delay_candidate_ms']==pytest.approx(37,abs=1)
    assert report['independent_holdout']['effective_delay_candidate_ms']==pytest.approx(37,abs=1)
    assert report['preparation']['matched_visual_samples']>=740
    no_grant(report)
    assert original==before


def test_usb_diagnostic_arrival_delay_not_added_to_measured_trs_lag():
    doc=capture()
    for row in doc['events']:
        if row['kind']=='calibration_visual':row['at_ns']+=100_000_000
    doc['events'].sort(key=lambda r:r['at_ns'])
    report=analyze(doc)
    assert report['candidate_identified'],report['reasons']
    assert report['fit']['effective_delay_candidate_ms']==pytest.approx(37,abs=1)
    no_grant(report)


@pytest.mark.parametrize('fault',[
    'separate_ros_frame_topic','wrong_transform','clock_unknown','interval_unknown',
    'only_highres','clipping','sensor_changed','imu_clock_regression','visual_clock_regression',
    'frame_restarted','visual_reset','pose_route_usb','visual_rate_2hz','write_param',
    'overflow','untrusted_timestamp_mode','source_epoch_jump','missing_raw_pose',
])
def test_incomplete_or_unsafe_capture_is_rejected(fault):
    doc=capture();events=doc['events'];meta=doc['capture_metadata']
    def rows(kind):return [e for e in events if e['kind']=='calibration_'+kind]
    if fault=='separate_ros_frame_topic':meta['frame_pose_join']='latest_frame_topic'
    elif fault=='wrong_transform':meta['alignment_yaw_rad']=.1
    elif fault=='clock_unknown':meta['clock_uncertainty_ns']=None
    elif fault=='interval_unknown':meta['interval_uncertainty_ns']=5_000_000
    elif fault=='only_highres':
        for e in rows('imu'):e['source']='HIGHRES_IMU'
    elif fault=='clipping':rows('imu')[20]['clipping']=1
    elif fault=='sensor_changed':rows('imu')[20]['sensor_id']=8
    elif fault=='imu_clock_regression':rows('imu')[20]['sample_us']=rows('imu')[19]['sample_us']
    elif fault=='visual_clock_regression':rows('visual')[20]['sample_us']=rows('visual')[19]['sample_us']
    elif fault=='frame_restarted':rows('vicon')[20]['frame']=1
    elif fault=='visual_reset':rows('visual')[20]['reset_counter']=1
    elif fault=='pose_route_usb':rows('send')[20]['route']='usb'
    elif fault=='visual_rate_2hz':doc['events']=[e for e in events if e['kind']!='calibration_visual' or e in rows('visual')[::25]]
    elif fault=='write_param':doc['events'].append(dict(kind='calibration_tx',at_ns=events[-1]['at_ns'],message_type='PARAM_SET'))
    elif fault=='overflow':doc['dropped_events']=1
    elif fault=='untrusted_timestamp_mode':meta['timestamp_mode']='ros_epoch'
    elif fault=='source_epoch_jump':rows('vicon')[20]['ros_stamp_ns']=1
    else:doc['events']=[e for e in events if e['kind']!='calibration_vicon']
    report=prepare(doc)
    assert not report['prepared'],fault
    no_grant(report)


def test_negative_lag_and_too_short_holdout_not_silently_approved():
    report=analyze(capture(lag_ms=-10))
    assert not report['candidate_identified']
    no_grant(report)
    short=analyze(capture(seconds=6))
    assert not short['candidate_identified']
    no_grant(short)


def test_broad_peak_is_rejected_even_if_its_maximum_is_37ms():
    data=prepare(capture())['capture']
    def slow(t):return [math.sin(2.1*t),.5*math.sin(4.7*t),.3*math.sin(6.4*t)]
    for e in data['imu']:e['rate_body_rad_s']=slow(e['t_ns']/1e9)
    for e in data['vision']:e['rate_body_rad_s']=slow((e['t_ns']-37_000_000)/1e9)
    report=calibrate(data)
    assert report['effective_delay_candidate_ms']==pytest.approx(37,abs=1)
    assert not report['candidate_identified']
    assert 'broad_or_ambiguous_correlation_peak' in report['reasons']
    assert not report['parameter_approved'] and not report['physical_authorization']


def test_highres_kept_as_sensor_derived_not_fused_attitude_or_complete_raw_imu():
    recorder=DelayRecorder('SYNTHETIC',{})
    msg=SimpleNamespace(get_type=lambda:'HIGHRES_IMU',get_srcSystem=lambda:1,get_srcComponent=lambda:1,
        fields_updated=0x38,time_usec=900000,xgyro=.1,ygyro=.2,zgyro=.3)
    assert not recorder.observe_highres_imu(msg,at_ns=1,route='usb',crc_valid=False)
    assert recorder.observe_highres_imu(msg,at_ns=2,route='usb',crc_valid=True)
    row=recorder.export()['events'][0]
    assert row['sensor_derived'] and row['integration_us'] is None and row['sensor_id'] is None
    msg.get_type=lambda:'ATTITUDE_QUATERNION'
    assert not recorder.observe_highres_imu(msg,at_ns=3,route='usb',crc_valid=True)


def test_default_ulog_sensor_combined_is_real_gyro_with_original_timestamp():
    doc=capture()
    doc['capture_metadata']['sensor_selection_evidence']='SYNTHETIC stable selected sensor 7'
    for e in doc['events']:
        if e['kind']=='calibration_imu':
            e['source']='sensor_combined'
            for a,b in zip(('dx','dy','dz'),('gx','gy','gz')):
                e[b]=e.pop(a)*1e6/e['integration_us']
    report=analyze(doc)
    assert report['candidate_identified'],report['reasons']
    assert report['fit']['effective_delay_candidate_ms']==pytest.approx(37,abs=1)
    assert report['preparation']['capture']['alignment']['imu_timestamp_field']=='sensor_combined.timestamp'
    no_grant(report)
    del doc['capture_metadata']['sensor_selection_evidence']
    assert not prepare(doc)['prepared']


def test_ram_bound_and_existing_timing_kinds_remain_strict():
    recorder=DelayRecorder('SYNTHETIC',{},capacity=1)
    assert recorder.record('calibration_clock',at_ns=1,remote_ns=1)
    assert not recorder.record('calibration_clock',at_ns=2,remote_ns=2)
    assert recorder.export()['recorder_fault']=='diagnostic_capacity_exceeded'
    with pytest.raises(ValueError):TimingRecorder('x').record('calibration_clock',at_ns=1)


def test_offline_cli_refuses_device_options_and_outputs_no_approval(tmp_path,capsys):
    with pytest.raises(SystemExit):main(['delay-capture','--port','/dev/ttyACM0'])
    path=tmp_path/'synthetic.json';path.write_text(json.dumps(capture(seconds=6)))
    assert main(['delay-capture','--capture',str(path)])==2
    report=json.loads(capsys.readouterr().out)
    no_grant(report)


def test_new_module_has_no_hardware_or_send_api():
    tree=ast.parse((Path(__file__).parents[1]/'ev_delay_capture.py').read_text())
    imports=[]
    for node in ast.walk(tree):
        if isinstance(node,ast.Import):imports.extend(n.name for n in node.names)
        if isinstance(node,ast.ImportFrom):imports.append(node.module or '')
        if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute):
            assert node.func.attr not in ('recv_match','Serial','write','send','param_set_send',
                                         'command_long_send','serial_control_encode')
    assert not any(n.split('.')[0] in ('serial','rclpy','socket','subprocess','mavutil') for n in imports)
