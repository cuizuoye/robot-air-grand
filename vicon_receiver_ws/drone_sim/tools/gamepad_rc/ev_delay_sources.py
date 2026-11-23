"""B3 passive source adapters: no ROS executor, serial owner or transmitter.

Callbacks run in the EXISTING single owner. Atomic metadata may arrive AFTER
the ordinary pose was sent: join by exact source header, never nearest topic.
Both caches are bounded. ULog import occurs AFTER acquisition; its import time
is not an IMU/visual measurement time, nor a radio arrival timestamp.
"""
from collections import OrderedDict
from copy import deepcopy
import math
import hashlib
from pathlib import Path
import stat
import re

from ev_delay_capture import _integer, _finite, _quat, pose_route
from ekf_ground_acceptance import result


def _stamp(stamp):
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def _pose(pose):
    return [pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]


def frame_sample_fields(msg):
    """Typed vicon_receiver/FrameSample -> plain record, no topic guessing."""
    return dict(sequence=int(msg.sequence), frame=int(msg.sdk_frame), sdk_hz=float(msg.sdk_hz),
        sdk_ok=bool(msg.sdk_ok), occluded=bool(msg.occluded),
        pose_published=bool(msg.pose_published), epoch=int(msg.sdk_epoch),
        missing_frames=int(msg.missing_frames), queue_dropped=int(msg.queue_dropped),
        sdk_begin_ns=int(msg.sdk_begin_monotonic_ns), sdk_end_ns=int(msg.sdk_end_monotonic_ns),
        ros_received_ns=_stamp(msg.ros_received_stamp), ros_stamp_ns=_stamp(msg.source_header.stamp),
        frame_id=msg.source_header.frame_id, subject=msg.subject, segment=msg.segment,
        sdk_pose=_pose(msg.sdk_pose), world_pose=_pose(msg.world_pose),
        exposure_time_known=bool(msg.exposure_time_known))


class AtomicViconAdapter:
    """Bounded RAM observer; use source world_pose for existing ENU->NED transform.

    Raw SDK pose is also retained. Do NOT pass raw SDK coordinates as world
    coordinates when map_xyz/map_rpy is nonzero. No synchronous file writes.
    """
    def __init__(self, recorder, *, subject='MircoAir', segment='MircoAir', capacity=512):
        if type(capacity) is not int or not 1 <= capacity <= 2048:
            raise ValueError('bounded_atomic_cache_required')
        self.recorder, self.subject, self.segment = recorder, subject, segment
        self.route = pose_route(recorder.metadata)
        self.capacity = capacity
        self.sources, self.pending, self.sent_keys = OrderedDict(), OrderedDict(), set()
        self.previous = None
        self.missing_frames = 0
        self.delivery_policy = recorder.metadata.get('vicon_delivery_policy','capture_history')
        if self.delivery_policy not in ('capture_history','latest_only'):
            raise ValueError('unknown_vicon_delivery_policy')
        self.unrecorded_source_samples = 0
        self.fault = None

    def fail(self, reason):
        self.fault = self.fault or reason
        self.recorder.fault = self.recorder.fault or self.fault
        return False

    def observe(self, sample, *, callback_ns):
        if self.fault:
            return False
        if self.recorder.fault:
            return self.fail(self.recorder.fault)
        try:
            sample = deepcopy(sample)
            if (sample['subject'], sample['segment']) != (self.subject, self.segment):
                raise ValueError('unexpected_rigid_body')
            for k in ('sequence','frame','epoch','missing_frames','queue_dropped',
                      'sdk_begin_ns','sdk_end_ns','ros_received_ns','ros_stamp_ns'):
                _integer(sample[k], k)
            if not (0 < sample['sdk_begin_ns'] <= sample['sdk_end_ns'] <= callback_ns):
                raise ValueError('SDK_and_callback_must_share_MONOTONIC_domain')
            if (sample['sdk_ok'] is not True or sample['occluded'] is not False
                    or sample['pose_published'] is not True or sample['queue_dropped']):
                raise ValueError('invalid_occluded_unpublished_or_dropped_atomic_frame')
            if sample['exposure_time_known'] is not False:
                raise ValueError('unverified_exposure_clock_claim')
            if not 1 <= _finite(sample['sdk_hz'], 'SDK_rate') <= 1000:
                raise ValueError('SDK_rate_outside_range')
            key = (sample['frame_id'], sample['ros_stamp_ns'])
            if sample['ros_stamp_ns'] <= 0 or not sample['frame_id'] or key in self.sources:
                raise ValueError('invalid_or_duplicate_exact_header')
            if self.previous:
                prev = self.previous
                sequence_delta=sample['sequence']-prev['sequence']
                if (sequence_delta<1 or (self.delivery_policy=='capture_history' and sequence_delta!=1)
                        or sample['epoch'] != prev['epoch']
                        or sample['frame'] <= prev['frame'] or sample['sdk_hz'] != prev['sdk_hz']
                        or sample['ros_stamp_ns'] <= prev['ros_stamp_ns']):
                    raise ValueError('sidechannel_loss_or_SDK_ROS_epoch_rate_changed')
                gap = sample['frame']-prev['frame']-sequence_delta
                if gap<0 or sample['missing_frames']-prev['missing_frames'] != gap:
                    raise ValueError('SDK_gap_counter_inconsistent')
                self.missing_frames += gap # cadence gate decides if interval remains usable
                self.unrecorded_source_samples += sequence_delta-1
            for pose in (sample['sdk_pose'], sample['world_pose']):
                if len(pose)!=7 or not all(math.isfinite(v) for v in pose):
                    raise ValueError('invalid_atomic_pose')
                _quat([pose[6], *pose[3:6]])
            if len(self.sources) >= self.capacity:
                self.sources.popitem(last=False)
            self.sources[key]=sample
            self.previous=sample
            p,q=sample['world_pose'][:3],sample['world_pose'][3:]
            if not self.recorder.record('calibration_vicon', at_ns=callback_ns,
                    frame=sample['frame'],ros_stamp_ns=key[1],sdk_hz=sample['sdk_hz'],
                    qw=q[3],qx=q[0],qy=q[1],qz=q[2],px=p[0],py=p[1],pz=p[2],
                    occluded=False,source_observed_ns=sample['sdk_end_ns'],
                    sdk_begin_ns=sample['sdk_begin_ns'],ros_received_ns=sample['ros_received_ns'],
                    sequence=sample['sequence'],epoch=sample['epoch'],missing_frames=sample['missing_frames']):
                raise ValueError('recorder_overflow')
            raw=sample['sdk_pose']
            if not self.recorder.record('calibration_sdk',at_ns=callback_ns,frame=sample['frame'],
                ros_stamp_ns=key[1],frame_id=sample['frame_id'],px=raw[0],py=raw[1],pz=raw[2],
                qx=raw[3],qy=raw[4],qz=raw[5],qw=raw[6],exposure_time_known=False):
                raise ValueError('recorder_overflow')
            if key in self.pending:
                self._join(key,self.pending.pop(key),callback_ns)
            return self.fault is None
        except (ValueError,KeyError,TypeError) as exc:
            return self.fail(str(exc))

    def observe_sent(self, message, *, source_stamp_ns, source_frame_id,
                     tx_start_ns, tx_end_ns, callback_ns, route='trs'):
        """Observe owner's successful write. NEVER packs or sends anything.

        message must be decoded from the EXACT float32 wire payload, not an
        unsent high-precision quaternion. Original sender is unchanged.
        """
        if self.fault:
            return False
        if self.recorder.fault:
            return self.fail(self.recorder.fault)
        try:
            if message.get_type()!='VISION_POSITION_ESTIMATE' or route!=self.route:
                raise ValueError('original_bound_transport_VPE_only')
            if not (0<tx_start_ns<=tx_end_ns<=callback_ns):
                raise ValueError('send_observation_time_order')
            key=(source_frame_id,source_stamp_ns)
            if key in self.sent_keys or key in self.pending:
                raise ValueError('repeated_source_send')
            if len(self.sent_keys)>=1000 or len(self.pending)>=self.capacity:
                raise ValueError('send_join_cache_overflow')
            r,p,y=message.roll/2,message.pitch/2,message.yaw/2
            cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
            row=dict(px=message.x,py=message.y,pz=message.z,
                qw=cr*cp*cy+sr*sp*sy,qx=sr*cp*cy-cr*sp*sy,
                qy=cr*sp*cy+sr*cp*sy,qz=cr*cp*sy-sr*sp*cy,
                sent_ns=tx_end_ns,tx_start_ns=tx_start_ns,wire_usec=int(message.usec),route=route)
            if not all(math.isfinite(v) for v in row.values() if type(v) is float):
                raise ValueError('nonfinite_sent_pose')
            if key in self.sources:
                self._join(key,row,callback_ns)
            else:
                self.pending[key]=row
            return self.fault is None
        except (ValueError,KeyError,TypeError,AttributeError) as exc:
            return self.fail(str(exc))

    def _join(self, key, row, now_ns):
        source=self.sources[key]
        if row['tx_start_ns']<source['sdk_end_ns']:
            self.fail('send_precedes_atomic_frame')
            return
        self.sent_keys.add(key)
        if not self.recorder.record('calibration_send',at_ns=now_ns,frame=source['frame'],
                                   ros_stamp_ns=key[1],**row):
            self.fail('recorder_overflow')

    def finish(self):
        if self.recorder.fault:
            self.fail(self.recorder.fault)
        if self.pending:
            self.fail('sent_pose_missing_exact_atomic_frame')
        return result(complete=self.fault is None,reasons=[self.fault] if self.fault else [],
                      missing_sdk_frames=self.missing_frames,matched_sends=len(self.sent_keys),
                      delivery_policy=self.delivery_policy,unrecorded_source_samples=self.unrecorded_source_samples)


def _selection_brackets(evidence, *, boot_session, expected_info, artifact, lo, hi):
    """Review explicit cached-topic evidence; NEVER synthesize a ULog row.

    Same IDs alone cannot exclude switching away and back. The native topic
    publication timestamp must also be IDENTICAL in bracketing same-boot
    reads. Source/boot/clock declarations still need independent human review;
    a self-consistent JSON document is not hardware authorization.
    """
    if (not isinstance(evidence, dict) or type(evidence.get('schema')) is not int
            or evidence.get('schema') != 1
            or evidence.get('scope') != 'same-boot-readonly-sensor-selection'
            or evidence.get('review_status') != 'reviewed_offline'
            or evidence.get('fc_boot_session') != boot_session
            or evidence.get('identity') != expected_info):
        raise ValueError('selection_evidence_review_identity_or_boot_mismatch')
    for key in ('reviewer', 'boot_binding_evidence', 'fc_clock_bounds_evidence',
                'selection_publish_semantics_evidence'):
        if not isinstance(evidence.get(key), str) or not evidence[key].strip():
            raise ValueError('selection_evidence_missing:' + key)
    digest = evidence.get('ulog_sha256')
    if (not isinstance(digest, str) or re.fullmatch('[0-9a-f]{64}', digest) is None
            or not isinstance(artifact, dict) or artifact.get('sha256') != digest):
        raise ValueError('selection_evidence_ULog_hash_binding_required')
    parsed = []
    for name in ('before', 'after'):
        row = evidence.get(name)
        if (not isinstance(row, dict) or row.get('route') != 'usb_readonly_serial_control'
                or type(row.get('system_id')) is not int or type(row.get('component_id')) is not int
                or (row.get('system_id'), row.get('component_id')) != (1, 1)
                or row.get('command') != 'listener sensor_selection -n 1'
                or row.get('crc_validated') is not True):
            raise ValueError('selection_evidence_readonly_source_required:' + name)
        text = row.get('raw_reply')
        if (not isinstance(text, str) or not 1 <= len(text.encode('utf-8')) <= 16384
                or hashlib.sha256(text.encode('utf-8')).hexdigest() != row.get('reply_sha256')):
            raise ValueError('selection_evidence_reply_hash_or_size:' + name)
        plain = re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]', '', text)
        topics = re.findall(r'^\s*TOPIC:\s*(\w+)\s*$', plain, re.M)
        if topics != ['sensor_selection']:
            raise ValueError('selection_evidence_exact_topic_required:' + name)
        native = {}
        for key in ('timestamp', 'gyro_device_id', 'accel_device_id'):
            values = re.findall(r'^\s*' + key + r':\s*([0-9]+)(?=[ \t\r]|$)[^\n]*$', plain, re.M)
            if len(values) != 1:
                raise ValueError('selection_evidence_native_field:' + name + ':' + key)
            native[key] = int(values[0])
            maximum = 2**64 if key == 'timestamp' else 2**32
            if not 0 < native[key] < maximum:
                raise ValueError('selection_evidence_native_range:' + key)
        for key in ('host_received_ns', 'read_fc_lower_us', 'read_fc_upper_us'):
            if type(row.get(key)) is not int or row[key] <= 0:
                raise ValueError('selection_evidence_original_time_required:' + key)
        if not native['timestamp'] <= row['read_fc_lower_us'] <= row['read_fc_upper_us']:
            raise ValueError('selection_evidence_native_time_or_clock_bounds')
        parsed.append(dict(native=native, **{k: row[k] for k in
            ('host_received_ns', 'read_fc_lower_us', 'read_fc_upper_us', 'reply_sha256')}))
    before, after = parsed
    if (before['host_received_ns'] >= after['host_received_ns']
            or before['read_fc_upper_us'] > lo or after['read_fc_lower_us'] < hi):
        raise ValueError('selection_evidence_does_not_bracket_original_FC_window')
    if before['native'] != after['native']:
        raise ValueError('selection_publication_or_ID_changed_during_window')
    return before['native'], parsed


def ulog_events(ulog, *, boot_session, expected_info, import_ns, source='vehicle_imu',
                selection_evidence=None, visual_not_before_us=None, physical_conditions=None):
    """pyulog or mock -> native timestamped records; no fallback to EKF attitude.

    expected_info is independently verified sys_uuid/ver_sw/ver_hw. Board/build
    identity alone does NOT identify a boot: also require independently reviewed
    capture-window/FC-clock binding in boot_session and capture metadata.
    """
    if source not in ('vehicle_imu','sensor_combined') or not boot_session:
        raise ValueError('raw_IMU_source_and_boot_binding_required')
    if set(expected_info)!={'sys_uuid','ver_sw','ver_hw'} or any(
            not expected_info[k] or ulog.msg_info_dict.get(k)!=expected_info[k] for k in expected_info):
        raise ValueError('ULog_identity_missing_or_mismatch')
    if ulog.dropouts:
        raise ValueError('ULog_dropout')
    datasets={}
    for ds in ulog.data_list:
        datasets.setdefault(ds.name,[]).append(ds)

    def rows(name):
        for ds in datasets.get(name,[]):
            data=ds.data
            n=len(data['timestamp'])
            if any(len(v)!=n for v in data.values()):
                raise ValueError('ULog_field_length_mismatch')
            previous=-1
            for i in range(n):
                row={k:(v[i].item() if hasattr(v[i],'item') else v[i]) for k,v in data.items()}
                if row['timestamp']<=previous:
                    raise ValueError('ULog_topic_clock_regression')
                previous=row['timestamp']
                yield row

    visuals=list(rows('vehicle_visual_odometry'))
    visual_total = len(visuals)
    if visual_not_before_us is not None:
        boundary = _integer(visual_not_before_us, 'visual_motion_lower_bound_us', 1)
        # Conservative FC-clock lower bound: retain uncertain boundary rows
        # and EVERY interior row. Never select only successful pose matches.
        visuals = [r for r in visuals if r['timestamp_sample'] >= boundary]
        if visual_total-len(visuals) < 3:
            raise ValueError('visual_preroll_not_observed_in_ULog')
    if len(visuals)<40:
        raise ValueError('insufficient_actual_visual_observations')
    lo=min(r['timestamp_sample'] for r in visuals)-500_000
    hi=max(r['timestamp_sample'] for r in visuals)+100_000
    selection=sorted(rows('sensor_selection'),key=lambda r:r['timestamp'])
    external_selection = None
    selection_origin = 'native_ULog'
    if selection_evidence is not None:
        cached, brackets = _selection_brackets(selection_evidence, boot_session=boot_session,
            expected_info=expected_info, artifact=getattr(ulog, '_capture_provenance', None), lo=lo, hi=hi)
        # Never let a cached snapshot conceal native contrary evidence.
        relevant = [r for r in selection if lo <= r['timestamp'] <= hi]
        prior = [r for r in selection if r['timestamp'] <= lo]
        if prior:
            relevant.append(prior[-1])
        if any(any(r.get(k) != cached[k] for k in cached) for r in relevant):
            raise ValueError('cached_selection_conflicts_with_native_ULog')
        external_selection = dict(evidence=deepcopy(selection_evidence), brackets=brackets,
                                  native_topic_missing=not bool(selection),
                                  not_injected_into_ULog=True)
        if not prior:
            # This list is for evidence assessment only; ulog.data_list and
            # exported calibration_ulog sensor_selection rows stay untouched.
            selection = [cached] + selection
            selection_origin = 'reviewed_readonly_cache_brackets'
    if not selection or selection[0]['timestamp']>lo:
        raise ValueError('missing_pre_window_sensor_selection')
    active_ids={r['gyro_device_id'] for r in selection if lo<=r['timestamp']<=hi}
    active_ids.add([r for r in selection if r['timestamp']<=lo][-1]['gyro_device_id'])
    if len(active_ids)!=1 or 0 in active_ids:
        raise ValueError('IMU_sensor_switch_or_unknown_ID')
    sensor_id=active_ids.pop()
    events=[]
    for r in visuals:
        events.append(dict(kind='calibration_visual',sample_us=int(r['timestamp_sample']),
            published_us=int(r['timestamp']),pose_frame=int(r['pose_frame']),
            reset_counter=int(r['reset_counter']),quality=int(r['quality']),route='ulog',
            **{k:float(r[f'position[{i}]']) for i,k in enumerate(('px','py','pz'))},
            **{k:float(r[f'q[{i}]']) for i,k in enumerate(('qw','qx','qy','qz'))}))
    raw=[]
    for r in rows(source):
        sample=int(r['timestamp_sample'] if source=='vehicle_imu' else r['timestamp'])
        if not lo<=sample<=hi:
            continue
        if source=='vehicle_imu' and r['gyro_device_id']!=sensor_id:
            continue # do not confuse another physical instance with active gyro
        dt=int(r['delta_angle_dt'] if source=='vehicle_imu' else r['gyro_integral_dt'])
        clip=int(r['delta_angle_clipping'] if source=='vehicle_imu' else r['gyro_clipping'])
        if not 0<dt<=20000 or clip:
            raise ValueError('IMU_clipped_or_invalid_integration')
        fields=('dx','dy','dz') if source=='vehicle_imu' else ('gx','gy','gz')
        prefix='delta_angle' if source=='vehicle_imu' else 'gyro_rad'
        raw.append(dict(kind='calibration_imu',source=source,sample_us=sample,
            integration_us=dt,sensor_id=int(sensor_id),clipping=clip,
            published_us=int(r['timestamp']),gyro_calibration_count=int(r['gyro_calibration_count']),
            **{k:float(r[f'{prefix}[{i}]']) for i,k in enumerate(fields)}))
    # Never sort/repair the chosen IMU stream; regression/duplicate instance fails.
    gaps=[b['sample_us']-a['sample_us'] for a,b in zip(raw,raw[1:])]
    if len(raw)<40 or not gaps or min(gaps)<=0 or max(gaps)>20000:
        raise ValueError('IMU_cadence_below_50Hz_or_timestamp_regression')
    if len({r['gyro_calibration_count'] for r in raw})!=1:
        raise ValueError('IMU_calibration_changed')
    if raw[0]['sample_us']>lo+500000 or raw[-1]['sample_us']<hi-100000:
        raise ValueError('IMU_does_not_cover_visual_window')
    status=list(rows('vehicle_imu_status'))
    health=[r for r in status if lo<=r['timestamp']<=hi and r['gyro_device_id']==sensor_id]
    if not health:
        raise ValueError('missing_IMU_health_records')
    for key in ('gyro_error_count','gyro_clipping[0]','gyro_clipping[1]','gyro_clipping[2]'):
        if len({r[key] for r in health})!=1:
            raise ValueError('IMU_health_error_or_clipping_increment')
    armed=list(rows('actuator_armed'))
    before=[r for r in armed if r['timestamp']<=lo+500000]
    if not before:
        raise ValueError('missing_ULog_disarmed_lock_evidence')
    window_state=[before[-1]]+[r for r in armed if lo+500000<r['timestamp']<=hi]
    isolated=bool(isinstance(physical_conditions,dict)
        and all(physical_conditions.get(k) is True for k in
            ('battery_disconnected','usb_only','motor_power_isolated','independent_cutoff_ready','scope_manually_approved'))
        and (physical_conditions.get('props_removed'),physical_conditions.get('props_installed'))
            in ((True,False),(False,True)))
    if physical_conditions is not None and not isolated:
        raise ValueError('incomplete_operator_motor_isolation_evidence')
    if any(r['armed'] or (not isolated and not r['lockdown']) for r in window_state):
        raise ValueError('ULog_not_disarmed_locked_during_capture')
    events.extend(raw)
    for topic in ('sensor_selection','vehicle_imu_status','timesync_status','vehicle_status','actuator_armed'):
        for r in rows(topic):
            if not lo<=r['timestamp']<=hi:
                continue
            fields=[(k,v) for k,v in r.items() if type(v) in (int,float,bool) and math.isfinite(v)]
            for part in range(0,len(fields),20):
                events.append(dict(kind='calibration_ulog',topic=topic,part=part//20,
                    **dict(fields[part:part+20])))
    events.sort(key=lambda e:e.get('published_us',e.get('timestamp',0)))
    for i,e in enumerate(events):
        e['at_ns']=_integer(import_ns,'import_ns',1)+i # import order, NOT FC arrival
    return result(events=events,raw_imu_source=source,sensor_id=sensor_id,
        visual_log_total=visual_total, visual_preroll_excluded=visual_total-len(visuals),
        visual_not_before_us=visual_not_before_us,
        native_boot_binding=boot_session,identity=deepcopy(expected_info),
        output_lock_proven=all(r['lockdown'] for r in window_state),
        motor_isolation_evidence=deepcopy(physical_conditions),
        motor_isolation_source='operator_statement_not_telemetry' if isolated else None,
        ulog_artifact=deepcopy(getattr(ulog,'_capture_provenance',None)),
        imu_samples=len(raw),visual_samples=len(visuals),max_imu_gap_us=max(gaps),
        median_imu_gap_us=sorted(gaps)[len(gaps)//2],import_time_is_measurement_time=False,
        sensor_selection_evidence=(
            'ULog sensor_selection native timestamp, unique active gyro in window'
            if selection_origin == 'native_ULog' else
            'Reviewed same-boot USB cached sensor_selection: identical native publication timestamp/IDs before and after original FC window; original ULog unmodified'),
        selection_origin=selection_origin, external_selection_evidence=external_selection)


def load_ulog_file(path):
    """Explicit regular .ulg files only; never devices, sockets or live ports."""
    path=Path(path).resolve(strict=True)
    info=path.stat()
    if not stat.S_ISREG(info.st_mode) or path.suffix!='.ulg' or not 0<info.st_size<=512*1024*1024:
        raise ValueError('bounded_regular_ULog_file_required')
    from pyulog import ULog
    parsed=ULog(str(path))
    with path.open('rb') as stream:
        digest=hashlib.file_digest(stream,'sha256').hexdigest()
    after=path.stat()
    if (info.st_size,info.st_mtime_ns)!=(after.st_size,after.st_mtime_ns):
        raise ValueError('ULog_changed_during_offline_import')
    parsed._capture_provenance=dict(path=str(path),size=info.st_size,sha256=digest)
    return parsed


def merge_ulog_capture(document, ulog, *, expected_info, import_ns, source='sensor_combined',
                       selection_evidence=None, native_fc_time=None):
    """Offline merge; never infer identity/boot or change old observation stamps.

    Write the returned capture only after collection. All original ULog remains
    an evidence artifact. A short topic/identity string is NOT proof by itself.
    """
    from ev_delay_capture import DelayRecorder
    meta=document['capture_metadata']
    if not meta.get('shared_fc_clock_evidence') or not meta.get('ulog_boot_binding_evidence'):
        raise ValueError('independent_ULog_same_boot_window_binding_required')
    if document.get('recorder_fault') or document.get('dropped_events'):
        raise ValueError('capture_fault_or_overflow')
    existing=document['events']
    if any(e['kind'] in ('calibration_imu','calibration_visual','calibration_ulog') for e in existing):
        raise ValueError('do_not_mix_preexisting_FC_data_or_import_twice')
    if existing and import_ns<existing[-1]['at_ns']:
        raise ValueError('import_clock_precedes_capture_end')
    boundary = None
    preroll = meta.get('visual_preroll')
    if preroll is not None:
        from types import SimpleNamespace
        from frozen_target_stream import clock_bounds_us
        from lock_clock_probe import LockClockProbe
        window = meta.get('motion_window', {})
        bounds = preroll.get('motion_start_fc_bounds_us', [])
        if (preroll.get('scope') != 'stationary-before-GO-v1'
                or preroll.get('received_confirmed') is not True
                or preroll.get('received', 0) < 3
                or len(bounds) != 2 or any(type(v) is not int for v in bounds)
                or not 0 < bounds[0] <= bounds[1] <= bounds[0]+200_000
                or preroll.get('motion_start_ns') != window.get('started_ns')
                or not all(type(preroll.get(k)) is int for k in ('started_ns','ended_ns','last_sent_ns'))
                or not preroll['started_ns'] <= preroll['last_sent_ns'] <= preroll['ended_ns']
                or type(window.get('go_ns')) is not int or type(window.get('started_ns')) is not int
                or not preroll['ended_ns'] <= window['go_ns'] <= window['started_ns']-3_000_000_000):
            raise ValueError('invalid_stationary_preroll_boundary')
        # Recompute the conservative boundary from the ORIGINAL request/reply
        # reference, not an arbitrary cutoff chosen after seeing bad samples.
        ref = preroll.get('motion_start_clock_reference', {})
        if (meta.get('timestamp_mode') != 'fc_receive_time'
                or any(type(ref.get(k)) is not int for k in
                       ('received_ns','remote_ns','lower_ns','upper_ns','rtt_ns'))
                or not 0 <= ref['rtt_ns'] <= LockClockProbe.MAX_RTT_NS
                or ref['lower_ns'] != ref['remote_ns']-ref['received_ns']
                or ref['upper_ns']-ref['lower_ns'] != ref['rtt_ns']):
            raise ValueError('invalid_preroll_clock_reference')
        clock = SimpleNamespace(reference=ref, fault=None,
            MAX_AGE_NS=LockClockProbe.MAX_AGE_NS, DRIFT_PPM=LockClockProbe.DRIFT_PPM)
        try:
            calculated = list(clock_bounds_us(clock, window['started_ns']))
        except RuntimeError as exc:
            raise ValueError('stale_preroll_clock_reference') from exc
        if calculated != bounds:
            raise ValueError('preroll_boundary_does_not_match_original_clock')
        boundary = bounds[0]
    imported=ulog_events(ulog,boot_session=meta['fc_boot_session'],expected_info=expected_info,
                         import_ns=import_ns,source=source,selection_evidence=selection_evidence,
                         visual_not_before_us=boundary,
                         physical_conditions=meta.get('physical_conditions') if meta.get('pose_route')=='wifi' else None)
    combined=deepcopy(document)
    combined['capture_metadata']['sensor_selection_evidence']=imported['sensor_selection_evidence']
    if imported['external_selection_evidence'] is not None:
        combined['capture_metadata']['external_sensor_selection_evidence'] = deepcopy(
            imported['external_selection_evidence'])
    if native_fc_time is None:
        native_fc_time=meta.get('requested_timing_method')=='native_fc_observation_intervals'
    if native_fc_time:
        from ev_delay_capture import fc_pairing_digest
        artifact=imported['ulog_artifact']
        if not artifact or not artifact.get('sha256'):
            raise ValueError('Original_ULog_artifact_required_for_native_pairing')
        combined['capture_metadata'].update(timing_method='native_fc_observation_intervals',
            fc_time_pairing=dict(boot_session=meta['fc_boot_session'],ulog_artifact=artifact,
                identity=deepcopy(expected_info),timestamp_domain='same_ULog_FC_boot',
                timestamp_unit='microseconds',measurement_sha256=fc_pairing_digest(
                    imported['events'],meta['fc_boot_session'])))
    recorder=DelayRecorder(document['host_clock_session'],combined['capture_metadata'])
    for event in existing+imported['events']:
        if not recorder.record(event['kind'],**{k:v for k,v in event.items() if k!='kind'}):
            raise ValueError('merged_capture_overflow')
    return recorder.export(),{k:v for k,v in imported.items() if k!='events'}
