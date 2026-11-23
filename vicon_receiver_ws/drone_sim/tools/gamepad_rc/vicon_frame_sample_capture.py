#!/usr/bin/env python3
"""One authorized isolated Vicon FrameSample observation; NO FC/device API.

Reuse the built receiver and existing atomic adapter. Runtime records live in
bounded RAM until both observation and our child receiver have stopped. The
shared receiver is monitored, never stopped or reconfigured. No live work on
import or without the explicit one-shot authorization argument.
"""
from collections import Counter
from datetime import datetime, timezone
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
from queue import Empty, Full, Queue
import signal
import statistics
import subprocess
import threading
import time

import yaml

from ev_delay_sources import AtomicViconAdapter, frame_sample_fields
from ev_delay_capture import DelayRecorder
from vicon_receiver_provenance import (verify_current, child_environment,
    recheck_before_spawn, verify_runtime_maps)
from vicon_metadata_audit_process import MetadataAuditProcess, audit_timeout, persist_evidence

ROOT = Path(__file__).resolve().parents[2]
BUILD = Path('/home/czy/vicon_receiver_ws/drone_sim/isolated/vicon_b3_recovery_20261022_135653')
BINARY = BUILD/'install/vicon_receiver/lib/vicon_receiver/vicon_client'
SHARED_BINARY = ROOT.parent/'install/vicon_receiver/lib/vicon_receiver/vicon_client'
DOMAIN = 83
TOPIC = '/vicon_b3_check/maintenance/frame_sample'
WINDOW_NS = 15_000_000_000
CAPACITY = 20_000
RUNTIME_AUDIT_INTERVAL_NS = 100_000_000  # 原100ms核验周期不变。


class RuntimeAuditWorker:
    """One bounded transport, no ROS/SDK calls or mutable capture/report state.

    Keep the existing complete provenance inspection off the executor thread.
    There is at most ONE outstanding inspection/result, never a work backlog.
    The default timeout remains 100ms; callers may explicitly select up to 1s
    for metadata inspection. This never changes pose/FC freshness. Shutdown
    is bounded even if a filesystem operation stalls, and cannot grant a pass.
    """
    def __init__(self, inspect, *, clock=None, timeout_ns=RUNTIME_AUDIT_INTERVAL_NS):
        self.timeout_ns=audit_timeout(timeout_ns)
        self._inspect=inspect
        self._clock=clock or time.monotonic_ns
        self._jobs=Queue(maxsize=1); self._results=Queue(maxsize=1)
        self._stop=threading.Event(); self._pending=None
        self._closed=False; self._fault=None; self._completed=0
        self._thread=threading.Thread(target=self._work,name='b3_metadata_audit',daemon=True)
        self._thread.start()

    @property
    def pending(self):
        return self._pending is not None

    def submit(self, at_ns, context):
        if self._closed or self._fault or self.pending:
            raise RuntimeError('runtime_audit_closed_failed_or_already_pending')
        job=dict(submitted_ns=at_ns,deadline_ns=at_ns+self.timeout_ns,
                 context=dict(context),done=threading.Event())
        self._pending=job
        try: self._jobs.put_nowait(job)
        except Full:
            self._fault='runtime_audit_job_queue_full'
            raise RuntimeError(self._fault)

    def _work(self):
        while not self._stop.is_set():
            try: job=self._jobs.get(timeout=.01)
            except Empty: continue
            if self._stop.is_set(): return
            start=self._clock()
            try: payload=self._inspect(job['context'])
            except BaseException as exc:
                payload=dict(error=type(exc).__name__+': '+str(exc))
            result=dict(submitted_ns=job['submitted_ns'],deadline_ns=job['deadline_ns'],
                        start_ns=start,end_ns=self._clock(),context=job['context'],payload=payload)
            try: self._results.put_nowait(result)
            except Full:
                self._fault='runtime_audit_result_queue_full'
            finally: job['done'].set()

    def poll(self, at_ns):
        if not self.pending: return None
        try: result=self._results.get_nowait()
        except Empty:
            if self._fault or not self._thread.is_alive():
                self._fault=self._fault or 'runtime_audit_worker_exited'
                raise RuntimeError(self._fault)
            if at_ns>=self._pending['deadline_ns']:
                self._fault='runtime_audit_deadline_missed'
                raise RuntimeError(self._fault)
            return None
        self._pending=None; self._completed+=1
        if result['end_ns']>result['deadline_ns']:
            result['payload']['error']=result['payload'].get('error') or 'runtime_audit_deadline_missed'
        if result['payload'].get('error'): self._fault=result['payload']['error']
        return result

    def wait(self):
        """Used only BEFORE reception or AFTER its window, never in the loop."""
        if not self.pending: return None
        remaining=max(0,self._pending['deadline_ns']-self._clock())/1e9
        if not self._pending['done'].wait(remaining):
            self._fault='runtime_audit_deadline_missed'
            raise RuntimeError(self._fault)
        result=self.poll(self._clock())
        if result is None:
            self._fault='runtime_audit_completed_without_result'
            raise RuntimeError(self._fault)
        return result

    def close(self):
        self._closed=True; self._stop.set()
        deadline=time.monotonic_ns()+self.timeout_ns
        transport_ok=True
        if isinstance(self._inspect,MetadataAuditProcess):
            transport_ok=self._inspect.close(deadline_ns=deadline)
        self._thread.join(timeout=max(0,deadline-time.monotonic_ns())/1e9)
        return transport_ok and not self._thread.is_alive()

    def status(self):
        status=dict(closed=self._closed,thread_alive=self._thread.is_alive(),pending=self.pending,
                    fault=self._fault,completed=self._completed,maximum_outstanding=1,
                    interval_ns=self.timeout_ns,timeout_ns=self.timeout_ns)
        if isinstance(self._inspect,MetadataAuditProcess): status['metadata_process']=self._inspect.status()
        return status


def runtime_audit_snapshot(pid, provenance, shared_before, context, *, snapshot_path=None):
    """Existing full checks, read-only, executed by our metadata worker only."""
    payload={}
    try:
        payload['snapshot']=dict(at_monotonic_ns=time.monotonic_ns(),text=read_runtime_maps(pid))
        if snapshot_path is not None:
            persist_evidence(Path(snapshot_path).parent,Path(snapshot_path).name,
                dict(target_pid=pid,snapshot=payload['snapshot']))
        payload['classification']={}
        payload['loaded']=verify_runtime_maps(payload['snapshot']['text'],provenance,
                                              details=payload['classification'])
        if context['monitor_shared']:
            payload['shared_observation']=shared_receiver_observation(shared_before)
    except Exception as exc:
        # Preserve the actual raw map and partial rejection details on failure.
        payload['error']=type(exc).__name__+': '+str(exc)
    return payload


class _RosParameterSafeLoader(yaml.SafeLoader):
    """Local ROS-launch compatibility; never enable Python object loading."""


def _construct_ros_tuple(loader,node):
    # ROS launch may dump a numeric array with this tag. Only its sequence
    # representation is supported; construction never imports/calls objects.
    if not isinstance(node,yaml.SequenceNode):
        raise yaml.constructor.ConstructorError(None,None,
            'ROS python/tuple tag requires a sequence',node.start_mark)
    return tuple(loader.construct_sequence(node,deep=True))


_RosParameterSafeLoader.add_constructor('tag:yaml.org,2002:python/tuple',_construct_ros_tuple)


def distribution(values):
    values = sorted(values)
    return dict(count=len(values), minimum=min(values) if values else None,
                median=statistics.median(values) if values else None,
                p95=values[max(0, math.ceil(.95*len(values))-1)] if values else None,
                maximum=max(values) if values else None)


class CaptureContinuity:
    """RAM-only static recorder policy, NOT a flight/control safety policy.

    Isolated gaps/duplicates are retained, not repaired or approved. Severe
    loss, source clock changes, overflow and >500ms source backlog stop this
    maintenance capture. The original strict adapter remains unchanged.
    """
    def __init__(self, *, event_capacity=CAPACITY):
        self.previous=None; self.events=[]; self.event_capacity=event_capacity
        self.received=0; self.frame_missing=0; self.sequence_missing=0
        self.missing=0; self.duplicates=0; self.regressions=0; self.consecutive=0
        self.gap_events=0; self.max_backlog_ns=0; self.fault=None

    def observe(self,row):
        self.received+=1
        if (row['subject'],row['segment'])!=('MircoAir','MircoAir'):
            self.fault=self.fault or 'unexpected_rigid_body'
        if not 0<row['sdk_begin_ns']<=row['sdk_end_ns']<=row['callback_ns']:
            self.fault=self.fault or 'source_monotonic_order_invalid'
        backlog=row['callback_ns']-row['sdk_end_ns']
        self.max_backlog_ns=max(self.max_backlog_ns,backlog)
        if backlog>500_000_000:
            self.fault=self.fault or 'source_backlog_above_500ms'
        if row['queue_dropped']:
            self.fault=self.fault or 'source_queue_overflow'
        previous=self.previous
        if previous is not None:
            df=row['frame']-previous['frame']; ds=row['sequence']-previous['sequence']
            fm=max(0,df-1); sm=max(0,ds-1)
            sdk_delta=row['missing_frames']-previous['missing_frames']
            queue_delta=row['queue_dropped']-previous['queue_dropped']
            self.frame_missing+=fm; self.sequence_missing+=sm
            self.missing+=max(fm,sm)  # same lost observations, never double-count
            duplicate=df==0 or ds==0
            regression=df<0 or ds<0
            self.duplicates+=duplicate; self.regressions+=regression
            exceptional=(fm or sm or duplicate or regression or sdk_delta or queue_delta)
            self.consecutive=self.consecutive+1 if (fm or sm or duplicate) else 0
            if exceptional:
                if sdk_delta>0 and sm>0: stage='mixed_SDK_and_after_construction_gap'
                elif queue_delta>0: stage='source_queue_overflow_reported'
                elif sm>0: stage='after_FrameSample_construction_before_callback_unresolved'
                elif sdk_delta>0: stage='SDK_GetFrame_gap_reported'
                else: stage='source_frame_or_sequence_anomaly'
                event=dict(kind='regression' if regression else 'duplicate' if duplicate else 'gap',
                    at_monotonic_ns=row['callback_ns'],previous_callback_ns=previous['callback_ns'],
                    previous_frame=previous['frame'],current_frame=row['frame'],
                    previous_sequence=previous['sequence'],current_sequence=row['sequence'],
                    missing_frame_range=[previous['frame']+1,row['frame']-1] if fm else None,
                    missing_sequence_range=[previous['sequence']+1,row['sequence']-1] if sm else None,
                    missing_frames=fm,missing_sequences=sm,source_missing_delta=sdk_delta,
                    source_queue_drop_delta=queue_delta,stage=stage)
                if len(self.events)>=self.event_capacity:
                    self.fault=self.fault or 'continuity_event_capacity_exceeded'
                else: self.events.append(event)
                if fm or sm: self.gap_events+=1
            if regression: self.fault=self.fault or 'source_frame_or_sequence_regression'
            if (row['epoch']!=previous['epoch'] or row['sdk_hz']!=previous['sdk_hz'] or
                    row.get('frame_id')!=previous.get('frame_id') or
                    row['ros_stamp_ns']<previous['ros_stamp_ns'] or sdk_delta<0 or queue_delta<0):
                self.fault=self.fault or 'source_clock_epoch_rate_or_counter_changed'
            # Explicit bounded maintenance thresholds. A single 5-frame gap
            # does not abort, but repeated severe loss still does.
            if self.consecutive>=5:
                self.fault=self.fault or 'five_consecutive_loss_or_duplicate_transitions'
            if self.gap_events>=5 and self.missing/(self.received+self.missing)>=.25:
                self.fault=self.fault or 'sustained_missing_fraction_at_least_25_percent'
        self.previous=row
        return self.fault is None

    def export(self):
        return dict(mode='static_record_only_not_flight_control',events=self.events,
            received=self.received,inferred_missing_frames=self.frame_missing,
            inferred_missing_sequences=self.sequence_missing,
            missing_observations_estimate=self.missing,
            missing_fraction=(self.missing/(self.received+self.missing)
                              if self.received and not (self.duplicates or self.regressions) else None),
            duplicate_transitions=self.duplicates,regression_transitions=self.regressions,
            max_source_to_callback_ns=self.max_backlog_ns,fatal_reason=self.fault,
            thresholds=dict(consecutive_bad_transitions=5,sustained_gap_events=5,
                sustained_missing_fraction=.25,source_backlog_ns=500_000_000),
            interpolated_frames=0,flight_authorized=False,EV_DELAY_approved=False)


def analyze(rows, *, adapter_fault=None, ram_dropped=0):
    """Only measured diagnostics; no delay estimate or preparation authority."""
    reasons = []
    if not rows:
        return dict(passed=False, reasons=['no_FrameSample_received'], total=0,
                    flight_authorized=False, EV_DELAY_approved=False)
    continuity=CaptureContinuity()
    for row in rows: continuity.observe(row)
    frames = [r['frame'] for r in rows]
    seq = [r['sequence'] for r in rows]
    frame_gaps = [b-a for a,b in zip(frames, frames[1:])]
    seq_gaps = [b-a for a,b in zip(seq,seq[1:])]
    valid, bad_quat, occluded, unpublished, unexpected = [], 0, 0, 0, 0
    norms, jumps, delays, work = [], [], [], []
    for r in rows:
        identity = (r['subject'],r['segment']) == ('MircoAir','MircoAir')
        unexpected += not identity
        finite = all(math.isfinite(x) for k in ('sdk_pose','world_pose') for x in r[k])
        qnorms = [math.sqrt(sum(x*x for x in r[k][3:])) for k in ('sdk_pose','world_pose')]
        qok = finite and all(.99 <= n <= 1.01 for n in qnorms)
        occluded += r['occluded']
        unpublished += not r['pose_published']
        bad_quat += not qok
        norms.extend(n for n in qnorms if math.isfinite(n))
        if identity and r['sdk_ok'] and not r['occluded'] and r['pose_published'] and qok:
            valid.append(r)
        delays.append((r['callback_ns']-r['sdk_end_ns'])/1e6)
        work.append(r['processing_ns']/1e6)
    for a,b in zip(valid,valid[1:]):
        jumps.append(math.dist(a['world_pose'][:3],b['world_pose'][:3]))
    duplicate = sum(g == 0 for g in frame_gaps)
    backwards = sum(g < 0 for g in frame_gaps)
    missing = sum(max(0,g-1) for g in frame_gaps)
    seq_loss = sum(max(0,g-1) for g in seq_gaps)
    regression = {}
    for key in ('sdk_begin_ns','sdk_end_ns','ros_received_ns','ros_stamp_ns','callback_ns'):
        stamps = [r[key] for r in rows if key != 'ros_stamp_ns' or r['pose_published']]
        regression[key] = sum(b <= a for a,b in zip(stamps,stamps[1:]))
    callback_span = (rows[-1]['callback_ns']-rows[0]['callback_ns'])/1e9
    sdk_rate = sorted({r['sdk_hz'] for r in rows})
    host_rate = (len(rows)-1)/callback_span if callback_span > 0 else None
    if len(rows)<20: reasons.append('insufficient_frames')
    if callback_span < 14: reasons.append('observed_frame_span_below_14s')
    if unexpected: reasons.append('rigid_body_mismatch')
    if len(valid) != len(rows): reasons.append('invalid_occluded_or_unpublished_frame')
    if duplicate or backwards or missing or seq_loss: reasons.append('frame_or_sequence_discontinuity')
    if any(regression.values()): reasons.append('timestamp_nonincreasing')
    if len(sdk_rate)!=1 or not 1 <= sdk_rate[0] <= 1000: reasons.append('invalid_or_changing_SDK_rate')
    if len(sdk_rate)==1 and host_rate is not None and not .9 <= host_rate/sdk_rate[0] <= 1.1:
        reasons.append('observed_rate_inconsistent_with_SDK')
    if any(r['queue_dropped'] for r in rows) or ram_dropped: reasons.append('queue_or_RAM_overflow')
    if any(r['epoch'] != rows[0]['epoch'] for r in rows): reasons.append('SDK_epoch_changed')
    if any(r['exposure_time_known'] for r in rows): reasons.append('unverified_exposure_timestamp_claim')
    if any(not 0 < r['sdk_begin_ns'] <= r['sdk_end_ns'] <= r['callback_ns'] for r in rows):
        reasons.append('monotonic_domain_or_time_order_invalid')
    if any(not r['ros_received_ns'] <= r['ros_stamp_ns'] <= r['callback_ros_ns']
           for r in rows if r['pose_published']): reasons.append('ROS_time_order_invalid')
    if jumps and max(jumps)>.05: reasons.append('static_position_jump_above_5cm')
    if adapter_fault: reasons.append('atomic_adapter:'+adapter_fault)
    if continuity.fault: reasons.append('capture_continuity:'+continuity.fault)
    return dict(passed=not reasons, reasons=sorted(set(reasons)), total=len(rows), valid=len(valid),
        observed_frame_span_s=callback_span, sdk_reported_rates_hz=sdk_rate,
        actual_host_observed_rate_hz=host_rate,
        SDK_frame_interval_based_rate_hz=((frames[-1]-frames[0])/callback_span if callback_span>0 else None),
        first_frame=frames[0], last_frame=frames[-1], inferred_missing_frame_numbers=missing,
        received_duplicate_frames=duplicate, frame_regressions=backwards, sidechannel_sequence_loss=seq_loss,
        source_missing_counter_first_last=[rows[0]['missing_frames'],rows[-1]['missing_frames']],
        source_filtered_SDK_duplicate_count=None,
        source_filtered_duplicate_note='Receiver filters equal SDK frames BEFORE FrameSample; count unavailable, not zero',
        occluded_frames=occluded, occluded_fraction=occluded/len(rows), unpublished_frames=unpublished,
        invalid_quaternions=bad_quat, quaternion_norm=distribution(norms),
        maximum_adjacent_position_change_m=max(jumps) if jumps else None,
        timestamp_nonincreasing_counts=regression,
        continuity=continuity.export(),
        ROS_callback_interval_ms=distribution([(b['callback_ns']-a['callback_ns'])/1e6 for a,b in zip(rows,rows[1:])]),
        stage_counters=dict(FrameSample_constructed_sequence_first_last=[seq[0],seq[-1]],
            source_SDK_missing_first_last=[rows[0]['missing_frames'],rows[-1]['missing_frames']],
            source_queue_drop_first_last=[rows[0]['queue_dropped'],rows[-1]['queue_dropped']],
            publisher_attempts=None,publisher_successes=None,DDS_reader_received=None,
            Python_callbacks_in_saved_window=len(rows),RAM_rows=len(rows),RAM_dropped=ram_dropped),
        source_queue_dropped_max=max(r['queue_dropped'] for r in rows), RAM_dropped=ram_dropped,
        SDK_return_to_callback_ms=distribution(delays), callback_processing_ms=distribution(work),
        ROS_receive_to_callback_ms=distribution([(r['callback_ros_ns']-r['ros_received_ns'])/1e6 for r in rows]),
        source_header_to_callback_ms=distribution([(r['callback_ros_ns']-r['ros_stamp_ns'])/1e6 for r in rows if r['pose_published']]),
        exposure_timestamp_known=False, frame_pose_source='same GetFrame snapshot, FrameSample only',
        dynamic_calibration_performed=False, flight_authorized=False, EV_DELAY_approved=False)


def process_identity(pid, *, proc_root=Path('/proc')):
    """Read-only identity; bracket metadata reads to detect exit/PID reuse.

    The start tick, executable inode and command line are compared, not just a
    PID. Only the ROS domain is retained from environ (never unrelated secrets).
    """
    path=proc_root/str(pid)
    before=(path/'stat').read_text().rsplit(')',1)[1].split()[19]
    exe=os.readlink(path/'exe')
    info=(path/'exe').stat()
    argv=[os.fsdecode(x) for x in (path/'cmdline').read_bytes().split(b'\0') if x]
    domain=[x.split(b'=',1)[1] for x in (path/'environ').read_bytes().split(b'\0')
            if x.startswith(b'ROS_DOMAIN_ID=')]
    after=(path/'stat').read_text().rsplit(')',1)[1].split()[19]
    if before!=after:
        raise ProcessLookupError(f'process_changed_during_identity_read:{pid}')
    return dict(pid=pid,start_ticks=before,exe=exe,exe_device=info.st_dev,
                exe_inode=info.st_ino,argv=argv,
                ros_domain=os.fsdecode(domain[-1]) if domain else '0')


def shared_receiver_parameters(argv):
    """Inspect the existing ROS launch arguments/YAML, without querying ROS."""
    params={}
    for index,arg in enumerate(argv):
        if arg=='--params-file':
            if index+1>=len(argv): raise ValueError('missing_params_file_argument')
            document=yaml.load(Path(argv[index+1]).read_text(),Loader=_RosParameterSafeLoader)
            if not isinstance(document,dict): raise ValueError('invalid_ROS_parameter_file')
            for key,value in document.items():
                if key not in ('/**','vicon_client','/vicon_client'):
                    raise ValueError('unrecognized_ROS_parameter_node:'+str(key))
                if not isinstance(value,dict) or not isinstance(value.get('ros__parameters'),dict):
                    raise ValueError('invalid_ROS_parameters')
                params.update(value['ros__parameters'])
        elif arg in ('-p','--param'):
            if index+1>=len(argv) or ':=' not in argv[index+1]:
                raise ValueError('invalid_ROS_parameter_argument')
            key,value=argv[index+1].split(':=',1)
            params[key]=yaml.load(value,Loader=_RosParameterSafeLoader)
    # Match communicator.cpp's vector<double>[3]/bool expectations without
    # casting, reordering, converting angles or inserting default values.
    for key in ('map_xyz','map_rpy'):
        if key in params:
            vector=params[key]
            if (not isinstance(vector,(list,tuple)) or len(vector)!=3 or
                    any(type(v) is not float or not math.isfinite(v) for v in vector)):
                raise ValueError('invalid_ROS_parameter:'+key+':expected_three_finite_doubles')
    if 'map_rpy_in_degrees' in params and type(params['map_rpy_in_degrees']) is not bool:
        raise ValueError('invalid_ROS_parameter:map_rpy_in_degrees:expected_bool')
    for key in ('hostname','namespace'):
        if key in params and type(params[key]) is not str:
            raise ValueError('invalid_ROS_parameter:'+key+':expected_string')
    return params


def discover_shared_receiver(*, proc_root=Path('/proc')):
    """Find one verified shared receiver, or None for standalone capture.

    Discovery is performed anew for each run. No PID is a configuration value.
    A same-name receiver with a different source/configuration is an explicit
    conflict, not permission to guess which process owns the shared topics.
    """
    candidates=[]
    for path in sorted(proc_root.iterdir(),key=lambda p:p.name):
        if not path.name.isdecimal(): continue
        try: executable=os.readlink(path/'exe')
        except (FileNotFoundError,ProcessLookupError): continue
        except PermissionError: continue  # unrelated/inaccessible process
        if Path(executable).name.removesuffix(' (deleted)')!='vicon_client': continue
        if Path(executable).resolve()==BINARY.resolve(): continue  # isolated receiver
        try: identity=process_identity(int(path.name),proc_root=proc_root)
        except (FileNotFoundError,ProcessLookupError): continue
        except OSError as exc:
            raise RuntimeError(f'shared_receiver_identity_unreadable:{path.name}:{exc}') from exc
        candidates.append(identity)
    if len(candidates)>1:
        raise RuntimeError('multiple_shared_receiver_candidates:'+','.join(str(p['pid']) for p in candidates))
    if not candidates: return None
    identity=candidates[0]
    try:
        expected=SHARED_BINARY.resolve(strict=True)
        expected_info=expected.stat()
        argv=identity['argv']
        if (Path(identity['exe'])!=expected or not argv or
                Path(argv[0]).resolve()!=expected or '--ros-args' not in argv or
                (identity['exe_device'],identity['exe_inode'])!=(expected_info.st_dev,expected_info.st_ino)):
            raise ValueError('executable_or_command_line_mismatch')
        if identity['ros_domain']!='0': raise ValueError('not_shared_ROS_domain_0')
        params=shared_receiver_parameters(argv)
        if params.get('hostname')!='10.10.10.2' or params.get('namespace')!='vicon':
            raise ValueError('shared_tracker_or_topic_namespace_mismatch')
        if process_identity(identity['pid'],proc_root=proc_root)!=identity:
            raise ValueError('process_changed_during_discovery')
    except (OSError,ValueError,yaml.YAMLError) as exc:
        raise RuntimeError(f'shared_receiver_identity_unverified:{identity["pid"]}:{exc}') from exc
    return identity


def shared_receiver_observation(identity):
    """A shared receiver is an optional non-interference observer, not a source.

    Never attach to a reused PID or send a signal. Losing this observer does
    not invalidate independent FrameSample data; report the missing evidence.
    """
    if identity is None: return dict(status='not_present',identity=None)
    try: current=process_identity(identity['pid'])
    except (FileNotFoundError,ProcessLookupError):
        return dict(status='exited_or_restarted',identity=None)
    except OSError as exc:
        return dict(status='identity_unavailable',identity=None,error=str(exc))
    return dict(status='unchanged' if current==identity else 'identity_changed',identity=current)


def read_runtime_maps(pid):
    """Bounded OS metadata only; never process memory or a device port."""
    with Path(f'/proc/{pid}/maps').open() as source:
        text=source.read(262145)
    if len(text)>262144: raise RuntimeError('runtime_maps_snapshot_capacity_exceeded')
    return text


def run(authorization_id):
    # No device or network initialization precedes this explicit scope check.
    if not authorization_id or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-' for c in authorization_id):
        raise ValueError('explicit_safe_authorization_id_required')
    # Gate BEFORE importing/initializing ROS or spawning any SDK process.
    try:
        provenance=verify_current()
        if Path(provenance['installed']).resolve()!=BINARY.resolve():
            raise RuntimeError('manifest_is_not_for_the_isolated_receiver')
    except Exception as exc:
        failure_dir=ROOT/'preparation_runs'/('b3_3_vicon_provenance_rejected_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        failure_dir.mkdir()
        with (failure_dir/'report.json').open('x') as f:
            json.dump(dict(error=type(exc).__name__+': '+str(exc),receiver_started=False,
                ROS_initialized=False,automatic_retries=0,flight_authorized=False),f,indent=2)
        raise
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import serialize_message, deserialize_message
    from vicon_receiver.msg import FrameSample
    from std_msgs.msg import Int64

    run_dir=ROOT/'preparation_runs'/('b3_3_vicon_frame_sample_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    run_dir.mkdir()
    marker_dir=ROOT/'preparation_runs/vicon_frame_authorizations'
    marker_dir.mkdir(exist_ok=True)
    with (marker_dir/(authorization_id+'.json')).open('x') as f:
        json.dump(dict(scope='one isolated stationary Vicon-only observation',run=str(run_dir)),f)
    report=dict(scope='One Vicon-only FrameSample observation, no FC/MAVLink/TRS',
        authorization_id=authorization_id, started_utc=datetime.now(timezone.utc).isoformat(),
        binary=str(BINARY), binary_sha256=hashlib.sha256(BINARY.read_bytes()).hexdigest(),
        ROS_DOMAIN_ID=DOMAIN,shared_domain=0,topic=TOPIC,shared_before=None,
        shared_receiver_required=False,shared_observations=[],
        requested_window_ns=WINDOW_NS,flight_authorized=False,EV_DELAY_approved=False,
        PX4_connected=False,MAVLink_sent=0,shared_receiver_modified=False,automatic_retries=0,
        provenance=provenance,capture_mode='static_RAM_record_then_strict_offline_review',
        runtime_audit_windows=[],runtime_audit_windows_dropped=0,
        runtime_audit_scheduling=dict(mode='single_metadata_process_with_bounded_transport_thread',
            interval_ns=RUNTIME_AUDIT_INTERVAL_NS,maximum_outstanding=1,
            initial_check_before_reception=True,final_pending_check_required=True,
            audit_failure_ignored=False,active_result='bounded_summary',
            full_evidence_export='after_observation_loop'),
        configured_FrameSample_qos=dict(reliability='BEST_EFFORT',history='KEEP_LAST',
            depth=5,durability='VOLATILE',changed=False,
            evidence='pinned_C++_SensorDataQoS_and_installed_rclpy_sensor_data_profile'))
    rows=[]; warmup=[]; shared=[]; console=bytearray(); proc=None; auditor=None; metadata=None; contexts=[]; nodes=[]; executors=[]
    ram_dropped=0; raw_roundtrip_checked=0; failure=None
    callback_counts=dict(entered=0,decoded=0,saved=0,warmup=0)
    shared_before=None; shared_monitor=False; shared_limit=None
    recorder=DelayRecorder(authorization_id,dict(scope='static Vicon source only; no FC clock'),capacity=CAPACITY*2)
    adapter=AtomicViconAdapter(recorder)
    continuity=CaptureContinuity()

    def audit_context():
        return dict(monitor_shared=shared_monitor,received_rows=len(rows),
                    last_callback_ns=rows[-1]['callback_ns'] if rows else None)

    def accept_audit(completed):
        nonlocal shared_monitor
        if completed is None: return
        payload=completed['payload']
        if 'snapshot' in payload: report['runtime_maps_last_snapshot']=payload['snapshot']
        if 'classification' in payload: report['runtime_maps_last_classification']=payload['classification']
        if 'loaded' in payload: report['verified_child_library_paths']=payload['loaded']
        if 'audit_reference' in payload:
            report['runtime_audit_last_reference']=payload['audit_reference']
        audit=dict(submitted_ns=completed['submitted_ns'],start_ns=completed['start_ns'],
            end_ns=completed['end_ns'],duration_ns=completed['end_ns']-completed['start_ns'],
            received_rows=completed['context']['received_rows'],
            last_callback_ns=completed['context']['last_callback_ns'],
            received_rows_at_result=len(rows),consumed_ns=time.monotonic_ns())
        if 'metadata_process_timing' in payload:
            audit['metadata_process_timing']=payload['metadata_process_timing']
        if 'audit_reference' in payload: audit['audit_reference']=payload['audit_reference']
        if len(report['runtime_audit_windows'])<512: report['runtime_audit_windows'].append(audit)
        else: report['runtime_audit_windows_dropped']+=1
        if payload.get('error'): raise RuntimeError('runtime_audit:'+payload['error'])
        observation=payload.get('shared_observation')
        if shared_monitor and observation and observation['status']!='unchanged':
            report['shared_observations'].append(dict(stage='capture',
                at_monotonic_ns=completed['end_ns'],**observation))
            report['shared_continuity_note']='Shared identity changed/unavailable; no continuity claim for that process'
            shared_monitor=False

    try:
        shared_before=discover_shared_receiver()
        report['shared_before']=shared_before
        report['shared_discovery']='identified' if shared_before else 'none_standalone_capture'
        for domain in (0,DOMAIN):
            ctx=Context(); rclpy.init(args=[],context=ctx,domain_id=domain); contexts.append(ctx)
            nodes.append(Node('b3_vicon_shared_watch' if domain==0 else 'b3_vicon_FrameSample_record',context=ctx))
            executor=SingleThreadedExecutor(context=ctx); executor.add_node(nodes[-1]); executors.append(executor)
        watch, node=nodes
        if shared_before is not None:
            watch.create_subscription(Int64,'/vicon/frame_number',lambda m:shared.append((time.monotonic_ns(),int(m.data))),qos_profile_sensor_data)
        def callback(message):
            nonlocal ram_dropped,raw_roundtrip_checked
            start=time.monotonic_ns()
            callback_counts['entered']+=1
            row=frame_sample_fields(message)
            callback_counts['decoded']+=1
            row.update(callback_ns=start,callback_ros_ns=node.get_clock().now().nanoseconds)
            # Segment publisher creation is asynchronous. Preserve these early
            # rows separately; the valid 15s window starts only once ready.
            if not rows and not row['pose_published']:
                row['processing_ns']=time.monotonic_ns()-start
                warmup.append(row)
                callback_counts['warmup']+=1
                if len(warmup)>1000 or row['occluded'] or not row['sdk_ok']:
                    raise RuntimeError('source_invalid_during_bounded_publisher_startup')
                return
            if len(rows)>=CAPACITY:
                ram_dropped+=1
                raise RuntimeError('RAM_capacity_exceeded')
            if raw_roundtrip_checked<3:
                restored=deserialize_message(serialize_message(message),FrameSample)
                if frame_sample_fields(restored)!=frame_sample_fields(message):
                    raise RuntimeError('FrameSample_serialization_roundtrip_mismatch')
                raw_roundtrip_checked+=1
            rows.append(row)
            callback_counts['saved']+=1
            continuity.observe(row)
            row['processing_ns']=time.monotonic_ns()-start
        node.create_subscription(FrameSample,TOPIC,callback,qos_profile_sensor_data)
        if shared_before is not None:
            baseline_end=time.monotonic_ns()+3_000_000_000
            while time.monotonic_ns()<baseline_end:
                executors[0].spin_once(timeout_sec=.01)
            baseline=list(shared)
            observation=shared_receiver_observation(shared_before)
            report['shared_observations'].append(dict(stage='before_spawn',**observation))
            shared_monitor=(observation['status']=='unchanged' and len(baseline)>=20
                            and len({f for _,f in baseline})>=20)
            report['shared_baseline_count']=len(baseline)
            if shared_monitor:
                baseline_gaps=[b[0]-a[0] for a,b in zip(baseline,baseline[1:])]
                shared_limit=max(300_000_000,5*max(baseline_gaps))
                report.update(shared_baseline_max_gap_ns=max(baseline_gaps),
                              shared_watch_gap_limit_ns=shared_limit)
        report['shared_continuity_monitor_started']=shared_monitor
        if not shared_monitor:
            report['shared_continuity_note']='No verified advancing shared baseline; independent capture does not depend on it'
        others=[n for n,ns in node.get_node_names_and_namespaces() if n!=node.get_name()]
        if others: raise RuntimeError('isolated_ROS_domain_already_has_nodes:'+str(others))
        command=[str(BINARY),'--ros-args','-r','__node:=b3_vicon_isolated_receiver',
            '-p','hostname:=10.10.10.2','-p','namespace:=vicon_b3_check',
            '-p','capture_frame_samples:=true','-p','capture_subject:=MircoAir','-p','capture_segment:=MircoAir',
            '-p','stream_mode:=ServerPush','-p','buffer_size:=200','-p','world_frame:=map',
            '-p','vicon_frame:=vicon_b3_check','-r','/tf:=/vicon_b3_check/tf',
            '-r','/tf_static:=/vicon_b3_check/tf_static','--log-level','warn']
        env=child_environment(os.environ,provenance['policy'])
        report['command']=command
        recheck_before_spawn(provenance)
        launch_ns=time.monotonic_ns()
        proc=subprocess.Popen(command,env=env,cwd=run_dir,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True)
        report.update(child_pid=proc.pid,child_exe=os.readlink(f'/proc/{proc.pid}/exe'))
        if Path(report['child_exe']).resolve()!=BINARY.resolve(): raise RuntimeError('running_wrong_receiver_binary')
        os.set_blocking(proc.stdout.fileno(),False)
        start=None; deadline=launch_ns+8_000_000_000
        report['capture_state']='WAITING_FRAME_SAMPLE'
        # Its interpreter initializes before first reception. The existing
        # metadata thread now only transports bounded messages; classification
        # and path/ELF loops have a DIFFERENT GIL from the ROS executor.
        metadata=MetadataAuditProcess(proc.pid,provenance,shared_before,startup_deadline_ns=deadline)
        auditor=RuntimeAuditWorker(metadata,clock=time.monotonic_ns)
        last_maps_check=time.monotonic_ns()
        auditor.submit(last_maps_check,audit_context())
        # The initial full runtime check remains a startup gate. Its bounded
        # wait is OUTSIDE the effective reception/15s observation window.
        accept_audit(auditor.wait())
        while True:
            executors[1].spin_once(timeout_sec=.002)
            executors[0].spin_once(timeout_sec=0)
            now=time.monotonic_ns()
            # Nonblocking handoff only: metadata I/O/classification and shared
            # process reads no longer pause this ROS executor service loop.
            accept_audit(auditor.poll(now))
            if not auditor.pending and now-last_maps_check>=RUNTIME_AUDIT_INTERVAL_NS:
                auditor.submit(now,audit_context())
                last_maps_check=now
            try:
                chunk=os.read(proc.stdout.fileno(),4096)
                if len(console)+len(chunk)>128_000: raise RuntimeError('receiver_console_RAM_limit')
                console.extend(chunk)
            except BlockingIOError: pass
            if proc.poll() is not None: raise RuntimeError('isolated_receiver_exited:'+str(proc.returncode))
            if shared_monitor and now-shared[-1][0]>shared_limit:
                raise RuntimeError('shared_receiver_stalled_after_new_connection')
            if rows and start is None:
                start=rows[0]['callback_ns']; deadline=start+WINDOW_NS
                report['capture_start_monotonic_ns']=start
                report['capture_state']='RECORDING_FRAME_SAMPLE'
                print('FrameSample received; starting the single 15-second RAM window',flush=True)
            if start is None and now>=deadline: raise RuntimeError('FrameSample_start_timeout_no_retry')
            if rows and now-rows[-1]['callback_ns']>500_000_000: raise RuntimeError('FrameSample_stream_stalled')
            if rows and (rows[-1]['occluded'] or not rows[-1]['sdk_ok'] or rows[-1]['queue_dropped']):
                raise RuntimeError('source_occlusion_SDK_failure_or_queue_overflow')
            if continuity.fault: raise RuntimeError('capture_continuity:'+continuity.fault)
            if start is not None and now>=deadline:
                report['capture_end_monotonic_ns']=now
                # Never approve a finished window while a late audit (which
                # might reject a library) is still outstanding. No new ROS
                # callbacks occur during this AFTER-window bounded wait.
                if auditor.pending: accept_audit(auditor.wait())
                report['capture_state']='OBSERVATION_COMPLETE'
                break
        report['shared_after']=shared_receiver_observation(shared_before)
    except (Exception,KeyboardInterrupt) as exc:
        failure=type(exc).__name__+': '+str(exc)
        report['error']=failure
        report['capture_state_at_failure']=report.get('capture_state','PRE_START')
        report['capture_state']='FAILED'
    finally:
        # The reception loop is over. Drain its last bounded result before a
        # one-way, full-diagnostic export; no heavy result decoding while ROS
        # callbacks are being serviced. Failure cannot authorize a pass.
        if auditor is not None and metadata is not None:
            try:
                if auditor.pending:
                    try: accept_audit(auditor.wait())
                    except Exception as exc:
                        report['runtime_audit_final_pending_error']=type(exc).__name__+': '+str(exc)
                        if failure is None:
                            failure='runtime_audit:'+str(exc); report['error']=failure
                            report['capture_state_at_failure']=report.get('capture_state','PRE_START')
                            report['capture_state']='FAILED'
                if auditor.pending: raise RuntimeError('metadata_evidence_export_with_unresolved_audit')
                records=metadata.finish_and_export()
                if not records or not any(r['audit_reference']==report.get('runtime_audit_last_reference') for r in records):
                    raise RuntimeError('metadata_latest_evidence_missing')
                report['runtime_audit_evidence_export']=dict(verified=True,
                    phase='after_observation_loop',references=[r['audit_reference'] for r in records])
                for record in records:
                    if record['audit_reference']==report.get('runtime_audit_last_reference'):
                        payload=record['payload']
                        if 'snapshot' in payload: report['runtime_maps_last_snapshot']=payload['snapshot']
                        if 'classification' in payload: report['runtime_maps_last_classification']=payload['classification']
                        if 'loaded' in payload: report['verified_child_library_paths']=payload['loaded']
                    if record['payload'].get('error'):
                        report['runtime_audit_first_failure_evidence']=record
            except Exception as exc:
                report['runtime_audit_evidence_export']=dict(verified=False,
                    phase='after_observation_loop',error=type(exc).__name__+': '+str(exc))
                if failure is None:
                    failure='metadata_evidence_export:'+str(exc); report['error']=failure
                    report['capture_state_at_failure']=report.get('capture_state','PRE_START')
                    report['capture_state']='FAILED'
        if auditor is not None:
            try:
                closed=auditor.close()
                report['runtime_audit_worker']=auditor.status()
                if not closed: raise RuntimeError('runtime_audit_worker_shutdown_incomplete')
            except Exception as exc:
                report['runtime_audit_shutdown_error']=type(exc).__name__+': '+str(exc)
                if failure is None:
                    failure=report['runtime_audit_shutdown_error']; report['error']=failure
                    report['capture_state_at_failure']=report.get('capture_state','PRE_START')
                    report['capture_state']='FAILED'
        elif metadata is not None:
            if not metadata.close():
                report['metadata_process_cleanup_failed']=True
                failure=failure or 'metadata_process_cleanup_failed'; report['error']=failure
                report['capture_state']='FAILED'
        if proc is not None:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try: proc.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    report['child_required_terminate']=True
                    proc.terminate()
                    try: proc.wait(timeout=2)
                    except subprocess.TimeoutExpired: proc.kill(); proc.wait(timeout=2)
            report.update(child_returncode=proc.returncode,child_stopped=proc.poll() is not None)
            console.extend(proc.stdout.read() or b''); proc.stdout.close()
        for executor in reversed(executors): executor.shutdown()
        for node in reversed(nodes): node.destroy_node()
        for ctx in reversed(contexts):
            if ctx.ok(): ctx.shutdown()
        report['shared_after_cleanup']=shared_receiver_observation(shared_before)
        report['shared_continuity_monitor_completed']=(shared_monitor and
            report['shared_after_cleanup']['status']=='unchanged' and failure is None)
        # Strict adapter work (deep copies, matching/cache/event creation) is
        # offline, after OUR child and ROS contexts have stopped. Never clear
        # its fault or change the adapter used by other maintenance/control code.
        replayed=0
        for row in rows:
            replayed+=1
            if not adapter.observe(row,callback_ns=row['callback_ns']): break
        report['strict_adapter_replay']=dict(phase='after_receiver_and_ROS_stopped',
            attempted_rows=replayed,total_raw_rows=len(rows),fault=adapter.fault,
            all_raw_rows_replayed=replayed==len(rows) and adapter.fault is None)
        report['capture_completed']=report.get('capture_state')=='OBSERVATION_COMPLETE'
        report['continuity_capture']=continuity.export()
        report['callback_counters']=callback_counts
        report.update(FrameSample_serialization_roundtrips=raw_roundtrip_checked,
            warmup_unpublished_rows=len(warmup),analysis=analyze(rows,adapter_fault=adapter.fault,ram_dropped=ram_dropped))
        report['passed']=failure is None and report['analysis']['passed']
        report['quality_passed']=report['passed']
        report['raw_saved_after_receiver_stopped']=proc is None or proc.poll() is not None
        report['finished_utc']=datetime.now(timezone.utc).isoformat()
        # No disk output happened in any FrameSample callback or active window.
        artifacts={'frames.json':dict(schema=1,rows=rows,warmup_rows=warmup,exposure_timestamp_known=False,
                       flight_authorized=False), 'adapter_events.json':recorder.export(),
                   'shared_receiver_watch.json':dict(samples=shared)}
        report['artifact_sha256']={}
        for name,document in artifacts.items():
            data=(json.dumps(document,allow_nan=False,indent=2)+'\n').encode()
            with (run_dir/name).open('xb') as f: f.write(data)
            report['artifact_sha256'][name]=hashlib.sha256(data).hexdigest()
        with (run_dir/'receiver_console.txt').open('xb') as f: f.write(console)
        with (run_dir/'report.json').open('x') as f: json.dump(report,f,indent=2,allow_nan=False)
        print(json.dumps(dict(run=str(run_dir),passed=report['passed'],error=failure,
                             capture_completed=report['capture_completed'],quality_passed=report['quality_passed'],
                             analysis=report['analysis']),indent=2),flush=True)
    return 0 if report['passed'] else 2


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    mode=ap.add_mutually_exclusive_group(required=True)
    mode.add_argument('--authorized-one-static-vicon-frame-capture',action='store_true')
    mode.add_argument('--verify-provenance-only',action='store_true',help='Offline ELF/library check; never initializes ROS or SDK')
    ap.add_argument('--authorization-id')
    ap.add_argument('--output',type=Path,help='New offline provenance report only')
    args=ap.parse_args()
    if args.verify_provenance_only:
        if not args.output or args.output.exists() or args.authorization_id:
            ap.error('offline check requires a new --output and no authorization id')
        try: document=verify_current()
        except Exception as exc:
            document=dict(verified=False,error=type(exc).__name__+': '+str(exc),
                receiver_started=False,ROS_initialized=False,flight_authorized=False)
        with args.output.open('x') as f: json.dump(document,f,indent=2)
        print(json.dumps(dict(output=str(args.output),verified=document.get('verified',False),
            library_count=document.get('library_count'),error=document.get('error'))))
        raise SystemExit(0 if document.get('verified') else 2)
    if args.output or not args.authorization_id:
        ap.error('capture needs explicit authorization id; --output is offline-only')
    raise SystemExit(run(args.authorization_id))
