"""Offline only: real metadata thread with synthetic readers + fake full runs.

No ROS initialization, actual receiver, SDK, socket, serial or hardware access.
Mock results can demonstrate scheduling logic, never live capture acceptance.
"""
import ast
import hashlib
import json
from pathlib import Path
import socket
import threading

import pytest

import vicon_frame_sample_capture as entry
from test_vicon_frame_startup_mock import install_fake_runtime


RealWorker=entry.RuntimeAuditWorker


@pytest.fixture(autouse=True)
def no_live_connections_or_child_processes(monkeypatch):
    def forbidden(*args,**kwargs):
        pytest.fail('OFFLINE scheduling test attempted a socket or actual child process')
    monkeypatch.setattr(socket,'socket',forbidden)
    monkeypatch.setattr(entry.subprocess,'Popen',forbidden)


def report(tmp_path):
    return json.loads(next((tmp_path/'preparation_runs').glob('b3_3_vicon_frame_sample_*/report.json')).read_text())


def test_real_worker_runs_off_owner_thread_without_stalling_local_reception():
    entered=threading.Event(); release=threading.Event(); ids=[]
    def inspect(context):
        ids.append(threading.get_ident()); entered.set()
        assert release.wait(1), 'test forgot to release synthetic inspector'
        return dict(loaded=['/SYNTHETIC/approved'])
    worker=RealWorker(inspect,clock=lambda:1_000_000_000)
    try:
        worker.submit(1_000_000_000,dict(received_rows=0))
        assert entered.wait(1)
        # Metadata inspection remains blocked, but owner can service every
        # synthetic producer tick; poll never performs its filesystem work.
        received=[]
        for i in range(20):
            received.append(i)
            assert worker.poll(1_000_000_000+i*1_000_000) is None
        assert len(received)==20 and ids==[worker._thread.ident]
        assert ids[0]!=threading.get_ident()
        release.set()
        result=worker.wait()
        assert result['payload']['loaded']==['/SYNTHETIC/approved']
        assert not worker.pending and worker.status()['completed']==1
    finally:
        release.set(); assert worker.close()
    assert worker.status()['closed'] and not worker.status()['thread_alive']


def test_real_worker_has_one_bounded_job_result_and_refuses_overlap():
    release=threading.Event()
    def inspect(ctx):
        release.wait(1)
        return {}
    worker=RealWorker(inspect,clock=lambda:1_000_000_000)
    try:
        worker.submit(1_000_000_000,{})
        with pytest.raises(RuntimeError,match='already_pending'):
            worker.submit(1_000_000_001,{})
        assert worker._jobs.maxsize==worker._results.maxsize==1
        release.set(); worker.wait()
        assert worker.status()['maximum_outstanding']==1
    finally:
        release.set(); assert worker.close()


@pytest.mark.parametrize('error',[RuntimeError('unreviewed_library'),SystemExit('synthetic worker exit')])
def test_real_worker_latches_inspection_errors_and_never_retries(error):
    calls=[]
    def inspect(ctx):
        calls.append(ctx)
        raise error
    worker=RealWorker(inspect,clock=lambda:1_000_000_000)
    try:
        worker.submit(1_000_000_000,{})
        result=worker.wait()
        assert str(error) in result['payload']['error']
        assert worker.status()['fault']
        with pytest.raises(RuntimeError,match='failed'):
            worker.submit(1_000_000_001,{})
        assert len(calls)==1
    finally:
        assert worker.close()


def test_pending_audit_deadline_fails_closed_while_owner_remains_responsive():
    entered=threading.Event(); release=threading.Event(); clock=[1_000_000_000]
    def inspect(ctx):
        entered.set(); release.wait(1); return {}
    worker=RealWorker(inspect,clock=lambda:clock[0])
    try:
        worker.submit(clock[0],{})
        assert entered.wait(1)
        clock[0]+=entry.RUNTIME_AUDIT_INTERVAL_NS
        with pytest.raises(RuntimeError,match='deadline_missed'): worker.poll(clock[0])
        assert worker.status()['fault']=='runtime_audit_deadline_missed'
    finally:
        release.set(); assert worker.close()


def test_completed_but_late_audit_cannot_be_accepted():
    clock=[1_000_000_000]
    def inspect(ctx):
        clock[0]+=entry.RUNTIME_AUDIT_INTERVAL_NS+1
        return dict(loaded=['/SYNTHETIC/approved'])
    worker=RealWorker(inspect,clock=lambda:clock[0])
    try:
        worker.submit(clock[0],{})
        assert worker._pending['done'].wait(1)
        result=worker.poll(clock[0])
        assert result['payload']['error']=='runtime_audit_deadline_missed'
        assert worker.status()['fault']
    finally: assert worker.close()


def test_bounded_startup_wait_cannot_silently_return_without_a_result():
    entered=threading.Event(); release=threading.Event()
    def inspect(ctx):
        entered.set(); release.wait(1); return {}
    worker=RealWorker(inspect,clock=lambda:1_000_000_000)
    try:
        worker.submit(1_000_000_000,{})
        assert entered.wait(1)
        with pytest.raises(RuntimeError,match='deadline_missed'): worker.wait()
    finally:
        release.set(); assert worker.close()


def test_stalled_reader_shutdown_is_bounded_and_reportable_not_a_pass():
    entered=threading.Event(); release=threading.Event()
    def inspect(ctx):
        entered.set(); release.wait(2); return {}
    worker=RealWorker(inspect,clock=lambda:1_000_000_000)
    try:
        worker.submit(1_000_000_000,{})
        assert entered.wait(1)
        assert not worker.close()
        assert worker.status()['closed'] and worker.status()['thread_alive']
    finally:
        release.set(); worker._thread.join(timeout=1)
    assert not worker.status()['thread_alive']


def test_full_original_mapping_check_runs_in_worker_and_preserves_rejection(monkeypatch):
    calls=[]
    def raw(pid):
        assert pid==123
        calls.append(threading.get_ident())
        return 'this is a malformed synthetic maps line'
    monkeypatch.setattr(entry,'read_runtime_maps',raw)
    worker=RealWorker(lambda ctx:entry.runtime_audit_snapshot(123,{},None,ctx),clock=lambda:1_000_000_000)
    try:
        worker.submit(1_000_000_000,dict(monitor_shared=False))
        completed=worker.wait(); payload=completed['payload']
        assert payload['snapshot']['text']=='this is a malformed synthetic maps line'
        assert payload['classification']['rejection']=='malformed_proc_maps_line'
        assert 'malformed_proc_maps_line' in payload['error']
        assert calls==[worker._thread.ident]
    finally: assert worker.close()


@pytest.mark.parametrize('delay_ns',[25_000_000,49_168_732,80_000_000])
def test_mock_329hz_full_window_with_slow_audit_has_continuous_input(monkeypatch,tmp_path,delay_ns):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    state.frame_period_ns=3_033_333; state.sdk_hz=1e9/state.frame_period_ns
    state.audit_delay_ns=delay_ns
    assert entry.run('OFFLINE_MOCK_ASYNC_329HZ')==0
    r=report(tmp_path)
    assert r['capture_completed'] and r['quality_passed']
    assert r['analysis']['total']>=4900 and r['analysis']['inferred_missing_frame_numbers']==0
    assert r['analysis']['received_duplicate_frames']==0 and r['analysis']['RAM_dropped']==0
    assert r['analysis']['actual_host_observed_rate_hz']==pytest.approx(state.sdk_hz)
    assert max(w['duration_ns'] for w in r['runtime_audit_windows'])==delay_ns
    assert any(w['received_rows_at_result']>w['received_rows'] for w in r['runtime_audit_windows'])
    assert r['configured_FrameSample_qos']==dict(reliability='BEST_EFFORT',history='KEEP_LAST',
        depth=5,durability='VOLATILE',changed=False,
        evidence='pinned_C++_SensorDataQoS_and_installed_rclpy_sensor_data_profile')
    assert r['runtime_audit_worker']['closed'] and not r['runtime_audit_worker']['thread_alive']
    assert state.spawn_count==1 and state.pipe_closed and r['child_stopped']
    assert all(n.destroyed for n in state.nodes) and all(not c.live for c in state.contexts)
    assert not r['flight_authorized'] and not r['EV_DELAY_approved'] and r['MAVLink_sent']==0


def test_mid_window_mapping_rejection_still_stops_and_preserves_raw(monkeypatch,tmp_path):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    original=entry.read_runtime_maps
    def altered(pid):
        if state.now>=6_000_000_000: return 'bad maps introduced during active capture'
        return original(pid)
    monkeypatch.setattr(entry,'read_runtime_maps',altered)
    assert entry.run('OFFLINE_MOCK_MID_CAPTURE_AUDIT_FAILURE')==2
    r=report(tmp_path)
    assert not r['capture_completed'] and r['capture_state_at_failure']=='RECORDING_FRAME_SAMPLE'
    assert 'malformed_proc_maps_line' in r['error']
    assert r['runtime_maps_last_snapshot']['text']=='bad maps introduced during active capture'
    assert r['runtime_maps_last_classification']['rejection']=='malformed_proc_maps_line'
    assert r['analysis']['total']>0 and r['child_stopped']
    assert state.spawn_count==1 and r['automatic_retries']==0 and state.pipe_closed
    assert r['runtime_audit_worker']['closed'] and not r['flight_authorized']


def test_audit_exceeding_original_cadence_refuses_startup_not_reduced_frequency(monkeypatch,tmp_path):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    state.audit_delay_ns=101_000_000
    assert entry.run('OFFLINE_MOCK_AUDIT_MISSED_CADENCE')==2
    r=report(tmp_path)
    assert 'runtime_audit_deadline_missed' in r['error']
    assert r['capture_state_at_failure']=='WAITING_FRAME_SAMPLE'
    assert not r['capture_completed'] and r['analysis']['total']==0
    assert r['child_stopped'] and r['runtime_audit_worker']['closed']


def test_late_final_rejection_cannot_be_hidden_by_observation_complete(monkeypatch,tmp_path):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    state.frame_period_ns=7_000_000; state.sdk_hz=1e9/state.frame_period_ns
    state.audit_delay_ns=25_000_000
    original=entry.read_runtime_maps
    def rejected_only_after_window(pid):
        if state.now>=19_035_000_000: return 'final pending audit rejected'
        return original(pid)
    monkeypatch.setattr(entry,'read_runtime_maps',rejected_only_after_window)
    assert entry.run('OFFLINE_MOCK_FINAL_PENDING_REJECT')==2
    r=report(tmp_path)
    assert not r['capture_completed'] and not r['quality_passed']
    assert 'malformed_proc_maps_line' in r['error'] and r['child_stopped']
    assert r['runtime_audit_windows'][-1]['consumed_ns']>r['capture_end_monotonic_ns']


def test_worker_shutdown_failure_revokes_otherwise_good_mock_run(monkeypatch,tmp_path):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    state.audit_shutdown_ok=False
    assert entry.run('OFFLINE_MOCK_WORKER_SHUTDOWN_FAILED')==2
    r=report(tmp_path)
    assert not r['quality_passed'] and not r['capture_completed']
    assert 'runtime_audit_worker_shutdown_incomplete' in r['error']
    assert r['child_stopped'] and state.pipe_closed and r['automatic_retries']==0


def test_user_interrupt_still_cleans_only_our_mock_resources(monkeypatch,tmp_path):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    make_worker=entry.RuntimeAuditWorker
    def make_and_interrupt(inspect,*,clock):
        worker=make_worker(inspect,clock=clock)
        original=state.executors[1].spin_once
        def interrupted(**kwargs):
            original(**kwargs)
            if state.now>=6_000_000_000: raise KeyboardInterrupt()
        state.executors[1].spin_once=interrupted
        return worker
    monkeypatch.setattr(entry,'RuntimeAuditWorker',make_and_interrupt)
    assert entry.run('OFFLINE_MOCK_USER_INTERRUPT')==2
    r=report(tmp_path)
    assert r['error'].startswith('KeyboardInterrupt:')
    assert not r['capture_completed'] and r['child_stopped'] and state.pipe_closed
    assert r['runtime_audit_worker']['closed']
    assert all(n.destroyed for n in state.nodes) and all(not c.live for c in state.contexts)
    assert not r['shared_receiver_modified'] and r['automatic_retries']==0


def test_previous_real_15s_raw_data_still_fails_strict_continuity_without_repair():
    history=Path(entry.__file__).parents[2]/'preparation_runs/b3_3_vicon_frame_sample_20261022_163334_922557'
    if not (history/'frames.json').exists():
        pytest.skip('real historical artifact not distributed with this checkout')
    raw=(history/'frames.json').read_bytes()
    previous=json.loads((history/'report.json').read_text())
    assert hashlib.sha256(raw).hexdigest()==previous['artifact_sha256']['frames.json']
    data=json.loads(raw)['rows']
    result=entry.analyze(data)
    assert result['total']==4573 and result['inferred_missing_frame_numbers']==373
    assert result['sidechannel_sequence_loss']==373 and not result['passed']
    assert 'frame_or_sequence_discontinuity' in result['reasons']
    assert not result['EV_DELAY_approved'] and not result['flight_authorized']
    assert (history/'frames.json').read_bytes()==raw


def test_startup_verification_and_live_loop_boundaries_are_explicit():
    tree=ast.parse(Path(entry.__file__).read_text())
    run=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='run')
    loops=[n for n in ast.walk(run) if isinstance(n,ast.While) and isinstance(n.test,ast.Constant) and n.test.value is True]
    assert len(loops)==1
    calls=[n.func.id for n in ast.walk(loops[0]) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name)]
    assert 'read_runtime_maps' not in calls and 'verify_runtime_maps' not in calls
    assert 'shared_receiver_observation' not in calls
    assert entry.RUNTIME_AUDIT_INTERVAL_NS==100_000_000
