"""Real local pipe/interpreter tests with SAVED/SYNTHETIC metadata only.

Every child installs a fail-on-network/device/ROS/SDK-launch audit hook. No
receiver, node, SDK, PX4/TRS, serial or live Vicon access is allowed.
"""
import ast
import hashlib
import io
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time

import pytest

import vicon_frame_sample_capture as entry
import vicon_metadata_audit_process as transport
from test_vicon_frame_startup_mock import install_fake_runtime


HISTORY=Path(entry.__file__).parents[2]/'preparation_runs/b3_3_vicon_frame_sample_20261026_114906_803538'
REAL_POPEN=subprocess.Popen


@pytest.fixture
def local_helper(monkeypatch):
    children=[]
    def install(mode='saved',*,response_bytes=0):
        # The only genuine subprocess is a local interpreter of this harness;
        # the production receiver command must never reach REAL_POPEN.
        script=f'''
import sys,os,json,time
def fence(event,args):
    if event in ('socket.__new__','socket.connect','socket.bind','subprocess.Popen'):
        raise AssertionError('OFFLINE child attempted network/process access')
    if event=='import' and args[0].split('.')[0] in ('rclpy','vicon_receiver'):
        raise AssertionError('OFFLINE child attempted ROS import')
    if event=='open' and isinstance(args[0],(str,bytes)):
        p=os.fsdecode(args[0])
        if p.startswith(('/dev/tty','/dev/serial','/dev/input','/dev/hid','/dev/bus')):
            raise AssertionError('OFFLINE child attempted device open')
sys.addaudithook(fence)
sys.path.insert(0,{str(Path(entry.__file__).parent)!r})
if {mode!r}=='production_cli':
    import runpy
    sys.argv=[{str(Path(transport.__file__).resolve())!r},'--metadata-worker']
    runpy.run_path(sys.argv[0],run_name='__main__')
    raise AssertionError('production CLI did not take its clean exit')
import vicon_frame_sample_capture as e
import vicon_metadata_audit_process as m
history=json.loads(open({str(HISTORY/'report.json')!r}).read())
original=e.runtime_audit_snapshot
def inspect(pid,provenance,shared,context):
    assert pid==123456789 and shared is None
    mode={mode!r}
    if mode=='exit': os._exit(7)
    if mode in ('cpu','stuck'):
        end=time.thread_time_ns()+(40_000_000 if mode=='cpu' else 500_000_000)
        n=0
        while time.thread_time_ns()<end: n+=1
        return dict(synthetic_cpu_iterations=n,pid=os.getpid())
    e.read_runtime_maps=lambda ignored: ('bad maps line' if mode=='bad_maps' else history['runtime_maps_last_snapshot']['text'])
    result=original(pid,provenance,shared,context)
    if {response_bytes}: result['synthetic_padding']='p'*{response_bytes}
    return result
e.runtime_audit_snapshot=inspect
if {mode!r} in ('corrupt_export','wrong_ref_export','missing_export'):
    original_export=m.AuditEvidenceStore.export
    def damaged(self,reference):
        if {mode!r}=='missing_export': raise RuntimeError('SYNTHETIC evidence unavailable')
        record=original_export(self,reference)
        if {mode!r}=='corrupt_export': record['payload']['SYNTHETIC_TAMPER']=True
        else: record['audit_reference']=dict(audit_serial=99999,sha256='0'*64)
        return record
    m.AuditEvidenceStore.export=damaged
if {mode!r}=='invalid_summary':
    original_record=m.AuditEvidenceStore.record
    def invalid(self,serial,payload):
        result=original_record(self,serial,payload); result['verified']='yes'; return result
    m.AuditEvidenceStore.record=invalid
if {mode!r}=='bad_ready':
    print('{{"ready":false}}',flush=True)
elif {mode!r}=='bad_serial':
    print(json.dumps(dict(ready=True,pid=os.getpid(),role='read_only_metadata')),flush=True)
    sys.stdin.buffer.readline()
    print(json.dumps(dict(serial=99,worker_pid=os.getpid(),payload={{}})),flush=True)
elif {mode!r}=='oversized_result':
    print(json.dumps(dict(ready=True,pid=os.getpid(),role='read_only_metadata')),flush=True)
    sys.stdin.buffer.readline()
    sys.stdout.buffer.write(b'x'*(m.MAX_PACKET_BYTES+1)); sys.stdout.buffer.flush()
else:
    m.metadata_worker_main()
    os._exit(0)
'''
        def allowed(command,**kwargs):
            assert command==[sys.executable,'-B',str(Path(transport.__file__).resolve()),'--metadata-worker']
            child=REAL_POPEN([sys.executable,'-B','-c',script],**kwargs)
            children.append(child); return child
        monkeypatch.setattr(transport.subprocess,'Popen',allowed)
        return children
    yield install
    for child in children:
        if child.poll() is None:
            child.kill(); child.wait(timeout=1)
            pytest.fail('test left metadata helper alive')


@pytest.fixture(autouse=True)
def no_network(monkeypatch):
    monkeypatch.setattr(socket,'socket',lambda *a,**kw:pytest.fail('OFFLINE test attempted socket'))


def provenance():
    return json.loads((HISTORY/'report.json').read_text())['provenance']


def make(): return transport.MetadataAuditProcess(123456789,provenance(),None)


def test_saved_606_maps_are_checked_unchanged_in_distinct_interpreter(local_helper):
    children=local_helper(); client=make()
    try:
        one=client(dict(monitor_shared=False)); two=client(dict(monitor_shared=False))
        assert one['verified'] and two['verified']
        assert not any(k in one or k in two for k in ('snapshot','classification','loaded'))
        record,=client.finish_and_export(); full=record['payload']
        assert record['audit_reference']==two['audit_reference']
        assert full['snapshot']['text']==json.loads((HISTORY/'report.json').read_text())['runtime_maps_last_snapshot']['text']
        assert len(full['loaded'])==99 and len(full['classification']['mappings'])==606
        assert one['metadata_process_timing']['worker_pid']==client.proc.pid!=os.getpid()
        assert len(children)==1 and client.status()['requests']==3
    finally: assert client.close()
    assert not client.status()['alive'] and client.status()['returncode']==0


def test_exact_production_helper_entry_imports_without_ROS_network_or_SDK(local_helper):
    local_helper('production_cli'); client=make()
    try:
        payload=client(dict(monitor_shared=False))
        # No such synthetic PID exists. The real, unmodified read-only source
        # fails safely, and the child audit hook rejects all network/ROS use.
        assert 'FileNotFoundError' in payload['error']
        assert payload['metadata_process_timing']['worker_pid']!=os.getpid()
    finally: assert client.close()


def test_cpu_bound_check_has_separate_GIL_and_bounded_transport(local_helper):
    local_helper('cpu'); client=make(); worker=entry.RuntimeAuditWorker(client)
    try:
        start=time.monotonic_ns(); worker.submit(start,dict(monitor_shared=False))
        ticks=[]; result=None
        while result is None:
            ticks.append(time.monotonic_ns()); time.sleep(.001)
            result=worker.poll(time.monotonic_ns())
        assert len(ticks)>2  # simultaneous owner activity, not a real rate guarantee
        timing=result['payload']['metadata_process_timing']
        assert timing['worker_pid']!=os.getpid()
        assert timing['cpu_ns']>=40_000_000
        assert timing['transport_thread_cpu_ns']<timing['cpu_ns']
        record,=client.finish_and_export()
        assert record['payload']['pid']==client.proc.pid
        assert worker._jobs.maxsize==worker._results.maxsize==1
    finally: assert worker.close()
    assert not worker.status()['metadata_process']['alive']


def test_original_mapping_rejection_is_not_hidden_by_transport(local_helper):
    local_helper('bad_maps'); client=make(); worker=entry.RuntimeAuditWorker(client)
    try:
        worker.submit(time.monotonic_ns(),dict(monitor_shared=False)); result=worker.wait()
        assert 'malformed_proc_maps_line' in result['payload']['error']
        record,=client.finish_and_export()
        assert record['payload']['snapshot']['text']=='bad maps line'
        assert worker.status()['fault']
        with pytest.raises(RuntimeError,match='failed'): worker.submit(time.monotonic_ns(),{})
    finally: assert worker.close()


def test_partial_pipe_reads_preserve_whole_bounded_message(local_helper):
    local_helper(response_bytes=300_000); client=make()
    try:
        payload=client(dict(monitor_shared=False))
        assert payload['verified'] and 'synthetic_padding' not in payload
        record,=client.finish_and_export(); full=record['payload']
        assert len(full['synthetic_padding'])==300_000
        assert full['classification']['verified_elf_paths']==full['loaded']
    finally: assert client.close()


@pytest.mark.parametrize('mode,reason',[
    ('exit','exited_or_pipe_closed'),('bad_serial','identity_mismatch'),
    ('oversized_result','capacity_exceeded'),('stuck','deadline_missed')])
def test_helper_failure_is_latched_and_no_retry(local_helper,mode,reason):
    children=local_helper(mode); client=make()
    try:
        with pytest.raises(RuntimeError,match=reason): client(dict(monitor_shared=False))
        with pytest.raises(RuntimeError,match='closed_or_failed'): client({})
        assert len(children)==1 and client.status()['fault']
    finally: client.close()
    assert not client.status()['alive']


def test_invalid_readiness_closes_own_child_before_start(local_helper):
    children=local_helper('bad_ready')
    with pytest.raises(RuntimeError,match='invalid_readiness'): make()
    assert len(children)==1 and children[0].poll() is not None


def test_helper_cannot_extend_existing_absolute_startup_deadline(local_helper):
    children=local_helper()
    with pytest.raises(RuntimeError,match='deadline_missed'):
        transport.MetadataAuditProcess(123456789,provenance(),None,
            startup_deadline_ns=time.monotonic_ns()-1)
    assert len(children)==1 and children[0].poll() is not None


def test_request_capacity_failure_cannot_start_more_helpers(local_helper):
    children=local_helper(); client=make()
    try:
        with pytest.raises(RuntimeError,match='request_capacity'):
            client(dict(monitor_shared=False,extra='x'*transport.MAX_PACKET_BYTES))
        assert client.status()['fault'] and len(children)==1
    finally: assert client.close()


def test_close_is_idempotent_and_refuses_reuse(local_helper):
    local_helper(); client=make(); assert client.close(); assert client.close()
    with pytest.raises(RuntimeError,match='closed_or_failed'): client({})
    assert client.proc.stdin.closed and client.proc.stdout.closed


def test_busy_helper_stop_is_bounded_reported_and_not_left_alive(local_helper):
    local_helper('stuck'); client=make(); worker=entry.RuntimeAuditWorker(client)
    worker.submit(time.monotonic_ns(),dict(monitor_shared=False))
    time.sleep(.01)
    start=time.monotonic_ns(); clean=worker.close(); elapsed=time.monotonic_ns()-start
    assert elapsed<250_000_000  # test scheduling allowance, NOT production deadline
    assert not clean and client.status()['forced_stop']
    assert not client.status()['alive'] and not worker._thread.is_alive()


@pytest.mark.parametrize('line',[
    b'x\n', b'{"serial":2,"context":{}}\n', b'x'*2_000_001,
])
def test_worker_rejects_malformed_or_out_of_order_requests_without_inspecting(monkeypatch,line):
    monkeypatch.setattr(entry,'runtime_audit_snapshot',lambda *a:pytest.fail('invalid request inspected'))
    with pytest.raises((RuntimeError,ValueError,KeyError)):
        transport.metadata_worker_main(io.BytesIO(line),io.BytesIO())


def test_worker_refuses_second_initialization(monkeypatch):
    monkeypatch.setattr(entry,'runtime_audit_snapshot',lambda *a:{})
    initialize=dict(target_pid=1,provenance={},shared_before=None)
    requests=[dict(serial=i,initialize=initialize,context={}) for i in (1,2)]
    raw=''.join(json.dumps(x)+'\n' for x in requests).encode()
    with pytest.raises(RuntimeError,match='reinitialization_refused'):
        transport.metadata_worker_main(io.BytesIO(raw),io.BytesIO())


def test_current_real_4945_rows_remain_strictly_failed_and_unmodified():
    raw=(HISTORY/'frames.json').read_bytes(); r=json.loads((HISTORY/'report.json').read_text())
    assert hashlib.sha256(raw).hexdigest()==r['artifact_sha256']['frames.json']
    result=entry.analyze(json.loads(raw)['rows'])
    assert result['total']==4945 and result['inferred_missing_frame_numbers']==3
    assert len(result['continuity']['events'])==2 and not result['passed']
    assert not result['flight_authorized'] and not result['EV_DELAY_approved']
    assert (HISTORY/'frames.json').read_bytes()==raw


def test_callback_quality_qos_commands_and_original_constants_match_before_repair():
    before=Path(entry.__file__).parents[2]/'preparation_runs/b3_3_vicon_gil_offline_20261026_115251/vicon_frame_sample_capture.py.before'
    old=ast.parse(before.read_text()); new=ast.parse(Path(entry.__file__).read_text())
    for name in ('CaptureContinuity','analyze','process_identity','discover_shared_receiver',
                 'read_runtime_maps','runtime_audit_snapshot'):
        a=next(n for n in old.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name==name)
        b=next(n for n in new.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name==name)
        assert ast.dump(a)==ast.dump(b),name
    def selected(tree):
        run=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='run')
        callback=next(n for n in ast.walk(run) if isinstance(n,ast.FunctionDef) and n.name=='callback')
        command=next(n for n in ast.walk(run) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='command' for t in n.targets))
        constants=[n for n in tree.body if isinstance(n,ast.Assign)]
        return ast.dump(callback),ast.dump(command),[ast.dump(n) for n in constants]
    assert selected(old)==selected(new)


def test_full_mock_cleanup_also_closes_helper_when_worker_construction_fails(monkeypatch,tmp_path):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    def failed(*args,**kwargs): raise RuntimeError('SYNTHETIC transport construction failed')
    monkeypatch.setattr(entry,'RuntimeAuditWorker',failed)
    assert entry.run('OFFLINE_MOCK_CONSTRUCTOR_FAILURE')==2
    r=json.loads(next((tmp_path/'preparation_runs').glob('b3_3_vicon_frame_sample_*/report.json')).read_text())
    assert state.metadata_helpers[0].closed and r['child_stopped']
    assert r['automatic_retries']==0 and not r['flight_authorized'] and r['MAVLink_sent']==0


def test_full_mock_helper_startup_failure_cleans_receiver_without_observation(monkeypatch,tmp_path):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    def failed(*args,**kwargs): raise RuntimeError('SYNTHETIC metadata readiness failure')
    monkeypatch.setattr(entry,'MetadataAuditProcess',failed)
    assert entry.run('OFFLINE_MOCK_HELPER_STARTUP_FAILURE')==2
    r=json.loads(next((tmp_path/'preparation_runs').glob('b3_3_vicon_frame_sample_*/report.json')).read_text())
    assert r['analysis']['total']==0 and not r['capture_completed'] and r['child_stopped']
    assert state.spawn_count==1 and r['automatic_retries']==0
    assert all(n.destroyed for n in state.nodes) and all(not c.live for c in state.contexts)


def test_full_mock_transport_failure_midwindow_stops_without_retry(monkeypatch,tmp_path):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    original=entry.MetadataAuditProcess.__call__
    def failed(self,context):
        if state.now>=6_000_000_000: raise RuntimeError('SYNTHETIC metadata IPC lost')
        return original(self,context)
    monkeypatch.setattr(entry.MetadataAuditProcess,'__call__',failed)
    assert entry.run('OFFLINE_MOCK_HELPER_MID_CAPTURE_FAILURE')==2
    r=json.loads(next((tmp_path/'preparation_runs').glob('b3_3_vicon_frame_sample_*/report.json')).read_text())
    assert 'metadata IPC lost' in r['error'] and not r['capture_completed']
    assert r['analysis']['total']>0 and r['child_stopped'] and state.metadata_helpers[0].closed
    assert state.spawn_count==1 and not r['flight_authorized'] and r['MAVLink_sent']==0


def test_metadata_entry_has_no_real_capture_or_control_call_paths():
    tree=ast.parse(Path(transport.__file__).read_text())
    calls=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
    assert not set(calls)&{'run','create_node','create_subscription','mavlink_connection',
                          'send','connect','bind','Serial','socket'}
    assert transport.AUDIT_NS==entry.RUNTIME_AUDIT_INTERVAL_NS==100_000_000
    assert not set(calls)&{'disable','set_threshold','setswitchinterval','setpriority','sched_setaffinity'}


def test_evidence_store_is_bounded_and_payload_cannot_be_changed_after_recording():
    store=transport.AuditEvidenceStore()
    payload=dict(snapshot=dict(text='SYNTHETIC'),classification={},loaded=[])
    for serial in range(1,101): summary=store.record(serial,payload)
    payload['snapshot']['text']='mutated input must not alter stored evidence'
    assert set(store.__dict__)=={'latest','first_failure','frozen'}
    assert store.first_failure is None and store.latest['audit_reference']==summary['audit_reference']
    record=store.export(summary['audit_reference'])
    assert record['payload']['snapshot']['text']=='SYNTHETIC'
    assert hashlib.sha256(transport.evidence_bytes(record['payload'])).hexdigest()==summary['audit_reference']['sha256']
    with pytest.raises(RuntimeError,match='frozen'): store.record(101,{})


def test_first_failure_is_latched_and_old_success_evidence_cannot_be_substituted():
    store=transport.AuditEvidenceStore()
    old=store.record(1,dict(loaded=['SYNTHETIC']))
    failed=store.record(2,dict(error='SYNTHETIC unapproved library',snapshot=dict(text='raw bad map')))
    assert not failed['verified'] and failed['error']
    assert store.first_failure is store.latest
    with pytest.raises(RuntimeError,match='failure_latched'): store.record(3,{})
    with pytest.raises(RuntimeError,match='not_retained'): store.export(old['audit_reference'])
    assert store.export(failed['audit_reference'])['payload']['snapshot']['text']=='raw bad map'


def test_full_evidence_retention_is_bounded():
    store=transport.AuditEvidenceStore()
    with pytest.raises(RuntimeError,match='capacity_exceeded'):
        store.record(1,dict(extra='x'*transport.MAX_PACKET_BYTES))
    assert store.latest is None


def test_active_pipe_never_decodes_large_full_details(local_helper,monkeypatch):
    local_helper(response_bytes=300_000); client=make()
    loads=json.loads; measured=[]
    def watched(raw,*a,**kw):
        measured.append((client._frozen,len(raw)))
        if not client._frozen: assert len(raw)<=transport.MAX_SUMMARY_BYTES
        return loads(raw,*a,**kw)
    monkeypatch.setattr(transport.json,'loads',watched)
    try:
        for _ in range(3): assert client(dict(monitor_shared=False))['verified']
        record,=client.finish_and_export()
        assert len(record['payload']['synthetic_padding'])==300_000
        assert all(size<=4096 for frozen,size in measured if not frozen)
        assert any(frozen and size>300_000 for frozen,size in measured)
        with pytest.raises(RuntimeError,match='frozen'): client(dict(monitor_shared=False))
    finally: assert client.close()


@pytest.mark.parametrize('mode,reason',[
    ('corrupt_export','reference_or_hash_mismatch'),
    ('wrong_ref_export','reference_or_hash_mismatch'),
    ('missing_export','exited_or_pipe_closed')])
def test_export_failure_cannot_become_valid_evidence_or_retry(local_helper,mode,reason):
    children=local_helper(mode); client=make()
    try:
        assert client(dict(monitor_shared=False))['verified']
        with pytest.raises(RuntimeError,match=reason): client.finish_and_export()
        assert client.status()['fault'] and client.status()['frozen']
        assert len(children)==1
    finally: client.close()
    assert not client.status()['alive']


def test_invalid_summary_is_rejected_instead_of_implicit_success(local_helper):
    local_helper('invalid_summary'); client=make()
    try:
        with pytest.raises(RuntimeError,match='invalid_audit_summary'): client(dict(monitor_shared=False))
        with pytest.raises(RuntimeError,match='closed_or_failed'): client({})
    finally: client.close()


def test_no_summary_means_no_export_or_readiness(local_helper):
    local_helper(); client=make()
    try:
        with pytest.raises(RuntimeError,match='no_audit_evidence'): client.finish_and_export()
    finally: assert client.close()


def test_full_mock_failed_export_revokes_otherwise_good_window(monkeypatch,tmp_path):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    def fail(self): raise RuntimeError('SYNTHETIC full evidence missing')
    monkeypatch.setattr(entry.MetadataAuditProcess,'finish_and_export',fail)
    assert entry.run('OFFLINE_MOCK_MISSING_EVIDENCE')==2
    r=json.loads(next((tmp_path/'preparation_runs').glob('b3_3_vicon_frame_sample_*/report.json')).read_text())
    assert not r['passed'] and not r['runtime_audit_evidence_export']['verified']
    assert 'full evidence missing' in r['error'] and r['child_stopped']
    assert state.metadata_helpers[0].closed and r['MAVLink_sent']==0
    assert not r['flight_authorized'] and r['automatic_retries']==0


def test_full_mock_wrong_latest_reference_refuses_success(monkeypatch,tmp_path):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    original=entry.MetadataAuditProcess.finish_and_export
    def wrong(self):
        records=original(self); records[0]['audit_reference']=dict(audit_serial=0,sha256='0'*64)
        return records
    monkeypatch.setattr(entry.MetadataAuditProcess,'finish_and_export',wrong)
    assert entry.run('OFFLINE_MOCK_WRONG_LATEST_EVIDENCE')==2
    r=json.loads(next((tmp_path/'preparation_runs').glob('b3_3_vicon_frame_sample_*/report.json')).read_text())
    assert not r['runtime_audit_evidence_export']['verified'] and 'latest_evidence_missing' in r['error']
    assert r['child_stopped'] and state.metadata_helpers[0].closed


def test_export_only_after_last_mock_callback_and_same_original_maps_retained(monkeypatch,tmp_path):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    original=entry.MetadataAuditProcess.finish_and_export; seen=[]
    def checked(self):
        seen.append((state.now,state.nodes[1].count))
        assert state.now>=19_010_000_000  # shared baseline + actual 15s mock window
        return original(self)
    monkeypatch.setattr(entry.MetadataAuditProcess,'finish_and_export',checked)
    assert entry.run('OFFLINE_MOCK_EXPORT_AFTER_RX')==0
    r=json.loads(next((tmp_path/'preparation_runs').glob('b3_3_vicon_frame_sample_*/report.json')).read_text())
    assert len(seen)==1 and r['runtime_audit_evidence_export']['phase']=='after_observation_loop'
    assert r['runtime_maps_last_snapshot']['text']==state.raw_maps
    assert r['runtime_audit_evidence_export']['references']==[r['runtime_audit_last_reference']]
    assert all('audit_reference' in a for a in r['runtime_audit_windows'])


def test_real_six_frame_gap_is_not_approved_by_compact_protocol():
    history=HISTORY.parent/'b3_3_vicon_frame_sample_20261026_144658_024492'
    raw=(history/'frames.json').read_bytes(); result=entry.analyze(json.loads(raw)['rows'])
    assert result['inferred_missing_frame_numbers']==6 and not result['passed']
    assert len(result['continuity']['events'])==1 and not result['flight_authorized']
    report=json.loads((history/'report.json').read_text())
    assert hashlib.sha256(raw).hexdigest()==report['artifact_sha256']['frames.json']


def test_failed_summary_allows_evidence_export_but_not_another_audit(local_helper):
    local_helper('bad_maps'); client=make()
    try:
        summary=client(dict(monitor_shared=False))
        assert not summary['verified'] and 'malformed_proc_maps_line' in summary['error']
        with pytest.raises(RuntimeError,match='failure_latched'): client(dict(monitor_shared=False))
        assert client.status()['requests']==1
        record,=client.finish_and_export()
        assert record['audit_reference']==summary['audit_reference']
        assert record['payload']['classification']['rejection']=='malformed_proc_maps_line'
    finally: assert client.close()


def test_final_pending_rejection_still_exports_its_raw_first_failure(monkeypatch,tmp_path):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    state.frame_period_ns=7_000_000; state.sdk_hz=1e9/state.frame_period_ns; state.audit_delay_ns=25_000_000
    original=entry.read_runtime_maps
    def final_bad(pid):
        if state.now>=19_035_000_000: return 'SYNTHETIC final pending bad maps'
        return original(pid)
    monkeypatch.setattr(entry,'read_runtime_maps',final_bad)
    assert entry.run('OFFLINE_MOCK_FINAL_PENDING_EVIDENCE')==2
    r=json.loads(next((tmp_path/'preparation_runs').glob('b3_3_vicon_frame_sample_*/report.json')).read_text())
    assert not r['passed'] and r['runtime_audit_evidence_export']['verified']
    assert r['runtime_audit_first_failure_evidence']['payload']['snapshot']['text']=='SYNTHETIC final pending bad maps'
    assert r['child_stopped'] and state.metadata_helpers[0].closed and r['automatic_retries']==0
