"""Only the joint collector's new metadata budget/evidence path; OFFLINE."""
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
from types import SimpleNamespace as NS
import pytest
import ev_delay_vicon_source as source
import vicon_frame_sample_capture as frame
import vicon_metadata_audit_process as transport
from ev_delay_live_owner import Owner

REAL_POPEN=subprocess.Popen


@pytest.fixture
def helper(monkeypatch):
    children=[]
    def install(delay=.2,fail=False):
        script=f'''
import sys,os,time
sys.path.insert(0,{str(Path(transport.__file__).parent)!r})
def fence(event,args):
 if event.startswith('socket.') or event=='subprocess.Popen':raise AssertionError('OFFLINE network/process access')
 if event=='import' and args[0].split('.')[0] in ('rclpy','vicon_receiver'):raise AssertionError('OFFLINE ROS access')
 if event=='open' and isinstance(args[0],(str,bytes)) and os.fsdecode(args[0]).startswith(('/dev/tty','/dev/serial','/dev/input','/dev/bus')):raise AssertionError('OFFLINE device access')
sys.addaudithook(fence)
import vicon_frame_sample_capture as f
import vicon_metadata_audit_process as m
f.read_runtime_maps=lambda pid:'SYNTHETIC RAW MAPS; no real PID inspected'
def verify(raw,provenance,details):
 time.sleep({delay!r})
 if {fail!r}:raise RuntimeError('synthetic_unreviewed_library')
 return ['SYNTHETIC approved library']
f.verify_runtime_maps=verify
m.metadata_worker_main()
os._exit(0)
'''
        def spawn(command,**kwargs):
            assert command==[sys.executable,'-B',str(Path(transport.__file__).resolve()),'--metadata-worker']
            p=REAL_POPEN([sys.executable,'-B','-c',script],**kwargs);children.append(p);return p
        monkeypatch.setattr(transport.subprocess,'Popen',spawn)
        monkeypatch.setattr(socket,'socket',lambda *a,**k:pytest.fail('OFFLINE socket'))
    yield install
    for child in children:
        if child.poll() is None:
            child.kill();child.wait(timeout=1);pytest.fail('metadata child leaked')


def test_200ms_metadata_does_not_stall_owner_or_abort_and_keeps_raw(tmp_path,helper):
    helper(.2)
    client=transport.MetadataAuditProcess(123,{},None,request_timeout_ns=source.METADATA_TIMEOUT_NS,evidence_dir=tmp_path/'evidence')
    worker=frame.RuntimeAuditWorker(client,timeout_ns=source.METADATA_TIMEOUT_NS)
    try:
        start=time.monotonic_ns();worker.submit(start,dict(monitor_shared=False));ticks=0;result=None
        while result is None:
            result=worker.poll(time.monotonic_ns());ticks+=1;time.sleep(.001)
        assert ticks>10 and result['payload']['verified']
        assert result['end_ns']-result['start_ns']>=200_000_000
        assert worker.status()['maximum_outstanding']==1
        exported,=client.finish_and_export()
        disk=json.loads((tmp_path/'evidence/latest_result.json').read_text())
        assert disk==exported
        assert 'SYNTHETIC RAW MAPS' in json.loads((tmp_path/'evidence/raw_maps.json').read_text())['snapshot']['text']
    finally:assert worker.close()


def test_stuck_metadata_stops_at_one_second_and_retains_preclassification_snapshot(tmp_path,helper):
    helper(2)
    client=transport.MetadataAuditProcess(123,{},None,request_timeout_ns=source.METADATA_TIMEOUT_NS,evidence_dir=tmp_path/'evidence')
    worker=frame.RuntimeAuditWorker(client,timeout_ns=source.METADATA_TIMEOUT_NS)
    try:
        start=time.monotonic_ns();worker.submit(start,dict(monitor_shared=False))
        with pytest.raises(RuntimeError,match='deadline_missed'):
            while True:
                row=worker.poll(time.monotonic_ns())
                if row and row['payload'].get('error'):raise RuntimeError(row['payload']['error'])
                time.sleep(.001)
        elapsed=(time.monotonic_ns()-start)/1e9
        assert .95<=elapsed<1.5
        assert (tmp_path/'evidence/raw_maps.json').exists()
        assert json.loads((tmp_path/'evidence/request.json').read_text())['serial']==1
        assert client.status()['request_timeout_ns']==1_000_000_000
    finally:worker.close()
    assert not client.status()['alive']


def test_actual_provenance_rejection_still_fails_and_persists_first_failure(tmp_path,helper):
    helper(.15,True)
    client=transport.MetadataAuditProcess(123,{},None,request_timeout_ns=source.METADATA_TIMEOUT_NS,evidence_dir=tmp_path/'evidence')
    worker=frame.RuntimeAuditWorker(client,timeout_ns=source.METADATA_TIMEOUT_NS)
    try:
        worker.submit(time.monotonic_ns(),dict(monitor_shared=False));row=worker.wait()
        s=object.__new__(source.ViconSource);s.report={'audits':[]}
        with pytest.raises(RuntimeError,match='unreviewed_library'):s.accept(row)
        failed=json.loads((tmp_path/'evidence/first_failure.json').read_text())
        assert 'unreviewed_library' in failed['payload']['error']
        assert s.report['audits']==[row]
        with pytest.raises(RuntimeError,match='failed'):worker.submit(time.monotonic_ns(),{})
    finally:assert worker.close()




@pytest.mark.parametrize('value',[0,99_999_999,1_000_000_001,True,1.0])
def test_budget_cannot_be_disabled_or_unbounded(value):
    with pytest.raises(ValueError):frame.RuntimeAuditWorker(lambda c:{},timeout_ns=value)


def test_completed_after_joint_deadline_is_not_accepted():
    clock=[0]
    def inspect(context):clock[0]=1_000_000_001;return {'verified':True}
    worker=frame.RuntimeAuditWorker(inspect,clock=lambda:clock[0],timeout_ns=source.METADATA_TIMEOUT_NS)
    try:
        worker.submit(0,{})
        assert worker._pending['done'].wait(1)
        row=worker.poll(clock[0]);assert row['payload']['error']=='runtime_audit_deadline_missed'
    finally:assert worker.close()


def test_joint_pose_staleness_is_still_150ms_with_one_second_metadata_budget():
    o=object.__new__(Owner)
    o.source=NS(latest=dict(subject='MircoAir',segment='MircoAir',sdk_ok=True,occluded=False,pose_published=True,queue_dropped=0,frame_id='map',ros_stamp_ns=0),now_ros_ns=lambda:150_000_000)
    with pytest.raises(RuntimeError,match='stale'):o.source_pose()


def test_atomic_snapshot_write_error_is_not_success(monkeypatch,tmp_path):
    monkeypatch.setattr(frame,'read_runtime_maps',lambda pid:'SYNTHETIC maps')
    monkeypatch.setattr(frame,'persist_evidence',lambda *a:(_ for _ in ()).throw(OSError('synthetic disk full')))
    payload=frame.runtime_audit_snapshot(123,{},None,dict(monitor_shared=False),snapshot_path=tmp_path/'raw_maps.json')
    assert 'disk full' in payload['error'] and payload['snapshot']['text']=='SYNTHETIC maps'
