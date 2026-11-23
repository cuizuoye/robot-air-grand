"""Exercise the REAL entry state/cleanup code with fake ROS, child and clock.

No real Context, subprocess, SDK connection, or device access is permitted.
Mock results remain in pytest temporary directories and grant no authority.
"""
from copy import deepcopy
import json
import os
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import pytest

import vicon_frame_sample_capture as entry
import vicon_receiver_provenance as p
from test_vicon_receiver_provenance import mapped


def install_fake_runtime(monkeypatch,tmp_path,mode,*,shared_present=True):
    policy=json.loads(p.MANIFEST.read_text())
    reviewed=p.verify_current(resolver=lambda target,env:policy['loader_resolutions'][target])
    monkeypatch.setattr(entry,'verify_current',lambda:deepcopy(reviewed))
    (tmp_path/'preparation_runs').mkdir()
    monkeypatch.setattr(entry,'ROOT',tmp_path)
    state=NS(now=1_000_000_000,spawned=False,spawn_count=0,contexts=[],nodes=[],executors=[],commands=[],maps_calls=0,
             shared_identity=dict(pid=321,start_ticks='MOCK_SESSION',exe='/MOCK/shared/receiver'),
             shared_change=None,shared_stall=False,shared_publish=True,audit_delay_ns=0,
             audit_shutdown_ok=True,auditors=[],metadata_helpers=[],frame_period_ns=10_000_000,sdk_hz=100.)
    monkeypatch.setattr(entry,'time',NS(monotonic_ns=lambda:state.now))

    def stamp(ns): return NS(sec=ns//1_000_000_000,nanosec=ns%1_000_000_000)
    def pose(): return NS(position=NS(x=1.,y=2.,z=.3),orientation=NS(x=0.,y=0.,z=0.,w=1.))

    class Context:
        def __init__(self): self.live=False; state.contexts.append(self)
        def ok(self): return self.live
        def shutdown(self): self.live=False
    def init(*,args,context,domain_id):
        assert domain_id in (0,83)
        context.domain=domain_id; context.live=True

    class Node:
        def __init__(self,name,*,context):
            self.name,self.context=name,context; self.callback=None; self.count=0; self.destroyed=False
            state.nodes.append(self)
        def create_subscription(self,kind,topic,callback,qos):
            assert topic in ('/vicon/frame_number',entry.TOPIC)
            self.callback=callback; return NS()
        def get_clock(self): return NS(now=lambda:NS(nanoseconds=100_000_000_000+state.now))
        def get_name(self): return self.name
        def get_node_names_and_namespaces(self): return [(self.name,'/')]
        def destroy_node(self): self.destroyed=True

    class Executor:
        def __init__(self,*,context): self.context=context; self.closed=False; state.executors.append(self)
        def add_node(self,node): self.node=node
        def shutdown(self): self.closed=True
        def spin_once(self,*,timeout_sec):
            node=self.node
            if self.context.domain==0:
                if not state.spawned: state.now+=10_000_000
                node.count+=1
                if node.callback and state.shared_publish and not (state.spawned and state.shared_stall):
                    node.callback(NS(data=node.count))
            else:
                assert state.spawned
                state.now+=state.frame_period_ns
                if mode not in ('normal','single_gap','many_small_gaps','severe_gaps',
                                'duplicate','regression','backlog'): return
                node.count+=1
                sequence=node.count
                if mode=='single_gap':
                    if node.count==30: state.now+=50_000_000
                    if node.count>=30: sequence+=5
                if mode=='many_small_gaps':
                    if node.count in (30,100,170): state.now+=10_000_000
                    sequence+=sum(node.count>=n for n in (30,100,170))
                if mode=='severe_gaps' and node.count>=20:
                    sequence+=node.count-19; state.now+=10_000_000
                if mode=='duplicate' and node.count>=31: sequence-=1
                if mode=='regression' and node.count==30: sequence=10
                lag=600_000_000 if mode=='backlog' else 1_000_000
                # Real native shape, but SYNTHETIC stationary values/time.
                node.callback(NS(sequence=sequence,sdk_frame=10000+sequence,sdk_hz=state.sdk_hz,
                    sdk_ok=True,occluded=False,pose_published=True,sdk_epoch=0,missing_frames=0,queue_dropped=0,
                    sdk_begin_monotonic_ns=state.now-lag-1_000_000,sdk_end_monotonic_ns=state.now-lag,
                    ros_received_stamp=stamp(100_000_000_000+state.now-1_000_000),
                    source_header=NS(stamp=stamp(100_000_000_000+state.now-900_000),frame_id='map'),
                    subject='MircoAir',segment='MircoAir',sdk_pose=pose(),world_pose=pose(),exposure_time_known=False))

    specs={
        'rclpy':dict(init=init), 'rclpy.context':dict(Context=Context),
        'rclpy.executors':dict(SingleThreadedExecutor=Executor),'rclpy.node':dict(Node=Node),
        'rclpy.qos':dict(qos_profile_sensor_data=object()),
        'rclpy.serialization':dict(serialize_message=lambda msg:msg,deserialize_message=lambda msg,cls:msg),
        'vicon_receiver':{},'vicon_receiver.msg':dict(FrameSample=NS),
        'std_msgs':{},'std_msgs.msg':dict(Int64=NS),
    }
    for name,fields in specs.items():
        module=ModuleType(name); module.__dict__.update(fields); module.__path__=[]
        monkeypatch.setitem(sys.modules,name,module)

    class Pipe:
        def fileno(self): return 987654
        def read(self): return b''
        def close(self): state.pipe_closed=True
    class Child:
        def __init__(self,command,**kwargs):
            assert str(entry.BINARY)==command[0]
            assert kwargs['env']['ROS_DOMAIN_ID']=='83'
            assert 'namespace:=vicon_b3_check' in command and 'capture_frame_samples:=true' in command
            assert 'capture_subject:=MircoAir' in command
            assert '/tf:=/vicon_b3_check/tf' in command
            assert '/tf_static:=/vicon_b3_check/tf_static' in command
            assert command.count('--ros-args')==1
            assert command[-2:]==['--log-level','warn']
            assert all('PX4' not in x and 'trs_' not in x and 'Offboard' not in x for x in command)
            self.pid=123456; self.stdout=Pipe(); self.returncode=None
            state.spawned=True; state.spawn_count+=1; state.commands.append(command)
        def poll(self):
            if mode=='receiver_exit' and state.now>=5_000_000_000: self.returncode=-6
            return self.returncode
        def send_signal(self,signal): self.returncode=0
        def wait(self,timeout): return 0
        def terminate(self): pytest.fail('unnecessary termination/retry')
        def kill(self): pytest.fail('unnecessary kill/retry')
    monkeypatch.setattr(entry.subprocess,'Popen',Child)
    original_link=os.readlink; original_read=os.read; original_block=os.set_blocking
    monkeypatch.setattr(entry.os,'readlink',lambda path,*a,**kw:str(entry.BINARY) if str(path)=='/proc/123456/exe' else original_link(path,*a,**kw))
    monkeypatch.setattr(entry.os,'read',lambda fd,n:b'' if fd==987654 else original_read(fd,n))
    monkeypatch.setattr(entry.os,'set_blocking',lambda fd,flag:None if fd==987654 else original_block(fd,flag))
    monkeypatch.setattr(entry,'discover_shared_receiver',
                        lambda:deepcopy(state.shared_identity) if shared_present else None)
    def identity(pid):
        assert shared_present and pid==state.shared_identity['pid']
        if state.spawned and state.now>=5_000_000_000:
            if state.shared_change=='exit': raise ProcessLookupError('mock receiver exited')
            if state.shared_change=='reuse':
                return dict(pid=pid,start_ticks='OTHER_PROCESS',exe='/MOCK/unrelated')
        return deepcopy(state.shared_identity)
    monkeypatch.setattr(entry,'process_identity',identity)
    raw=(mapped(entry.BINARY)+mapped('/etc/ld.so.cache','r--p')+
         '3000-4000 rw-p 0 00:00 0\n4000-5000 r-xp 0 00:00 0 [vdso]\n')
    if mode=='unknown_exec':
        bad=tmp_path/'unexpected'; bad.write_bytes(b'non-ELF code')
        raw+=mapped(bad,'r-xp')
    if mode=='bad_maps': raw+='malformed maps line\n'
    def maps(pid):
        assert pid==123456
        state.maps_calls+=1
        return raw
    monkeypatch.setattr(entry,'read_runtime_maps',maps)
    state.raw_maps=raw

    class FakeMetadataAuditProcess:
        """No real helper is started by full ROS/SDK mock tests."""
        def __init__(self,pid,provenance,shared_before,*,startup_deadline_ns=None):
            assert pid==123456
            assert startup_deadline_ns==state.now+8_000_000_000
            self.pid=pid; self.provenance=provenance; self.shared_before=shared_before; self.closed=False
            from vicon_metadata_audit_process import AuditEvidenceStore
            self.store=AuditEvidenceStore(); self.serial=0; self.exported=False
            state.metadata_helpers.append(self)
        def __call__(self,context):
            self.serial+=1
            return self.store.record(self.serial,entry.runtime_audit_snapshot(
                self.pid,self.provenance,self.shared_before,context))
        def finish_and_export(self):
            self.exported=True
            return [self.store.export(self.store.latest['audit_reference'])]
        def close(self,**kwargs): self.closed=True; return state.audit_shutdown_ok
        def status(self):
            return dict(pid=654321,alive=not self.closed,closed=self.closed,
                        role='SYNTHETIC_metadata_only',forced_stop=False)
    monkeypatch.setattr(entry,'MetadataAuditProcess',FakeMetadataAuditProcess)

    class FakeAuditWorker:
        """Deterministic asynchronous completion, NOT a real thread or I/O.

        Delayed audit completion never advances the live producer's clock.
        Only initial/final wait() may advance it, outside the active loop.
        Real threading, queue bounds and failure behavior have separate tests.
        """
        def __init__(self,inspect,*,clock):
            self.inspect=inspect; self.clock=clock; self.job=None
            self.closed=False; self.fault=None; self.completed=0
            state.auditors.append(self)
        @property
        def pending(self): return self.job is not None
        def submit(self,at_ns,context):
            assert not self.pending and not self.closed and not self.fault
            self.job=dict(submitted_ns=at_ns,deadline_ns=at_ns+entry.RUNTIME_AUDIT_INTERVAL_NS,
                          ready_ns=at_ns+state.audit_delay_ns,context=dict(context))
        def poll(self,at_ns):
            if not self.pending: return None
            job=self.job
            if job['ready_ns']>job['deadline_ns'] or at_ns<job['ready_ns']:
                if at_ns>=job['deadline_ns']:
                    self.fault='runtime_audit_deadline_missed'
                    raise RuntimeError(self.fault)
                return None
            self.job=None; self.completed+=1
            payload=self.inspect(job['context'])
            if payload.get('error'): self.fault=payload['error']
            return dict(submitted_ns=job['submitted_ns'],deadline_ns=job['deadline_ns'],
                        start_ns=job['submitted_ns'],end_ns=job['ready_ns'],context=job['context'],payload=payload)
        def wait(self):
            if self.pending:
                state.now=max(state.now,min(self.job['ready_ns'],self.job['deadline_ns']))
                return self.poll(state.now)
        def close(self):
            self.closed=True
            if isinstance(self.inspect,FakeMetadataAuditProcess): self.inspect.close()
            return state.audit_shutdown_ok
        def status(self):
            return dict(closed=self.closed,thread_alive=not state.audit_shutdown_ok,
                        pending=self.pending,fault=self.fault,completed=self.completed,
                        maximum_outstanding=1,interval_ns=entry.RUNTIME_AUDIT_INTERVAL_NS)
    monkeypatch.setattr(entry,'RuntimeAuditWorker',FakeAuditWorker)
    return state


@pytest.mark.parametrize('mode',['normal','no_frames','unknown_exec','bad_maps','receiver_exit'])
def test_full_entry_start_wait_record_or_abort_and_cleanup_is_mock_only(monkeypatch,tmp_path,mode):
    state=install_fake_runtime(monkeypatch,tmp_path,mode)
    code=entry.run('OFFLINE_MOCK_'+mode)
    docs=list((tmp_path/'preparation_runs').glob('b3_3_vicon_frame_sample_*/report.json'))
    assert len(docs)==1
    report=json.loads(docs[0].read_text())
    assert state.spawn_count==1 and state.maps_calls>0
    assert all(n.destroyed for n in state.nodes)
    assert all(e.closed for e in state.executors)
    assert all(not c.live for c in state.contexts)
    assert state.pipe_closed and report['child_stopped']
    assert len(state.metadata_helpers)==1 and state.metadata_helpers[0].closed
    assert state.metadata_helpers[0].exported and report['runtime_audit_evidence_export']['verified']
    assert report['shared_before']==report['shared_after_cleanup']['identity']
    assert report['shared_after_cleanup']['status']=='unchanged'
    assert not report['PX4_connected'] and report['MAVLink_sent']==0
    assert not report['flight_authorized'] and not report['EV_DELAY_approved']
    assert not report['shared_receiver_modified'] and report['automatic_retries']==0
    assert report['runtime_maps_last_snapshot']['text']==state.raw_maps
    if mode=='normal':
        assert code==0 and report['passed']
        assert report['capture_state']=='OBSERVATION_COMPLETE'
        assert report['analysis']['total']>=1500
        assert report['runtime_maps_last_classification']['category_counts']['non_elf_file']==1
    else:
        assert code==2 and not report['passed']
        assert report['capture_state_at_failure']=='WAITING_FRAME_SAMPLE'
        if mode=='no_frames': assert 'start_timeout' in report['error']
        elif mode=='receiver_exit': assert 'isolated_receiver_exited:-6' in report['error']
        else: assert 'rejection' in report['runtime_maps_last_classification']


def test_runtime_maps_read_is_bounded(monkeypatch):
    from io import StringIO
    monkeypatch.setattr(entry.Path,'open',lambda self:StringIO('x'*262145))
    with pytest.raises(RuntimeError,match='snapshot_capacity_exceeded'):
        entry.read_runtime_maps(123456)


@pytest.mark.parametrize('shared_present,change',[(False,None),(True,'exit'),(True,'reuse')])
def test_independent_capture_does_not_require_shared_process_lifetime(monkeypatch,tmp_path,shared_present,change):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal',shared_present=shared_present)
    state.shared_change=change
    assert entry.run('OFFLINE_MOCK_OPTIONAL_SHARED')==0
    report=json.loads(next((tmp_path/'preparation_runs').glob('b3_3_vicon_frame_sample_*/report.json')).read_text())
    assert report['passed'] and not report['shared_receiver_required']
    assert report['analysis']['observed_frame_span_s']>=15
    assert state.spawn_count==1 and report['child_stopped']
    assert not report['shared_continuity_monitor_completed']
    if not shared_present:
        assert report['shared_discovery']=='none_standalone_capture'
        assert report['shared_after_cleanup']['status']=='not_present'
    else:
        assert report['shared_observations'][-1]['stage']=='capture'
        assert report['shared_observations'][-1]['status'] in ('exited_or_restarted','identity_changed')
    assert all(n.destroyed for n in state.nodes)
    assert all(not c.live for c in state.contexts)
    assert all(e.closed for e in state.executors)
    assert not report['shared_receiver_modified'] and report['MAVLink_sent']==0


def test_same_shared_identity_stall_still_stops_our_capture_only(monkeypatch,tmp_path):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    state.shared_stall=True
    assert entry.run('OFFLINE_MOCK_SHARED_STALL')==2
    report=json.loads(next((tmp_path/'preparation_runs').glob('b3_3_vicon_frame_sample_*/report.json')).read_text())
    assert 'shared_receiver_stalled_after_new_connection' in report['error']
    assert report['child_stopped'] and state.spawn_count==1
    assert report['shared_before']==report['shared_after_cleanup']['identity']
    assert not report['shared_receiver_modified'] and report['automatic_retries']==0


def test_discovery_conflict_is_recorded_without_ros_or_sdk_start(monkeypatch,tmp_path):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    def conflict(): raise RuntimeError('multiple_shared_receiver_candidates:321,654')
    monkeypatch.setattr(entry,'discover_shared_receiver',conflict)
    assert entry.run('OFFLINE_MOCK_SHARED_CONFLICT')==2
    report=json.loads(next((tmp_path/'preparation_runs').glob('b3_3_vicon_frame_sample_*/report.json')).read_text())
    assert report['capture_state_at_failure']=='PRE_START'
    assert 'multiple_shared_receiver_candidates' in report['error']
    assert state.spawn_count==0 and not state.contexts
    assert report['automatic_retries']==0 and not report['flight_authorized']


def test_unavailable_shared_topic_baseline_is_not_an_independent_source_dependency(monkeypatch,tmp_path):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    state.shared_publish=False
    assert entry.run('OFFLINE_MOCK_NO_SHARED_TOPIC')==0
    report=json.loads(next((tmp_path/'preparation_runs').glob('b3_3_vicon_frame_sample_*/report.json')).read_text())
    assert report['shared_before'] is not None and report['shared_baseline_count']==0
    assert not report['shared_continuity_monitor_started']
    assert not report['shared_continuity_monitor_completed']
    assert report['passed'] and state.spawn_count==1 and report['child_stopped']
    assert 'No verified advancing shared baseline' in report['shared_continuity_note']


@pytest.mark.parametrize('invalid_tag',[False,True])
def test_complete_startup_uses_real_configuration_copy_and_actual_discovery_code(monkeypatch,tmp_path,invalid_tag):
    # Do not mock away the discovery/parser stage responsible for the live
    # failure. Only proc metadata, ROS, child execution and clocks are fake.
    real_discover=entry.discover_shared_receiver
    real_identity=entry.process_identity
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    fixtures=Path(__file__).parent/'fixtures'
    meta=json.loads((fixtures/'vicon_shared_process_20261021.json').read_text())
    source=fixtures/meta['config_fixture']
    original=source.read_bytes()
    local=tmp_path/'copied_ROS_parameters.yaml'
    local.write_bytes(original+(b'    unrecognized: !!python/object/new:tuple []\n' if invalid_tag else b''))
    argv=list(meta['argv']); argv[argv.index('--params-file')+1]=str(local)
    proc_root=tmp_path/'offline_proc'
    path=proc_root/str(meta['pid']); path.mkdir(parents=True)
    (path/'exe').symlink_to(meta['exe'])
    (path/'cmdline').write_bytes(b'\0'.join(x.encode() for x in argv)+b'\0')
    (path/'stat').write_text(f'{meta["pid"]} (vicon_client) '+
        ' '.join(['S']+['0']*18+[meta['start_ticks'],'0']))
    (path/'environ').write_bytes(b'ROS_DOMAIN_ID=0\0')
    monkeypatch.setattr(entry,'process_identity',lambda pid,**kw:real_identity(pid,proc_root=proc_root))
    monkeypatch.setattr(entry,'discover_shared_receiver',lambda:real_discover(proc_root=proc_root))
    monkeypatch.setattr(entry.os,'kill',lambda *a,**k:pytest.fail('must not signal shared receiver'))
    result=entry.run('OFFLINE_MOCK_REAL_CONFIGURATION')
    report=json.loads(next((tmp_path/'preparation_runs').glob('b3_3_vicon_frame_sample_*/report.json')).read_text())
    assert source.read_bytes()==original and not report['shared_receiver_modified']
    assert not report['flight_authorized'] and not report['EV_DELAY_approved']
    assert report['MAVLink_sent']==0 and not report['PX4_connected']
    assert report['automatic_retries']==0
    if invalid_tag:
        assert result==2 and state.spawn_count==0 and not state.contexts
        assert report['capture_state_at_failure']=='PRE_START'
        assert 'python/object/new:tuple' in report['error']
    else:
        assert result==0 and report['passed'] and state.spawn_count==1
        assert report['capture_state']=='OBSERVATION_COMPLETE'
        assert report['analysis']['observed_frame_span_s']>=15
        assert report['shared_before']['pid']==meta['pid']  # fixture data, not a live binding
        assert report['shared_before']==report['shared_after_cleanup']['identity']
        assert report['shared_continuity_monitor_completed']
        assert report['provenance']['change']=='exact_CMake_RUNPATH_rewrite_only'
        assert report['runtime_maps_last_classification']['category_counts']['non_elf_file']==1
        assert report['child_stopped'] and state.pipe_closed
        assert all(n.destroyed for n in state.nodes)
        assert all(e.closed for e in state.executors)
        assert all(not c.live for c in state.contexts)


@pytest.mark.parametrize('mode',['normal','no_frames'])
def test_mock_command_is_accepted_by_installed_rcl_and_cleanup_is_complete(monkeypatch,tmp_path,mode):
    from test_vicon_ros_arguments import parse_ros_arguments
    state=install_fake_runtime(monkeypatch,tmp_path,mode)
    result=entry.run('OFFLINE_MOCK_REAL_RCL_'+mode)
    assert result==(0 if mode=='normal' else 2)
    assert len(state.commands)==1
    parsed=parse_ros_arguments(state.commands[0])
    assert parsed['returncode']==0 and parsed['default_log_level']==30
    assert parsed['unparsed_ros_count']==0
    assert parsed['remaps']==dict(node='b3_vicon_isolated_receiver',
        tf='/vicon_b3_check/tf',tf_static='/vicon_b3_check/tf_static')
    assert state.pipe_closed and all(n.destroyed for n in state.nodes)
    assert all(e.closed for e in state.executors) and all(not c.live for c in state.contexts)
