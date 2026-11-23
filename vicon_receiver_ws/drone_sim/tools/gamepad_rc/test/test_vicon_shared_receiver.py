"""Shared receiver discovery from synthetic /proc only; never start ROS/SDK."""
from copy import deepcopy
from pathlib import Path

import pytest

import vicon_frame_sample_capture as entry


@pytest.fixture
def processes(tmp_path,monkeypatch):
    proc=tmp_path/'proc'; proc.mkdir()
    executable=tmp_path/'build/vicon_client'
    executable.parent.mkdir(); executable.write_bytes(b'SYNTHETIC_NOT_EXECUTED')
    installed=tmp_path/'install/vicon_client'
    installed.parent.mkdir(); installed.symlink_to(executable)
    isolated=tmp_path/'isolated/vicon_client'
    isolated.parent.mkdir(); isolated.write_bytes(b'SYNTHETIC_ISOLATED')
    monkeypatch.setattr(entry,'SHARED_BINARY',installed)
    monkeypatch.setattr(entry,'BINARY',isolated)
    def create(pid,*,ticks=100,exe=None,domain='0',argv=None,hostname='10.10.10.2',namespace='vicon'):
        path=proc/str(pid); path.mkdir()
        exe=exe or executable
        (path/'exe').symlink_to(exe)
        (path/'stat').write_text(f'{pid} (vicon client) '+' '.join(['S']+['0']*18+[str(ticks),'0']))
        config=tmp_path/f'params_{pid}.yaml'
        config.write_text('/**:\n  ros__parameters:\n'+f'    hostname: {hostname}\n    namespace: {namespace}\n')
        argv=argv if argv is not None else [str(installed),'--ros-args','--params-file',str(config)]
        (path/'cmdline').write_bytes(b'\0'.join(x.encode() for x in argv)+b'\0')
        (path/'environ').write_bytes(f'ROS_DOMAIN_ID={domain}\0PRIVATE_SECRET=not_retained\0'.encode())
        return path
    return proc,create,executable,installed,isolated


def test_normal_receiver_identity_includes_executable_arguments_and_start_tick(processes):
    proc,create,exe,installed,_=processes
    create(1101,ticks=77)
    actual=entry.discover_shared_receiver(proc_root=proc)
    assert actual['pid']==1101 and actual['start_ticks']=='77'
    assert actual['exe']==str(exe)
    assert actual['exe_inode']==exe.stat().st_ino
    assert actual['argv'][0]==str(installed) and actual['ros_domain']=='0'
    assert 'PRIVATE_SECRET' not in repr(actual)


def test_pid_changes_and_receiver_restart_are_rediscovered(processes):
    proc,create,*_=processes
    old=create(1101,ticks=20)
    first=entry.discover_shared_receiver(proc_root=proc)
    # An exited PID may still have a transient proc entry, but no executable.
    (old/'exe').unlink()
    assert entry.discover_shared_receiver(proc_root=proc) is None
    create(2202,ticks=30)
    second=entry.discover_shared_receiver(proc_root=proc)
    assert first['pid']==1101 and second['pid']==2202
    assert first['start_ticks']!=second['start_ticks']


def test_same_pid_new_start_tick_is_a_new_session(processes):
    proc,create,*_=processes
    path=create(1101,ticks=20)
    first=entry.discover_shared_receiver(proc_root=proc)
    (path/'stat').write_text('1101 (vicon_client) '+' '.join(['S']+['0']*18+['99','0']))
    second=entry.discover_shared_receiver(proc_root=proc)
    assert second['pid']==first['pid'] and second!=first


def test_pid_reused_by_other_executable_is_never_selected(processes,tmp_path):
    proc,create,*_=processes
    unrelated=tmp_path/'sleep'; unrelated.write_bytes(b'SYNTHETIC_OTHER')
    create(1101,exe=unrelated,argv=[str(unrelated)])
    assert entry.discover_shared_receiver(proc_root=proc) is None


def test_no_shared_receiver_is_explicit_standalone_case(processes):
    proc,*_=processes
    assert entry.discover_shared_receiver(proc_root=proc) is None


def test_multiple_same_name_candidates_report_conflict_without_selecting(processes):
    proc,create,*_=processes
    create(1101); create(2202)
    with pytest.raises(RuntimeError,match='multiple_shared_receiver_candidates:1101,2202'):
        entry.discover_shared_receiver(proc_root=proc)


@pytest.mark.parametrize('case,reason',[
    ('wrong_exe','executable_or_command_line_mismatch'),
    ('wrong_argv','executable_or_command_line_mismatch'),
    ('no_ros_args','executable_or_command_line_mismatch'),
    ('wrong_domain','not_shared_ROS_domain_0'),
    ('wrong_host','shared_tracker_or_topic_namespace_mismatch'),
    ('wrong_namespace','shared_tracker_or_topic_namespace_mismatch'),
    ('missing_yaml','No such file'),
])
def test_unverified_receiver_is_not_silently_used(processes,tmp_path,case,reason):
    proc,create,exe,installed,_=processes
    if case=='wrong_exe':
        wrong=tmp_path/'other/vicon_client'; wrong.parent.mkdir(); wrong.write_bytes(b'OTHER')
        create(1101,exe=wrong)
    elif case=='wrong_argv': create(1101,argv=['/wrong/vicon_client','--ros-args'])
    elif case=='no_ros_args': create(1101,argv=[str(installed)])
    elif case=='wrong_domain': create(1101,domain='83')
    elif case=='wrong_host': create(1101,hostname='192.0.2.1')
    elif case=='wrong_namespace': create(1101,namespace='other')
    else:
        create(1101); (tmp_path/'params_1101.yaml').unlink()
    with pytest.raises(RuntimeError,match='shared_receiver_identity_unverified:1101:') as error:
        entry.discover_shared_receiver(proc_root=proc)
    assert reason in str(error.value)


def test_isolated_receiver_is_not_mistaken_for_shared(processes):
    proc,create,_,_,isolated=processes
    create(1101,exe=isolated,domain='83',argv=[str(isolated),'--ros-args'])
    assert entry.discover_shared_receiver(proc_root=proc) is None


def test_identity_rejects_pid_reuse_during_metadata_read(processes,monkeypatch):
    proc,create,*_=processes
    create(1101)
    read_text=Path.read_text; calls=0
    def changed(self,*args,**kwargs):
        nonlocal calls
        text=read_text(self,*args,**kwargs)
        if self==proc/'1101/stat':
            calls+=1
            if calls==2: return text.replace('100','101')
        return text
    monkeypatch.setattr(Path,'read_text',changed)
    with pytest.raises(ProcessLookupError,match='process_changed_during_identity_read:1101'):
        entry.process_identity(1101,proc_root=proc)


def test_identity_change_before_discovery_finishes_is_refused(processes,monkeypatch):
    proc,create,*_=processes
    create(1101)
    original=entry.process_identity; calls=0
    def changed(pid,**kwargs):
        nonlocal calls
        result=original(pid,**kwargs); calls+=1
        if calls==2: result['start_ticks']='REUSED'
        return result
    monkeypatch.setattr(entry,'process_identity',changed)
    with pytest.raises(RuntimeError,match='process_changed_during_discovery'):
        entry.discover_shared_receiver(proc_root=proc)


def test_executable_inode_must_match_expected_source(processes,monkeypatch):
    proc,create,*_=processes
    create(1101)
    original=entry.process_identity
    def wrong_inode(pid,**kwargs):
        result=original(pid,**kwargs); result['exe_inode']+=1
        return result
    monkeypatch.setattr(entry,'process_identity',wrong_inode)
    with pytest.raises(RuntimeError,match='executable_or_command_line_mismatch'):
        entry.discover_shared_receiver(proc_root=proc)


def test_inline_ros_parameters_and_default_ros_domain_are_supported(processes):
    proc,create,_,installed,_=processes
    path=create(1101,argv=[str(installed),'--ros-args','-p','hostname:=10.10.10.2',
                           '-p','namespace:=vicon'])
    (path/'environ').write_bytes(b'UNRELATED=not_retained\0')
    actual=entry.discover_shared_receiver(proc_root=proc)
    assert actual['pid']==1101 and actual['ros_domain']=='0'


@pytest.mark.parametrize('change,status',[(None,'unchanged'),('exit','exited_or_restarted'),('reuse','identity_changed')])
def test_monitor_is_read_only_and_never_follows_reused_pid(monkeypatch,change,status):
    original=dict(pid=111,start_ticks='1',exe='/mock/vicon_client')
    def read(pid):
        assert pid==111
        if change=='exit': raise ProcessLookupError('gone')
        result=deepcopy(original)
        if change=='reuse': result.update(start_ticks='2',exe='/mock/other')
        return result
    monkeypatch.setattr(entry,'process_identity',read)
    monkeypatch.setattr(entry.os,'kill',lambda *a:pytest.fail('must not signal shared process'))
    assert entry.shared_receiver_observation(original)['status']==status
    assert original==dict(pid=111,start_ticks='1',exe='/mock/vicon_client')
