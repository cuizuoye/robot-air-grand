"""Offline GUI commands with fake subprocess only; no ROS/device connection."""
import io
import json
from pathlib import Path
import sys
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from joint_console_run import JointConsoleRun,PLANS,review_conditions,explain_error,make_review
from joint_console_run import DEDICATED_WIFI,prepare_network


class FakeNetwork:
    """No real subprocess, sockets or NetworkManager changes."""
    def __init__(self,*,connected=False):
        self.connected={s[1]:connected for s in DEDICATED_WIFI}
        self.calls=[];self.bad_device=None;self.bad_profile=None;self.other_profile=None
        self.bad_route=False;self.bad_ip=False;self.fail_up=False;self.timeout_up=False
        self.on_up=lambda:None

    def __call__(self,args,**kwargs):
        import subprocess
        self.calls.append(args)
        assert kwargs['capture_output'] and kwargs['text'] and kwargs['timeout']<=35
        if args[:2]==['nmcli','--wait']:
            spec=next(s for s in DEDICATED_WIFI if s[3]==args[6])
            assert args==['nmcli','--wait','30','connection','up','uuid',spec[3],'ifname',spec[1]]
            self.on_up()
            if self.timeout_up:raise subprocess.TimeoutExpired(args,35)
            if self.fail_up:return subprocess.CompletedProcess(args,10,'','热点未找到')
            self.connected[spec[1]]=True
            return subprocess.CompletedProcess(args,0,'connected','')
        if args[0]=='ip':
            spec=next(s for s in DEDICATED_WIFI if s[6]==args[5])
            assert args[6:8]==['from',spec[5]]
            assert args[8:]==(['oif',spec[1]] if spec[0]=='小车' else [])
            data=json.dumps([{'dev':'wlp114s0' if self.bad_route else spec[1],'prefsrc':spec[5]}])
        elif args[5]=='device':
            spec=next(s for s in DEDICATED_WIFI if s[1]==args[-1])
            if self.bad_device==spec[1]:return subprocess.CompletedProcess(args,10,'','设备不存在')
            active=self.connected[spec[1]]
            data='\n'.join(['wifi',spec[2],'100 (connected)' if active else '30 (disconnected)',
                'office-profile' if self.other_profile==spec[1] else spec[3] if active else '',
                ('192.168.4.99' if self.bad_ip else spec[5])+'/24' if active else ''])+'\n'
        else:
            spec=next(s for s in DEDICATED_WIFI if s[3]==args[-1])
            data='\n'.join([spec[1],spec[4],'no' if self.bad_profile==spec[1] else 'yes','yes','','','disabled'])+'\n'
        return subprocess.CompletedProcess(args,0,data,'')

    def activations(self):return [c for c in self.calls if c[:2]==['nmcli','--wait']]


def test_network_both_connected_skips_activation(tmp_path):
    net=FakeNetwork(connected=True)
    assert prepare_network(tmp_path,execute=net)==0
    assert not net.activations()
    result=json.loads((tmp_path/'network_status.json').read_text())
    assert result['phase']=='ready' and len(result['adapters'])==2
    assert not any(x['reconnected'] for x in result['adapters'])


def test_network_only_reconnects_missing_dedicated_link(tmp_path):
    net=FakeNetwork(connected=True);net.connected[DEDICATED_WIFI[1][1]]=False
    assert prepare_network(tmp_path,execute=net)==0
    assert len(net.activations())==1
    assert net.activations()[0][-1]==DEDICATED_WIFI[1][1]
    # Every mutation is an explicit saved UUID + fixed dedicated adapter.
    assert not any('modify' in c or 'disconnect' in c or 'down' in c for c in net.calls)


@pytest.mark.parametrize('fault',['bad_device','bad_profile'])
def test_network_checks_both_bindings_before_any_reconnect(tmp_path,fault):
    net=FakeNetwork();setattr(net,fault,DEDICATED_WIFI[1][1])
    assert prepare_network(tmp_path,execute=net)==1
    assert not net.activations()
    assert json.loads((tmp_path/'network_status.json').read_text())['phase']=='failed'


def test_network_does_not_take_over_other_use_of_dedicated_adapter(tmp_path):
    net=FakeNetwork();net.other_profile=DEDICATED_WIFI[0][1]
    assert prepare_network(tmp_path,execute=net)==1
    assert not net.activations()


@pytest.mark.parametrize('fault',['bad_route','bad_ip','fail_up','timeout_up'])
def test_network_failure_records_reason_without_blind_retry(tmp_path,fault):
    net=FakeNetwork();setattr(net,fault,True)
    assert prepare_network(tmp_path,execute=net)==1
    result=json.loads((tmp_path/'network_status.json').read_text())
    assert result['phase']=='failed' and result['raw']
    assert len(net.activations())==1  # does not proceed to the second adapter


def test_network_cancel_during_reconnect_stops_next_adapter(tmp_path):
    net=FakeNetwork();net.on_up=lambda:(tmp_path/'network_cancel').touch()
    assert prepare_network(tmp_path,execute=net)==1
    assert len(net.activations())==1
    assert json.loads((tmp_path/'network_status.json').read_text())['phase']=='cancelled'


def test_network_cancel_before_start_executes_nothing(tmp_path):
    net=FakeNetwork();(tmp_path/'network_cancel').touch()
    assert prepare_network(tmp_path,execute=net)==1 and not net.calls


class FakeProcess:
    def __init__(self):self.stdin=io.StringIO();self.returncode=None
    def poll(self):return self.returncode


def running(tmp_path):
    proc=FakeProcess();calls=[];reviews=[];now=[100.]
    def spawn(argv,**kwargs):calls.append((argv,kwargs));return proc
    def review(run,profile,plan,flight,consent):
        assert all(consent.values())
        reviews.append(flight);p=run/('execution_review.json' if flight else 'ground_review.json')
        p.write_text('{}');return p
    r=JointConsoleRun(popen=spawn,review_factory=review,clock=lambda:now[0])
    r.launch(PLANS['矩形 3×2 m'],dict.fromkeys(review_conditions(),True),base=tmp_path)
    return r,proc,calls,reviews,now


def ready(r,phase='WAIT_START'):
    r.status={'phase':'WAIT_EXPLICIT_START' if phase=='WAIT_TAKEOFF' else 'PREPARE',
        'missing':[],'failure':None,'operator_action':None,'cooperative':{
            'phase':'WAIT_START','car_task_phase':phase,'car_stopped':True,'fault':None}}
    r.status_at=1.;r.progress_at=100.


def test_gui_launch_only_starts_existing_owner_and_freezes_plan(tmp_path):
    r,p,calls,reviews,_=running(tmp_path)
    assert len(calls)==1 and 'vicon_fixed_point_entry.py' in calls[0][0][-1]
    assert '--cooperative-plan' in calls[0][0][-1]
    command=calls[0][0][-1]
    assert command.index('--prepare-network')<command.index(' && source ')<command.index('vicon_fixed_point_entry.py')
    assert calls[0][1]['start_new_session'] is True
    assert calls[0][1]['stdout'].name.endswith('backend_output.log')
    assert p.stdin.getvalue()=='' and not reviews
    assert json.loads((r.run/'plan.json').read_text())['car_speed_mps']==.25
    with pytest.raises(RuntimeError,match='不重复启动'):
        r.launch(PLANS['矩形 3×2 m'],{},base=tmp_path)


def test_failed_network_child_never_sends_ground_and_allows_retry(tmp_path):
    r,p,calls,reviews,_=running(tmp_path)
    (r.run/'network_status.json').write_text(json.dumps(dict(phase='failed',message='无人机热点连接失败',raw='SSID not found')))
    p.returncode=1;r.poll();r.poll()
    assert not reviews and p.stdin.getvalue()==''
    assert r.finished and r.cleanup_ok and r.stage()=='网卡准备未完成'
    assert r.issue['raw']=='SSID not found'
    result=json.loads((r.run/'gui_result.json').read_text())
    assert result['hardware_backend_started'] is False
    assert 'cleanup_verified' not in result
    r.launch(PLANS['矩形 3×2 m'],dict.fromkeys(review_conditions(),True),base=tmp_path)
    assert len(calls)==2
    r.stdout.close()


def test_missing_consent_or_external_owner_cannot_launch(tmp_path):
    calls=[];r=JointConsoleRun(popen=lambda *a,**k:calls.append(a))
    with pytest.raises(PermissionError):r.launch(PLANS['矩形 3×2 m'],{},base=tmp_path)
    with pytest.raises(RuntimeError):r.launch(PLANS['矩形 3×2 m'],{},base=tmp_path,existing='owned')
    assert not calls


def test_ground_review_once_and_no_automatic_car_or_flight(tmp_path):
    r,p,_,reviews,_=running(tmp_path)
    s=r.run/'session';s.mkdir();(s/'flight_recovery.json').write_text('{}')
    r.poll();r.poll()
    assert reviews==[False]
    assert p.stdin.getvalue().count('ground ')==1
    assert 'CAR_START' not in p.stdin.getvalue() and 'TAKEOFF' not in p.stdin.getvalue()


def test_prepare_button_needs_no_dialog_and_cannot_start_motion(tmp_path,monkeypatch):
    import drone_console
    from types import SimpleNamespace
    proc=FakeProcess();calls=[];reviews=[]
    def spawn(*args,**kwargs):calls.append(args);return proc
    def review(run,profile,plan,flight,consent):
        reviews.append(flight)
        assert consent['_confirmation_method']=='single_prepare_click'
        path=run/'ground_review.json';path.write_text('{}');return path
    runner=JointConsoleRun(popen=spawn,review_factory=review)
    launch=runner.launch
    monkeypatch.setattr(runner,'launch',lambda plan,consent,**kw:launch(plan,consent,base=tmp_path,**kw))
    monkeypatch.setattr(drone_console,'active_joint_directory',lambda _:None)
    ui=drone_console.Console.__new__(drone_console.Console)
    ui.runner=runner;ui.task_choice=SimpleNamespace(get=lambda:'矩形 3×2 m')
    ui.source=SimpleNamespace(set=lambda _:None)
    def unexpected_dialog(*args,**kwargs):raise AssertionError('ground preparation must not open a dialog')
    ui.consent=unexpected_dialog
    ui.prepare_run()
    assert len(calls)==1 and runner.issue is None and proc.stdin.getvalue()==''
    saved=json.loads((runner.run/'operator_ground_consent.json').read_text())
    assert saved['_confirmation_method']=='single_prepare_click'
    assert all(saved[key] is True for key in review_conditions())
    session=ui.owned_directory;session.mkdir();(session/'flight_recovery.json').write_text('{}')
    runner.poll();runner.poll()
    assert reviews==[False] and proc.stdin.getvalue().count('ground ')==1
    ui.prepare_run()  # a rapid second click must not spawn another owner
    assert len(calls)==1 and '不重复启动' in runner.issue['raw']
    assert 'CAR_START' not in proc.stdin.getvalue() and 'TAKEOFF' not in proc.stdin.getvalue()
    runner.stdout.close()


def test_stable_status_uses_live_progress_not_event_age_and_click_only(tmp_path):
    r,p,_,_,now=running(tmp_path);ready(r)
    assert r.can_align()  # unchanged status event at t=1, fresh progress t=100
    r.align();assert p.stdin.getvalue()=='CAR_START\n'
    with pytest.raises(RuntimeError):r.align()
    assert not r.takeoff_sent
    now[0]=110;assert not r.can_preview()


def test_preview_is_not_takeoff_and_confirmation_is_one_use(tmp_path):
    r,p,_,reviews,now=running(tmp_path);ready(r,'WAIT_TAKEOFF')
    r.request_preview(dict.fromkeys(review_conditions(True,r.profile),True))
    assert reviews==[True] and 'preview\n' in p.stdin.getvalue() and 'TAKEOFF' not in p.stdin.getvalue()
    preview={'confirmation_text':'TAKEOFF test-only','height_m':.5}
    r.accept_event({'at_ns':100_000_000_000,'text':json.dumps(preview)})
    with pytest.raises(PermissionError):r.takeoff(preview)
    assert 'TAKEOFF' not in p.stdin.getvalue()
    r.takeoff(preview,confirmed=True)
    assert p.stdin.getvalue().count('TAKEOFF test-only')==1
    with pytest.raises(PermissionError):r.takeoff(preview,confirmed=True)


def test_old_historical_preview_cannot_arm_and_expiry_requires_new_preview(tmp_path):
    r,p,_,_,now=running(tmp_path);ready(r,'WAIT_TAKEOFF')
    token={'confirmation_text':'TAKEOFF old'}
    r.accept_event({'at_ns':1,'text':json.dumps(token)})
    assert r.preview is None
    r.preview_requested=True;r.accept_event({'at_ns':100_000_000_000,'text':json.dumps(token)})
    now[0]=121;r.progress_at=121
    with pytest.raises(PermissionError):r.takeoff(token,confirmed=True)
    assert not r.takeoff_sent and 'TAKEOFF' not in p.stdin.getvalue()


def test_cancel_keeps_child_alive_and_blocks_new_motion(tmp_path):
    r,p,_,_,_=running(tmp_path);ready(r)
    r.stop();r.stop()
    assert p.stdin.getvalue()=='cancel\n' and r.running
    with pytest.raises(RuntimeError):r.align()
    assert not r.can_preview()


def test_command_rejection_is_explained_and_not_automatically_retried(tmp_path):
    r,p,_,_,_=running(tmp_path);ready(r);r.align()
    r.accept_event({'at_ns':100_000_000_000,'text':'Command rejected: car_must_start_on_clear_left_approach_side'})
    assert '左侧' in r.issue['reason'] and r.issue['raw'].startswith('Command rejected:')
    assert p.stdin.getvalue().count('CAR_START')==1
    assert not r.car_requested


def test_first_failure_keeps_original_phase_and_raw_reason(tmp_path):
    r,_,_,_,_=running(tmp_path);r.takeoff_sent=True
    r.status={'phase':'HOVER','cooperative':{'phase':'FOLLOW'}}
    for fault in ('CAR_TRANSPORT_LOST','LAND_TIMEOUT'):
        r.accept_event({'at_ns':100_000_000_000,'text':json.dumps({'phase':'BLOCKED','failure':fault,'cooperative':{},'failure_snapshot':{'phase':'BLOCKED'}})})
    assert r.first_failure['phase']=='车机跟随' and r.issue['raw']=='CAR_TRANSPORT_LOST'


def test_exit_code_zero_is_not_cleanup_proof(tmp_path):
    r,p,_,_,_=running(tmp_path);p.returncode=0;r.poll()
    assert r.finished and not r.cleanup_ok and r.issue
    assert '尚未确认' in r.guidance()


def test_verified_cleanup_allows_next_round_without_auto_restart(tmp_path):
    r,p,calls,_,_=running(tmp_path);s=r.run/'session';s.mkdir()
    for n in ('flight_recovery.json','rates_recovery.json'):
        (s/n).write_text(json.dumps({'verified_closed':True,'needs_restore':False}))
    (s/'joint_stop.json').write_text(json.dumps({'car_stop_confirmed':True,'errors':[]}))
    p.returncode=0;r.poll();assert r.cleanup_ok and r.finished and len(calls)==1


def test_unknown_error_is_not_presented_as_known_hardware_fault():
    d=explain_error('unexpected-value-abc','移至车上方')
    assert d['phase']=='移至车上方' and d['raw']=='unexpected-value-abc'
    assert '不足以确定根因' in d['reason']


def test_missing_physical_review_never_creates_authorized_file(tmp_path):
    with pytest.raises(PermissionError):make_review(tmp_path,{}, {},False,{})
    assert not list(tmp_path.iterdir())


def test_complete_stage_sequence_only_sends_explicit_commands(tmp_path):
    r,p,_,_,_=running(tmp_path);ready(r);r.align()
    ready(r,'WAIT_TAKEOFF')
    r.request_preview(dict.fromkeys(review_conditions(True,r.profile),True))
    preview={'confirmation_text':'TAKEOFF offline-only','height_m':.5}
    r.accept_event({'at_ns':100_000_000_000,'text':json.dumps(preview)})
    r.takeoff(preview,confirmed=True)
    for phase,label in [('TAKEOFF','无人机起飞'),('RENDEZVOUS','移至车上方'),('FOLLOW','车机跟随'),
                        ('WAIT_CAR_STOP','小车停车'),('ALIGN','返回起飞点'),('DESCEND','缓慢下降'),
                        ('LAND','接地与停桨'),('COMPLETE','任务完成 / 恢复回传')]:
        r.accept_event({'at_ns':100_000_000_000,'text':json.dumps({'phase':'HOVER','failure':None,
            'cooperative':{'phase':phase,'car_task_phase':'WAIT_LANDING'}})})
        assert r.stage()==label
    sent=p.stdin.getvalue().splitlines()
    assert sent[0]=='CAR_START' and sent[1].startswith('review ')
    assert sent[2:]==['preview','TAKEOFF offline-only']


def test_raw_core_phase_does_not_hide_plain_language_failure_stage(tmp_path):
    r,_,_,_,_=running(tmp_path);r.takeoff_sent=True
    r.status={'phase':'HOVER','cooperative':{'phase':'DESCEND'}}
    r.accept_event({'at_ns':100_000_000_000,'text':json.dumps({'phase':'LAND',
        'failure':'LAND_TIMEOUT','failure_snapshot':{'phase':'HOVER'},'cooperative':{'phase':'LAND'}})})
    assert r.issue['phase']=='缓慢下降' and r.issue['snapshot']['phase']=='HOVER'


def test_circle_launch_uses_existing_owner_and_keeps_takeoff_explicit(tmp_path):
    from joint_console_run import route_description
    calls=[];proc=FakeProcess()
    def spawn(args,**kwargs):calls.append(args);return proc
    runner=JointConsoleRun(popen=spawn)
    runner.launch(PLANS['圆周 直径1.5 m'],dict.fromkeys(review_conditions(),True),base=tmp_path)
    assert len(calls)==1 and 'vicon_fixed_point_entry.py' in calls[0][-1]
    assert proc.stdin.getvalue()==''
    assert runner.plan['circle_route']['diameter_m']==1.5
    assert len(runner.plan['route_local_forward_left_m'])==48
    assert '逆时针' in route_description(runner.plan)
    assert len(route_description(runner.plan))<100


@pytest.mark.parametrize('source_key',['from','src','prefsrc'])
@pytest.mark.parametrize('wrong_source',[False,True])
def test_network_explicit_source_iproute2_schema(tmp_path,source_key,wrong_source):
    net=FakeNetwork(connected=True)
    def execute(args,**kwargs):
        result=net(args,**kwargs)
        if args[0]=='ip':
            rows=json.loads(result.stdout)
            src=rows[0].pop('prefsrc')
            rows[0][source_key]='192.168.4.99' if wrong_source else src
            result.stdout=json.dumps(rows)
        return result
    assert prepare_network(tmp_path,execute=execute)==(1 if wrong_source else 0)
    assert not net.activations()
