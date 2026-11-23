"""New console lifecycle only; no devices and no repeated fusion/pilot checks."""
from types import SimpleNamespace
import pytest
import vicon_fixed_point_entry as entry

class Controller:
    origin_monotonic=0.
    def __init__(self):
        self.done=False;self.steps=0;self.cancelled=False;self.cancel_step=None
        self.confirmations=[];self.previews=0;self.closed=False
        self.start=SimpleNamespace(create=self.preview,confirm=self.confirm)
    def preview(self):
        self.previews+=1
        return {'confirmation_text':'TAKEOFF synthetic','flight_authorized':False}
    def confirm(self,text):
        if text!='TAKEOFF synthetic':raise PermissionError('token mismatch')
        self.confirmations.append(text)
    def step(self,now):
        self.steps+=1
        if self.cancelled and self.steps>=self.cancel_step+3:self.done=True
        if self.confirmations and self.steps>=6:self.done=True
        if self.steps>12:raise AssertionError('console failed to finish test')
        return self.status()
    def status(self):return dict(phase='COMPLETE' if self.done else 'WAIT_EXPLICIT_START',missing_conditions=[],intervention_required=False)
    def cancel(self):
        if not self.cancelled:self.cancel_step=self.steps
        self.cancelled=True
    def close(self):
        assert self.done
        self.closed=True
    def install_bootstrap_review(self,path):raise PermissionError('unapproved review')

def run(c,lines,**kwargs):
    it=iter(lines)
    return entry.serve_console(c,lambda:next(it,None),clock=lambda:0.,idle=lambda:None,emit=lambda s:None,**kwargs)

def test_preview_and_invalid_token_never_start():
    c=Controller();run(c,['preview\n','TAKEOFF wrong\n',''])
    assert c.previews==1 and not c.confirmations and c.cancelled and c.closed

def test_only_explicit_exact_token_is_forwarded():
    c=Controller();run(c,['preview\n',None,'TAKEOFF synthetic\n'])
    assert c.confirmations==['TAKEOFF synthetic'] and c.closed

def test_eof_keeps_servicing_until_safe_completion():
    c=Controller();run(c,[''])
    assert c.steps==c.cancel_step+3 and c.closed and not c.confirmations


def test_cleanup_error_is_visible_to_existing_console_mirror():
    c=Controller();out=[]
    def close():raise RuntimeError('小车停车未确认，见 joint_stop.json')
    c.close=close
    with pytest.raises(RuntimeError,match='停车未确认'):
        entry.serve_console(c,lambda:'',clock=lambda:0.,idle=lambda:None,emit=out.append)
    assert any('资源收尾未完全确认' in line and 'joint_stop.json' in line for line in out)

def test_interrupt_requests_cancel_without_dropping_owner():
    c=Controller();calls=0
    def reader():
        nonlocal calls
        calls+=1
        if calls==1:raise KeyboardInterrupt()
    entry.serve_console(c,reader,emit=lambda s:None,clock=lambda:0.,idle=lambda:None)
    assert c.steps==c.cancel_step+3 and c.closed

def test_rejected_review_does_not_end_service_or_create_start():
    c=Controller();run(c,['ground unapproved.json\n',''])
    assert c.cancel_step==2 and c.closed and not c.confirmations


def test_console_reports_mode_action_and_failure_snapshot_without_per_turn_spam():
    import json
    c=Controller();original=c.status;out=[]
    snapshot=dict(at=3.,reason='send_pose_stale',ages_s={'source_age':.17})
    def status():
        row=original()
        row.update(operator_action='退出遗留 Offboard',failure='send_pose_stale',failure_snapshot=snapshot)
        return row
    c.status=status;lines=iter([None,None,''])
    entry.serve_console(c,lambda:next(lines,None),clock=lambda:0.,idle=lambda:None,emit=out.append)
    rows=[json.loads(line) for line in out if line.startswith('{')]
    assert len(rows)==2  # Initial state and terminal phase, not every turn.
    assert rows[0]['operator_action']=='退出遗留 Offboard'
    assert rows[0]['failure_snapshot']==snapshot


def test_receiver_console_keeps_bounded_tail_without_stopping_pose_input():
    from ev_delay_vicon_source import ViconSource
    source=ViconSource.__new__(ViconSource)
    source.stopped=False;source.count=source.delivered_count=0;source.latest_only=True
    source.proc=SimpleNamespace(poll=lambda:None,stdout=SimpleNamespace(read=lambda n:b'x'*n))
    source.executor=SimpleNamespace(spin_once=lambda **kwargs:None)
    source.report=dict(console='a'*65536)
    for _ in range(20):source.poll()
    assert len(source.report['console'])==65536
    assert source.report['console_dropped_chars']==20*4096
    assert source.report['console']=='x'*65536
    source.proc.poll=lambda:1
    with pytest.raises(RuntimeError,match='receiver exited'):source.poll()


def test_loaded_car_command_is_explicit_and_does_not_forward_takeoff():
    c=Controller();requests=[]
    c.start_car=lambda:requests.append('car') or {'flight_started':False}
    run(c,['CAR_START\n',''])
    assert requests==['car'] and not c.confirmations and not c.previews and c.closed
