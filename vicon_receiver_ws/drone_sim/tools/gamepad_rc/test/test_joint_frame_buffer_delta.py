"""New burst-drain budget and failed-frame evidence; no hardware."""
from types import SimpleNamespace as NS
import pytest
import ev_delay_vicon_source as source
from ev_delay_live_owner import Owner


@pytest.mark.parametrize('callback_cost_ns,expected',[(0,64),(300_000,7)])
def test_burst_drain_yields_for_other_owners(monkeypatch,callback_cost_ns,expected):
    now=[0];calls=[];audits=[]
    s=object.__new__(source.ViconSource);s.stopped=False;s.report={'console':''}
    s.latest_only=False;s.count=0
    s.proc=NS(poll=lambda:None,stdout=NS(read=lambda size:b''))
    def spin(*,timeout_sec):
        assert timeout_sec==0
        now[0]+=callback_cost_ns;calls.append(now[0])
    s.executor=NS(spin_once=spin);s.audit=lambda at:audits.append(at)
    monkeypatch.setattr(source.time,'monotonic_ns',lambda:now[0])
    s.poll()
    assert len(calls)==expected and audits==[]  # No metadata scan in the receive loop.
    assert now[0]<=2_000_000+callback_cost_ns


def test_rejected_sequence_keeps_exact_previous_and_failed_frames():
    o=object.__new__(Owner);o.pose_enabled=True;o.stop_at_ns=100;o.report={}
    previous={'sequence':1,'frame':20};bad={'sequence':3,'frame':22}
    o.session=NS(adapter=NS(previous=previous),gate=NS(fault='sequence_gap'),
                 observe_frame=lambda *a,**k:False)
    with pytest.raises(RuntimeError,match='sequence_gap'):o.frame(bad,50)
    proof=o.report['first_failed_source_frame']
    assert proof==dict(previous=previous,rejected=bad,callback_ns=50,reason='sequence_gap')
    bad['frame']=99
    assert proof['rejected']['frame']==22


@pytest.mark.parametrize('latest_only,expected_depth,expected_buffer',[(False,64,200),(True,1,1)])
def test_flight_policy_is_explicit_and_capture_default_unchanged(monkeypatch,tmp_path,latest_only,expected_depth,expected_buffer):
    monkeypatch.setattr(source.ViconSource,'_open',lambda *a:None)
    s=source.ViconSource(tmp_path,{},**({'latest_only':True} if latest_only else {}))
    assert (s.subscription_depth,s.sdk_buffer_size)==(expected_depth,expected_buffer)


@pytest.mark.parametrize('latest_only',[False,True])
def test_delayed_turn_logs_latest_once_without_retimestamping_or_hiding_invalid(monkeypatch,tmp_path,latest_only):
    monkeypatch.setattr(source.ViconSource,'_open',lambda *a:None)
    monkeypatch.setattr(source,'frame_sample_fields',lambda row:dict(row))
    clock=[1_000_000_000]
    monkeypatch.setattr(source.time,'monotonic_ns',lambda:clock[0])
    s=source.ViconSource(tmp_path,{},latest_only=latest_only)
    s.node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=clock[0])))
    s.proc=NS(poll=lambda:None,stdout=NS(read=lambda n:b''))
    log=[];s.on_frame=lambda row,at:log.append((row,at))
    # Burst after a delayed owner turn; final sample is occluded. Never keep
    # sending an earlier valid pose merely because the newest one is invalid.
    rows=[dict(frame=i,ros_stamp_ns=800_000_000+i,occluded=i==2) for i in range(3)]
    queue=list(rows)
    def spin(**kw):
        clock[0]+=300_000
        if queue:s.receive(queue.pop(0))
    s.executor=NS(spin_once=spin)
    s.poll()
    assert [r['frame'] for r,_ in log]==([2] if latest_only else [0,1,2])
    assert s.latest['occluded'] and s.latest['ros_stamp_ns']==800_000_002
    assert clock[0]-s.latest['ros_stamp_ns']>150_000_000  # Still stale, not made fresh.
    count=len(log);s.poll()
    assert len(log)==count  # No repeated log callback when no new frame arrived.


def test_latest_policy_keeps_bounded_turn_under_continuous_input(monkeypatch,tmp_path):
    monkeypatch.setattr(source.ViconSource,'_open',lambda *a:None)
    monkeypatch.setattr(source,'frame_sample_fields',lambda row:dict(row))
    clock=[0];monkeypatch.setattr(source.time,'monotonic_ns',lambda:clock[0])
    s=source.ViconSource(tmp_path,{},latest_only=True)
    s.node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=clock[0])))
    s.proc=NS(poll=lambda:None,stdout=NS(read=lambda n:b''))
    log=[];s.on_frame=lambda row,at:log.append(row)
    def spin(**kw):
        clock[0]+=300_000;s.receive(dict(frame=s.count+1,ros_stamp_ns=clock[0]))
    s.executor=NS(spin_once=spin)
    s.poll()
    assert s.count==7 and len(log)==1 and log[0]['frame']==7
    assert clock[0]==2_100_000  # One callback can cross the existing 2 ms budget.
