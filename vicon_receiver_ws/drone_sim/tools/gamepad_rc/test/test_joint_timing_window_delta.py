"""New action synchronization and same-ULog pairing only; synthetic I/O."""
from copy import deepcopy
from types import SimpleNamespace as NS
import json
import sys
import pytest

from ev_delay_capture import analyze, prepare
from ev_delay_sources import merge_ulog_capture
from ev_delay_live_owner import Owner
from test_ev_delay_sources import fake_ulog


def native_capture(jitter=False):
    doc,u=fake_ulog('sensor_combined')
    doc['events']=[e for e in doc['events'] if e['kind'] not in ('calibration_imu','calibration_visual')]
    doc['capture_metadata'].update(ulog_boot_binding_evidence='SYNTHETIC same boot/window',
        requested_timing_method='native_fc_observation_intervals',clock_uncertainty_ns=None,
        interval_uncertainty_ns=None)
    u._capture_provenance=dict(path='SYNTHETIC.ulg',sha256='a'*64,size=123)
    if jitter:
        v=next(d.data for d in u.data_list if d.name=='vehicle_visual_odometry')
        for i in range(len(v['timestamp'])):
            for key in ('timestamp','timestamp_sample'):v[key][i]+=(8000 if i%2 else -8000)
    return merge_ulog_capture(doc,u,expected_info=u.msg_info_dict,import_ns=10**15)[0]


def test_native_same_clock_recovers_known_lag_without_host_precision_claim():
    d=native_capture();r=analyze(d)
    assert r['candidate_identified'],r
    assert r['fit']['effective_delay_candidate_ms']==pytest.approx(37,abs=1)
    assert not r['parameter_approved']
    assert r['uncertainty']['host_timesync_used_for_fit'] is False
    assert r['uncertainty']['sdk_interval_bound_ns'] is None
    assert d['capture_metadata']['clock_uncertainty_ns'] is None  # not forged
    before=prepare(d)['capture']
    # Arbitrarily delayed import and host-clock uncertainty do not move FC samples.
    for e in d['events']:e['at_ns']+=10**12
    d['capture_metadata']['clock_uncertainty_ns']=90_000_000
    assert prepare(d)['capture']==before


@pytest.mark.parametrize('fault',['sample','boot','artifact','route','duplicate_pose','faulted_capture'])
def test_native_path_does_not_approve_changed_or_ambiguous_evidence(fault):
    d=native_capture();meta=d['capture_metadata']
    if fault=='sample':next(e for e in d['events'] if e['kind']=='calibration_visual')['sample_us']+=1
    elif fault=='boot':meta['fc_boot_session']='different'
    elif fault=='artifact':meta['fc_time_pairing']['ulog_artifact']={}
    elif fault=='route':next(e for e in d['events'] if e['kind']=='calibration_visual')['route']='usb_diagnostic'
    elif fault=='duplicate_pose':
        e=deepcopy(next(e for e in d['events'] if e['kind']=='calibration_send'))
        e['at_ns']=d['events'][-1]['at_ns'];d['events'].append(e)
        # One ambiguity can be excluded, never silently assigned to a frame.
        r=prepare(d);assert r['prepared'] and r['excluded_samples']['ambiguous_or_unmatched_visual']>=1
        return
    else:d['recorder_fault']='saved incomplete window'
    assert not prepare(d)['prepared']


def test_transport_jitter_is_not_removed_by_using_native_clock():
    r=analyze(native_capture(jitter=True))
    assert not r['candidate_identified']
    assert not r['uncertainty']['transport_jitter_removed']
    assert r['uncertainty']['max_arrival_vs_sdk_interval_difference_ns']==16_000_000
    assert r['reasons']==['native_arrival_variation_exceeds_plus_minus_5ms_budget']


@pytest.mark.parametrize('field,value',[('cue_ns',None),('cue_ns',2_600_000_000),('ended_ns',16_900_000_000)])
def test_incomplete_or_late_motion_window_not_calibration(field,value):
    d=native_capture();m=d['capture_metadata'];m['motion_window_required']=True
    m['motion_window']=dict(go_ns=1,started_ns=2_000_000_000,cue_ns=2_010_000_000,
        deadline_ns=17_000_000_000,ended_ns=17_000_000_000)
    m['motion_window'][field]=value
    assert not prepare(d)['prepared']


def test_delayed_local_go_does_not_spend_action_window_and_countdown_pumps():
    o=object.__new__(Owner);now=[0];notices=[]
    o.report={};o.clock=lambda:now[0];o.identity=lambda:True
    o.notice=lambda text,**kw:notices.append((now[0],text))
    o.pump_all=lambda:now.__setitem__(0,now[0]+100_000_000)
    o.motion_ready=lambda:now[0]>=30_000_000_000
    o.prepare_visual_preroll=lambda gate:None  # Isolate cue timing; pre-roll has its own delta tests.
    o.prepare_recording(None)
    assert o.report['motion_go_ns']==30_000_000_000 and now[0]==33_000_000_000
    assert not any('开始动作：' in text for _,text in notices)
    o.motion_cue_ns=None;o.session=NS(gate=NS(started_ns=now[0]),
        recorder=NS(metadata={'motion_window':{'cue_ns':None,'ended_ns':None}}))
    now[0]+=10_000_000;o.announce_motion();o.announce_motion()
    assert len([x for _,x in notices if '开始动作：' in x])==1


def test_native_factory_records_cue_only_after_write_and_preserves_full_window(tmp_path,monkeypatch):
    import ev_delay_joint_capture as live
    from test_ev_delay_joint_device import execute
    monkeypatch.setattr(sys,'stdin',NS(isatty=lambda:True,fileno=lambda:0))
    monkeypatch.setattr(live,'motion_cue_ready',lambda fd:True)
    sim,code,report=execute(tmp_path,monkeypatch)
    assert code==0,report
    raw=json.loads(next(tmp_path.glob('joint_ev_delay_*/capture_raw.json')).read_text())
    w=raw['capture_metadata']['motion_window']
    first_recorded=next(e['sent_ns'] for e in raw['events'] if e['kind']=='calibration_send')
    assert sim.first_pose < w['go_ns']
    assert w['go_ns']+3_000_000_000<=w['started_ns']<=first_recorded<=w['cue_ns']
    assert 0<=w['cue_ns']-w['started_ns']<=500_000_000
    assert w['deadline_ns']-w['started_ns']==15_000_000_000 and w['ended_ns']>=w['deadline_ns']
    assert report['hardware_capture_completed']  # synthetic factory result only
    assert not report['calibration_approved']
