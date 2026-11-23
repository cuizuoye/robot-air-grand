"""Static recorder gap handling only. Hardware/flight guards are unchanged."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import pytest

import vicon_frame_sample_capture as entry
from ev_delay_capture import DelayRecorder
from ev_delay_sources import AtomicViconAdapter
from test_vicon_frame_sample_capture import rows
from test_vicon_frame_startup_mock import install_fake_runtime


HISTORY=Path(__file__).parent/'fixtures/vicon_frames_17_20261021.json'


def inspect(data,**kwargs):
    monitor=entry.CaptureContinuity(**kwargs)
    for row in data: monitor.observe(row)
    return monitor.export()


def test_original_17_rows_are_replayed_without_repair_or_false_pass():
    raw=HISTORY.read_bytes()
    assert hashlib.sha256(raw).hexdigest()=='f1ae5b173487daf5af234c47b1a342462b548a36f00e266198ded23a1583315c'
    data=json.loads(raw)['rows']; before=deepcopy(data)
    result=inspect(data)
    assert result['received']==17 and result['fatal_reason'] is None
    assert result['inferred_missing_frames']==result['inferred_missing_sequences']==5
    assert result['missing_observations_estimate']==5
    assert result['missing_fraction']==pytest.approx(5/22)
    event,=result['events']
    assert event['previous_sequence']==55 and event['current_sequence']==61
    assert event['missing_sequence_range']==[56,60]
    assert event['missing_frame_range']==[8776025,8776029]
    assert event['stage']=='after_FrameSample_construction_before_callback_unresolved'
    assert event['source_missing_delta']==event['source_queue_drop_delta']==0
    assert event['at_monotonic_ns']==data[-1]['callback_ns']
    assert not entry.analyze(data)['passed']  # short window and loss still fail
    assert data==before and HISTORY.read_bytes()==raw
    assert result['interpolated_frames']==0 and not result['EV_DELAY_approved']


def test_continuous_15_second_data_passes_no_gap_authority_created():
    data=rows(); result=inspect(data)
    assert result['events']==[] and result['missing_fraction']==0
    assert result['fatal_reason'] is None and entry.analyze(data)['passed']
    assert not result['flight_authorized']


@pytest.mark.parametrize('holes',[
    set(range(100,105)), {100,101,300,500,501},
])
def test_isolated_or_multiple_small_gaps_are_counted_and_not_approved(holes):
    data=[r for i,r in enumerate(rows()) if i not in holes]
    result=inspect(data)
    assert result['fatal_reason'] is None
    assert result['inferred_missing_frames']==len(holes)
    assert result['inferred_missing_sequences']==len(holes)
    assert result['missing_observations_estimate']==len(holes)
    assert not entry.analyze(data)['passed']
    assert len(data)+len(holes)==1501 and result['interpolated_frames']==0


def test_repeated_severe_loss_stops_with_explicit_condition():
    result=inspect(rows()[::10][:6])
    assert result['fatal_reason']=='five_consecutive_loss_or_duplicate_transitions'
    assert result['inferred_missing_frames']==45
    assert len(result['events'])==5


def test_sustained_loss_with_some_good_frames_still_has_bounded_abort():
    data=rows(); chosen=[0]
    for jump in (4,1)*5: chosen.append(chosen[-1]+jump)
    result=inspect([data[i] for i in chosen])
    assert result['fatal_reason']=='sustained_missing_fraction_at_least_25_percent'


def test_duplicate_is_preserved_and_not_counted_as_a_missing_observation():
    data=rows(); duplicate=deepcopy(data[50]); duplicate['callback_ns']+=1
    data.insert(51,duplicate)
    result=inspect(data)
    assert result['fatal_reason'] is None and result['duplicate_transitions']==1
    assert result['inferred_missing_frames']==0 and result['missing_fraction'] is None
    assert result['events'][0]['kind']=='duplicate'
    assert len(data)==1502 and not entry.analyze(data)['passed']


def test_source_regression_is_not_sorted_or_retimed():
    data=rows()[:3]; data[2]['frame']=data[0]['frame']
    result=inspect(data)
    assert result['fatal_reason']=='source_frame_or_sequence_regression'
    assert result['events'][-1]['kind']=='regression'
    assert not entry.analyze(data)['passed']


def test_source_backlog_is_not_mistaken_for_fresh_measurements():
    row=rows()[0]; row['callback_ns']=row['sdk_end_ns']+600_000_000
    result=inspect([row])
    assert result['fatal_reason']=='source_backlog_above_500ms'
    assert result['max_source_to_callback_ns']==600_000_000


@pytest.mark.parametrize('field,value,reason',[
    ('epoch',1,'source_clock_epoch_rate_or_counter_changed'),
    ('sdk_hz',101.,'source_clock_epoch_rate_or_counter_changed'),
    ('subject','car01','unexpected_rigid_body'),
    ('sdk_end_ns',100_000_000_000,'source_monotonic_order_invalid'),
])
def test_existing_non_gap_identity_and_clock_boundaries_remain(field,value,reason):
    data=rows()[:2]; data[-1][field]=value
    assert inspect(data)['fatal_reason']==reason


def test_SDK_gap_and_source_queue_overflow_are_distinguishable():
    data=[rows()[0],rows()[6]]
    data[1]['sequence']=data[0]['sequence']+1
    data[1]['missing_frames']=5
    result=inspect(data)
    assert result['events'][0]['stage']=='SDK_GetFrame_gap_reported'
    assert result['inferred_missing_sequences']==0 and result['inferred_missing_frames']==5
    data[1]['queue_dropped']=1
    result=inspect(data)
    assert result['fatal_reason']=='source_queue_overflow'
    assert result['events'][0]['stage']=='source_queue_overflow_reported'


def test_gap_event_buffer_is_bounded():
    data=rows()[:6]
    duplicate=deepcopy(data[0])
    assert inspect([data[0],duplicate,duplicate],event_capacity=1)['fatal_reason']=='continuity_event_capacity_exceeded'


def test_strict_adapter_used_by_other_paths_still_rejects_same_gap():
    data=json.loads(HISTORY.read_text())['rows']
    strict=AtomicViconAdapter(DelayRecorder('OFFLINE_MOCK_STRICT_UNCHANGED',{}))
    for row in data: strict.observe(row,callback_ns=row['callback_ns'])
    assert strict.fault=='sidechannel_loss_or_SDK_ROS_epoch_rate_changed'


@pytest.mark.parametrize('mode,missing',[('single_gap',5),('many_small_gaps',3),('duplicate',0)])
def test_full_window_finishes_with_raw_anomalies_but_quality_stays_failed(monkeypatch,tmp_path,mode,missing):
    state=install_fake_runtime(monkeypatch,tmp_path,mode)
    assert entry.run('OFFLINE_MOCK_STATIC_'+mode)==2  # not silently passed
    report_path=next((tmp_path/'preparation_runs').glob('b3_3_vicon_frame_sample_*/report.json'))
    report=json.loads(report_path.read_text())
    raw=json.loads((report_path.parent/'frames.json').read_text())['rows']
    assert report['capture_completed'] and report['capture_state']=='OBSERVATION_COMPLETE'
    assert report.get('error') is None and report['continuity_capture']['fatal_reason'] is None
    assert report['analysis']['observed_frame_span_s']>=15 and len(raw)>1000
    assert report['analysis']['inferred_missing_frame_numbers']==missing
    assert not report['analysis']['passed'] and not report['passed']
    assert report['strict_adapter_replay']['fault'] is not None
    assert not report['strict_adapter_replay']['all_raw_rows_replayed']
    assert report['strict_adapter_replay']['phase']=='after_receiver_and_ROS_stopped'
    assert report['callback_counters']['saved']==len(raw)
    assert report['analysis']['stage_counters']['publisher_successes'] is None
    assert report['analysis']['continuity']['interpolated_frames']==0
    assert report['child_stopped'] and state.spawn_count==1 and state.pipe_closed
    assert all(n.destroyed for n in state.nodes) and all(not c.live for c in state.contexts)
    assert not report['shared_receiver_modified'] and report['MAVLink_sent']==0
    assert not report['flight_authorized'] and report['automatic_retries']==0


@pytest.mark.parametrize('mode,reason',[
    ('severe_gaps','five_consecutive_loss_or_duplicate_transitions'),
    ('regression','source_frame_or_sequence_regression'),
    ('backlog','source_backlog_above_500ms'),
])
def test_severe_fault_stops_and_preserves_partial_raw_data(monkeypatch,tmp_path,mode,reason):
    state=install_fake_runtime(monkeypatch,tmp_path,mode)
    assert entry.run('OFFLINE_MOCK_SEVERE_'+mode)==2
    report_path=next((tmp_path/'preparation_runs').glob('b3_3_vicon_frame_sample_*/report.json'))
    report=json.loads(report_path.read_text())
    assert not report['capture_completed'] and reason in report['error']
    assert report['capture_state_at_failure']=='RECORDING_FRAME_SAMPLE'
    assert report['analysis']['total']>0 and report['child_stopped']
    assert state.spawn_count==1 and state.pipe_closed and all(n.destroyed for n in state.nodes)
    assert not report['shared_receiver_modified'] and report['automatic_retries']==0


def test_RAM_overflow_still_stops_and_releases_own_resources(monkeypatch,tmp_path):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    monkeypatch.setattr(entry,'CAPACITY',20)
    assert entry.run('OFFLINE_MOCK_RAM_LIMIT')==2
    report=json.loads(next((tmp_path/'preparation_runs').glob('b3_3_vicon_frame_sample_*/report.json')).read_text())
    assert 'RAM_capacity_exceeded' in report['error']
    assert report['analysis']['total']==20 and report['analysis']['RAM_dropped']==1
    assert report['callback_counters']==dict(entered=21,decoded=21,saved=20,warmup=0)
    assert report['child_stopped'] and state.pipe_closed


def test_existing_audit_cost_is_recorded_without_blocking_reception(monkeypatch,tmp_path):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    state.audit_delay_ns=25_000_000
    # Same synthetic 25ms inspection, now concurrent with the producer.
    # Actual thread behavior is checked separately; this is not live proof.
    assert entry.run('OFFLINE_MOCK_TIMED_AUDIT')==0
    report=json.loads(next((tmp_path/'preparation_runs').glob('b3_3_vicon_frame_sample_*/report.json')).read_text())
    assert report['runtime_audit_windows']
    assert max(w['duration_ns'] for w in report['runtime_audit_windows'])==25_000_000
    assert report['capture_completed'] and report['analysis']['passed']
    assert report['analysis']['actual_host_observed_rate_hz']==100
    assert any(w['received_rows_at_result']>w['received_rows'] for w in report['runtime_audit_windows'])
    assert all(a.closed for a in state.auditors)
    assert report['configured_FrameSample_qos']['depth']==5
    assert report['configured_FrameSample_qos']['changed'] is False


def test_strict_adapter_never_runs_in_the_live_RAM_callback(monkeypatch,tmp_path):
    state=install_fake_runtime(monkeypatch,tmp_path,'normal')
    original=entry.AtomicViconAdapter.observe
    calls=0
    def offline_only(self,*args,**kwargs):
        nonlocal calls
        assert state.pipe_closed
        assert all(n.destroyed for n in state.nodes) and all(not c.live for c in state.contexts)
        calls+=1
        return original(self,*args,**kwargs)
    monkeypatch.setattr(entry.AtomicViconAdapter,'observe',offline_only)
    assert entry.run('OFFLINE_MOCK_ADAPTER_AFTER_CAPTURE')==0
    assert calls>=1500
