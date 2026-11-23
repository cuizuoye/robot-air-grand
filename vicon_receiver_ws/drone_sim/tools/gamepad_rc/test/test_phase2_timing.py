from copy import deepcopy

import pytest
from pymavlink.dialects.v20 import common as m

from ground_link_diagnostics import TimingRecorder, summarize, STREAMS
from mavlink_crc_stream import CrcResyncParser
from lock_clock_probe import LockClockProbe
from test_phase2_acceptance import no_grant


def trace():
    r=TimingRecorder('SYNTHETIC-clock-session')
    for i in range(11):
        at=1_000_000_000+i*100_000_000
        r.record('read',at_ns=at,start_ns=at-1000,bytes=200,in_waiting=0)
        r.record('parse',at_ns=at+1000,start_ns=at,read_end_ns=at)
        r.record('frame',at_ns=at+1000,valid=True,crc_error=False,system=1,component=1,seq=i)
        for name in STREAMS:r.record('stream',at_ns=at+1000,stream=name,source_ns=at,
                                     accepted=True,identity_verified=True)
        r.record('timesync_request',at_ns=at+2000,token=at+2000)
        r.record('timesync_reply',at_ns=at+3000,token=at+2000,decision='accepted')
        r.record('application_update',at_ns=at+4000,parse_end_ns=at+1000)
    return r.export()


def review(t=None):
    return summarize(trace() if t is None else t,start_ns=1_000_000_000,end_ns=2_000_005_000)


def test_complete_timing_trace_has_fields_and_no_hardware_grant():
    r=review()
    assert r['review_complete'] and r['timing_window_clear']
    assert r['counts']['valid_frames']==11
    assert r['counts']['crc_errors']==0
    assert r['streams']['ATTITUDE_QUATERNION']['observed_hz']==10.
    assert r['read_duration']['max_ns']==1000
    assert r['parse_duration']['max_ns']==1000
    assert r['parse_to_application']['max_ns']==3000
    assert not r['uart_arrival_measured'] and not r['data_loss_proven']
    no_grant(r)


def test_501ms_attitude_gap_even_fresh_vicon_is_not_hidden():
    t=trace()
    t['events']=[e for e in t['events'] if not(e['kind']=='stream' and
        e['stream']=='ATTITUDE_QUATERNION' and 1_100_000_000<e['at_ns']<1_800_000_000)]
    r=review(t)
    assert not r['timing_window_clear'] and r['attitude_gaps_over_500ms']
    assert r['streams']['VICON']['samples']==11


def test_trailing_gap_and_missing_data_not_reported_as_healthy():
    r=summarize(trace(),start_ns=1_000_000_000,end_ns=2_600_000_000)
    assert not r['timing_window_clear']
    assert r['attitude_gaps_over_500ms'][-1]['gap_ns']>500_000_000
    r=summarize(dict(host_clock_session='x',events=[]),start_ns=0,end_ns=60_000_000_000)
    assert not r['review_complete'] and not r['timing_window_clear']
    assert 'accepted_timesync_reference' in r['missing_evidence']


def test_timesync_reference_expiry_and_real_late_reply_decision():
    # Decisions taken from unchanged production clock algorithm, not a new gate.
    clock=LockClockProbe()
    encoder=m.MAVLink(None,srcSystem=1,srcComponent=1)
    request=clock.request(1_000_000_000)
    reply=m.MAVLink(None).parse_buffer(encoder.timesync_encode(10_000_000_000,request).pack(encoder))[0]
    assert not clock.ingest(reply,1_101_000_000)
    assert clock.history[-1]['event']=='rtt_rejected'
    t=trace()
    t['events'] += [dict(kind='timesync_request',at_ns=2_100_000_000,token=2_100_000_000),
        dict(kind='timesync_reply',at_ns=2_201_000_000,token=2_100_000_000,decision=clock.history[-1]['event']),
        dict(kind='clock_expired',at_ns=2_600_000_000),
        dict(kind='safety_revoke',at_ns=2_600_000_001,reason='clock_age')]
    r=summarize(t,start_ns=1_000_000_000,end_ns=2_700_000_000)
    assert r['counts']['timesync_rtt_over_100ms']==1
    assert r['counts']['late_replies_rejected']==1
    assert r['counts']['timesync_reference_expiry_events']==1
    assert r['authorization_revocation_reasons']=={'clock_age':1}
    assert not r['timing_window_clear'] and not r['retry_permitted']


def test_cached_status_or_wrong_identity_never_refreshes_statistics():
    t=trace()
    for e in t['events']:
        if e['kind']=='stream' and e['stream']=='ATTITUDE_QUATERNION':
            e['source_ns']=1
        if e['kind']=='stream' and e['stream']=='EKF_FUSION':e['identity_verified']=False
    r=review(t)
    assert r['streams']['ATTITUDE_QUATERNION']['samples']==1
    assert r['rejected_or_cached_updates']==21
    assert not r['review_complete']


def test_bounded_memory_saturation_explicit_and_snapshots_not_mutable():
    recorder=TimingRecorder('x',capacity=1)
    assert recorder.record('read',at_ns=1,start_ns=0,bytes=1)
    assert not recorder.record('read',at_ns=2,start_ns=1,bytes=1)
    r=recorder.export()
    assert r['dropped_events']==1 and r['recorder_fault']
    r['events'].clear()
    assert len(recorder.events)==1


def test_parse_adapter_uses_existing_strict_parser_and_never_reads_device():
    encoder=m.MAVLink(None,srcSystem=1,srcComponent=1)
    wire=encoder.heartbeat_encode(2,12,0,0,3).pack(encoder)
    recorder=TimingRecorder('SYNTHETIC',clock=iter([120,140]).__next__)
    messages=recorder.parse_chunk(CrcResyncParser(),wire,read_start_ns=100,read_end_ns=110,in_waiting=0)
    assert messages[0].get_type()=='HEARTBEAT'
    events=recorder.export()['events']
    assert events[0]['at_ns']==110 and events[1]['at_ns']==140 and events[2]['valid']
    assert not hasattr(recorder,'port') and not hasattr(recorder,'send')


def test_window_boundary_preserves_original_timesync_request():
    r=summarize(trace(),start_ns=1_000_002_500,end_ns=2_000_005_000)
    assert r['timesync_rtt']['count']==11
    assert r['timesync_rtt']['max_ns']==1000


@pytest.mark.parametrize('fault',['regression','float_time','early_parse','bad_reply'])
def test_malformed_log_rejected_not_repaired(fault):
    t=trace()
    if fault=='regression':t['events'][1]['at_ns']=1
    elif fault=='float_time':t['events'][0]['at_ns']=1.
    elif fault=='early_parse':t['events'][1]['start_ns']=0
    else:
        for e in t['events']:
            if e['kind']=='timesync_reply':e['token']=0;break
    with pytest.raises(ValueError):review(t)
