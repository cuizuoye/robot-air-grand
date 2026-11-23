"""Offline WiFi delay provenance and exact source timestamp regressions."""
from copy import deepcopy
import socket

import pytest
from pymavlink.dialects.v20 import common as m
from ev_delay_capture import DelayRecorder, analyze, prepare, pose_route
from ev_delay_sources import AtomicViconAdapter
from test_ev_delay_capture import capture, no_grant
from test_ev_delay_sources import sample, vpe
from test_pocket_wifi_delta import Peer
import native_rc_ground as ground


def wifi_metadata():
    return dict(pose_route='wifi',pose_transport=dict(kind='fixed_peer_udp',
        peer=['192.168.4.1',14550],bind=['192.168.4.2',14550],evidence='SYNTHETIC endpoint binding'))


@pytest.fixture(autouse=True)
def offline(monkeypatch):
    monkeypatch.setattr(socket,'socket',lambda *a,**kw:pytest.fail('Physical network attempted'))


def test_wifi_uses_existing_delay_fit_without_approving_parameter():
    doc=capture(); doc['capture_metadata'].update(wifi_metadata())
    for row in doc['events']:
        if row['kind']=='calibration_send': row['route']='wifi'
    original=deepcopy(doc)
    result=analyze(doc)
    assert result['candidate_identified'],result['reasons']
    assert result['fit']['effective_delay_candidate_ms']==pytest.approx(37,abs=1)
    assert result['independent_holdout']['effective_delay_candidate_ms']==pytest.approx(37,abs=1)
    assert result['preparation']['capture']['alignment']['pose_route']=='wifi'
    assert doc==original
    no_grant(result)


@pytest.mark.parametrize('bad', ['mixed','missing_binding','missing_declaration'])
def test_wifi_cannot_silently_accept_historical_trs_or_unbound_link(bad):
    doc=capture();doc['capture_metadata'].update(wifi_metadata())
    sends=[r for r in doc['events'] if r['kind']=='calibration_send']
    for row in sends:row['route']='wifi'
    if bad=='mixed': sends[20]['route']='trs'
    elif bad=='missing_binding': doc['capture_metadata'].pop('pose_transport')
    else: doc['capture_metadata'].pop('pose_route')
    result=prepare(doc)
    assert not result['prepared'] and not result['parameter_approved']


@pytest.mark.parametrize('binding', [None,{},dict(kind='fixed_peer_udp',evidence='test',
    peer=['0.0.0.0',14550],bind=['192.168.4.2',14550])])
def test_invalid_wifi_binding_is_reported_not_accepted(binding):
    with pytest.raises(ValueError): pose_route(dict(pose_route='wifi',pose_transport=binding))


def test_wifi_adapter_preserves_exact_frame_join_and_route():
    rec=DelayRecorder('SYNTHETIC_WIFI',wifi_metadata());adapter=AtomicViconAdapter(rec)
    assert adapter.observe(sample(stamp=1234567),callback_ns=30)
    assert adapter.observe_sent(vpe(),source_stamp_ns=1234567,source_frame_id='map',
        tx_start_ns=31,tx_end_ns=32,callback_ns=33,route='wifi')
    assert adapter.finish()['complete']
    assert rec.events[-1]['ros_stamp_ns']==1234567 and rec.events[-1]['route']=='wifi'
    assert not adapter.observe_sent(vpe(),source_stamp_ns=1234568,source_frame_id='map',
        tx_start_ns=34,tx_end_ns=35,callback_ns=36,route='trs')


def test_ground_evidence_keeps_nanoseconds_and_exact_serialized_payload(monkeypatch):
    rows=[];peer=Peer();now=100.
    ob=ground.GroundObserver(peer,rows.append,clock=lambda:now,fusion_observe=True)
    ob.identity={'synthetic':True};ob.rc.heartbeat={'armed':False,'received_at':now}
    ob.rc.observe_clock(100_000_000,now)
    ob.attitude=dict(at=now,boot_us=100_000_000,q=[1,0,0,0])
    ticks=iter([500,600]);monkeypatch.setattr(ground.time,'monotonic_ns',lambda:next(ticks))
    stamp=100_000_000_987
    ob.pose(dict(source_stamp_ns=stamp,position=[.123456789123,0,0],quaternion_wxyz=[1,0,0,0]),stamp+1)
    record=next(x for x in rows if x['direction']=='vicon_sent')
    assert record['source_stamp_ns']==stamp and record['wire_usec']==stamp//1000
    assert record['tx_start_ns']==500 and record['tx_end_ns']==600
    assert record['source_frame_id']=='map' and record['route']=='wifi'
    decoded=m.MAVLink(None).parse_buffer(bytes.fromhex(record['wire_hex']))[0]
    assert decoded.usec==stamp//1000
    assert decoded.x!=record['position'][0]  # Float32 wire value is the matching authority.
    assert record['wire_hex']==next(x['wire_hex'] for x in rows if x['direction']=='tx_attempt')
