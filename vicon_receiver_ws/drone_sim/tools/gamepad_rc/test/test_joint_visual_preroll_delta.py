"""New stationary pre-roll only. In-memory devices, no live acquisition."""
from copy import deepcopy
from types import SimpleNamespace
import json
import socket

import pytest
from pymavlink.dialects.v20 import common as m

from ev_delay_capture import analyze
from ev_delay_sources import merge_ulog_capture
from frozen_target_stream import clock_bounds_us
from lock_clock_probe import LockClockProbe
from test_ev_delay_joint_device import execute, Port, Source
from test_ev_delay_sources import fake_ulog


@pytest.fixture(autouse=True)
def forbid_network(monkeypatch):
    def forbidden(*args, **kwargs):
        raise AssertionError('Offline pre-roll test attempted network I/O')
    monkeypatch.setattr(socket.socket, 'connect', forbidden)
    monkeypatch.setattr(socket.socket, 'sendto', forbidden)


def test_factory_preroll_is_locked_received_and_outside_full_motion_window(tmp_path, monkeypatch):
    sim, code, report = execute(tmp_path, monkeypatch)
    assert code == 0, report
    pr = report['owner']['visual_preroll']
    assert pr['received_confirmed'] and not pr['logger_subscription_verified']
    assert pr['received'] >= 3 and pr['sent'] <= 160
    assert pr['ended_ns']-pr['first_received_ns'] >= 2_000_000_000
    raw = json.loads(next(tmp_path.glob('joint_ev_delay_*/capture_raw.json')).read_text())
    w = raw['capture_metadata']['motion_window']
    assert sim.first_pose < pr['ended_ns'] <= w['go_ns']
    assert w['started_ns'] >= w['go_ns']+3_000_000_000
    assert w['deadline_ns']-w['started_ns'] == 15_000_000_000
    assert w['ended_ns'] >= w['deadline_ns']
    sent = [e for e in raw['events'] if e['kind']=='calibration_send']
    assert 400 <= len(sent) <= 480 and min(e['sent_ns'] for e in sent) >= w['started_ns']
    assert raw['capture_metadata']['visual_preroll']['motion_start_ns'] == w['started_ns']
    assert sim.on_count == sim.off_count == 1 and sim.ports[1].interval == 0
    assert all(p.closed == 1 for p in sim.ports)
    for p in sim.ports:
        for _, msg in p.sent:
            assert msg.get_type() not in ('PARAM_SET','SET_MODE','SET_POSITION_TARGET_LOCAL_NED')
            if msg.get_type() == 'COMMAND_LONG': assert msg.command in (185,510,511,512)
    assert sum(msg.get_type()=='COMMAND_LONG' and msg.command==185
               for p in sim.ports for _,msg in p.sent) == 1
    assert not report['calibration_approved'] and not report['flight_authorized']


@pytest.mark.parametrize('fault',['missing','bad_crc','wrong_pose','replayed'])
def test_unproved_echo_never_unlocks_go_and_cleanup_restores_rates(tmp_path, monkeypatch, fault):
    original = Port.write
    replay = []
    def write(self, packet):
        inbox = self.sim.ports[0].inbox
        before = len(inbox)
        n = original(self, packet)
        if self.route == 1 and self.sent[-1][1].get_type() == 'VISION_POSITION_ESTIMATE':
            del inbox[before:]
            if fault == 'bad_crc':
                bad = bytearray(packet); bad[-1] ^= 1; inbox.extend(bad)
            elif fault == 'wrong_pose':
                msg = self.sent[-1][1]; msg.x += .1
                enc = m.MAVLink(None, srcSystem=250, srcComponent=213); enc.seq=msg.get_seq()
                inbox.extend(msg.pack(enc))
            elif fault == 'replayed':
                if not replay: replay.append(packet)
                inbox.extend(replay[0])
        return n
    monkeypatch.setattr(Port,'write',write)
    sim, code, report = execute(tmp_path, monkeypatch)
    assert code == 2
    assert not report['owner']['visual_preroll']['received_confirmed']
    assert 'motion_go_ns' not in report['owner']
    assert not report['lifecycle']['observation_window_completed']
    assert sim.on_count == sim.off_count == 1 and sim.ports[1].interval == 0
    assert all(p.closed == 1 for p in sim.ports)
    if fault=='bad_crc': assert report['routes'][0]['joint_rx_evidence']['rejected_candidates'] > 0


@pytest.mark.parametrize('fault',['occluded','stale_trs','short_write','motion'])
def test_preroll_source_link_or_write_failure_stops_before_go(tmp_path, monkeypatch, fault):
    if fault == 'motion':
        original = Source.poll
        def moved(self):
            original(self)
            if self.sim.first_pose and self.sim.now-self.sim.first_pose > 600_000_000:
                self.latest['world_pose'][0] += .1
        monkeypatch.setattr(Source,'poll',moved)
    sim, code, report = execute(tmp_path, monkeypatch, None if fault=='motion' else fault)
    assert code == 2 and 'motion_go_ns' not in report['owner']
    assert sim.on_count == sim.off_count == 1
    assert all(p.closed == 1 for p in sim.ports)


def preroll_ulog():
    doc, ulog = fake_ulog('sensor_combined')
    doc['events'] = [e for e in doc['events'] if e['kind'] not in ('calibration_visual','calibration_imu')]
    meta = doc['capture_metadata']
    meta['ulog_boot_binding_evidence'] = 'SYNTHETIC independently bound'
    meta['motion_window'] = dict(started_ns=8_000_000_000,go_ns=5_000_000_000)
    visual = next(d.data for d in ulog.data_list if d.name=='vehicle_visual_odometry')
    first = visual['timestamp_sample'][0]
    for key, values in visual.items():
        extra = ([first-100000,first-60000,first-20000] if key.startswith('timestamp') else [values[0]]*3)
        visual[key] = extra+values
    meta['visual_preroll'] = dict(scope='stationary-before-GO-v1', received_confirmed=True,
        received=50, started_ns=1_000_000_000, last_sent_ns=4_000_000_000, ended_ns=4_001_000_000,
        motion_start_ns=8_000_000_000, motion_start_fc_bounds_us=[first-1000,first])
    ref=dict(received_ns=7_999_000_000,remote_ns=first*1000-2_000_000,rtt_ns=200_000)
    ref.update(lower_ns=ref['remote_ns']-ref['received_ns'],
               upper_ns=ref['remote_ns']-ref['received_ns']+ref['rtt_ns'])
    meta['visual_preroll']['motion_start_clock_reference']=ref
    meta['visual_preroll']['motion_start_fc_bounds_us']=list(clock_bounds_us(SimpleNamespace(
        reference=ref,fault=None,MAX_AGE_NS=LockClockProbe.MAX_AGE_NS,DRIFT_PPM=LockClockProbe.DRIFT_PPM),8_000_000_000))
    return doc, ulog


def test_import_accounts_preroll_without_modifying_original_or_hiding_motion_gap():
    doc, ulog = preroll_ulog()
    visual = next(d.data for d in ulog.data_list if d.name=='vehicle_visual_odometry')
    # Interior missing measurements must remain a gap, not be patched or filtered.
    for k,v in visual.items(): visual[k] = v[:103]+v[113:]
    original = deepcopy(visual)
    merged, summary = merge_ulog_capture(doc,ulog,expected_info=ulog.msg_info_dict,import_ns=10**15)
    assert summary['visual_preroll_excluded'] == 3
    assert summary['visual_samples'] == len(visual['timestamp'])-3
    assert visual == original
    assert 'visual_gap_over_50ms_insufficient_recording_cadence' in analyze(merged)['reasons']


@pytest.mark.parametrize('fault',['unconfirmed','clock_width','countdown','start_mismatch','no_logged_preroll',
                                 'arbitrary_cutoff','stale_reference'])
def test_import_refuses_unproved_preroll_boundary(fault):
    doc, ulog = preroll_ulog();p=doc['capture_metadata']['visual_preroll']
    if fault=='unconfirmed': p['received_confirmed']=False
    elif fault=='clock_width':p['motion_start_fc_bounds_us'][1]+=200001
    elif fault=='countdown':doc['capture_metadata']['motion_window']['go_ns']+=1
    elif fault=='start_mismatch':p['motion_start_ns']+=1
    elif fault=='arbitrary_cutoff':p['motion_start_fc_bounds_us']=[v+1000 for v in p['motion_start_fc_bounds_us']]
    elif fault=='stale_reference':
        r=p['motion_start_clock_reference'];r['received_ns']-=600_000_000
        r['lower_ns']+=600_000_000;r['upper_ns']+=600_000_000
    else:
        visual=next(d.data for d in ulog.data_list if d.name=='vehicle_visual_odometry')
        for k,v in visual.items():visual[k]=v[3:]
    with pytest.raises(ValueError,match='preroll'):
        merge_ulog_capture(doc,ulog,expected_info=ulog.msg_info_dict,import_ns=10**15)
