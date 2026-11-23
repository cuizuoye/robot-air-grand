"""Only the new joint RX/recovery behavior; all I/O is synthetic."""
import json

import pytest
from pymavlink.dialects.v20 import common as m

from ev_delay_device_io import Endpoint
from ev_delay_live_owner import Owner
from preparation_authorization import SCOPE, require_no_pending_recovery
from restore_preparation_rates import allowed, retained_rates
from trs_message_interval_trial import interval_message
from test_ev_delay_joint_device import execute as factory, Port, Pad, IDENTITY


def execute(tmp_path, monkeypatch, failure=None):
    return factory(tmp_path, monkeypatch, failure, physical_overrides=dict(
        confirm_props_off_battery_disconnected_usb_only=False,
        confirm_props_installed_battery_disconnected_usb_only=True))


class BytesPort:
    is_open = True
    port = 'SYNTHETIC'
    def __init__(self): self.data = b''
    @property
    def in_waiting(self): return len(self.data)
    def read(self, n):
        raw, self.data = self.data[:n], self.data[n:]
        return raw


def receiver(tmp_path):
    now = [10_000_000_000]
    p = BytesPort()
    e = Endpoint(p, 213, IDENTITY, clock=lambda: now[0], joint_rx_evidence=tmp_path/'rx.jsonl')
    fc = m.MAVLink(None, srcSystem=1, srcComponent=1)
    good = bytes(fc.heartbeat_encode(2, 12, 0, 0, 0).pack(fc))
    p.data = good; e.poll()
    return now, p, e, fc, good


def test_split_bad_crc_retains_exact_reads_and_never_refreshes_heartbeat(tmp_path):
    now, p, e, fc, good = receiver(tmp_path)
    seen = []; e.hook = lambda msg, at: seen.append(msg.get_type())
    bad = bytearray(good); bad[-1] ^= 1; bad = bytes(bad)
    original_stamp = e.heartbeat_ns
    for piece in (bad[:7], bad[7:]):
        now[0] += 50_000_000; p.data = piece; e.poll()
    assert e.heartbeat_ns == original_stamp and not seen and e.fault is None
    now[0] += 50_000_000; p.data = good; e.poll()
    assert seen == ['HEARTBEAT'] and e.heartbeat_ns == now[0]
    e.close_rx_evidence()
    rows = [json.loads(s) for s in (tmp_path/'rx.jsonl').read_text().splitlines()]
    assert b''.join(bytes.fromhex(r['wire_hex']) for r in rows if r['kind']=='serial_read') == good+bad+good
    assert any(r['kind']=='rejected_candidate' and r['wire_hex']==bad.hex() for r in rows)
    assert e.parser.summary()['repaired_bytes'] == 0


def test_bad_or_other_peer_frames_cannot_keep_joint_link_fresh(tmp_path):
    now, p, e, fc, good = receiver(tmp_path)
    peer = m.MAVLink(None, srcSystem=250, srcComponent=219)
    now[0] += 500_000_000
    p.data = b'garbage'+bytes(peer.heartbeat_encode(6, 8, 0, 0, 0).pack(peer))
    with pytest.raises(RuntimeError, match='valid FC stream stale'): e.poll()
    assert not e.safe()
    e.close_rx_evidence()


def test_raw_evidence_write_failure_stops_before_ingest(tmp_path):
    now, p, e, fc, good = receiver(tmp_path)
    before = e.heartbeat_ns
    e.close_rx_evidence()
    now[0] += 1; p.data = good
    with pytest.raises(ValueError): e.poll()
    assert e.heartbeat_ns == before and e.fault


@pytest.mark.parametrize('route,phase', [(0,'waiting'), (1,'waiting'), (1,'capture')])
def test_factory_discards_corruption_and_completes_same_window(tmp_path, monkeypatch, route, phase):
    original = Port.telemetry
    injected = []
    log_started = []
    def telemetry(self):
        original(self)
        s = self.sim
        if s.logging and not log_started: log_started.append(s.now)
        active = s.logging and (s.first_pose is None and s.now-log_started[0] > 3_000_000_000 if phase=='waiting' else
                                 s.first_pose is not None and s.now-s.first_pose > 600_000_000)
        if self.route == route and active and not injected:
            bad = bytearray(self.fc.heartbeat_encode(2, 12, 0, 0, 0).pack(self.fc)); bad[-1] ^= 1
            self.inbox.extend(bad); injected.append(bytes(bad))
    monkeypatch.setattr(Port, 'telemetry', telemetry)
    # Explicitly wait in GO phase in the existing fixture (no real keyboard).
    original_prepare = Owner.prepare_recording
    def prepare(self):
        ready_at = self.clock()+4_000_000_000
        self.motion_ready = lambda: self.clock() >= ready_at
        original_prepare(self)
    monkeypatch.setattr(Owner, 'prepare_recording', prepare)
    s, code, report = execute(tmp_path, monkeypatch)
    assert code == 0, (report['errors'],report.get('lifecycle'),report.get('owner'))
    assert s.on_count == s.off_count == 1 and s.last_pose-s.first_pose > 14_000_000_000
    raw_path = next(tmp_path.glob('joint_ev_delay_*/capture_raw.json'))
    window = json.loads(raw_path.read_text())['capture_metadata']['motion_window']
    assert window['ended_ns'] >= window['deadline_ns']
    assert window['deadline_ns']-window['started_ns'] == 15_000_000_000
    rx = report['routes'][route]['joint_rx_evidence']
    assert rx['rejected_candidates'] and injected[0].hex() in open(rx['path']).read()
    assert report['owner']['rates']['restoration_verified'] and s.ports[1].interval == 0
    require_no_pending_recovery(tmp_path)


@pytest.mark.parametrize('failure', ['pad', 'occluded'])
def test_capture_abort_does_not_repoll_failed_input_during_restore(tmp_path, monkeypatch, failure):
    original = Pad.poll
    def poll(self):
        if self.sim.first_pose and self.sim.now-self.sim.first_pose > 600_000_000:
            raise RuntimeError('SYNTHETIC pad disconnected')
        return original(self)
    if failure == 'pad': monkeypatch.setattr(Pad, 'poll', poll)
    s, code, report = execute(tmp_path, monkeypatch, None if failure=='pad' else failure)
    assert code == 2 and not report['hardware_capture_completed']
    assert s.on_count == s.off_count == 1
    assert report['lifecycle']['logger']['file_closed_verified']
    assert report['owner']['rates']['restoration_verified'] and s.ports[1].interval == 0
    assert not report['owner']['cleanup_errors']
    require_no_pending_recovery(tmp_path)


@pytest.mark.parametrize('failure', ['ack_missing', 'readback_wrong'])
def test_unverified_restore_keeps_journal_pending_no_retries(tmp_path, monkeypatch, failure):
    original = Port.write
    def write(self, packet):
        n = original(self, packet)
        msg = self.sent[-1][1]
        if self.route == 1 and msg.get_type()=='COMMAND_LONG' and msg.command==511 and msg.param2==-1:
            if failure=='ack_missing': self.inbox.clear()
            else: self.interval = 123456
        return n
    monkeypatch.setattr(Port, 'write', write)
    s, code, report = execute(tmp_path, monkeypatch)
    assert code == 2 and not report['owner']['rates']['restoration_verified']
    assert report['lifecycle']['logger']['file_closed_verified']
    attempts = [msg for _,msg in s.ports[1].sent if msg.get_type()=='COMMAND_LONG' and msg.command==511]
    assert [msg.param2 for msg in attempts] == [200000, -1]
    with pytest.raises(ValueError, match='needs recovery'): require_no_pending_recovery(tmp_path)


@pytest.mark.parametrize('lose_replies', [False, True])
def test_corrupt_stream_aborts_and_restore_needs_valid_replies(tmp_path, monkeypatch, lose_replies):
    original = Port.telemetry
    def telemetry(self):
        s = self.sim
        if self.route==1 and s.first_pose and s.now-s.first_pose > 600_000_000:
            self.inbox.extend(b'bad')
        else: original(self)
    monkeypatch.setattr(Port, 'telemetry', telemetry)
    original_write = Port.write
    def write(self, packet):
        n = original_write(self, packet)
        s = self.sim
        if lose_replies and self.route==1 and s.first_pose and s.now-s.first_pose > 600_000_000:
            self.inbox.clear()
        return n
    monkeypatch.setattr(Port, 'write', write)
    s, code, report = execute(tmp_path, monkeypatch)
    assert code == 2 and s.last_pose-s.first_pose < 1_500_000_000
    assert report['lifecycle']['logger']['file_closed_verified']
    assert report['owner']['rates']['restoration_verified'] is not lose_replies
    assert report['routes'][1]['joint_rx_evidence']['rejected_candidates'] > 0
    if lose_replies:
        with pytest.raises(ValueError): require_no_pending_recovery(tmp_path)
    else: require_no_pending_recovery(tmp_path)


def test_existing_recovery_helper_accepts_only_saved_252_original():
    record = dict(scope=SCOPE, needs_restore=True,
                  rates=dict(original_intervals_us={'252':0}, possibly_changed=[252]))
    original, changed = retained_rates(record)
    enc = m.MAVLink(None)
    msg = interval_message(enc,252,-1,short_status=True)
    args = dict(safe=True, locked=True, original=original, changed=changed)
    assert allowed(msg, **args)
    for value in (0,200000,1):
        assert not allowed(interval_message(enc,252,value,short_status=True), **args)
    assert not allowed(msg, **{**args,'safe':False})
    assert not allowed(msg, **{**args,'locked':False})
    record['rates']['original_intervals_us']={'400':0}
    record['rates']['possibly_changed']=[400]
    with pytest.raises(ValueError): retained_rates(record)
