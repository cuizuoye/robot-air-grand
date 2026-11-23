"""In-memory USB + WiFi joint capture. Never connect real devices."""
import json
import socket

import pytest
from test_ev_delay_joint_device import execute, Port
from ev_delay_maintenance_host import VISUAL_QUERY
from ev_delay_maintenance import IsolatedGroundState
from pymavlink.dialects.v20 import common as m


@pytest.fixture(autouse=True)
def native_wifi(monkeypatch):
    monkeypatch.setattr(socket, 'socket', lambda *a, **kw: pytest.fail('Real network attempted'))
    original_write, original_shell = Port.write, Port.shell

    def write(self, packet):
        inbox = self.sim.ports[0].inbox
        before = len(inbox)
        result = original_write(self, packet)
        msg = self.sent[-1][1]
        if self.route == 1 and msg.get_type() == 'VISION_POSITION_ESTIMATE':
            # Native receipt works even with MAVLink forwarding disabled.
            del inbox[before:]
            self.sim.visual = dict(timestamp=self.sim.boot_ms()*1000,
                timestamp_sample=self.sim.boot_ms()*1000, pose_frame=1, velocity_frame=0,
                reset_counter=0, quality=0, position=[msg.x,msg.y,msg.z], q=[1.,0.,0.,0.])
        return result

    def shell(self, command):
        if command == VISUAL_QUERY:
            row = self.sim.visual
            return 'TOPIC: vehicle_visual_odometry\n'+'\n'.join(f' {k}: {v}' for k,v in row.items())
        answer = original_shell(self, command)
        if command == b'listener actuator_armed -n 1\n':
            answer = answer.replace('lockdown: True','lockdown: False')
        return answer

    monkeypatch.setattr(Port, 'write', write)
    monkeypatch.setattr(Port, 'shell', shell)


def test_wifi_full_window_without_gamepad_home_or_rate_commands(tmp_path, monkeypatch):
    s, code, report = execute(tmp_path, monkeypatch, transport='wifi')
    assert code == 0, report
    assert s.on_count == s.off_count == 1
    assert report['owner']['home'] is None
    assert report['owner']['visual_preroll']['received_confirmed']
    assert all(p.closed == 1 for p in s.ports)
    assert not report['calibration_approved'] and not report['flight_authorized']
    messages = [msg for p in s.ports for _,msg in p.sent]
    assert {msg.command for msg in messages if msg.get_type()=='COMMAND_LONG'} == {512}
    assert not any(msg.get_type() in ('PARAM_SET','PARAM_EXT_SET','SET_MODE',
        'SET_POSITION_TARGET_LOCAL_NED','RC_CHANNELS_OVERRIDE') for msg in messages)
    raw = json.loads(next(tmp_path.glob('joint_ev_delay_*/capture_raw.json')).read_text())
    assert raw['capture_metadata']['pose_route'] == 'wifi'
    window = raw['capture_metadata']['motion_window']
    assert window['started_ns'] >= window['go_ns']+3_000_000_000
    assert window['deadline_ns']-window['started_ns'] == 15_000_000_000
    sends = [e for e in raw['events'] if e['kind']=='calibration_send']
    assert 450 <= len(sends) <= 480
    assert raw['capture_metadata']['vicon_delivery_policy']=='latest_only'
    assert raw['capture_metadata']['requested_pose_period_ns']==31_250_000
    assert all(e['route']=='wifi' and e['sent_ns'] >= window['started_ns'] for e in sends)


@pytest.mark.parametrize('fault', ['stale_trs','short_write'])
def test_wifi_loss_retains_usb_log_close_export(tmp_path, monkeypatch, fault):
    s, code, report = execute(tmp_path, monkeypatch, fault, transport='wifi')
    assert code == 2 and not report['hardware_capture_completed']
    assert s.on_count == s.off_count == 1, report
    assert report['lifecycle']['logger']['file_closed_verified']
    assert report['lifecycle']['logger']['exported']
    assert all(p.closed == 1 for p in s.ports)


def test_wifi_visual_mismatch_never_starts_motion_window(tmp_path, monkeypatch):
    original = Port.shell
    def shell(self, command):
        if command == VISUAL_QUERY: self.sim.visual['position'] = [100.,100.,100.]
        return original(self, command)
    monkeypatch.setattr(Port,'shell',shell)
    s, code, report = execute(tmp_path, monkeypatch, transport='wifi')
    assert code == 2
    assert not report['lifecycle'].get('capture_started_ns')
    assert s.on_count == s.off_count == 1
    assert 'matching' in str(report)


def test_isolated_state_never_invents_lock_and_latches_armed_state():
    state = IsolatedGroundState()
    enc = m.MAVLink(None,srcSystem=1,srcComponent=1)
    def heartbeat(at, armed):
        wire = enc.heartbeat_encode(2,12,128 if armed else 0,0,0).pack(enc)
        state.ingest(m.MAVLink(None).parse_char(wire),at)
    heartbeat(1000,False)
    state.clock.reference = dict(received_ns=1000)
    proof = state.assess(1000,identity_verified=True)
    assert proof['valid'] and not proof['output_lock_proven']
    heartbeat(1001,True)
    heartbeat(1002,False)
    assert not state.assess(1002,identity_verified=True)['valid']


def test_wifi_complete_link_loss_keeps_usb_export(tmp_path, monkeypatch):
    original_telemetry, original_write = Port.telemetry, Port.write
    def lost(port):
        return port.route == 1 and port.sim.first_pose and port.sim.now-port.sim.first_pose > 600_000_000
    def telemetry(self):
        if not lost(self): original_telemetry(self)
    def write(self, packet):
        if lost(self): return len(packet)  # UDP send succeeds while all replies cease.
        return original_write(self,packet)
    monkeypatch.setattr(Port,'telemetry',telemetry)
    monkeypatch.setattr(Port,'write',write)
    s, code, report = execute(tmp_path,monkeypatch,transport='wifi')
    assert code == 2
    assert s.on_count == s.off_count == 1, report
    assert report['lifecycle']['logger']['exported']
    assert not report['hardware_capture_completed']


def test_default_wifi_preview_opens_no_resources(monkeypatch):
    import ev_delay_joint_capture as live
    monkeypatch.setattr(live,'local_inputs',lambda:(dict(revision='MOCK'),{}))
    monkeypatch.setattr(live,'execute',lambda *a:pytest.fail('Preview opened devices'))
    assert live.main(['--transport','wifi','--wifi-peer','192.168.4.1','--wifi-bind','192.168.4.2']) == 0
    with pytest.raises(SystemExit): live.main(['--wifi-peer','192.168.4.1'])
