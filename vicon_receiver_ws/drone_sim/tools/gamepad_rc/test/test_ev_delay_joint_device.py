"""New real adapters exercised through RAW MAVLink bytes, no physical devices.

The full factory is used with simulated serial/source/gamepad at its edges.
No previously passed static hardware checks are invoked by these tests.
"""
from dataclasses import replace
import json
import math
from pathlib import Path
import struct
from types import SimpleNamespace as NS

import pytest
from pymavlink.dialects.v20 import common as m

import ev_delay_joint_capture as live
from ev_delay_device_io import Endpoint, LoggerIO
from ev_delay_live_owner import Owner
from ev_delay_maintenance_host import maintenance_usb_allowed
from mavlink_crc_stream import CrcResyncParser
from output_lock_status import MAGIC
from fault_archive_service import HEADER as FTP_HEADER, nuttx_crc
from test_static_ulog_capture import HEADER as ULOG_HEADER, PATH as LOG_PATH
from test_ev_delay_recording_session import session
from test_ev_delay_sources import sample

IDENTITY = dict(firmware='1.14.3', board_version=54, uid=123, uid2=bytes(range(18)).hex())
VALUES = dict(SDLOG_PROFILE=(3, 6), SDLOG_MODE=(0, 6), EKF2_EV_NOISE_MD=(0, 6),
    EKF2_EVP_NOISE=(.1, 9), EKF2_EVA_NOISE=(.1, 9), EKF2_EV_CTRL=(2, 6),
    EKF2_EV_DELAY=(0., 9), EKF2_EV_POS_X=(0., 9), EKF2_EV_POS_Y=(0., 9), EKF2_EV_POS_Z=(0., 9))


def wire_value(value, kind):
    return struct.unpack('<f', struct.pack('<i', value))[0] if kind == 6 else value


class Simulation:
    def __init__(self, failure=None):
        self.now = 10_000_000_000
        self.start = self.now
        self.failure = failure
        self.ports = []
        self.logging = False
        self.home_locked = False
        self.on_count = self.off_count = 0
        self.first_pose = self.last_pose = None
        self.injected = False
    def ns(self): return self.now
    def sleep(self, seconds): self.now += max(500_000, round(seconds*1e9))
    def boot_ms(self): return (self.now+5_000_000_000)//1_000_000
    def serial(self, path, baud, **kwargs):
        assert baud == 115200 and kwargs == dict(timeout=0, write_timeout=.02, exclusive=True)
        p = Port(self, path, len(self.ports))
        self.ports.append(p)
        return p


class Port:
    def __init__(self, sim, path, route):
        self.sim, self.port, self.route = sim, path, route
        self.is_open = True
        self.inbox = bytearray()
        self.fc = m.MAVLink(None, srcSystem=1, srcComponent=1)
        self.decoder = CrcResyncParser()
        self.sent = []
        self.next_att = self.next_hb = self.next_lock = 0
        self.interval = 0
        self.closed = 0
    def append(self, msg):
        self.inbox.extend(msg.pack(self.fc))
        self.fc.seq = (self.fc.seq+1) % 256
    def telemetry(self):
        s = self.sim
        if s.now >= self.next_hb:
            self.append(self.fc.heartbeat_encode(m.MAV_TYPE_QUADROTOR, m.MAV_AUTOPILOT_PX4, 0, 0, 0))
            self.next_hb = s.now+500_000_000
        fail_active = s.first_pose is not None and s.now-s.first_pose > 600_000_000
        if s.now >= self.next_att:
            if not (s.failure == 'stale_trs' and self.route == 1 and fail_active):
                self.append(self.fc.attitude_quaternion_encode(s.boot_ms()-20, 1, 0, 0, 0, 0, 0, 0))
            self.next_att = s.now+20_000_000
        if self.route == 1 and self.interval > 0 and s.now >= self.next_lock:
            self.append(self.fc.named_value_int_encode(s.boot_ms()-10, b'OUTLOCK1', MAGIC|(5 if s.home_locked else 1)))
            self.next_lock = s.now+self.interval*1000
        if self.route == 1 and s.failure == 'crc' and fail_active and not s.injected:
            bad = bytearray(self.fc.heartbeat_encode(2, 12, 0, 0, 0).pack(self.fc)); bad[-1] ^= 1
            self.inbox.extend(bad); s.injected = True
    @property
    def in_waiting(self):
        self.telemetry()
        return len(self.inbox)
    def read(self, count):
        data = bytes(self.inbox[:count]); del self.inbox[:count]
        return data
    def write(self, packet):
        msg, = self.decoder.parse_buffer(packet)
        self.sent.append((self.sim.now, msg))
        s, kind = self.sim, msg.get_type()
        if kind == 'COMMAND_LONG':
            if msg.command == 512:
                self.append(self.fc.autopilot_version_encode(0, (1<<24)|(14<<16)|(3<<8), 0, 0, 54,
                    [1]*8, [0]*8, [0]*8, 0, 0, 123, list(range(18))))
            elif msg.command == 185:
                s.home_locked = True
                self.append(self.fc.command_ack_encode(185, 0, 0, 0, 250, 213))
            elif msg.command == 510:
                self.append(self.fc.message_interval_encode(252, self.interval))
            elif msg.command == 511:
                self.interval = max(0, int(msg.param2))
                self.append(self.fc.command_ack_encode(511, 0, 0, 0, 250, 213))
            else: raise AssertionError('Unexpected command '+str(msg.command))
        elif kind == 'TIMESYNC':
            self.append(self.fc.timesync_encode(s.now+5_000_000_000, msg.ts1))
        elif kind == 'PARAM_REQUEST_READ':
            value, ptype = VALUES[msg.param_id]
            self.append(self.fc.param_value_encode(msg.param_id.encode(), wire_value(value, ptype), ptype, 10, 0))
        elif kind == 'SERIAL_CONTROL' and msg.count:
            command = bytes(msg.data[:msg.count])
            answer = self.shell(command)
            raw = command+answer.encode()+b'\nnsh> '
            for i in range(0, len(raw), 70):
                part = raw[i:i+70]
                self.append(self.fc.serial_control_encode(10, 1, 0, 0, len(part), part.ljust(70, b'\0')))
        elif kind == 'FILE_TRANSFER_PROTOCOL':
            seq, _, op, size, _, _, _, offset = FTP_HEADER.unpack_from(bytes(msg.payload))
            if op == 4: body = struct.pack('<I', len(ULOG_HEADER))
            elif op == 14: body = struct.pack('<I', nuttx_crc(ULOG_HEADER))
            elif op == 5: body = ULOG_HEADER[offset:offset+size]
            elif op == 1: body = b''
            else: raise AssertionError('FTP write attempted')
            response = FTP_HEADER.pack((seq+1)%65536, 0, 128, len(body), op, 0, 0, offset)+body
            self.append(self.fc.file_transfer_protocol_encode(0, 250, 219, response.ljust(251, b'\0')))
        elif kind == 'VISION_POSITION_ESTIMATE':
            s.first_pose = s.first_pose or s.now
            s.last_pose = s.now
            if s.failure == 'short_write' and s.now-s.first_pose > 600_000_000:
                return len(packet)-1
            # Existing hardware forwards the original TRS frame over FC USB.
            if self.route == 1:
                s.ports[0].inbox.extend(packet)
        else:
            assert kind in ('HEARTBEAT', 'SERIAL_CONTROL'), kind
        return len(packet)
    def shell(self, command):
        s = self.sim
        if command == b'logger on\n': s.logging = True; s.on_count += 1; return ''
        if command == b'logger off\n': s.logging = False; s.off_count += 1; return ''
        if command == b'logger status\n':
            return 'Full File Logging Running:\nLog file: '+LOG_PATH if s.logging else 'Not logging'
        if command == b'mount\n': return '/fs/microsd type vfat'
        if command == b'ps\n': return ' 100 97 60 FIFO pthread Waiting 0 log_writer_file\n'
        if command == b'cat /proc/100/group/fd\n':
            return 'FD POS OFLAGS PATH\n 0 0 101 \n 1 0 2 /dev/console_buf\n'+(' 3 99543 6 \n' if s.logging else '')
        if command == b'listener actuator_armed -n 1\n':
            return f'TOPIC: actuator_armed\n timestamp: {s.boot_ms()*1000}\n armed: False\n lockdown: True\n manual_lockdown: False\n force_failsafe: False\n'
        if command == b'listener sensor_selection -n 1\n':
            return f'TOPIC: sensor_selection\n timestamp: 9000000\n gyro_device_id: 42\n accel_device_id: 43\n'
        raise AssertionError(command)
    def close(self): self.is_open = False; self.closed += 1


class Source:
    def __init__(self, sim):
        self.sim = sim
        self.latest = None
        self.stopped = False
        self.on_frame = lambda row, at: None
        self.report = {'simulated': True}
        self.index = 0
        self.next = 0
    def now_ros_ns(self): return self.sim.now+1_800_000_000_000_000_000
    def poll(self):
        if self.stopped or self.sim.now < self.next: return
        self.next = self.sim.now+3_000_000
        self.index += 1
        now = self.sim.now
        self.latest = sample(sequence=self.index, frame=self.index, stamp=self.now_ros_ns()-1000,
            sdk_hz=1e9/3e6, sdk_begin_ns=now-3000, sdk_end_ns=now-2000,
            callback_ns=now, callback_ros_ns=self.now_ros_ns(),
            world_pose=[math.sin((now-self.sim.start)/1e9)*.01, 0., 0., 0., 0., 0., 1.])
        if self.sim.failure == 'occluded' and self.sim.first_pose and now-self.sim.first_pose > 600_000_000:
            self.latest['occluded'] = True
        self.on_frame(self.latest, now)
    def stop_inputs(self): self.stopped = True
    def close(self): self.stop_inputs()


class Pad:
    generation = {'name': 'SIMULATED_ONLY', 'inode': 10}
    def __init__(self, sim): self.sim = sim; self.pressed = self.released = False; self.closed = 0
    def poll(self):
        elapsed = self.sim.now-self.sim.start
        events = []
        if elapsed >= 1_000_000_000 and not self.pressed: self.pressed = True; events = [True]
        elif elapsed >= 1_100_000_000 and not self.released: self.released = True; events = [False]
        return dict(home_events=events)
    def close(self): self.closed += 1


def execute(tmp_path, monkeypatch, failure=None, *, physical_overrides=None, transport='trs'):
    import sys
    import serial
    import preparation_host_bindings
    import ev_delay_vicon_source
    import vicon_receiver_provenance
    s = Simulation(failure)
    # Native capture now always needs a local GO; fixtures supply synthetic input.
    monkeypatch.setattr(sys, 'stdin', NS(isatty=lambda: True, fileno=lambda: 0))
    monkeypatch.setattr(live, 'motion_cue_ready', lambda fd: True)
    monkeypatch.setattr(live.time, 'monotonic_ns', s.ns)
    monkeypatch.setattr(live.time, 'monotonic', lambda: s.now/1e9)
    monkeypatch.setattr(live.time, 'sleep', s.sleep)
    monkeypatch.setattr(serial, 'Serial', s.serial)
    monkeypatch.setattr(preparation_host_bindings, 'LinuxGamepad', lambda: Pad(s))
    if transport == 'wifi':
        def wifi_port(peer, bind, *, on_datagram):
            p = Port(s, 'SIMULATED_WIFI', 1)
            p.check = lambda: None
            s.ports.append(p)
            return p
        monkeypatch.setattr(live, 'FixedPeerUDP', wifi_port)
        monkeypatch.setattr(preparation_host_bindings, 'LinuxGamepad',
                            lambda: pytest.fail('WiFi capture opened gamepad'))
    def source_factory(*a,latest_only=False):
        assert latest_only is (transport=='wifi')
        return Source(s)
    monkeypatch.setattr(ev_delay_vicon_source, 'ViconSource', source_factory)
    monkeypatch.setattr(vicon_receiver_provenance, 'verify_current', lambda: {})
    monkeypatch.setattr(live, 'require_port_free', lambda p: None)
    monkeypatch.setattr(live, 'RUNS', tmp_path)
    usb, trs = tmp_path/'usb', tmp_path/'trs'
    usb.touch(); trs.touch()
    monkeypatch.setattr(live, 'USB', usb)
    baseline = dict(identity=IDENTITY, after_save={})
    for name in ('SDLOG_PROFILE', 'SDLOG_MODE'):
        value, kind = VALUES[name]
        baseline['after_save'][name] = dict(value=value, type=kind,
            wire_value_hex=struct.pack('<f', wire_value(value, kind)).hex())
    options = dict(confirm_props_off_battery_disconnected_usb_only=True,
        confirm_props_installed_battery_disconnected_usb_only=False,
        confirm_motor_power_isolated=True, confirm_independent_usb_cutoff=True,
        authorization_id='MOCK_ONLY', trs_port=str(trs))
    if transport == 'wifi':
        options.update(transport='wifi', trs_port=str(live.TRS), wifi_peer='192.168.4.1',
                       wifi_bind='192.168.4.2', wifi_port=14550)
    options.update(physical_overrides or {})
    args = NS(**options)
    config = dict(proof=tmp_path/'proof', revision='MOCK', orientation=[0., 0., 0., 1.], origin=[0., 0., 0.])
    code = live.execute(args, config, baseline)
    report = json.loads(next(tmp_path.glob('joint_ev_delay_*/report.json')).read_text())
    return s, code, report


def test_real_factory_byte_level_full_capture_and_export(tmp_path, monkeypatch):
    s, code, report = execute(tmp_path, monkeypatch)
    assert code == 0, report
    assert report['hardware_capture_completed']  # SIMULATION; never save as physical evidence.
    assert not report['calibration_approved'] and not report['flight_authorized']
    assert s.on_count == s.off_count == 1
    assert all(p.closed == 1 for p in s.ports)
    assert s.ports[1].interval == 0
    traces = json.loads(next(tmp_path.glob('joint_ev_delay_*/capture_raw.json')).read_text())
    sends = [e for e in traces['events'] if e['kind'] == 'calibration_send']
    assert 400 <= len(sends) <= 480
    assert len({e['frame'] for e in sends}) == len(sends)
    assert s.last_pose-s.first_pose > 14_000_000_000
    assert report['lifecycle']['selection_snapshots']['before']['read_fc_upper_us'] < (s.first_pose+5_000_000_000)//1000-500000
    assert report['owner']['parameters_before'].keys() == report['owner']['parameters_after'].keys()
    forbidden = {'PARAM_SET', 'PARAM_EXT_SET', 'SET_MODE', 'SET_ATTITUDE_TARGET'}
    assert not any(msg.get_type() in forbidden for p in s.ports for _, msg in p.sent)
    assert sum(msg.get_type() == 'COMMAND_LONG' and msg.command == 185 for p in s.ports for _, msg in p.sent) == 1


# Isolated CRC continuation is now covered by test_joint_rx_recovery_delta.
@pytest.mark.parametrize('failure', ['occluded', 'short_write', 'stale_trs'])
def test_live_wiring_fault_stops_once_and_preserves_USB_cleanup(tmp_path, monkeypatch, failure):
    s, code, report = execute(tmp_path, monkeypatch, failure)
    assert code == 2
    assert not report['hardware_capture_completed']
    assert s.on_count == s.off_count == 1, report
    assert report['lifecycle']['logger']['file_closed_verified']
    assert all(p.closed == 1 for p in s.ports)
    assert s.last_pose-s.first_pose < 2_000_000_000


def test_continuity_keeps_UID_receipt_and_requires_actual_attitude_receipt():
    s, obs = session()
    obs = replace(obs, identity_continuity_received_ns=obs.attitude_received_ns)
    assert s.begin(obs)
    bad = replace(obs, now_ns=obs.now_ns+1, identity_continuity_received_ns=obs.now_ns+1)
    assert not s.gate.check(bad)
    assert s.gate.fault == 'identity_continuity_invalid'


def test_timesync_permits_real_execution_delay_not_wrong_or_future_token():
    s, obs = session()
    assert s.begin(obs)
    token = s.gate.lock.clock.request(obs.now_ns)
    msg = m.MAVLink(None).timesync_encode(0, token)
    assert s.gate.permits(msg, replace(obs, now_ns=obs.now_ns+1000))
    s2, obs2 = session()
    assert s2.begin(obs2)
    token2 = s2.gate.lock.clock.request(obs2.now_ns+2000)
    assert not s2.gate.permits(m.MAVLink(None).timesync_encode(0, token2), obs2)


def test_default_entry_does_not_instantiate_hardware(monkeypatch):
    monkeypatch.setattr(live, 'local_inputs', lambda: (dict(revision='MOCK'), {}))
    monkeypatch.setattr(live, 'execute', lambda *a: pytest.fail('default opened hardware'))
    assert live.main([]) == 0
    with pytest.raises(SystemExit): live.main(['--run'])


def test_post_selection_failure_still_exports_closed_raw_ULog(tmp_path):
    from test_ev_delay_maintenance_host import fixture
    host, owner, io = fixture()
    io.selection_fail_after = True
    out = host.run_injected(tmp_path)
    assert not out['injected_cycle_completed']
    assert out['logger']['exported'] and out['logger']['file_closed_verified']
    assert any('selection_after' in e for e in out['cleanup_errors'])


@pytest.mark.parametrize('kind', ['arm', 'param', 'mode', 'shell'])
def test_USB_adapter_rejects_actuation_and_unlisted_shell_before_write(kind):
    sim = Simulation()
    port = Port(sim, 'SIMULATED', 0)
    ep = Endpoint(port, 219, IDENTITY, clock=sim.ns)
    io = LoggerIO(ep, {'identity': IDENTITY}, lambda: None,
                  clock_bounds=lambda n: (1, 2), boot_session='SIMULATED')
    enc = ep.enc
    messages = dict(arm=enc.command_long_encode(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0),
        param=enc.param_set_encode(1, 1, b'EKF2_EV_DELAY', 65, 9), mode=enc.set_mode_encode(1, 1, 1),
        shell=enc.serial_control_encode(10, 2, 0, 0, 7, b'reboot\n'.ljust(70, b'\0')))
    with pytest.raises(PermissionError): io.send(messages[kind])
    assert not port.sent


def test_USB_loss_after_start_keeps_power_flag_and_does_not_send_to_new_identity(tmp_path, monkeypatch):
    original = Port.telemetry
    def telemetry(self):
        original(self)
        s = self.sim
        if self.route == 0 and s.first_pose and s.now-s.first_pose > 600_000_000 and not s.injected:
            self.append(self.fc.attitude_quaternion_encode(1, 1, 0, 0, 0, 0, 0, 0))
            s.injected = True
    monkeypatch.setattr(Port, 'telemetry', telemetry)
    s, code, report = execute(tmp_path, monkeypatch)
    assert code == 2
    assert s.on_count == 1 and s.off_count == 0
    assert report['lifecycle']['keep_USB_power_required']
    assert all(p.closed == 1 for p in s.ports)


def test_factory_refuses_missing_physical_confirmation_before_any_resource(tmp_path, monkeypatch):
    args = NS(confirm_props_off_battery_disconnected_usb_only=False, authorization_id='MOCK')
    with pytest.raises(PermissionError, match='Current props-off'):
        live.execute(args, {}, {})
    assert not list(tmp_path.iterdir())
