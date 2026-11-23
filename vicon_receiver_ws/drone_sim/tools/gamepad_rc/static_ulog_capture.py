"""One-use, operator-authorized 12s STATIC capture; accept only >=10s actual IMU.

Only one logger on/off pair is authorized; no param/save/reboot/HOME/pose/control sends.
Reuse existing CRC decoder, identity evidence, startup framing and NSH parsing.
"""
import sys
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import time

import serial
from pymavlink.dialects.v20 import common as m
from mavlink_crc_stream import CrcResyncParser
from mavlink_readonly_probe import Evidence, decode_parameter, readonly_message
from persistent_shell import reply_complete
from estop_only_policy import parse_actuator_snapshot

from types import SimpleNamespace
import re
from static_ulog_capture_policy import allowed as capture_allowed
from static_ulog_capture_actions import (FileReader, perform, stop_and_finalize,
    finalize_resources, abort_capture)

"""USB maintenance-only startup framing; no I/O or flight authorization.

The shared CRC decoder is unchanged. One pre-sync orphan FE is quarantined
only until the immediately following parsed item proves an intact MAVLink2
packet from expected FC 1/1. No CRC failure is ignored, before or after sync.
"""

class StartupPrefixSync:
    WINDOW_NS = 2_000_000_000
    MAX_PREFIX = 4096

    def __init__(self, start_ns):
        self.start_ns = start_ns
        self.first_valid_ns = None
        self.skipped = []
        self.skipped_bytes = 0
        self.pending_orphan = None
        self.confirmed_orphans = []

    def _bounded(self, now, count=0):
        return (0 <= now-self.start_ns < self.WINDOW_NS
                and self.skipped_bytes+count <= self.MAX_PREFIX)

    def _record(self, msg, at_ns):
        data = bytes(msg.data)
        self.skipped_bytes += len(data)
        row = dict(at_ns=at_ns, reason=msg.reason, bytes_hex=data.hex())
        self.skipped.append(row)
        return row

    def may_skip(self, msg, at_ns, *, discard_corrupt_prefix=False):
        if msg.get_type() != 'BAD_DATA':
            if self.first_valid_ns is None:
                if not self._bounded(at_ns):
                    raise RuntimeError('Initial frame synchronization deadline/budget exceeded')
                if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
                    raise RuntimeError('First valid frame is not expected FC 1/1')
                if self.pending_orphan is not None:
                    if bytes(msg.get_msgbuf())[:1] != b'\xfd':
                        raise RuntimeError('Orphan FE not immediately followed by intact expected MAVLink2 frame')
                    self.confirmed_orphans.append(dict(**self.pending_orphan,
                        confirmed_ns=at_ns, next_type=msg.get_type(), next_system=1, next_component=1))
                    self.pending_orphan = None
                self.first_valid_ns = at_ns
            return False

        data = bytes(msg.data)
        if (self.first_valid_ns is not None or not data or not self._bounded(at_ns, len(data))
                or self.pending_orphan is not None):
            return False
        if discard_corrupt_prefix:
            # Joint capture persists the original reads and rejected candidates.
            # Discarding a candidate is NOT accepting its contents as a frame.
            # Original 2s/4096-byte startup bounds and FC 1/1 proof still apply;
            # all other callers keep the historical strict startup policy.
            self._record(msg, at_ns)
            return True
        if msg.reason == 'Bad prefix':
            self._record(msg, at_ns)
            return True
        if msg.reason == 'Invalid source ID' and data == b'\xfe':
            # Quarantine only, not a valid message/state/identity proof. A CRC
            # valid expected v2 packet must be the next decoded item, including
            # when the packet is split across serial reads.
            self.pending_orphan = self._record(msg, at_ns)
            return True
        return False

    def check_deadline(self, at_ns):
        if self.first_valid_ns is None and not self._bounded(at_ns):
            raise RuntimeError('No verified initial frame within 2s; no retry')

    def summary(self):
        return dict(window_ns=self.WINDOW_NS, max_bytes=self.MAX_PREFIX,
            first_valid_ns=self.first_valid_ns, skipped_prefix_bytes=self.skipped_bytes,
            skipped=self.skipped, pending_unverified_orphan=self.pending_orphan,
            confirmed_orphans=self.confirmed_orphans, CRC_bypass=False,
            post_sync_bad_data_tolerated=False, flight_authorization=False)


ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'preparation_runs/b3_3_profile3_save_retry1_20261021_112307_783498/report.json'
BASE_SHA = 'c34eafdbaa984a5408abc4843a47c7ec665a26f24ebada258514503e074a777a'
USB = Path('/dev/serial/by-id/usb-MicoAir_MicoAir743v2_0-if00')
BASELINE = None  # Load evidence only after explicit maintenance authorization.
PARAMS = ('SDLOG_PROFILE', 'SDLOG_MODE')
COMMANDS = (b'mount\n', b'logger status\n', b'listener actuator_armed -n 1\n')

def pname(msg):
    v = msg.param_id
    return (v.decode('ascii') if isinstance(v, bytes) else v).rstrip('\0')

def permitted(msg, safe):
    # Offline default context has no logger start/stop authorization.
    return capture_allowed(msg, safe=safe, phase='PRECHECK', on_sent=False, off_sent=False)

def selftest():
    enc = m.MAVLink(None)
    assert len(PARAMS) == 2 and len(set(PARAMS)) == 2
    assert permitted(enc.param_request_read_encode(1, 1, b'SDLOG_PROFILE', -1), True)
    assert not permitted(enc.param_request_read_encode(1, 1, b'SDLOG_PROFILE', -1), False)
    assert not permitted(enc.param_request_read_encode(2, 1, b'SDLOG_PROFILE', -1), True)
    assert not permitted(enc.param_request_read_encode(1, 1, b'UNAUTHORIZED', -1), True)
    assert not permitted(enc.param_set_encode(1, 1, b'SDLOG_PROFILE', 3, 6), True)
    assert not permitted(enc.param_ext_set_encode(1, 1, b'SDLOG_PROFILE', b'3', 6), True)
    for command in (176, 185, 241, 245, 246, 400, 511):
        assert not permitted(enc.command_long_encode(1, 1, command, 0, 0, 0, 0, 0, 0, 0, 0), True)
    assert permitted(enc.command_long_encode(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0), False)
    for p in COMMANDS:
        assert permitted(enc.serial_control_encode(10, 2, 0, 0, len(p), p.ljust(70, b'\0')), True)
    for p in (b'param save\n', b'reboot\n', b'logger on\n', b'logger off\n', b'commander arm\n', b'param set SDLOG_PROFILE 3\n'):
        assert not permitted(enc.serial_control_encode(10, 2, 0, 0, len(p), p.ljust(70, b'\0')), True)
    assert not permitted(enc.set_mode_encode(1, 0, 0), True)
    assert not permitted(enc.vision_position_estimate_encode(0, 0, 0, 0, 0, 0, 0), True)
    source = m.MAVLink(None, srcSystem=1, srcComponent=1)
    packet = bytes(source.heartbeat_encode(m.MAV_TYPE_QUADROTOR, m.MAV_AUTOPILOT_PX4, 0, 0, 0).pack(source))
    dec = CrcResyncParser(); sync = StartupPrefixSync(10)
    messages = dec.parse_buffer(b'\x01\x02' + packet)
    assert sync.may_skip(messages[0], 20)
    assert not sync.may_skip(messages[1], 21)
    assert not sync.may_skip(m.MAVLink_bad_data(b'x', 'Bad prefix'), 22)
    damaged = bytearray(packet); damaged[-1] ^= 1
    assert not StartupPrefixSync(10).may_skip(CrcResyncParser().parse_buffer(damaged)[0], 20)
    print('PASS offline read-only TX whitelist and bounded startup framing; no device opened', flush=True)

def main(authorization_id=None, physical_confirmed=False):
    global BASELINE
    selftest()
    if (not physical_confirmed or not isinstance(authorization_id, str)
            or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,79}', authorization_id)):
        raise RuntimeError('Fresh scoped authorization ID and physical confirmation required; no serial opened')
    BASELINE = json.loads(BASE.read_text())
    if hashlib.sha256(BASE.read_bytes()).hexdigest() != BASE_SHA:
        raise RuntimeError('Prior evidence hash mismatch; no serial opened')
    if not BASELINE['save_completed'] or BASELINE['save_exit_code'] != 0 or BASELINE['error']:
        raise RuntimeError('Prior save not verified; no serial opened')
    auth_dir = ROOT/'preparation_runs/static_ulog_authorizations'
    auth_dir.mkdir(exist_ok=True)
    marker = auth_dir/(authorization_id+'.json')
    if marker.exists():
        raise RuntimeError('One-use authorization consumed; no retry')
    target = USB.resolve(strict=True)
    owners = subprocess.run(['fuser', str(target)], capture_output=True, text=True)
    if owners.returncode != 1 or target.name != 'ttyACM0':
        raise RuntimeError('Expected free FC USB required; no serial opened')
    run = ROOT / 'preparation_runs' / ('b3_3_static_ulog_capture_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    run.mkdir(parents=True, exist_ok=False)
    start = time.monotonic_ns()
    report = dict(schema=1, scope='Authorized ONE ~12s static logger on/off, >=10s actual IMU acceptance, verified file close and READONLY export; no parameters, save, reboot, HOME, flight',
        utc_start=datetime.now(timezone.utc).isoformat(), started_monotonic_ns=start,
        operator_confirmed=dict(props_removed=True, battery_disconnected=True, USB_only=True,
            stationary=True, independent_power_removal=True, serial_programs_closed=True, stable_USB=True),
        baseline_path=str(BASE), baseline_sha256=BASE_SHA, port=str(USB), resolved_port=str(target),
        probe_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        expected_identity=BASELINE['identity'], parameters={}, mismatches=[], shell_records=[],
        prior_failed_run=str(ROOT/'preparation_runs/b3_3_static_ulog_precheck_readonly_20261021_113628_038667/report.json'),
        capture_authorization_id=authorization_id,
        capture_authorization='Separate explicit authorization required for this one capture; does not confer flight permission',
        startup_sync_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        rx_startup_raw_chunks=[], rx_bad_data=[],
        tx_audit=[], status_texts=[], attitudes=[], state_proofs=[], error=None,
        precheck_verified=False, capture_completed=False, file_closed_verified=False,
        flight_authorized=False, stage='CONNECTING', serial_closed=False)
    with marker.open('x') as stream:
        json.dump(dict(run=str(run), scope=report['scope']), stream)
    enc = m.MAVLink(None, srcSystem=250, srcComponent=219)
    dec = CrcResyncParser(); sync = StartupPrefixSync(start)
    ev = Evidence(source_component=219, expected_parameters=PARAMS)
    rx = Counter(); tx = Counter(); values = {}; buffer = ''; next_hb = 0; last_boot = None
    port = None
    state = dict(phase='PRECHECK', on_sent=False, off_sent=False, on_at_ns=None, off_at_ns=None,
        writer_pids=(), logfile=None, ftp_replies=[])
    last_attitude_rx = None

    def elapsed(): return (time.monotonic_ns() - start) / 1e9
    def safe():
        return bool(not ev.fatal and ev.identity == BASELINE['identity'] and ev.heartbeats
                    and not ev.heartbeats[-1]['armed'] and 0 <= elapsed() - ev.heartbeats[-1]['elapsed_s'] < 2)
    def send(msg):
        if not capture_allowed(msg, safe=safe(), phase=state['phase'], on_sent=state['on_sent'],
                off_sent=state['off_sent'], writer_pids=state['writer_pids'], logfile=state['logfile']):
            raise RuntimeError('Exact static capture TX policy rejected ' + msg.get_type())
        if msg.get_type() == 'SERIAL_CONTROL':
            command = bytes(msg.data[:msg.count])
            if command == b'logger on\n':
                now = time.monotonic_ns()
                if not report['precheck_verified'] or not report['state_proofs'] or not (
                        0 <= now-report['state_proofs'][-1]['at_ns'] < 500_000_000) or last_attitude_rx is None or not (
                        0 <= now-last_attitude_rx < 500_000_000):
                    raise RuntimeError('Fresh precheck/unarmed/attitude proof required for logging start')
                state['on_sent'] = True; state['on_at_ns'] = now
            elif command == b'logger off\n':
                state['off_sent'] = True; state['off_at_ns'] = time.monotonic_ns()
        packet = msg.pack(enc)
        row = dict(at_ns=time.monotonic_ns(), message_type=msg.get_type(), wire_bytes=len(packet))
        if msg.get_type() == 'PARAM_REQUEST_READ': row['name'] = pname(msg)
        if msg.get_type() == 'COMMAND_LONG': row['command'] = int(msg.command); row['param1'] = msg.param1
        if msg.get_type() == 'SERIAL_CONTROL': row['shell_payload'] = bytes(msg.data[:msg.count]).decode('ascii')
        if msg.get_type() == 'FILE_TRANSFER_PROTOCOL':
            from fault_archive_service import HEADER
            seq, session, op, size, _, _, _, offset = HEADER.unpack_from(bytes(msg.payload))
            row.update(ftp_seq=seq, ftp_session=session, ftp_opcode=op, ftp_size=size, ftp_offset=offset)
        report['tx_audit'].append(row)
        if port.write(packet) != len(packet): raise RuntimeError('Short serial write; stop without retry')
        enc.seq = (enc.seq + 1) % 256; tx[msg.get_type()] += 1
    def pump():
        nonlocal next_hb, last_boot, buffer, last_attitude_rx
        if elapsed() > 360: raise RuntimeError('360s bounded acquisition/export deadline')
        if not USB.exists() or USB.resolve() != target: raise RuntimeError('USB disappeared/reassigned')
        raw = port.read(max(1, min(port.in_waiting, 8192)))
        at = time.monotonic_ns()
        if sync.first_valid_ns is None and raw:
            report['rx_startup_raw_chunks'].append(dict(at_ns=at, bytes_hex=raw.hex()))
        for msg in dec.parse_buffer(raw) or []:
            kind = msg.get_type(); rx[kind] += 1
            if kind == 'BAD_DATA':
                report['rx_bad_data'].append(dict(at_ns=at, reason=msg.reason, bytes_hex=bytes(msg.data).hex()))
            skipped = sync.may_skip(msg, at)
            if kind == 'BAD_DATA':
                if skipped: continue
                raise RuntimeError('Invalid frame beyond bounded startup prefix sync; no retry')
            ev.ingest(msg, (at-start)/1e9)
            if ev.fatal: raise RuntimeError(ev.fatal)
            if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1): continue
            if ev.identity and ev.identity != BASELINE['identity']: raise RuntimeError('FC identity mismatch')
            if kind == 'SYS_STATUS' and 2000 < msg.voltage_battery < 65535:
                raise RuntimeError('Unexpected battery voltage')
            if kind == 'ATTITUDE_QUATERNION':
                boot = int(msg.time_boot_ms)
                if last_boot is not None and boot < last_boot: raise RuntimeError('Unexpected reboot during readback')
                last_boot = boot; last_attitude_rx = at
                report['attitudes'].append(dict(at_ns=at, time_boot_ms=boot))
            if kind == 'PARAM_VALUE':
                name = pname(msg)
                if name in ev.requests:
                    values[name] = dict(value=decode_parameter(msg), type=int(msg.param_type),
                        received_ns=at, system=msg.get_srcSystem(), component=msg.get_srcComponent(),
                        wire_value_hex=struct.pack('<f', msg.param_value).hex())
            if kind == 'FILE_TRANSFER_PROTOCOL':
                state['ftp_replies'].append(msg)
                if len(state['ftp_replies']) > 4096: raise RuntimeError('Bounded FTP reply queue overflow')
            if kind == 'SERIAL_CONTROL':
                buffer += bytes(msg.data[:msg.count]).decode('utf8', 'replace')
                if len(buffer) > 65536: raise RuntimeError('Shell reply overflow')
            if kind == 'STATUSTEXT': report['status_texts'].append(dict(at_ns=at, severity=int(msg.severity), text=str(msg.text)))
        sync.check_deadline(at)
        if state['on_sent'] and not state['off_sent'] and (last_attitude_rx is None or at-last_attitude_rx > 500_000_000):
            raise RuntimeError('Attitude exceeded unchanged 500ms during static recording')
        if sync.first_valid_ns is not None and at >= next_hb:
            send(enc.heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0)); next_hb = at + 1_000_000_000
    def read(name):
        if not safe(): raise RuntimeError('Fresh unarmed identity required for parameter read')
        values.pop(name, None); ev.requests[name] = elapsed(); sent = time.monotonic_ns()
        send(enc.param_request_read_encode(1, 1, name.encode(), -1)); deadline = time.monotonic() + 2
        while name not in values or values[name]['received_ns'] < sent:
            pump()
            if not safe() or time.monotonic() > deadline: raise RuntimeError('Read failed; no retry: ' + name)
        row = dict(values[name]); reference = BASELINE['after_save'][name]
        row['expected_value'] = reference['value']; row['expected_type'] = reference['type']
        row['matches_baseline'] = (row['type'], row['wire_value_hex']) == (reference['type'], reference['wire_value_hex'])
        report['parameters'][name] = row
        if not row['matches_baseline']:
            report['mismatches'].append(name); raise RuntimeError('Parameter differs from saved maintenance baseline: ' + name)
        return row
    def shell(command, *, deadline_ns=None):
        nonlocal buffer
        if not safe(): raise RuntimeError('Fresh unarmed identity required for status query')
        buffer = ''; row = dict(command=command.decode().strip(), started_ns=time.monotonic_ns(), complete=False)
        report['shell_records'].append(row)
        send(enc.serial_control_encode(10, 2, 0, 0, len(command), command.ljust(70, b'\0')))
        deadline = min(time.monotonic_ns()+10_000_000_000,
                       deadline_ns if deadline_ns is not None else time.monotonic_ns()+10_000_000_000)
        poll = time.monotonic() + .2
        while time.monotonic_ns() < deadline:
            pump(); row['raw_reply'] = buffer
            if not safe(): raise RuntimeError('Unarmed identity freshness lost')
            if reply_complete(buffer, command):
                row.update(complete=True, completed_ns=time.monotonic_ns())
                if command == COMMANDS[2]:
                    snapshot = parse_actuator_snapshot(buffer)
                    if snapshot is None or snapshot['armed']: raise RuntimeError('Unarmed state unknown or armed')
                    report['state_proofs'].append(dict(at_ns=row['completed_ns'], **snapshot))
                print('READONLY SHELL ' + row['command'] + '\n' + buffer, flush=True)
                return buffer
            if time.monotonic() >= poll:
                send(enc.serial_control_encode(10, 2, 0, 0, 0, b'\0'*70)); poll = time.monotonic() + .2
        raise RuntimeError('Incomplete status query; no retry')
    io = SimpleNamespace(enc=enc, send=send, pump=pump, shell=shell, read=read, safe=safe)
    io.reader = FileReader(io, state, report)
    try:
        port = serial.Serial(str(USB), 115200, timeout=.02, write_timeout=.5, exclusive=True)
        until = time.monotonic() + 2
        while time.monotonic() < until: pump()
        if not ev.heartbeats: raise RuntimeError('No PX4 heartbeat')
        ev.version_requested_at = elapsed()
        send(enc.command_long_encode(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0))
        until = time.monotonic() + 4
        while not safe():
            pump()
            if time.monotonic() > until: raise RuntimeError('Identity timeout')
        report['identity'] = dict(ev.identity); report['system_id'] = 1; report['component_id'] = 1
        print('IDENTITY ' + json.dumps(ev.identity), flush=True)
        report['stage'] = 'CAPTURE'
        perform(io, state, report, run)
        report['stage'] = 'COMPLETE'
    except BaseException as exc:
        report['error'] = type(exc).__name__ + ': ' + str(exc)
        print('STOP ' + report['error'], flush=True)
        abort_capture(io, state, report)
        if report.get('keep_USB_power_required'):
            print('File close unconfirmed: maintain USB power; no automatic retry', flush=True)
    finally:
        finalize_resources(port, io.reader, report)
        if not report['serial_closed'] or report.get('ftp_close_error') or report.get('serial_close_error'):
            report['error'] = report['error'] or 'Resource cleanup failed'
        report.update(ended_monotonic_ns=time.monotonic_ns(),
            utc_end=datetime.now(timezone.utc).isoformat(), tx_counts=dict(tx), rx_counts=dict(rx),
            parser=dec.summary(), startup_prefix_sync=sync.summary(), heartbeats=ev.heartbeats,
            logger_on_off_authorized=True, logger_on_sent=int(state['on_sent']), logger_off_sent=int(state['off_sent']),
            on_at_ns=state['on_at_ns'], off_at_ns=state['off_at_ns'], parameter_writes_sent=0,
            capture_phase=state['phase'],
            prohibited_tx_count=sum(v for k, v in tx.items() if k not in ('HEARTBEAT', 'COMMAND_LONG', 'PARAM_REQUEST_READ', 'SERIAL_CONTROL', 'FILE_TRANSFER_PROTOCOL')))
        destination = run / 'report.json'
        with destination.open('x') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False); stream.write('\n')
        print(json.dumps(dict(report_path=str(destination), sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),
            error=report['error'], precheck_verified=report['precheck_verified'], capture_completed=report['capture_completed'],
            file_closed_verified=report['file_closed_verified'], ulog_path=report.get('ulog_path'),
            ulog_size=report.get('ulog_size'), parameters_read=len(report['parameters']),
            mismatches=report['mismatches'], identity=report.get('identity'), sd_mounted=report.get('sd_mounted'),
            logger_not_logging=report.get('logger_not_logging'), tx_counts=dict(tx), serial_closed=report['serial_closed']),
            ensure_ascii=False, indent=2), flush=True)
    return 0 if report['capture_completed'] and report['file_closed_verified'] and state['phase'] == 'COMPLETE' and not report['error'] else 2

if __name__ == '__main__':
    if sys.argv[1:] == ['--selftest']: selftest()
    elif (len(sys.argv) == 4 and sys.argv[1] == '--authorized-one-static-capture-and-readonly-export'
          and sys.argv[2] == '--props-removed-battery-disconnected-usb-only-confirmed'
          and sys.argv[3].startswith('--authorization-id=')):
        raise SystemExit(main(sys.argv[3].split('=', 1)[1], physical_confirmed=True))
    else: raise SystemExit('Explicit scoped option required; no serial opened')
