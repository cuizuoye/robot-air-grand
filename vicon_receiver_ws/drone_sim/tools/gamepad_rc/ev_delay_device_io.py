"""Real nonblocking serial adapters for the joint maintenance collector.

Construction requires an already exclusively opened port. No device discovery,
reconnect, parameter writes, mode/arm API or automatic capture on import.
Logger protocol/policy/file ownership are reused from static_ulog_capture_*.
"""
import hashlib
import json
from pathlib import Path
import struct
import time
from collections import Counter, deque

from pymavlink.dialects.v20 import common as m
from mavlink_crc_stream import CrcResyncParser
from mavlink_readonly_probe import Evidence, decode_parameter
from static_ulog_capture import StartupPrefixSync, pname
from static_ulog_capture_actions import FileReader
from ev_delay_maintenance_host import maintenance_usb_allowed, SELECTION_QUERY, VISUAL_QUERY
from persistent_shell import reply_complete
from estop_only_policy import parse_actuator_snapshot


class Endpoint:
    def __init__(self, port, component, expected_identity, *, clock=None,
                 path_check=lambda: None, battery_static_observer=False,
                 joint_rx_evidence=None):
        # Explicit powered observer reuse; the caller must enforce its own
        # disarmed/output-lock TX whitelist. Existing USB collectors stay strict.
        if type(battery_static_observer) is not bool:
            raise ValueError('Explicit boolean battery observer scope required')
        self.battery_static_observer = battery_static_observer
        self.observer_rx_errors = deque(maxlen=64)
        self.port, self.clock, self.path_check = port, clock or time.monotonic_ns, path_check
        self.start_ns = self.clock()
        self.enc = m.MAVLink(None, srcSystem=250, srcComponent=component)
        self.parser = CrcResyncParser()
        self.sync = StartupPrefixSync(self.start_ns)
        self.evidence = Evidence(source_component=component, expected_parameters=())
        self.expected = expected_identity
        self.identity_ns = self.heartbeat_ns = self.attitude_ns = None
        self.attitude = None
        self.attitudes = deque(maxlen=32)
        self.values = {}
        self.version_wire = None
        self.rx = Counter()
        self.tx = []
        self.fault = None
        self.next_hb = 0
        self.hook = lambda msg, at: None
        self.peer_hook = lambda msg, at: None
        self.writer = self.write
        self.fw_custom = None
        self.incomplete_version = None
        self.v2_fc_received_ns = None
        self.forwarded_peer_frames = Counter()
        # Explicit joint-capture scope only; other callers retain their policy.
        # Save every read BEFORE parsing, including incomplete/split candidates.
        self.joint_rx_path = None if joint_rx_evidence is None else Path(joint_rx_evidence)
        self.joint_rx_file = None if self.joint_rx_path is None else self.joint_rx_path.open('x')
        self.joint_rx_bytes = self.joint_rx_chunks = self.joint_rx_error_count = 0
        self.joint_rx_errors = deque(maxlen=64)
        self.last_valid_fc_ns = None

    def _rx_record(self, row):
        if self.joint_rx_file is not None:
            self.joint_rx_file.write(json.dumps(row, separators=(',', ':'))+'\n')
            self.joint_rx_file.flush()  # Evidence failure aborts; never continue silently.

    def close_rx_evidence(self):
        if self.joint_rx_file is not None:
            self.joint_rx_file.close()

    def safe(self):
        now = self.clock()
        return bool(not self.fault and not self.evidence.fatal and self.port.is_open
            and self.evidence.identity == self.expected and self.heartbeat_ns is not None
            and 0 <= now-self.heartbeat_ns < 2_000_000_000
            and (self.joint_rx_path is None or self.last_valid_fc_ns is not None
                 and 0 <= now-self.last_valid_fc_ns < 500_000_000)
            and not self.evidence.heartbeats[-1]['armed'])

    def write(self, msg):
        packet = bytes(msg.pack(self.enc))
        if self.port.write(packet) != len(packet):
            self.fault = 'short_write_delivery_uncertain'
            raise OSError(self.fault)
        self.enc.seq = (self.enc.seq+1) % 256
        self.tx.append(dict(at_ns=self.clock(), type=msg.get_type(), wire_hex=packet.hex()))

    def version(self):
        self.evidence.version_requested_at = (self.clock()-self.start_ns)/1e9
        self.writer(self.enc.command_long_encode(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0))

    def poll(self):
        try:
            self.path_check()
            raw = self.port.read(min(self.port.in_waiting, 8192))
            at = self.clock()
            if raw and self.joint_rx_file is not None:
                self._rx_record(dict(kind='serial_read', at_ns=at, offset=self.joint_rx_bytes,
                                     wire_hex=raw.hex()))
                self.joint_rx_chunks += 1
                self.joint_rx_bytes += len(raw)
                if self.joint_rx_bytes > 64*1024*1024:
                    raise RuntimeError('Joint RX evidence byte budget exceeded')
            for msg in self.parser.parse_buffer(raw) or []:
                kind = msg.get_type()
                self.rx[kind] += 1
                if kind == 'BAD_DATA' and self.joint_rx_file is not None:
                    row = dict(kind='rejected_candidate', at_ns=at, reason=str(msg.reason),
                               wire_hex=bytes(msg.data).hex())
                    self._rx_record(row)
                    self.joint_rx_errors.append(row)
                    self.joint_rx_error_count += 1
                if kind.startswith('UNKNOWN_') and self.sync.pending_orphan is None:
                    # CRC/schema were validated, but pymavlink's unknown-message
                    # wrapper has synthetic 0/0 sender fields. No state proof here;
                    # wait for a decoded FC frame within the original deadline.
                    continue
                if (kind != 'BAD_DATA' and self.sync.pending_orphan is None
                        and (msg.get_srcSystem(), msg.get_srcComponent()) in ((250, 219), (250, 213))):
                    # PX4 forwards the other owned endpoint's GCS frames.
                    # CRC was checked above; these are never FC sync/state proof.
                    self.forwarded_peer_frames[str(msg.get_srcComponent())+':'+kind] += 1
                    if self.joint_rx_file is not None:
                        self.peer_hook(msg, at)  # Receipt only; never identity/lock freshness.
                    continue
                skip = self.sync.may_skip(msg, at,
                    discard_corrupt_prefix=self.joint_rx_file is not None)
                if kind == 'BAD_DATA':
                    if skip: continue
                    if self.joint_rx_file is not None and self.sync.first_valid_ns is not None:
                        # Resynchronize using the same strict CRC parser. Rejected
                        # bytes NEVER refresh state, clock, ACKs, or callbacks.
                        # Continuation depends on fresh valid FC input and the
                        # unchanged capture/lock/attitude/window guards below it.
                        continue
                    if self.battery_static_observer:
                        # Static diagnostics reuse the strict parser's existing
                        # resynchronization. Damaged bytes never reach Evidence,
                        # hooks, identity, heartbeat or attitude freshness.
                        self.observer_rx_errors.append(dict(at_ns=at,
                            reason=str(msg.reason), wire_hex=bytes(msg.data).hex()))
                        recent = sum(at-row['at_ns'] < 1_000_000_000
                                     for row in self.observer_rx_errors)
                        if recent > 4:
                            raise RuntimeError('Static observer RX error burst; stopped')
                        continue
                    raise RuntimeError('CRC/framing error; no automatic retry')
                if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
                    # Ground peers may share a telemetry link; never use their proofs.
                    continue
                self.last_valid_fc_ns = at
                if bytes(msg.get_msgbuf())[:1] == b'\xfd':
                    self.v2_fc_received_ns = at
                if (kind == 'AUTOPILOT_VERSION' and self.identity_ns is None
                        and bytes(msg.get_msgbuf())[:1] == b'\xfe'):
                    # MAVLink 1 has no uid2 extension. A queued startup broadcast
                    # cannot prove the full identity, nor is an absent uid2 a new FC.
                    v = msg.flight_sw_version
                    core = dict(firmware=f'{v >> 24}.{(v >> 16) & 255}.{(v >> 8) & 255}',
                                board_version=msg.board_version, uid=msg.uid)
                    if any(core[k] != self.expected[k] for k in core):
                        raise RuntimeError('FC identity changed')
                    self.incomplete_version = dict(at_ns=at, wire_hex=bytes(msg.get_msgbuf()).hex(),
                        reason='MAVLink1 lacks uid2; awaiting complete MAVLink2 identity')
                    continue  # No safe(), identity timestamp or version proof granted.
                self.evidence.ingest(msg, (at-self.start_ns)/1e9)
                if self.evidence.fatal: raise RuntimeError(self.evidence.fatal)
                if kind == 'AUTOPILOT_VERSION' and self.evidence.version_requested_at is not None:
                    if self.evidence.identity != self.expected: raise RuntimeError('FC identity changed')
                    custom = bytes(msg.flight_custom_version).hex()
                    if self.fw_custom is not None and self.fw_custom != custom:
                        raise RuntimeError('FC firmware changed')
                    self.fw_custom = custom
                    self.identity_ns = at
                    self.version_wire = bytes(msg.get_msgbuf()).hex()
                if kind == 'HEARTBEAT': self.heartbeat_ns = at
                if (kind == 'SYS_STATUS' and 2000 < msg.voltage_battery < 65535
                        and not self.battery_static_observer):
                    raise RuntimeError('Propulsion battery voltage detected; USB-only scope')
                if kind == 'ATTITUDE_QUATERNION':
                    if self.attitude is not None and msg.time_boot_ms < self.attitude.time_boot_ms:
                        raise RuntimeError('FC boot clock regressed')
                    self.attitude, self.attitude_ns = msg, at
                    self.attitudes.append((msg, at))
                if kind == 'PARAM_VALUE' and pname(msg) in self.evidence.requests:
                    self.values[pname(msg)] = dict(value=decode_parameter(msg), type=int(msg.param_type),
                        wire_value_hex=struct.pack('<f', msg.param_value).hex(), received_ns=at)
                self.hook(msg, at)
            self.sync.check_deadline(at)
            if (self.joint_rx_path is not None and self.last_valid_fc_ns is not None
                    and at-self.last_valid_fc_ns >= 500_000_000):
                raise RuntimeError('Joint RX valid FC stream stale (500ms); capture stopped')
        except Exception as exc:
            self.fault = self.fault or str(exc)
            raise

    def heartbeat(self):
        now = self.clock()
        if now >= self.next_hb:
            self.writer(self.enc.heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0))
            self.next_hb = self.clock()+1_000_000_000

    def read(self, name, pump):
        if not self.safe(): raise RuntimeError('Fresh unarmed identity required')
        self.values.pop(name, None)
        sent = self.clock()
        self.evidence.requests[name] = (sent-self.start_ns)/1e9
        self.writer(self.enc.param_request_read_encode(1, 1, name.encode(), -1))
        while name not in self.values or self.values[name]['received_ns'] < sent:
            pump()
            if not self.safe() or self.clock()-sent >= 2_000_000_000:
                raise RuntimeError('Parameter read failed without retry: '+name)
        return dict(self.values[name])

    def report(self):
        return dict(identity=self.evidence.identity, identity_received_ns=self.identity_ns,
            identity_wire=self.version_wire, firmware_custom_hex=self.fw_custom,
            attitude_received_ns=self.attitude_ns,
            last_boot_ms=None if self.attitude is None else int(self.attitude.time_boot_ms),
            parser=self.parser.summary(), startup={**self.sync.summary(),
                'bounded_corrupt_prefix_discard': self.joint_rx_path is not None,
                'post_sync_bad_data_tolerated': self.joint_rx_path is not None or self.battery_static_observer}, rx=dict(self.rx),
            tx=self.tx, fault=self.fault, incomplete_version=self.incomplete_version,
            battery_static_observer=self.battery_static_observer,
            observer_rx_errors=list(self.observer_rx_errors),
            joint_rx_evidence=dict(path=None if self.joint_rx_path is None else str(self.joint_rx_path),
                bytes=self.joint_rx_bytes, chunks=self.joint_rx_chunks,
                rejected_candidates=self.joint_rx_error_count,
                recent_errors=list(self.joint_rx_errors), last_valid_fc_ns=self.last_valid_fc_ns,
                crc_bypass=False, post_sync_discard_enabled=self.joint_rx_path is not None),
            v2_fc_received_ns=self.v2_fc_received_ns,
            forwarded_peer_frames=dict(self.forwarded_peer_frames))


class LoggerIO:
    def __init__(self, endpoint, baseline, pump, *, clock_bounds, boot_session):
        self.endpoint, self.port, self.enc = endpoint, endpoint.port, endpoint.enc
        self.pump, self.clock_ns = pump, endpoint.clock
        self.clock_bounds, self.boot_session = clock_bounds, boot_session
        self.baseline = baseline
        self.buffer = ''
        self.state = dict(phase='PRECHECK', on_sent=False, off_sent=False, on_at_ns=None,
            off_at_ns=None, writer_pids=(), logfile=None, ftp_replies=[])
        self.report = dict(identity=baseline['identity'], parameters={}, mismatches=[], shell_records=[],
            state_proofs=[], tx_audit=[], precheck_verified=False, serial_closed=False,
            file_closed_verified=False, capture_completed=False)
        self.reader = FileReader(self, self.state, self.report)
        endpoint.writer, endpoint.hook = self.send, self.receive

    def safe(self): return self.endpoint.safe()

    def receive(self, msg, at):
        if msg.get_type() == 'SERIAL_CONTROL':
            if msg.device != 10: raise RuntimeError('Unexpected shell device')
            self.buffer += bytes(msg.data[:msg.count]).decode('utf8', 'replace')
            if len(self.buffer) > 65536: raise RuntimeError('Shell reply overflow')
        if msg.get_type() == 'FILE_TRANSFER_PROTOCOL':
            self.state['ftp_replies'].append(msg)
            if len(self.state['ftp_replies']) > 4096: raise RuntimeError('FTP reply overflow')

    def send(self, msg):
        s, r = self.state, self.report
        if not maintenance_usb_allowed(msg, safe=self.safe(), phase=s['phase'],
                on_sent=s['on_sent'], off_sent=s['off_sent'], writer_pids=s['writer_pids'],
                logfile=s['logfile']):
            raise PermissionError('USB maintenance policy rejected '+msg.get_type())
        if msg.get_type() == 'SERIAL_CONTROL':
            command = bytes(msg.data[:msg.count])
            now = self.clock_ns()
            if command == b'logger on\n':
                if (not r['precheck_verified'] or not r['state_proofs']
                        or not 0 <= now-r['state_proofs'][-1]['at_ns'] < 500_000_000
                        or self.endpoint.attitude_ns is None
                        or not 0 <= now-self.endpoint.attitude_ns < 500_000_000):
                    raise RuntimeError('Fresh native unarmed/attitude proof required for Logger')
                s.update(on_sent=True, on_at_ns=now)  # Before possibly partial delivery.
            elif command == b'logger off\n': s.update(off_sent=True, off_at_ns=now)
        self.endpoint.write(msg)
        r['tx_audit'].append(self.endpoint.tx[-1])

    def read(self, name):
        row = self.endpoint.read(name, self.pump)
        expected = self.baseline['after_save'][name]
        row['matches_baseline'] = (row['type'], row['wire_value_hex']) == (expected['type'], expected['wire_value_hex'])
        self.report['parameters'][name] = row
        if not row['matches_baseline']: raise RuntimeError('Saved Logger parameter differs: '+name)
        return row

    def shell(self, command, *, deadline_ns=None):
        if not self.safe(): raise RuntimeError('Unarmed identity lost before shell request')
        self.buffer = ''
        start = self.clock_ns()
        row = dict(command=command.decode().strip(), started_ns=start, complete=False)
        self.report['shell_records'].append(row)
        self.send(self.enc.serial_control_encode(10, 2, 0, 0, len(command), command.ljust(70, b'\0')))
        deadline = min(start+10_000_000_000, deadline_ns or start+10_000_000_000)
        poll = start+200_000_000
        while self.clock_ns() < deadline:
            self.pump()
            row['raw_reply'] = self.buffer
            if not self.safe(): raise RuntimeError('Unarmed identity lost during shell reply')
            if reply_complete(self.buffer, command):
                row.update(complete=True, completed_ns=self.clock_ns())
                if command == b'listener actuator_armed -n 1\n':
                    snapshot = parse_actuator_snapshot(self.buffer)
                    if snapshot is None or snapshot['armed']: raise RuntimeError('Native unarmed proof missing')
                    self.report['state_proofs'].append(dict(at_ns=row['completed_ns'], **snapshot))
                return self.buffer
            if self.clock_ns() >= poll:
                self.send(self.enc.serial_control_encode(10, 2, 0, 0, 0, b'\0'*70))
                poll = self.clock_ns()+200_000_000
        raise RuntimeError('Incomplete shell reply; no retry')

    def read_sensor_selection(self):
        before = self.clock_bounds(self.clock_ns())
        raw = self.shell(SELECTION_QUERY)
        at = self.clock_ns()
        after = self.clock_bounds(at)
        return dict(route='usb_readonly_serial_control', system_id=1, component_id=1,
            command=SELECTION_QUERY.decode().strip(), crc_validated=True,
            raw_reply=raw, reply_sha256=hashlib.sha256(raw.encode()).hexdigest(),
            host_received_ns=at, read_fc_lower_us=before[0], read_fc_upper_us=after[1],
            fc_boot_session=self.boot_session, review_status='raw_unreviewed')

    def read_visual(self):
        from usb_vision_policy import parse_visual_snapshot
        raw = self.shell(VISUAL_QUERY)
        row = parse_visual_snapshot(raw)
        if not row: raise RuntimeError('Incomplete native visual snapshot')
        bounds = self.clock_bounds(self.clock_ns())
        if not 0 <= bounds[1]-row['timestamp'] < 500_000:
            raise RuntimeError('Native visual snapshot stale/future')
        return row
