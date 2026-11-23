"""Bounded one-shot static logging and READ-ONLY export; no serial owner here."""
from pathlib import Path
import hashlib
import math
import re
import statistics
import struct
import time

from fault_archive_service import HEADER, payload, nuttx_crc, Nak
from static_ulog_capture_policy import (parse_writer_pids, parse_fd_table, valid_log_path,
    writer_open_descriptors, writer_returned_to_baseline)

START_TIMEOUT_NS = 5_000_000_000
STOP_TIMEOUT_NS = 15_000_000_000
VERIFY_TIMEOUT_NS = 5_000_000_000
REQUIRED_IMU_SPAN_NS = 10_000_000_000
HOST_RECORDING_SPAN_NS = 12_000_000_000
POLL_NS = 100_000_000

class FileReader:
    def __init__(self, io, state, report):
        self.io, self.state, self.report = io, state, report
        self.seq = 21000
        self.active = False

    def request(self, op, data=b'', *, offset=0, size=None):
        seq = self.seq; self.seq = (self.seq+2) % 65536
        self.state['ftp_replies'].clear()
        sent = time.monotonic_ns()
        self.io.send(self.io.enc.file_transfer_protocol_encode(0, 1, 1,
            payload(seq, op, data, offset=offset, size=size)))
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            self.io.pump()
            while self.state['ftp_replies']:
                reply = self.state['ftp_replies'].pop(0)
                if (reply.target_system, reply.target_component) != (250, 219): continue
                raw = bytes(reply.payload)
                rseq, session, opcode, length, request_op, _, _, roff = HEADER.unpack_from(raw)
                if rseq != (seq+1) % 65536 or request_op != op: continue
                if opcode not in (128, 129) or session != 0 or length > 239 or roff != offset:
                    raise RuntimeError('Malformed correlated file-read reply')
                body = raw[12:12+length]
                self.report.setdefault('ftp_transactions', []).append(dict(
                    seq=seq, opcode=op, offset=offset, request_size=size if size is not None else len(data),
                    reply_opcode=opcode, reply_size=length, sent_ns=sent, received_ns=time.monotonic_ns()))
                if opcode == 129: raise Nak(body)
                return body
            if not self.io.safe(): raise RuntimeError('Fresh unarmed identity lost during file read')
        raise RuntimeError('Read-only file request timed out; no retry')

    def open(self, path, *, allow_empty=False):
        body = self.request(4, path.encode())
        self.active = True
        if len(body) != 4: raise RuntimeError('Invalid read-only file size reply')
        size, = struct.unpack('<I', body)
        if size > 16*1024*1024 or (size == 0 and not allow_empty):
            raise RuntimeError('Unexpected closed/export ULog size')
        return size

    def close(self):
        if self.active:
            self.active = False  # single close attempt only
            self.request(1)

    def size(self, path):
        # A newly created FAT file can legitimately report size zero. Metadata
        # is not a data-integrity verdict. Download still requires nonempty.
        try: return self.open(path, allow_empty=True)
        finally: self.close()

    def crc(self, path):
        body = self.request(14, path.encode())
        if len(body) != 4: raise RuntimeError('Invalid remote CRC reply')
        return struct.unpack('<I', body)[0]

    def download(self, path, expected_size):
        crc_before = self.crc(path)
        data = bytearray()
        try:
            if self.open(path) != expected_size: raise RuntimeError('File changed after writer closed')
            next_progress = 65536
            while len(data) < expected_size:
                block = self.request(5, offset=len(data), size=min(239, expected_size-len(data)))
                if not block or len(block) > expected_size-len(data): raise RuntimeError('Invalid file block length')
                data.extend(block)
                if len(data) >= next_progress:
                    print(f'EXPORT {len(data)}/{expected_size} bytes', flush=True); next_progress += 65536
        finally: self.close()
        crc_after = self.crc(path)
        if crc_before != crc_after or crc_after != nuttx_crc(data):
            raise RuntimeError('Export remote/local CRC mismatch; preserve bytes, do not repeat experiment')
        return bytes(data), dict(before=crc_before, after=crc_after, local=nuttx_crc(data), matches=True)

def now_ns(io):
    return getattr(io, 'clock_ns', time.monotonic_ns)()


def transition(io, state, report, phase):
    state['phase'] = phase
    report.setdefault('state_transitions', []).append(dict(phase=phase, at_ns=now_ns(io)))


def wait_until(io, deadline_ns):
    # Continue receiving/validating telemetry. No fixed blocking sleep.
    while now_ns(io) < deadline_ns:
        io.pump()


def query(io, command, deadline_ns):
    if now_ns(io) >= deadline_ns:
        raise RuntimeError('Phase deadline expired before status query')
    answer = io.shell(command, deadline_ns=deadline_ns)
    if now_ns(io) > deadline_ns:
        raise RuntimeError('Phase deadline exceeded while receiving status')
    return answer


def writer_tables(io, state, deadline_ns):
    raw = {pid: query(io, f'cat /proc/{pid}/group/fd\n'.encode(), deadline_ns)
           for pid in state['writer_pids']}
    return raw, {pid: parse_fd_table(text) for pid, text in raw.items()}


def stopped_writer(current, state):
    baseline = state['writer_fd_baseline']
    return all(writer_returned_to_baseline(table, baseline[pid])
               for pid, table in current.items())


def stop_and_finalize(io, state, report, reader):
    if not state['on_sent']:
        return
    if state.get('stop_attempted'):
        raise RuntimeError('Stop/finalize already attempted; no automatic repeat')
    state['stop_attempted'] = True
    transition(io, state, report, 'STOPPING')
    deadline = now_ns(io)+STOP_TIMEOUT_NS
    if not state['off_sent']:
        query(io, b'logger off\n', deadline)
    while now_ns(io) < deadline:
        status = query(io, b'logger status\n', deadline)
        raw, current = writer_tables(io, state, deadline)
        closed = 'Not logging' in status and stopped_writer(current, state)
        report.setdefault('close_fd_checks', []).append(dict(
            at_ns=now_ns(io), raw=raw, writer_at_baseline=stopped_writer(current, state),
            not_logging='Not logging' in status))
        if closed:
            report.update(file_closed_verified=True, close_verified_ns=now_ns(io),
                          logger_not_logging=True, final_writer_fd_tables=raw)
            break
        wait_until(io, min(deadline, now_ns(io)+POLL_NS))
    else:
        raise RuntimeError('Logger stop/file-close timeout; keep USB power, do not retry')

    transition(io, state, report, 'VERIFYING')
    if state['logfile'] is None:
        # Startup failure before any identifiable logfile. Output remains
        # stopped, but no capture success/export can be claimed.
        report['no_identified_logfile'] = True
        return
    deadline = now_ns(io)+VERIFY_TIMEOUT_NS
    previous = None
    while now_ns(io) < deadline:
        size = reader.size(state['logfile'])  # zero is legal metadata, not damage
        raw, current = writer_tables(io, state, deadline)
        report.setdefault('file_size_checks', []).append(dict(
            at_ns=now_ns(io), size=size, writer_at_baseline=stopped_writer(current, state)))
        if not stopped_writer(current, state):
            raise RuntimeError('Writer reopened a file during verification; keep USB power')
        if size > 0 and size == previous:
            if parse_writer_pids(query(io, b'ps\n', deadline)) != state['writer_pids']:
                raise RuntimeError('Writer process changed during capture')
            report.update(closed_file_size=size, file_size_stable=True,
                          close_evidence='Logger Not logging + complete writer FD set restored to pre-start baseline; nonempty size stable across probes')
            return
        previous = size
        wait_until(io, min(deadline, now_ns(io)+POLL_NS))
    reason = 'Closed logfile stayed empty' if previous == 0 else 'Closed logfile size did not stabilize'
    raise RuntimeError(reason+' within verification deadline; preserve file, no automatic repeat')


def scan_records(binary):
    if len(binary) < 16 or binary[:7] != b'ULog\x01\x12\x35':
        return dict(ok=False, reason='invalid_ULog_header', parsed_bytes=0)
    offset = 16
    while offset < len(binary):
        if len(binary)-offset < 3:
            return dict(ok=False, reason='truncated_record_header', parsed_bytes=offset)
        size, = struct.unpack_from('<H', binary, offset)
        if offset+3+size > len(binary):
            return dict(ok=False, reason='truncated_record_payload', parsed_bytes=offset)
        offset += 3+size
    return dict(ok=True, reason=None, parsed_bytes=offset)


def verify_static_ulog(path, *, required_span_ns=REQUIRED_IMU_SPAN_NS,
                       expected_uid=None, parser=None):
    """Exact native IMU-time acceptance. No EV_DELAY estimate/flight permit."""
    if required_span_ns != REQUIRED_IMU_SPAN_NS:
        raise ValueError('Static acceptance requires the full 10s IMU interval')
    from ev_delay_sources import load_ulog_file
    path = Path(path)
    raw = path.read_bytes()
    result = dict(accepted=False, flight_authorization=False,
                  requested_imu_span_ns=required_span_ns,
                  size=len(raw), sha256=hashlib.sha256(raw).hexdigest(),
                  framing=scan_records(raw), reasons=[], topics={})
    if not result['framing']['ok']:
        result['reasons'].append(result['framing']['reason'])
        return result
    try:
        ulog = (parser or load_ulog_file)(path)
        result['file_corruption'] = bool(ulog.file_corruption)
        result['ulog_dropout_count'] = len(ulog.dropouts)
        if ulog.file_corruption: result['reasons'].append('ULog_parser_reported_corruption')
        if ulog.dropouts: result['reasons'].append('ULog_dropouts')
        if expected_uid is not None and ulog.msg_info_dict.get('sys_uuid') != expected_uid:
            result['reasons'].append('ULog_UID_mismatch')
        for name in ('sensor_combined','sensor_selection','vehicle_imu','vehicle_imu_status',
                     'actuator_armed','vehicle_status','logger_status'):
            result['topics'][name] = []
            for ds in (d for d in ulog.data_list if d.name == name):
                stamps = [int(t) for t in ds.data['timestamp']]
                gaps = [b-a for a,b in zip(stamps, stamps[1:])]
                span_us = stamps[-1]-stamps[0] if len(stamps)>1 else 0
                result['topics'][name].append(dict(
                    instance=int(ds.multi_id), count=len(stamps), span_us=span_us,
                    frequency_hz=(len(stamps)-1)*1e6/span_us if span_us>0 else None,
                    mean_gap_us=statistics.mean(gaps) if gaps else None,
                    max_gap_us=max(gaps) if gaps else None,
                    non_increasing_timestamps=sum(g<=0 for g in gaps)))
        primary = [d for d in ulog.data_list if d.name == 'sensor_combined']
        if len(primary) != 1:
            result['reasons'].append('missing_or_ambiguous_sensor_combined')
        else:
            data = primary[0].data
            row = result['topics']['sensor_combined'][0]
            span_ns = row['span_us']*1000
            result['imu_span_ns'] = span_ns
            result['imu_duration_s'] = span_ns/1e9
            if span_ns < required_span_ns: result['reasons'].append('IMU_effective_duration_below_10s')
            if row['frequency_hz'] is None or row['frequency_hz'] < 100:
                result['reasons'].append('IMU_logged_frequency_below_100Hz')
            if row['non_increasing_timestamps'] or row['max_gap_us'] is None or row['max_gap_us'] > 20000:
                result['reasons'].append('IMU_timestamp_gap_or_regression')
            periods = [int(t) for t in data['gyro_integral_dt']]
            result['gyro_integration_us'] = dict(min=min(periods), max=max(periods)) if periods else None
            if not periods or any(t<=0 or t>20000 for t in periods):
                result['reasons'].append('invalid_gyro_integration_interval')
            for field in ('gyro_clipping', 'accelerometer_clipping'):
                if any(int(t) for t in data[field]):
                    result['reasons'].append(field)
            keys = [*(f'gyro_rad[{i}]' for i in range(3)), *(f'accelerometer_m_s2[{i}]' for i in range(3))]
            if any(not math.isfinite(float(v)) for k in keys for v in data[k]):
                result['reasons'].append('nonfinite_IMU')
        armed = [int(v) for d in ulog.data_list if d.name == 'actuator_armed' for v in d.data['armed']]
        status = [int(v) for d in ulog.data_list if d.name == 'vehicle_status' for v in d.data['arming_state']]
        result['all_logged_unarmed'] = bool(armed and status and not any(armed) and 2 not in status)
        if not result['all_logged_unarmed']: result['reasons'].append('logged_unarmed_state_unknown_or_false')
        selections = [int(v) for d in ulog.data_list if d.name == 'sensor_selection' for v in d.data['gyro_device_id']]
        result['selected_gyro_ids'] = sorted(set(selections))
        if len(set(selections)) != 1 or 0 in selections:
            result['reasons'].append('IMU_sensor_switch_or_unknown')
        result['imu_status_counters'] = []
        for d in (d for d in ulog.data_list if d.name == 'vehicle_imu_status'):
            counters = dict(instance=int(d.multi_id), fields={})
            keys = ('gyro_error_count','accel_error_count', *(f'{sensor}_clipping[{axis}]'
                for sensor in ('gyro','accel') for axis in range(3)))
            for key in keys:
                values = [int(v) for v in d.data[key]]
                counters['fields'][key] = dict(first=values[0] if values else None,
                    last=values[-1] if values else None, trend_observable=len(values)>=2)
                if any(a!=b for a,b in zip(values,values[1:])):
                    result['reasons'].append(key+'_changed_during_log')
            result['imu_status_counters'].append(counters)
        result['exact_message_gap_count_known'] = bool(result['topics']['logger_status'])
        result['reasons'] = sorted(set(result['reasons']))
        result['accepted'] = not result['reasons']
    except (Exception,) as exc:
        result['reasons'].append('ULog_analysis_failed: '+type(exc).__name__+': '+str(exc))
    return result


def prepare_logger(io, state, report):
    """Reusable existing precheck; never starts Logger or opens a transport."""
    report['flight_authorized'] = False
    io.shell(b'listener actuator_armed -n 1\n')
    io.read('SDLOG_PROFILE'); io.read('SDLOG_MODE')
    report['parameters_before'] = dict(report['parameters'])
    report['sd_mount_reply'] = io.shell(b'mount\n')
    if '/fs/microsd type vfat' not in report['sd_mount_reply']: raise RuntimeError('SD mount missing')
    report['sd_mounted'] = True
    report['logger_before'] = io.shell(b'logger status\n')
    if 'Not logging' not in report['logger_before']: raise RuntimeError('Logger already recording; no second experiment')
    state['writer_pids'] = parse_writer_pids(io.shell(b'ps\n'))
    state['writer_fd_baseline'] = {pid: parse_fd_table(io.shell(f'cat /proc/{pid}/group/fd\n'.encode()))
                                  for pid in state['writer_pids']}
    io.shell(b'listener actuator_armed -n 1\n')
    report['precheck_verified'] = True


def start_logger(io, state, report):
    """Existing bounded active-status AND writer-FD start confirmation."""
    transition(io, state, report, 'STARTING')
    deadline = now_ns(io)+START_TIMEOUT_NS
    query(io, b'logger on\n', deadline)
    while now_ns(io) < deadline:
        status = query(io, b'logger status\n', deadline)
        report.setdefault('recording_status', []).append(status)
        if re.search(r'(?:ERROR|Failed to|Can.t open log file)', status, re.I):
            raise RuntimeError('Logger explicitly reported startup failure')
        match = re.search(r'Log file:\s*(/fs/microsd/log/[^\s]+\.ulg)', status)
        if 'Full File Logging Running:' in status and match:
            path = match.group(1)
            if not valid_log_path(path): raise RuntimeError('Unexpected log path')
            state['logfile'] = path
            report['remote_ulog_path'] = path
            raw, current = writer_tables(io, state, deadline)
            added = {}
            if any(set(table) == set(state['writer_fd_baseline'][pid]) for pid, table in current.items()):
                # Actual status may precede the async writer opening its FD.
                # Neither this transient nor a zero FAT size is corruption.
                wait_until(io, min(deadline, now_ns(io)+POLL_NS))
                continue
            for pid, table in current.items():
                added[pid] = writer_open_descriptors(table, state['writer_fd_baseline'][pid], path)
            report['writer_open_during_capture'] = raw
            report['recording_writer_fds'] = added
            report['logger_active_confirmed_ns'] = now_ns(io)
            break
        wait_until(io, min(deadline, now_ns(io)+POLL_NS))
    else:
        raise RuntimeError('Logger startup timed out without actual active status and writer FD')


def export_closed_ulog(io, state, report, run):
    """Same read-only export; only AFTER verified stop, FD closure and stability.

    Export/framing is not static/dynamic IMU quality or calibration approval.
    """
    if not (report.get('file_closed_verified') and report.get('file_size_stable')):
        raise RuntimeError('Verified closed stable logfile required before export')
    binary, crc = io.reader.download(state['logfile'], report['closed_file_size'])
    destination = Path(run)/Path(state['logfile']).name
    with destination.open('xb') as stream: stream.write(binary)
    if destination.read_bytes() != binary: raise RuntimeError('Local export readback mismatch')
    report.update(ulog_path=str(destination), ulog_size=len(binary),
                  ulog_sha256=hashlib.sha256(binary).hexdigest(), remote_crc_verification=crc, exported=True)
    return destination


def perform(io, state, report, run):
    """Existing static 12s test; common lifecycle also usable by offline hosts."""
    prepare_logger(io, state, report)
    start_logger(io, state, report)
    transition(io, state, report, 'RECORDING')
    report['recording_start_ns'] = now_ns(io)
    report['host_recording_target_ns'] = HOST_RECORDING_SPAN_NS
    deadline = report['recording_start_ns']+HOST_RECORDING_SPAN_NS
    midpoint_done = False
    print('STATIC RECORDING: confirmed active; start 12s host window; actual ULog IMU must span at least 10s', flush=True)
    while now_ns(io) < deadline:
        io.pump()
        if not io.safe(): raise RuntimeError('Fresh unarmed identity lost during recording')
        if not midpoint_done and now_ns(io) >= report['recording_start_ns']+HOST_RECORDING_SPAN_NS//2:
            status = query(io, b'logger status\n', deadline)
            report['recording_status'].append(status)
            if 'Full File Logging Running:' not in status:
                raise RuntimeError('Logger stopped unexpectedly during capture')
            midpoint_done = True
    report['recording_end_ns'] = now_ns(io)
    report['host_recording_duration_ns'] = report['recording_end_ns']-report['recording_start_ns']
    stop_and_finalize(io, state, report, io.reader)
    report['recording_command_duration_s'] = (state['off_at_ns']-state['on_at_ns'])/1e9
    destination = export_closed_ulog(io, state, report, run)
    checker = getattr(io, 'analyze_ulog', verify_static_ulog)
    quality = checker(destination, required_span_ns=REQUIRED_IMU_SPAN_NS,
                      expected_uid=report.get('identity', {}).get('uid2'))
    report['ulog_quality'] = quality
    if not quality['accepted']:
        raise RuntimeError('ULog acceptance failed: '+'; '.join(quality['reasons']))
    io.read('SDLOG_PROFILE'); io.read('SDLOG_MODE')
    io.shell(b'listener actuator_armed -n 1\n')
    report['logger_after'] = io.shell(b'logger status\n')
    if 'Not logging' not in report['logger_after']: raise RuntimeError('Logger active after export')
    report['logger_not_logging'] = True
    report['capture_completed'] = True
    transition(io, state, report, 'COMPLETE')


def finalize_resources(port, reader, report):
    """Same caller-owned port: even FTP close failure cannot skip serial close."""
    try:
        if reader.active:
            try: reader.close()
            except BaseException as exc: report['ftp_close_error'] = type(exc).__name__+': '+str(exc)
    finally:
        if port is not None:
            try: port.close()
            except BaseException as exc: report['serial_close_error'] = type(exc).__name__+': '+str(exc)
        report['serial_closed'] = port is None or not port.is_open


def abort_capture(io, state, report):
    """At most one stop after a failure; never start/restart/repeat a capture."""
    if state['on_sent'] and not report.get('file_closed_verified') and not state.get('stop_attempted'):
        try:
            stop_and_finalize(io, state, report, io.reader)
        except BaseException as exc:
            report['stop_cleanup_error'] = type(exc).__name__+': '+str(exc)
    if state['on_sent'] and not report.get('file_closed_verified'):
        report['keep_USB_power_required'] = True
    transition(io, state, report, 'FAILED')
