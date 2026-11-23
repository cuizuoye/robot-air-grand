"""Maintenance capture: in-memory transport, fake clock, offline ULog only.

Repository conftest forbids opening any real serial/device. Mock COMPLETE is
only a Logger tool result; it never creates flight/preparation authorization.
"""
from copy import deepcopy
import hashlib
from pathlib import Path
import struct
from types import SimpleNamespace as NS

import pytest
from pymavlink.dialects.v20 import common as m

import static_ulog_capture as entry
import static_ulog_capture_actions as a
import static_ulog_capture_policy as policy

PATH = '/fs/microsd/log/sess200/log001.ulg'
HEADER = b'ULog\x01\x12\x35\x01'+struct.pack('<Q', 1000000)
UID = 'MOCK_ONLY_NOT_HARDWARE_AUTHORITY'


def fake_ulog(span_us=10000000, step_us=5000):
    stamps = list(range(1000000, 1000000+span_us+1, step_us))
    count = len(stamps)
    sensor = dict(timestamp=stamps, gyro_integral_dt=[step_us]*count,
                  gyro_clipping=[0]*count, accelerometer_clipping=[0]*count)
    for prefix in ('gyro_rad', 'accelerometer_m_s2'):
        sensor.update({f'{prefix}[{i}]': [0.]*count for i in range(3)})
    datasets = [NS(name='sensor_combined', multi_id=0, data=sensor),
        NS(name='sensor_selection', multi_id=0, data=dict(timestamp=[900000], gyro_device_id=[42])),
        NS(name='actuator_armed', multi_id=0, data=dict(timestamp=[1000000], armed=[0])),
        NS(name='vehicle_status', multi_id=0, data=dict(timestamp=[1000000], arming_state=[1]))]
    return NS(file_corruption=False, dropouts=[], msg_info_dict=dict(sys_uuid=UID), data_list=datasets)


class FakePort:
    def __init__(self): self.is_open = True; self.close_calls = 0
    def close(self): self.close_calls += 1; self.is_open = False


class FakeReader:
    def __init__(self, io): self.io = io; self.active = False; self.sizes = []
    def size(self, path):
        assert path == PATH
        self.io.now += 10000000
        self.sizes.append((self.io.now, self.io.state['off_sent']))
        # No data size is used to reject STARTING/RECORDING. During stopped
        # filesystem verification a transient zero is likewise tolerated.
        if self.io.now-self.io.state['off_at_ns'] < self.io.size_growth_delay_ns:
            return 0
        return len(self.io.binary)
    def download(self, path, size):
        assert path == PATH and size == len(self.io.binary)
        assert self.io.state['phase'] == 'VERIFYING'
        return self.io.binary, dict(matches=True, before=123, after=123, local=123)
    def close(self): self.active = False


class FakeIO:
    def __init__(self, **options):
        self.now = 1000000000
        self.commands = []; self.params = []
        self.startup_delay_ns = 500000000
        self.fd_start_delay_ns = 100000000
        self.stop_delay_ns = 300000000
        self.fd_stop_delay_ns = 500000000
        self.size_growth_delay_ns = 700000000
        self.explicit_start_failure = False
        self.never_start = False
        self.never_stop = False
        self.record_fault_ns = None
        self.record_fault_raised = False
        self.binary = HEADER
        self.ulog = fake_ulog()
        self.state = dict(phase='PRECHECK', on_sent=False, off_sent=False,
            on_at_ns=None, off_at_ns=None, writer_pids=(), logfile=None, ftp_replies=[])
        self.report = dict(parameters={}, identity=dict(uid2=UID), file_closed_verified=False,
            capture_completed=False, error=None, flight_authorized=False)
        self.__dict__.update(options)
        self.reader = FakeReader(self)
    def clock_ns(self): return self.now
    def safe(self): return True
    def pump(self):
        self.now += 20000000
        if (self.record_fault_ns is not None and not self.record_fault_raised
                and self.state['phase'] == 'RECORDING'
                and self.now-self.report['recording_start_ns'] >= self.record_fault_ns):
            self.record_fault_raised = True
            raise RuntimeError('Mock mid-recording telemetry fault')
    def read(self, name):
        self.params.append(name)
        self.report['parameters'][name] = dict(value=3 if name=='SDLOG_PROFILE' else 0, type=6)
    def analyze_ulog(self, path, **kwargs):
        return a.verify_static_ulog(path, parser=lambda unused: self.ulog, **kwargs)
    def shell(self, command, *, deadline_ns=None):
        self.now += 10000000
        if deadline_ns is not None and self.now > deadline_ns:
            raise RuntimeError('Mock bounded command deadline')
        self.commands.append(command)
        s = self.state
        if command == b'logger on\n':
            assert s['phase']=='STARTING' and not s['on_sent']
            s['on_sent'] = True; s['on_at_ns'] = self.now
            return 'logger on\nnsh>'
        if command == b'logger off\n':
            assert s['on_sent'] and not s['off_sent']
            s['off_sent'] = True; s['off_at_ns'] = self.now
            return 'logger off\nnsh>'
        if command == b'listener actuator_armed -n 1\n': return 'armed: False'
        if command == b'mount\n': return '/fs/microsd type vfat'
        if command == b'ps\n': return ' 100 97 60 FIFO pthread Waiting 0 log_writer_file\n'
        if command == b'logger status\n':
            if self.explicit_start_failure and s['on_sent'] and not s['off_sent']:
                return 'ERROR Failed to open log file'
            active = (s['on_sent'] and not self.never_start and
                      self.now-s['on_at_ns'] >= self.startup_delay_ns)
            if s['off_sent'] and not self.never_stop and self.now-s['off_at_ns'] >= self.stop_delay_ns:
                active = False
            if not active: return 'Not logging'
            # Even while FAT size remains zero, the actual Logger state is the
            # primary start criterion. All counts here are explicitly MOCK.
            return 'Full File Logging Running:\nLog file: '+PATH+'\nWrote 0.00 KiB'
        if command == b'cat /proc/100/group/fd\n':
            exists = (s['on_sent'] and not self.never_start and
                self.now-s['on_at_ns'] >= self.startup_delay_ns+self.fd_start_delay_ns)
            if s['off_sent'] and not self.never_stop and self.now-s['off_at_ns'] >= self.fd_stop_delay_ns:
                exists = False
            # Exact pathless FAT rows observed in the actual prior capture.
            return ('FD POS OFLAGS PATH\n 0 6144 101 \n 1 0 2 /dev/console_buf\n 2 195 101 \n'
                    + (' 3 99543 6 \n' if exists else ''))
        raise AssertionError('Unapproved mock command '+str(command))


def execute(io, path):
    """Same lifecycle/abort/finalizer used by the actual single-owner entry."""
    port = FakePort()
    try:
        a.perform(io, io.state, io.report, path)
    except BaseException as exc:
        io.report['error'] = str(exc)
        a.abort_capture(io, io.state, io.report)
    finally:
        a.finalize_resources(port, io.reader, io.report)
    assert port.close_calls == 1 and not port.is_open and io.report['serial_closed']
    assert io.commands.count(b'logger on\n') <= 1
    assert io.commands.count(b'logger off\n') <= 1
    assert not io.report['flight_authorized']
    return io.report


def test_size_zero_is_valid_metadata_but_never_a_valid_export():
    reader = a.FileReader(NS(), {}, {})
    requests = []
    reader.request = lambda op, *args, **kwargs: requests.append(op) or (struct.pack('<I',0) if op==4 else b'')
    assert reader.size(PATH) == 0
    assert requests == [4, 1] and not reader.active
    with pytest.raises(RuntimeError, match='closed/export'): reader.open(PATH)
    reader.close()
    assert requests == [4,1,4,1]


@pytest.mark.parametrize('delay', [0, 100000000, 900000000, 2200000000])
def test_delayed_zero_size_and_pathless_fd_do_not_abort_capture(tmp_path, delay):
    io = FakeIO(size_growth_delay_ns=delay)
    report = execute(io, tmp_path)
    assert report['error'] is None and report['capture_completed']
    assert io.state['phase'] == 'COMPLETE'
    assert all(off_sent for _, off_sent in io.reader.sizes), 'No FAT-size admission gate while recording'
    assert report['file_closed_verified'] and report['file_size_stable']
    assert report['ulog_quality']['imu_span_ns'] >= 10000000000


def test_ten_seconds_begins_only_after_actual_active_state_and_writer_fd(tmp_path):
    io = FakeIO(startup_delay_ns=1500000000)
    report = execute(io, tmp_path)
    assert report['error'] is None
    assert report['recording_start_ns']-io.state['on_at_ns'] >= io.startup_delay_ns+io.fd_start_delay_ns
    assert report['host_recording_duration_ns'] >= 10000000000
    assert io.state['off_at_ns']-report['recording_start_ns'] >= 10000000000
    assert [v['phase'] for v in report['state_transitions']] == [
        'STARTING','RECORDING','STOPPING','VERIFYING','COMPLETE']


def test_authorized_twelve_second_host_window_keeps_actual_imu_minimum_ten(tmp_path):
    io=FakeIO()
    report=execute(io,tmp_path)
    assert report['host_recording_target_ns']==12000000000
    assert report['host_recording_duration_ns']>=12000000000
    assert report['ulog_quality']['requested_imu_span_ns']==10000000000
    assert not report['flight_authorized']


@pytest.mark.parametrize('options,reason', [
    (dict(explicit_start_failure=True),'explicitly reported startup failure'),
    (dict(never_start=True),'startup timed out'),
    (dict(startup_delay_ns=6000000000),'startup timed out'),
])
def test_start_failure_is_bounded_and_stops_once_without_repeating(tmp_path,options,reason):
    io = FakeIO(**options)
    report = execute(io,tmp_path)
    assert reason in report['error']
    assert not report['capture_completed'] and io.state['phase']=='FAILED'
    assert io.commands.count(b'logger on\n')==1 and io.commands.count(b'logger off\n')==1
    assert io.now < 30000000000


def test_record_fault_retains_reason_and_stops_once(tmp_path):
    io = FakeIO(record_fault_ns=700000000)
    report = execute(io,tmp_path)
    assert 'mid-recording' in report['error']
    assert io.state['phase']=='FAILED' and report['file_closed_verified']
    assert not report['capture_completed']
    before = list(io.commands)
    a.abort_capture(io,io.state,report)
    assert io.commands == before, 'No second stop or capture transaction after failure'


@pytest.mark.parametrize('stop_delay,fd_delay', [(0,0), (1200000000,1800000000)])
def test_stop_ack_does_not_mean_writer_already_closed(tmp_path,stop_delay,fd_delay):
    io=FakeIO(stop_delay_ns=stop_delay,fd_stop_delay_ns=fd_delay)
    report=execute(io,tmp_path)
    assert report['capture_completed']
    if fd_delay:
        assert any(not v['writer_at_baseline'] for v in report['close_fd_checks'])
    assert all(3 not in policy.parse_fd_table(t) for t in report['final_writer_fd_tables'].values())


def test_stop_failure_requires_keeping_power_no_export_no_second_stop(tmp_path):
    io=FakeIO(never_stop=True)
    report=execute(io,tmp_path)
    assert report['keep_USB_power_required']
    assert not report.get('exported') and not report['file_closed_verified']
    assert io.commands.count(b'logger off\n')==1
    assert 'close' in report['error'].lower() or 'deadline' in report['error'].lower()


def test_incomplete_ulog_is_retained_but_rejected(tmp_path):
    io=FakeIO(binary=HEADER+b'\x05')
    report=execute(io,tmp_path)
    assert report['exported'] and Path(report['ulog_path']).read_bytes()==io.binary
    assert not report['capture_completed']
    assert 'truncated_record_header' in report['ulog_quality']['reasons']


def test_host_ten_seconds_cannot_approve_short_actual_imu_span(tmp_path):
    io=FakeIO(ulog=fake_ulog(span_us=9000000))
    report=execute(io,tmp_path)
    assert report['host_recording_duration_ns']>=10000000000
    assert report['ulog_quality']['imu_duration_s']==9.0
    assert 'IMU_effective_duration_below_10s' in report['error']
    assert not report['capture_completed']


@pytest.mark.parametrize('point',['before_start','starting','recording','stopping','verifying','complete'])
def test_serial_finalizer_always_closes_owned_port(point):
    port=FakePort(); report={}
    reader=NS(active=True,close=lambda: (_ for _ in ()).throw(RuntimeError('Mock FTP close failure at '+point)))
    try:
        if point!='complete': raise RuntimeError(point)
    except RuntimeError: pass
    finally: a.finalize_resources(port,reader,report)
    assert report['serial_closed'] and port.close_calls==1
    assert point in report['ftp_close_error']


@pytest.mark.parametrize('p', [b'param save\n',b'reboot\n',b'param set SDLOG_PROFILE 3\n',
    b'param set EKF2_EV_CTRL 11\n',b'commander arm\n',b'logger start\n',b'logger stop\n'])
def test_lifecycle_does_not_open_other_control_or_parameter_paths(p):
    e=m.MAVLink(None)
    msg=e.serial_control_encode(10,2,0,0,len(p),p.ljust(70,b'\0'))
    for phase in ('STARTING','RECORDING','STOPPING','VERIFYING','COMPLETE','FAILED'):
        assert not policy.allowed(msg,safe=True,phase=phase,on_sent=True,off_sent=False)


def test_no_blanket_widening_of_acceptance_duration(tmp_path):
    path=tmp_path/'test.ulg';path.write_bytes(HEADER)
    with pytest.raises(ValueError,match='full 10s'):
        a.verify_static_ulog(path,required_span_ns=0,parser=lambda unused:fake_ulog())


def test_previous_real_short_ulog_replay_cannot_pass_ten_seconds():
    path=Path(__file__).resolve().parents[3]/'preparation_runs/b3_3_static_ulog_export_only_20261021_115034_421613/log100.ulg'
    if not path.exists(): pytest.skip('Historical raw ULog is not distributed with this checkout')
    before=hashlib.sha256(path.read_bytes()).hexdigest()
    result=a.verify_static_ulog(path,expected_uid='000600000000343434323533511500280022')
    assert not result['accepted'] and result['framing']['ok']
    assert result['imu_duration_s']==pytest.approx(1.275788)
    assert 'IMU_effective_duration_below_10s' in result['reasons']
    assert hashlib.sha256(path.read_bytes()).hexdigest()==before


def test_import_and_selftest_never_authorize_or_open_hardware():
    entry.selftest()
    with pytest.raises(RuntimeError,match='authorization ID'):
        entry.main()


@pytest.mark.parametrize('payload,reason', [(b'', 'invalid_ULog_header'),
    (HEADER+b'\x03\x00D\x01','truncated_record_payload')])
def test_other_incomplete_file_boundaries_are_rejected(tmp_path,payload,reason):
    path=tmp_path/'bad.ulg';path.write_bytes(payload)
    result=a.verify_static_ulog(path,parser=lambda unused:fake_ulog())
    assert not result['accepted'] and reason in result['reasons']


@pytest.mark.parametrize('case', ['parser_error','parser_corruption','wrong_uid','missing_imu',
    'arming','time_reversal','large_gap','low_rate','clipping','short_by_one_microsecond'])
def test_post_export_quality_cannot_pass_on_nonempty_file_alone(tmp_path,case):
    u=fake_ulog()
    if case=='parser_corruption': u.file_corruption=True
    if case=='wrong_uid': u.msg_info_dict['sys_uuid']='OTHER'
    if case=='missing_imu': u.data_list=u.data_list[1:]
    if case=='arming': u.data_list[2].data['armed']=[1]
    if case=='time_reversal': u.data_list[0].data['timestamp'][5]=999999
    if case=='large_gap':
        u.data_list[0].data['timestamp']=[t+(30000 if i>100 else 0) for i,t in enumerate(u.data_list[0].data['timestamp'])]
    if case=='low_rate': u=fake_ulog(step_us=25000)
    if case=='clipping': u.data_list[0].data['gyro_clipping'][1]=1
    if case=='short_by_one_microsecond': u.data_list[0].data['timestamp'][-1]-=1
    def parsed(unused):
        if case=='parser_error': raise ValueError('Mock ULog definitions truncated')
        return u
    path=tmp_path/'test.ulg';path.write_bytes(HEADER)
    result=a.verify_static_ulog(path,expected_uid=UID,parser=parsed)
    assert not result['accepted'] and result['reasons']
    assert not result['flight_authorization']


def test_start_stop_and_ftp_policy_remains_one_shot_and_read_only():
    from fault_archive_service import payload
    e=m.MAVLink(None)
    def message(p): return e.serial_control_encode(10,2,0,0,len(p),p.ljust(70,b'\0'))
    def allowed(msg,**changes):
        context=dict(safe=True,phase='STARTING',on_sent=False,off_sent=False,logfile=PATH)
        context.update(changes)
        return policy.allowed(msg,**context)
    assert allowed(message(b'logger on\n'))
    assert not allowed(message(b'logger on\n'),on_sent=True)
    assert allowed(message(b'logger off\n'),phase='STOPPING',on_sent=True)
    assert not allowed(message(b'logger off\n'),phase='STOPPING',on_sent=True,off_sent=True)
    assert not allowed(e.param_set_encode(1,1,b'SDLOG_PROFILE',3,6))
    assert not allowed(e.param_ext_set_encode(1,1,b'EKF2_EV_CTRL',b'11',6))
    for opcode in (2,6,7,8,9,10,11,12,13,15):
        assert not allowed(e.file_transfer_protocol_encode(0,1,1,payload(1,opcode,PATH.encode())),phase='VERIFYING')
    read=e.file_transfer_protocol_encode(0,1,1,payload(1,5,offset=0,size=239))
    assert allowed(read,phase='VERIFYING')
    assert not allowed(read,phase='RECORDING')


def test_file_close_proof_cannot_ignore_pathless_fat_descriptor():
    base=policy.parse_fd_table('FD POS OFLAGS PATH\n0 6144 101\n1 0 2 /dev/console_buf\n2 195 101\n')
    active=policy.parse_fd_table('FD POS OFLAGS PATH\n0 6144 101\n1 0 2 /dev/console_buf\n2 195 101\n3 99543 6\n')
    assert policy.writer_open_descriptors(active,base,PATH)==[3]
    assert not policy.writer_returned_to_baseline(active,base)
    assert policy.writer_returned_to_baseline(base,base)
    with pytest.raises(RuntimeError): policy.parse_fd_table('Not logging')
