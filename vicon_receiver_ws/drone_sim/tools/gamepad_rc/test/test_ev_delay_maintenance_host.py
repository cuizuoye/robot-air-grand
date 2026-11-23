"""Injected lifecycle only: no real devices, ROS, SDK, node or hardware grant.

Reuse the actual static Logger lifecycle fixture and existing owner write
boundary. Every fresh sensor/clock/identity event below is SYNTHETIC.
"""
from dataclasses import replace
from pathlib import Path

import pytest
from pymavlink.dialects.v20 import common as m

from ev_delay_maintenance_host import (MaintenanceHost, ObservationCache,
    maintenance_usb_allowed, run_live, LIVE_ENTRY_ENABLED, SELECTION_QUERY)
from ev_delay_maintenance import WINDOW_NS
from mavlink_crc_stream import CrcResyncParser
from trs_powered_fusion_bench import _write_owner_packet
from test_ev_delay_recording_session import session, frame, message
from test_lock_clock_probe import reply, sample as lock_sample
from test_static_ulog_capture import FakeIO, FakePort


class TrsPort(FakePort):
    def __init__(self):
        super().__init__()
        self.writes = []
        self.short = False
        self.write_error = None
    def write(self, wire):
        self.writes.append(bytes(wire))
        if self.write_error: raise self.write_error
        return len(wire)-1 if self.short else len(wire)


class Owner:
    """Original writer and explicit synthetic evidence, not hardware inputs."""
    def __init__(self, io, recording, base):
        self.io, self.recording, self.base = io, recording, base
        self.port = TrsPort()
        self.decoder = CrcResyncParser()
        self.cache = ObservationCache()
        self.bound = None
        self.index = 0
        self.stop_at = None
        self.next_pose = 0
        self.next_clock = 0
        self.quiet = self.inputs_stopped = False
        self.calls = []
        self.options = {}
        self.publish()

    def publish(self):
        now = self.io.now
        # Explicit SYNTHETIC accepted messages, never a clock refresh inside
        # ObservationCache.snapshot(). Existing source/time gates stay intact.
        if now >= self.next_clock:
            token = self.recording.gate.lock.clock.request(now-2_000_000)
            if token is not None:
                self.recording.gate.lock.clock.ingest(
                    reply(remote=now+4_000_000_000-1_000_000, token=token), now)
                if self.bound: self.bound.observe_clock(at_ns=now)
            self.next_clock = now+100_000_000
        native = lock_sample((now+4_000_000_000-5_000_000)//1_000_000)
        native['received'] = now/1e9
        self.recording.gate.lock.status.sample = native
        if self.options.get('verify_home_at') is not None and now>=self.options['verify_home_at']:
            self.recording.gate.home.observe_short(self.recording.gate.lock,now_ns=now,
                post_home_ns=950000000,identity_verified=True)
        changes = self.options.get('observation_change', {})
        if self.bound and self.options.get('fault_after_ns') is not None:
            elapsed = now-self.recording.gate.started_ns
            if elapsed < self.options['fault_after_ns']: changes = {}
        fields = dict(now_ns=now, identity_received_ns=now,
                      attitude_received_ns=now, vicon_received_ns=now)
        fields.update(changes)
        self.cache.publish(replace(self.base, **fields))

    def observation(self, now_ns):
        obs = self.cache.snapshot(now_ns)
        return replace(obs, log_ready='logger_active_confirmed_ns' in self.io.report
                       and not self.io.state['off_sent'])

    def tick(self):
        self.publish()
        now = self.io.now
        if not self.bound or self.quiet or self.inputs_stopped or now >= self.stop_at:
            return
        src = frame(replace(self.base, now_ns=now), self.index)
        src['sdk_hz'] = 50.
        src['ros_stamp_ns'] = 1_800_000_000_000_000_000+self.index*20_000_000
        if self.options.get('occluded') and self.index >= 5: src['occluded'] = True
        if not self.bound.observe_frame(src, callback_ns=now):
            raise RuntimeError('MOCK source recording fault: '+str(self.bound.gate.fault))
        self.index += 1
        if now >= self.next_pose:
            msg = message()
            msg.usec = now//1000
            msg.pack(m.MAVLink(None,srcSystem=250,srcComponent=213))
            def clock():
                self.io.now += 1000
                return self.io.now
            _write_owner_packet(self.port, bytes(msg.get_msgbuf()), recording=self.bound,
                original=msg, observation=self.observation, clock=clock,
                wire_decoder=self.decoder, source_stamp_ns=src['ros_stamp_ns'],
                source_frame_id=src['frame_id'])
            self.next_pose = self.io.now+40_000_000

    def bind_recording(self, value, *, stop_at_ns):
        assert value is self.recording and self.bound is None
        assert self.io.report['logger_active_confirmed_ns'] <= self.io.now
        self.calls.append('bind')
        self.bound, self.stop_at = value, stop_at_ns
        if self.options.get('bind_failure'): raise RuntimeError('MOCK bind failure')

    def stop_pose_writes(self):
        self.calls.append('stop_poses')
        self.quiet = True
        if self.options.get('quiesce_failure'): raise RuntimeError('MOCK quiesce failure')
    def stop_inputs(self):
        self.calls.append('stop_inputs')
        self.inputs_stopped = True
    def detach_recording(self):
        self.calls.append('detach')
        self.bound = None
    def close_owned(self):
        self.calls.append('close')
        self.port.close()
        if self.options.get('close_failure'): raise OSError('MOCK close failure')
    def verify_configuration(self):
        self.calls.append('config_readonly')
        return not (self.options.get('config_changed') and self.inputs_stopped)


class LoggerIO(FakeIO):
    def __init__(self):
        super().__init__()
        self.port = FakePort()
        self.boot_session = 'BOOT'
        self.wire = []
        self.owner = None
        self.last_native_read_ns = None
        self.selection_fail_after = False
        self.enc = m.MAVLink(None, srcSystem=250, srcComponent=219)

    def pump(self):
        super().pump()
        if self.owner: self.owner.tick()

    def shell(self, command, *, deadline_ns=None):
        if command == b'logger on\n':
            assert self.last_native_read_ns is not None
            assert 0<=self.now-self.last_native_read_ns<500_000_000, 'Original USB startup proof stale'
        msg = self.enc.serial_control_encode(10, 2, 0, 0, len(command), command.ljust(70,b'\0'))
        assert maintenance_usb_allowed(msg, safe=True, phase=self.state['phase'],
            on_sent=self.state['on_sent'], off_sent=self.state['off_sent'],
            writer_pids=self.state['writer_pids'], logfile=self.state['logfile'])
        self.wire.append(msg)
        answer = super().shell(command, deadline_ns=deadline_ns)
        if command == b'listener actuator_armed -n 1\n':
            self.last_native_read_ns = self.now
        return answer

    def read_sensor_selection(self):
        if self.selection_fail_after and self.state['off_sent']:
            raise RuntimeError('MOCK after selection unavailable')
        msg = self.enc.serial_control_encode(10, 2, 0, 0, len(SELECTION_QUERY),
                                            SELECTION_QUERY.ljust(70,b'\0'))
        assert maintenance_usb_allowed(msg,safe=True,phase=self.state['phase'],
            on_sent=self.state['on_sent'],off_sent=self.state['off_sent'],
            writer_pids=self.state['writer_pids'],logfile=self.state['logfile'])
        self.wire.append(msg)
        # Captured raw evidence only, NOT an importer-ready reviewed bracket.
        return dict(raw_reply='MOCK sensor_selection\n gyro_device_id: 7\n',
                    host_received_ns=self.now, review_status='unreviewed')


def fixture():
    recording, obs = session()
    recording.gate.lock.clock.target = (250, 213)
    io = LoggerIO()
    io.now = obs.now_ns+10_000_000
    io.report['identity']['uid2'] = recording.gate.uid
    owner = Owner(io, recording, obs)
    io.owner = owner
    host = MaintenanceHost(owner, io, recording, state=io.state, logger_report=io.report)
    return host, owner, io


def verify_cleanup(host, owner, io, out):
    assert io.port.close_calls == 1 and not io.port.is_open
    assert owner.port.close_calls == 1 and not owner.port.is_open
    assert owner.calls.index('stop_poses') < owner.calls.index('stop_inputs') < owner.calls.index('detach')
    assert owner.calls[-1] == 'close'
    assert io.commands.count(b'logger on\n') <= 1
    assert io.commands.count(b'logger off\n') <= 1
    assert not out['physical_authorization'] and not out['control_ready']
    assert not out['flight_authorized'] and not out['live_enabled']
    assert not out['data_quality_verified'] and not out['hardware_verified']
    assert out['automatic_retries'] == 0 and out['parameter_writes'] == 0
    assert set(io.params) <= {'SDLOG_PROFILE', 'SDLOG_MODE'}
    decoder = CrcResyncParser()
    for wire in owner.port.writes:
        msg, = decoder.parse_buffer(wire)
        assert msg.get_type() == 'VISION_POSITION_ESTIMATE'
    assert not decoder.buffer
    assert not any(b'param ' in c or b'commander' in c or b'reboot' in c for c in io.commands)


def test_complete_injected_15_second_cycle_reuses_writer_logger_and_export(tmp_path):
    host, owner, io = fixture()
    out = host.run_injected(tmp_path)
    verify_cleanup(host, owner, io, out)
    assert out['injected_cycle_completed'], out
    assert out['capture_ended_ns']-out['capture_started_ns'] >= WINDOW_NS
    assert out['capture_deadline_ns']-out['capture_started_ns'] == WINDOW_NS
    assert 300 <= len(owner.port.writes) <= 480
    assert out['record']['atomic_source']['matched_sends'] == len(owner.port.writes)
    assert out['record']['callbacks_closed_cleanly']
    assert out['capture_trace']['events']
    assert not out['capture_trace']['physical_authorization']
    assert not out['capture_trace']['flight_authorized']
    assert not out['capture_trace']['recorder_fault']
    events=out['capture_trace']['events']
    assert all(a['at_ns']<=b['at_ns'] for a,b in zip(events,events[1:]))
    assert out['selection_snapshots']['before']['host_received_ns'] < out['capture_started_ns']
    assert out['selection_snapshots']['after']['host_received_ns'] > out['capture_ended_ns']
    assert io.report['logger_active_confirmed_ns'] <= out['capture_started_ns']
    assert io.state['off_at_ns'] >= out['capture_ended_ns']
    assert out['logger']['exported'] and out['logger']['file_closed_verified']
    assert Path(out['logger']['ulog_path']).read_bytes() == io.binary
    # Fake ULog header has no real sensor data. Lifecycle completion MUST NOT
    # approve delay/IMU quality, real resources, parameters or flight.
    assert out['ULog_framing']['ok'] and not out['data_quality_verified']
    assert [p['name'] for p in out['phases']] == ['PRECHECK','WAIT_HOME','LOGGER_STARTING',
        'CAPTURING','QUIESCING','LOGGER_STOPPING','EXPORTING','COMPLETE']
    with pytest.raises(RuntimeError, match='One maintenance lifecycle'):
        host.run_injected(tmp_path)
    assert io.commands.count(b'logger on\n') == 1


@pytest.mark.parametrize('kind', ['source', 'short_write', 'write_error', 'interrupt',
    'identity', 'reboot', 'pad', 'attitude_501ms', 'vicon_151ms', 'config',
    'bind', 'quiesce', 'owner_close', 'never_start', 'never_stop', 'empty_file',
    'broken_ulog', 'post_selection', 'post_configuration', 'home_missing', 'home_aborted'])
def test_failure_stops_without_retry_or_EKF_restore(tmp_path,kind):
    host, owner, io = fixture()
    if kind == 'source': owner.options['occluded'] = True
    elif kind == 'short_write': owner.port.short = True
    elif kind in ('write_error','interrupt'):
        owner.port.write_error = OSError('MOCK write') if kind=='write_error' else KeyboardInterrupt('MOCK interrupt')
    elif kind in ('identity','reboot','pad','attitude_501ms','vicon_151ms','config'):
        now=io.now
        changes=dict(identity=dict(uid='OTHER'),reboot=dict(fc_boot='OTHER'),
            pad=dict(gamepad_generation='NEW'),attitude_501ms=dict(attitude_received_ns=now-501000000),
            vicon_151ms=dict(vicon_received_ns=now-151000000),config=dict(config_readonly_unchanged=False))
        owner.options.update(observation_change=changes[kind],fault_after_ns=200000000)
    elif kind == 'bind': owner.options['bind_failure'] = True
    elif kind == 'quiesce': owner.options['quiesce_failure'] = True
    elif kind == 'owner_close': owner.options['close_failure'] = True
    elif kind == 'never_start': io.never_start = True
    elif kind == 'never_stop': io.never_stop = True
    elif kind == 'empty_file': io.binary = b''
    elif kind == 'broken_ulog': io.binary += b'\x05'
    elif kind == 'post_selection': io.selection_fail_after = True
    elif kind == 'post_configuration': owner.options['config_changed'] = True
    elif kind == 'home_missing': host.session.gate.home.verified = False
    elif kind == 'home_aborted': host.session.gate.home.aborted = True
    out=host.run_injected(tmp_path)
    verify_cleanup(host,owner,io,out)
    assert not out['injected_cycle_completed'],out
    assert out['errors'] or out['cleanup_errors'] or not out['record']['callbacks_closed_cleanly']
    expected=dict(source='source recording fault',short_write='Short TRS serial write',
        write_error='MOCK write',interrupt='KeyboardInterrupt',
        identity='identity_boot_or_gamepad_changed',reboot='identity_boot_or_gamepad_changed',
        pad='identity_boot_or_gamepad_changed',attitude_501ms='attitude_received_ns_stale',
        vicon_151ms='vicon_received_ns_stale',config='config_readonly_unchanged',
        bind='MOCK bind failure',quiesce='stop_pose_writes',owner_close='MOCK close failure',
        never_start='Logger startup timed out',never_stop='Logger stop/file-close timeout',
        empty_file='Closed logfile stayed empty',broken_ulog='ULog framing',
        post_selection='MOCK after selection unavailable',post_configuration='configuration changed after',
        home_missing='HOME proof aborted or timed out',home_aborted='HOME fault remains latched')[kind]
    assert expected in str(out['errors']+out['cleanup_errors']),out
    assert 'capture_trace' in out, 'Failed partial RAM records must be retained too'
    assert io.now<120_000_000_000
    if kind=='home_missing': assert b'logger on\n' not in io.commands
    if kind=='never_stop': assert out['keep_USB_power_required'] and not io.report.get('exported')


@pytest.mark.parametrize('change', ['uid','boot','used_logger'])
def test_precheck_binding_failure_never_starts_logger(tmp_path,change):
    host, owner, io=fixture()
    if change=='uid': io.report['identity']['uid2']='WRONG'
    elif change=='boot': io.boot_session='WRONG'
    else: io.state['on_sent']=True;io.state['off_sent']=True;io.state['stop_attempted']=True
    out=host.run_injected(tmp_path)
    verify_cleanup(host,owner,io,out)
    assert not out['injected_cycle_completed']
    assert b'logger on\n' not in io.commands and not owner.port.writes


def test_snapshot_assessment_does_not_refresh_original_receipts():
    _, obs=session()
    cache=ObservationCache()
    with pytest.raises(RuntimeError,match='missing'): cache.snapshot(obs.now_ns)
    cache.publish(obs)
    later=cache.snapshot(obs.now_ns+501000000)
    assert later.identity_received_ns==obs.identity_received_ns
    assert later.attitude_received_ns==obs.attitude_received_ns
    assert later.vicon_received_ns==obs.vicon_received_ns
    with pytest.raises(ValueError,match='precedes'): cache.snapshot(obs.now_ns-1)
    with pytest.raises(ValueError,match='regressed'): cache.publish(replace(obs,now_ns=obs.now_ns-1))
    with pytest.raises(TypeError): cache.publish({})


@pytest.mark.parametrize('phase',['PRECHECK','STARTING','STOPPING','VERIFYING'])
def test_only_fixed_selection_query_extends_USB_read_gate(phase):
    enc=m.MAVLink(None)
    msg=enc.serial_control_encode(10,2,0,0,len(SELECTION_QUERY),SELECTION_QUERY.ljust(70,b'\0'))
    assert maintenance_usb_allowed(msg,safe=True,phase=phase,on_sent=False,off_sent=False)
    assert not maintenance_usb_allowed(msg,safe=False,phase=phase,on_sent=False,off_sent=False)
    assert not maintenance_usb_allowed(msg,safe=True,phase='RECORDING',on_sent=True,off_sent=False)


@pytest.mark.parametrize('payload',[b'param save\n',b'param set EKF2_EV_CTRL 11\n',
    b'reboot\n',b'commander arm\n',b'listener sensor_selection -n 100\n',b'logger on\n'])
def test_extra_shell_and_logger_start_in_wrong_phase_remain_rejected(payload):
    msg=m.MAVLink(None).serial_control_encode(10,2,0,0,len(payload),payload.ljust(70,b'\0'))
    assert not maintenance_usb_allowed(msg,safe=True,phase='VERIFYING',on_sent=True,off_sent=True)


def test_live_entry_remains_closed_without_creating_resources():
    assert not LIVE_ENTRY_ENABLED
    with pytest.raises(PermissionError,match='remains closed'): run_live()


def test_clock_failure_during_capture_cannot_skip_resource_cleanup(tmp_path):
    host, owner, io=fixture()
    original=io.pump
    tripped=False
    def pump():
        nonlocal tripped
        original()
        if owner.bound and not tripped and owner.index>10:
            io.now-=200000000
            tripped=True
    io.pump=pump
    out=host.run_injected(tmp_path)
    verify_cleanup(host,owner,io,out)
    assert not out['injected_cycle_completed'] and out['errors']
    assert any(p['at_ns'] is None for p in out['phases'])


@pytest.mark.parametrize('state', [dict(on_sent=True), dict(off_sent=True),
                                  dict(stop_attempted=True)])
def test_preexisting_logger_lifecycle_is_never_stopped_or_exported(tmp_path,state):
    host, owner, io=fixture()
    io.state.update(state)
    out=host.run_injected(tmp_path)
    verify_cleanup(host,owner,io,out)
    assert io.commands == []
    assert not io.report.get('exported') and not out['selection_snapshots']
    assert not out['injected_cycle_completed']
    assert 'config_readonly' not in owner.calls


def test_same_port_cannot_be_owned_by_USB_and_TRS(tmp_path):
    host, owner, io=fixture()
    owner.port=io.port
    out=host.run_injected(tmp_path)
    assert not out['injected_cycle_completed']
    assert io.port.close_calls==1
    assert not io.commands


def test_missing_observation_cannot_be_filled_from_old_readonly_report(tmp_path):
    host, owner, io=fixture()
    owner.observation=lambda now: None
    out=host.run_injected(tmp_path)
    verify_cleanup(host,owner,io,out)
    assert not out['injected_cycle_completed'] and not owner.port.writes
    assert any('original owner evidence' in reason for reason in out['errors'])


def test_successful_close_always_precedes_any_log_download(tmp_path):
    host, owner, io=fixture()
    download=io.reader.download
    def guarded_download(*args):
        assert owner.quiet and owner.inputs_stopped and owner.bound is None
        assert host.session.closed and io.report['file_closed_verified']
        return download(*args)
    io.reader.download=guarded_download
    assert host.run_injected(tmp_path)['injected_cycle_completed']


def test_manual_HOME_wait_pumps_state_but_never_emits_poses(tmp_path):
    host, owner, io=fixture()
    host.session.gate.home.verified=False
    owner.options['verify_home_at']=io.now+800000000
    out=host.run_injected(tmp_path)
    verify_cleanup(host,owner,io,out)
    assert out['injected_cycle_completed'],out
    assert out['capture_started_ns']>=owner.options['verify_home_at']
    assert io.state['on_at_ns']>=owner.options['verify_home_at']
    assert len(host.session.gate.home.events)==2, 'Only existing MOCK press/release; no automatic HOME'


def test_download_failure_never_restarts_log_or_masks_cleanup(tmp_path):
    host, owner, io=fixture()
    def fail(*args): raise OSError('MOCK export failure')
    io.reader.download=fail
    out=host.run_injected(tmp_path)
    verify_cleanup(host,owner,io,out)
    assert not out['injected_cycle_completed']
    assert out['logger']['file_closed_verified']
    assert any('MOCK export failure' in e for e in out['cleanup_errors'])
    assert not io.report.get('exported')


def test_USB_close_failure_never_converts_Mock_cycle_to_hardware_success(tmp_path):
    host, owner, io=fixture()
    def fail():
        io.port.close_calls+=1
        raise OSError('MOCK USB close failed')
    io.port.close=fail
    out=host.run_injected(tmp_path)
    assert not out['injected_cycle_completed'] and not out['flight_authorized']
    assert io.port.close_calls==1 and owner.port.close_calls==1
    assert 'USB_serial_not_closed' in out['cleanup_errors']
