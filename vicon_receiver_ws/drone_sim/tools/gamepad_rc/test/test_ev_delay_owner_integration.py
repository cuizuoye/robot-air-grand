"""Actual existing owner write boundary with memory-only transports/clock."""
from dataclasses import replace
from types import SimpleNamespace as NS

import pytest
from pymavlink.dialects.v20 import common as m

import trs_powered_fusion_bench as owner
from mavlink_crc_stream import CrcResyncParser
from test_ev_delay_recording_session import session as recording_session, message, frame
from test_mission_preparation_io import MemoryPort
from test_lock_clock_probe import reply, sample as lock_sample


def session():
    value, obs = recording_session()
    # Original prepared() replaces the clock with a legacy target 250:203.
    # This owner's real encoder/StaticLockGate use 250:213. Match the MOCK
    # fixture explicitly; never relax the runtime producer identity check.
    value.gate.lock.clock.target = (250, 213)
    return value, obs


def run_write(s, obs, source, port=None, msg=None, *, times=None, observer=None):
    port = port or MemoryPort()
    msg = msg or message()
    stamps = iter(times or [obs.now_ns+i*1000 for i in range(4)])
    return owner._write_owner_packet(port, bytes(msg.get_msgbuf()), recording=s,
        original=msg, observation=observer or (lambda ns: replace(obs, now_ns=ns)),
        clock=lambda: next(stamps), wire_decoder=CrcResyncParser(),
        source_stamp_ns=source['ros_stamp_ns'], source_frame_id=source['frame_id'])


def test_existing_owner_write_calls_bookkeeping_around_exactly_one_actual_write():
    s, obs = session()
    assert s.begin(obs)
    source = frame(obs)
    assert s.observe_frame(source, callback_ns=obs.now_ns)
    port = MemoryPort()
    port.read = lambda *args: pytest.fail('Write hooks added another serial reader')
    length = run_write(s, obs, source, port)
    assert port.order == ['write'] and len(port.writes) == 1
    assert length == len(port.writes[0])
    out = s.close()
    assert out['callbacks_closed_cleanly'] and out['completed_pose_writes'] == 1
    send = [e for e in s.recorder.events if e['kind'] == 'calibration_send'][0]
    assert send['tx_start_ns'] == obs.now_ns+1000
    assert send['sent_ns'] == obs.now_ns+2000
    assert send['at_ns'] == obs.now_ns+3000
    assert out['parameter_writes'] == 0 and not out['flight_authorized']


@pytest.mark.parametrize('fault', ['short', 'write_error', 'interrupt', 'postwrite_proof'])
def test_delivery_or_postwrite_failure_is_not_retried_and_cleanup_is_silent(fault):
    s, obs = session()
    assert s.begin(obs)
    source = frame(obs)
    assert s.observe_frame(source, callback_ns=obs.now_ns)
    port = MemoryPort()
    if fault == 'short': port.short = True
    original_write = port.write

    def write(wire):
        amount = original_write(wire)
        if fault == 'write_error': raise OSError('MOCK failed delivery')
        if fault == 'interrupt': raise KeyboardInterrupt('MOCK interrupted delivery')
        return amount
    port.write = write

    def observed(ns):
        value = replace(obs, now_ns=ns)
        if fault == 'postwrite_proof' and ns == obs.now_ns+3000:
            value = replace(value, attitude_received_ns=ns-500000000)
        return value

    with pytest.raises((OSError, KeyboardInterrupt, RuntimeError)):
        run_write(s, obs, source, port, observer=observed)
    assert len(port.writes) == 1
    before = list(port.writes)
    assert not s.close()['callbacks_closed_cleanly']
    assert not s.before_tx(message(), obs)
    assert port.writes == before


@pytest.mark.parametrize('fault', ['source_header', 'CRC', 'producer', 'changed_fields',
                                  'dirty_decoder', 'clock_regression', 'expired_before_write'])
def test_prewrite_failure_cannot_send_any_bytes(fault):
    s, obs = session()
    assert s.begin(obs)
    source = frame(obs)
    assert s.observe_frame(source, callback_ns=obs.now_ns)
    port, msg, decoder = MemoryPort(), message(), CrcResyncParser()
    packet = bytes(msg.get_msgbuf())
    if fault == 'source_header': source['ros_stamp_ns'] = None
    elif fault == 'CRC':
        msg._msgbuf[-1] ^= 1
        packet = bytes(msg.get_msgbuf())
    elif fault == 'producer':
        enc = m.MAVLink(None, srcSystem=250, srcComponent=220)
        packet = bytes(msg.pack(enc))
    elif fault == 'changed_fields': msg.x += 1
    elif fault == 'dirty_decoder': decoder.buffer.extend(b'\xfd')
    times = iter([obs.now_ns, obs.now_ns-1 if fault == 'clock_regression' else
                  obs.now_ns+500000000 if fault == 'expired_before_write' else obs.now_ns+1000])
    with pytest.raises((ValueError, PermissionError)):
        owner._write_owner_packet(port, packet, recording=s, original=msg,
            observation=lambda ns: replace(obs, now_ns=ns), clock=lambda: next(times),
            wire_decoder=decoder, source_stamp_ns=source['ros_stamp_ns'],
            source_frame_id=source['frame_id'])
    assert not port.writes
    assert not s.close()['callbacks_closed_cleanly']


@pytest.mark.parametrize('kind', ['PARAM_SET', 'COMMAND_LONG', 'SET_POSITION_TARGET_LOCAL_NED'])
def test_original_maintenance_policy_still_blocks_parameter_and_control_packets(kind):
    s, obs = session()
    assert s.begin(obs)
    source = frame(obs)
    enc = m.MAVLink(None, srcSystem=250, srcComponent=213)
    if kind == 'PARAM_SET': msg = enc.param_set_encode(1, 1, b'EKF2_EV_CTRL', 11, 6)
    elif kind == 'COMMAND_LONG': msg = enc.command_long_encode(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0)
    else: msg = enc.set_position_target_local_ned_encode(1, 1, 1, 1, 0, *([0.]*11))
    msg.pack(enc)
    port = MemoryPort()
    with pytest.raises((PermissionError, ValueError)):
        run_write(s, obs, source, port, msg)
    assert not port.writes
    assert s.close()['parameter_writes'] == 0


def test_old_sender_default_path_is_the_same_one_write_no_new_clock_or_checks():
    port = MemoryPort()
    assert owner._write_owner_packet(port, b'unchanged-existing-packet') == 25
    assert port.writes == [b'unchanged-existing-packet']
    port.short = True
    with pytest.raises(OSError, match='Short TRS serial write'):
        owner._write_owner_packet(port, b'unchanged-existing-packet')
    assert len(port.writes) == 2  # Separate explicit test call, no internal retry.


def test_no_live_maintenance_or_fixed_mode_entry_is_enabled():
    with pytest.raises(PermissionError, match='offline-only'):
        owner.run(NS(maintenance_recording=object()))
    assert owner.FIXED_GROUND_LIVE_ENABLED is False
    with pytest.raises(PermissionError, match='offline-only'):
        owner.run(NS(fixed_ekf_profile='/NOT_READ_OR_OPENED.json'))


def test_actual_owner_boundary_handles_one_virtual_15s_window_without_new_reader():
    s, base = session()
    assert s.begin(base)
    port = MemoryPort()
    port.read = lambda *args: pytest.fail('Second reader prohibited')
    for i in range(1500):
        now = base.now_ns+i*10000000
        obs = replace(base, now_ns=now, identity_received_ns=now,
                      attitude_received_ns=now, vicon_received_ns=now)
        if i % 10 == 0:
            token = s.gate.lock.clock.request(now-2000000)
            assert s.gate.lock.clock.ingest(reply(remote=now+4000000000-1000000,
                                                   token=token), now)
            assert s.observe_clock(at_ns=now)
        native = lock_sample((now+4000000000-5000000)//1000000)
        native['received'] = now/1e9
        s.gate.lock.status.sample = native
        source = frame(obs, i)
        assert s.observe_frame(source, callback_ns=now)
        if i % 4 == 0:
            run_write(s, obs, source, port)
    out = s.close()
    assert out['callbacks_closed_cleanly']
    assert out['completed_pose_writes'] == len(port.writes) == 375
    decoded = CrcResyncParser()
    assert all(decoded.parse_buffer(w)[0].get_type() == 'VISION_POSITION_ESTIMATE' for w in port.writes)
    assert not out['flight_authorized'] and out['parameter_writes'] == 0


@pytest.mark.parametrize('valid_noise', [True, False])
def test_existing_48byte_profile_is_preserved_only_with_explicit_equivalent_noise(valid_noise):
    import low_bandwidth_vision
    s, obs = session()
    assert s.begin(obs)
    source = frame(obs)
    assert s.observe_frame(source, callback_ns=obs.now_ns)
    params = dict(EKF2_EV_NOISE_MD=0, EKF2_EVP_NOISE=.1, EKF2_EVA_NOISE=.1)
    enc = m.MAVLink(None, srcSystem=250, srcComponent=213)
    msg = low_bandwidth_vision.message(enc, 123, [4., 5., 6.], [1., 0., 0., 0.], 0, params)[0]
    wire = bytes(msg.pack(enc))
    assert len(wire) == 48
    if not valid_noise: params['EKF2_EVP_NOISE'] = 0
    port = MemoryPort()
    times = iter([obs.now_ns+i*1000 for i in range(4)])

    def call():
        return owner._write_owner_packet(port, wire, recording=s, original=msg,
            observation=lambda ns: replace(obs, now_ns=ns), clock=lambda: next(times),
            wire_decoder=CrcResyncParser(), source_stamp_ns=source['ros_stamp_ns'],
            source_frame_id=source['frame_id'], noise_parameters=params)
    if valid_noise:
        assert call() == 48
        assert port.writes == [wire]
        assert s.close()['callbacks_closed_cleanly']
    else:
        with pytest.raises(PermissionError): call()
        assert not port.writes
        assert not s.close()['callbacks_closed_cleanly']


def test_existing_GCS_heartbeat_and_requested_TIMESYNC_use_same_owner_and_no_source_fabrication():
    s, obs = session()
    assert s.begin(obs)
    source = frame(obs)
    assert s.observe_frame(source, callback_ns=obs.now_ns)
    port = MemoryPort()
    run_write(s, obs, source, port)
    enc = m.MAVLink(None, srcSystem=250, srcComponent=213)
    hb_obs = replace(obs, now_ns=obs.now_ns+10000)
    hb = enc.heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    hb.pack(enc)
    run_write(s, hb_obs, source, port, hb)
    clock_obs = replace(obs, now_ns=obs.now_ns+20000)
    token = s.gate.lock.clock.request(clock_obs.now_ns)
    clock_msg = enc.timesync_encode(0, token)
    clock_msg.pack(enc)
    run_write(s, clock_obs, source, port, clock_msg)
    parser = CrcResyncParser()
    assert [parser.parse_buffer(p)[0].get_type() for p in port.writes] == [
        'VISION_POSITION_ESTIMATE', 'HEARTBEAT', 'TIMESYNC']
    assert s.close()['callbacks_closed_cleanly']
