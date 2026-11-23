"""New callback integration only. Every transport/clock/identity is MOCK.

No serial/ROS executor/SDK/logger. Existing conftest prevents real devices.
These tests cannot approve EV_DELAY or create a real maintenance/flight permit.
"""
from copy import deepcopy
from dataclasses import replace

import pytest
from pymavlink.dialects.v20 import common as m

import compact_vision
from ev_delay_capture import DelayRecorder
from ev_delay_maintenance import MaintenanceRecordingSession, LIVE_ENTRY_ENABLED
from test_ev_delay_sources import maintenance, sample
from test_lock_clock_probe import reply, sample as lock_sample


def session():
    gate, obs = maintenance()
    recorder = DelayRecorder('MOCK_CALLBACKS_ONLY', {'evidence': 'SYNTHETIC'})
    return MaintenanceRecordingSession(gate, recorder), obs


def message():
    enc = m.MAVLink(None, srcSystem=250, srcComponent=213)
    outbound = compact_vision.message(enc, 123, [4., 5., 6.], [1., 0., 0., 0.], 0)[0]
    outbound.pack(enc)
    return outbound


def frame(obs, index=0):
    now = obs.now_ns
    return sample(sequence=index+1, frame=100+index, stamp=1800000000000000000+index*10000000,
                  sdk_begin_ns=now-2000, sdk_end_ns=now-1000)


def sent(s, obs, msg, source):
    assert s.before_tx(msg, obs)
    decoded = m.MAVLink(None).parse_buffer(bytes(msg.get_msgbuf()))[0]
    return s.after_tx(decoded, replace(obs, now_ns=obs.now_ns+3000),
                      tx_start_ns=obs.now_ns+1000, tx_end_ns=obs.now_ns+2000,
                      callback_ns=obs.now_ns+3000,
                      source_stamp_ns=source['ros_stamp_ns'], source_frame_id=source['frame_id'])


def test_callbacks_preserve_actual_source_TX_and_clock_stamps_without_transport():
    s, obs = session()
    assert s.begin(obs)
    assert s.observe_clock(at_ns=obs.now_ns)
    source = frame(obs)
    assert s.observe_frame(source, callback_ns=obs.now_ns)
    assert sent(s, obs, message(), source)
    out = s.close()
    assert out['callbacks_closed_cleanly']
    assert out['atomic_source']['matched_sends'] == 1
    events = s.recorder.export()['events']
    written = [e for e in events if e['kind'] == 'calibration_send'][0]
    assert written['sent_ns'] == obs.now_ns+2000
    assert written['at_ns'] == obs.now_ns+3000
    assert written['frame'] == source['frame']
    assert events[0]['original_event_ns'] == 1040000000
    assert not out['physical_authorization'] and not out['flight_authorized']
    assert not out['control_ready'] and not out['live_enabled']
    assert not out['real_IO_integrated'] and not LIVE_ENTRY_ENABLED
    assert not out['calibration_performed'] and not out['parameter_approved']
    assert out['parameter_writes'] == out['devices_opened'] == 0
    before = deepcopy(s.recorder.export())
    assert s.close() == out
    assert s.recorder.export() == before
    assert not s.before_tx(message(), obs)


def test_late_FrameSample_exact_join_never_replaces_TX_time_with_callback_time():
    s, obs = session()
    assert s.begin(obs)
    source = frame(obs)
    assert sent(s, obs, message(), source)
    assert s.observe_frame(source, callback_ns=obs.now_ns+9000)
    out = s.close()
    assert out['callbacks_closed_cleanly']
    written = s.recorder.events[-1]
    assert written['kind'] == 'calibration_send'
    assert written['sent_ns'] == obs.now_ns+2000
    assert written['at_ns'] == obs.now_ns+9000


@pytest.mark.parametrize('fault', ['missing_header', 'changed_message', 'wrong_time',
                                  'no_before', 'incomplete_write'])
def test_TX_observation_faults_latch_without_retry_or_cleanup_writes(fault):
    s, obs = session()
    assert s.begin(obs)
    source = frame(obs)
    assert s.observe_frame(source, callback_ns=obs.now_ns)
    msg = message()
    if fault != 'no_before':
        assert s.before_tx(msg, obs)
    if fault == 'changed_message':
        msg.x += 1
    if fault == 'incomplete_write':
        assert not s.abort('MOCK_partial_serial_write')
    else:
        assert not s.after_tx(msg, replace(obs, now_ns=obs.now_ns+3000),
            tx_start_ns=obs.now_ns-1 if fault == 'wrong_time' else obs.now_ns+1000,
            tx_end_ns=obs.now_ns+2000, callback_ns=obs.now_ns+3000,
            source_stamp_ns=None if fault == 'missing_header' else source['ros_stamp_ns'],
            source_frame_id=source['frame_id'])
    out = s.close()
    assert not out['callbacks_closed_cleanly']
    assert out['reasons'] and out['parameter_writes'] == 0
    assert not s.begin(obs)
    assert not s.before_tx(message(), obs)


@pytest.mark.parametrize('kind', ['PARAM_SET', 'PARAM_EXT_SET', 'COMMAND_LONG',
                                 'SET_POSITION_TARGET_LOCAL_NED', 'SET_MODE', 'SERIAL_CONTROL'])
def test_forbidden_TX_never_reaches_pending_write(kind):
    from types import SimpleNamespace
    s, obs = session()
    assert s.begin(obs)
    assert not s.before_tx(SimpleNamespace(get_type=lambda: kind), obs)
    assert s.pending is None
    out = s.close()
    assert not out['callbacks_closed_cleanly'] and out['parameter_writes'] == 0
    assert not [e for e in s.recorder.events if e['kind'] == 'calibration_tx']


@pytest.mark.parametrize('change', [dict(fc_boot='REBOOT'), dict(uid='OTHER'),
    dict(gamepad_generation='NEW'), dict(attitude_received_ns=550000000),
    dict(vicon_received_ns=900000000), dict(source_valid=False), dict(log_ready=False)])
def test_original_identity_reboot_freshness_and_readiness_checks_still_apply(change):
    s, obs = session()
    assert not s.begin(replace(obs, **change))
    assert not s.close()['callbacks_closed_cleanly']
    assert not s.begin(obs)


@pytest.mark.parametrize('fault', ['clock_stale', 'armed', 'home_unknown', 'home_aborted'])
def test_original_HOME_lock_and_clock_are_not_replaced_by_recorder(fault):
    s, obs = session()
    if fault == 'clock_stale':
        s.gate.lock.clock.reference['received_ns'] = 1
    elif fault == 'armed':
        s.gate.lock.status.sample['bits'] |= 2
    elif fault == 'home_unknown':
        s.gate.home.verified = False
    else:
        s.gate.home.aborted = True
    assert not s.begin(obs)
    assert not s.close()['callbacks_closed_cleanly']


def test_window_rate_budget_and_actual_15s_callback_sequence_use_original_policy():
    s, base = session()
    assert s.begin(base)
    # SYNTHETIC 100Hz atomic frames and 25Hz VPE over a virtual 15s window.
    # No sleep, real process, network, ROS or hardware is involved.
    for i in range(1500):
        now = base.now_ns+i*10000000
        obs = replace(base, now_ns=now, identity_received_ns=now,
                      attitude_received_ns=now, vicon_received_ns=now)
        # Matched MOCK TIMESYNC exchanges, original source-state timestamps.
        if i % 10 == 0:
            token = s.gate.lock.clock.request(now-2000000)
            assert s.gate.lock.clock.ingest(reply(remote=now+4000000000-1000000,
                                                   token=token), now)
            assert s.observe_clock(at_ns=now)
        fresh = lock_sample((now+4000000000-5000000)//1000000)
        fresh['received'] = now/1e9
        s.gate.lock.status.sample = fresh
        source = frame(obs, i)
        assert s.observe_frame(source, callback_ns=now)
        if i % 4 == 0:
            assert sent(s, obs, message(), source)
    out = s.close()
    assert out['callbacks_closed_cleanly'], out['reasons']
    assert out['completed_pose_writes'] == 375
    assert out['atomic_source']['matched_sends'] == 375
    assert not out['calibration_performed'] and not out['real_IO_integrated']
    assert out['parameter_writes'] == 0 and not out['flight_permit_created']


def test_unsent_pending_empty_capture_and_queue_overflow_do_not_pass():
    s, obs = session()
    assert s.begin(obs)
    assert s.before_tx(message(), obs)
    assert not s.close()['callbacks_closed_cleanly']
    empty, obs = session()
    assert not empty.close()['callbacks_closed_cleanly']
    overflow, obs = session()
    assert overflow.begin(obs)
    overflow.recorder.capacity = 1
    assert not overflow.observe_frame(frame(obs), callback_ns=obs.now_ns)
    assert not overflow.close()['callbacks_closed_cleanly']


def test_post_write_500ms_proof_expiry_preserves_actual_send_then_stops():
    s, obs = session()
    assert s.begin(obs)
    source = frame(obs)
    assert s.observe_frame(source, callback_ns=obs.now_ns)
    msg = message()
    assert s.before_tx(msg, obs)
    after = replace(obs, now_ns=obs.now_ns+3000,
                    attitude_received_ns=obs.now_ns+3000-500000000)
    assert not s.after_tx(msg, after, tx_start_ns=obs.now_ns+1000,
        tx_end_ns=obs.now_ns+2000, callback_ns=after.now_ns,
        source_stamp_ns=source['ros_stamp_ns'], source_frame_id=source['frame_id'])
    assert any(e['kind'] == 'calibration_tx' for e in s.recorder.events)
    assert not s.close()['callbacks_closed_cleanly']


def test_original_encoder_and_decoded_float32_have_distinct_policy_roles():
    s, obs = session()
    assert s.begin(obs)
    source = frame(obs)
    assert s.observe_frame(source, callback_ns=obs.now_ns)
    original = message()
    decoded = m.MAVLink(None).parse_buffer(bytes(original.get_msgbuf()))[0]
    # The existing policy compares ORIGINAL covariance values. Do not loosen
    # it to accommodate a wrongly placed decoded-message policy call.
    assert compact_vision.pose_allowed(original)
    assert not compact_vision.pose_allowed(decoded)
    assert s.before_tx(original, obs)
    assert s.after_tx(decoded, replace(obs, now_ns=obs.now_ns+3000),
        tx_start_ns=obs.now_ns+1000, tx_end_ns=obs.now_ns+2000,
        callback_ns=obs.now_ns+3000,
        source_stamp_ns=source['ros_stamp_ns'], source_frame_id=source['frame_id'])
    assert s.close()['callbacks_closed_cleanly']


def test_existing_unknown_covariance_profile_is_not_changed_or_auto_selected():
    import low_bandwidth_vision
    s, obs = session()
    assert s.begin(obs)
    source = frame(obs)
    assert s.observe_frame(source, callback_ns=obs.now_ns)
    params = dict(EKF2_EV_NOISE_MD=0, EKF2_EVP_NOISE=.1, EKF2_EVA_NOISE=.1)
    enc = m.MAVLink(None, srcSystem=250, srcComponent=213)
    original = low_bandwidth_vision.message(enc, 123, [4., 5., 6.],
                                           [1., 0., 0., 0.], 0, params)[0]
    original.pack(enc)
    decoded = m.MAVLink(None).parse_buffer(bytes(original.get_msgbuf()))[0]
    assert s.before_tx(original, obs, noise_parameters=params)
    assert s.after_tx(decoded, replace(obs, now_ns=obs.now_ns+3000),
        tx_start_ns=obs.now_ns+1000, tx_end_ns=obs.now_ns+2000,
        callback_ns=obs.now_ns+3000,
        source_stamp_ns=source['ros_stamp_ns'], source_frame_id=source['frame_id'])
    assert s.close()['callbacks_closed_cleanly']


def test_explicit_window_end_and_overlapping_write_attempts_remain_fail_stop():
    s, obs = session()
    assert s.begin(obs)
    assert s.before_tx(message(), obs)
    assert not s.before_tx(message(), obs)
    assert not s.close()['callbacks_closed_cleanly']
    ended, obs = session()
    assert ended.begin(obs)
    at_end = obs.now_ns+15000000000
    token = ended.gate.lock.clock.request(at_end-2000000)
    assert ended.gate.lock.clock.ingest(reply(remote=at_end+4000000000-1000000,
                                              token=token), at_end)
    fresh = lock_sample((at_end+4000000000-5000000)//1000000)
    fresh['received'] = at_end/1e9
    ended.gate.lock.status.sample = fresh
    assert not ended.before_tx(message(), replace(obs, now_ns=at_end,
        identity_received_ns=at_end, attitude_received_ns=at_end, vicon_received_ns=at_end))
    assert 'maintenance_window_ended' in ended.close()['reasons']


def test_changed_wire_buffer_and_occluded_atomic_source_cannot_close_cleanly():
    s, obs = session()
    assert s.begin(obs)
    original = message()
    assert s.before_tx(original, obs)
    decoded = m.MAVLink(None).parse_buffer(bytes(original.get_msgbuf()))[0]
    decoded._msgbuf[-1] ^= 1
    assert not s.after_tx(decoded, replace(obs, now_ns=obs.now_ns+3000),
        tx_start_ns=obs.now_ns+1000, tx_end_ns=obs.now_ns+2000,
        callback_ns=obs.now_ns+3000)
    assert not s.close()['callbacks_closed_cleanly']
    bad, obs = session()
    assert bad.begin(obs)
    source = frame(obs)
    source['occluded'] = True
    assert not bad.observe_frame(source, callback_ns=obs.now_ns)
    assert not bad.close()['callbacks_closed_cleanly']
