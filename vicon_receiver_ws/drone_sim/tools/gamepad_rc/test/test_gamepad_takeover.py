"""Only the new takeover regressions; byte-encoded peers, no FC/HID access."""
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import gamepad_rc as g
from estop_only_policy import DurableStopLatch
from gamepad_estop_monitor import StopLatch
from flight_mission_core import Observation
from mission_live_inputs import LiveMissionInputs
from continuous_mission_runtime import ContinuousMissionRuntime
from test_flight_mission_core import airborne, observation, selector, ack
from test_mission_live_inputs import packet, identity, telemetry


def pad(keys=()):
    c = json.loads((Path(g.__file__).parent/'gamepad_calib.json').read_text())
    return c, NS(axes={g.CODE_OF[k]: v['center'] for k, v in c['axes'].items()},
                 held=lambda key: key in keys)


def peer(monkeypatch, core=None):
    clock = [10.]
    monkeypatch.setattr(g.time, 'monotonic', lambda: clock[0])
    sent = []
    class Wire:
        def write(self, data):
            msg = m.MAVLink(None).parse_char(data)
            assert msg is not None
            if core and msg.get_type() == 'COMMAND_LONG' and msg.command == 176:
                assert core.pilot_control  # Yield before the first command byte.
            sent.append(msg)
            return len(data)
    link = g.Link('offline', dry_run=True, mission_core=core)
    link.dry_run = False
    link.mavutil = NS(mavlink=m)
    link.mav = NS(mav=m.MAVLink(Wire(), srcSystem=250, srcComponent=0),
                  source_component=0, recv_match=lambda **kw: None)
    link._keepalive = NS(tick=lambda: None)
    return link, clock, sent


def fc_ack(result=0, system=1, target=250, component=0):
    return packet(m.MAVLink(None).command_ack_encode(176, result,
                  target_system=target, target_component=component), system=system)


def fc_mode(parameter=2, armed=True):
    return packet(m.MAVLink(None).heartbeat_encode(2, 12, 128 if armed else 0,
                  g.px4_modes.custom_mode_from_param(parameter), 0))


def test_short_home_tap_in_one_read_persists_through_release_and_restart(tmp_path):
    latch_path = tmp_path/'stop.json'
    latch = DurableStopLatch(latch_path, g.UID2)
    readfd, writefd = os.pipe()
    device = g.Gamepad.__new__(g.Gamepad)
    device.fd, device.stop_latch = readfd, latch
    device.axes, device.raw_buttons, device.key_edges = {}, set(), []
    try:
        os.write(writefd, b''.join(g.struct.pack(g.FMT, 0, 0, 1, 0x13c, v) for v in (1, 0)))
        assert device.poll(0)
        assert not device.held('HOME') and latch.latched
        c, idle = pad()
        assert g.Mapper(c, latch).channels(idle)[6] == 2000
        restarted = DurableStopLatch(latch_path, g.UID2)
        assert g.Mapper(c, restarted).channels(idle)[6] == 2000
        latch_path.write_text('damaged retained file')
        assert g.Mapper(c, DurableStopLatch(latch_path, g.UID2)).channels(idle)[6] == 2000
    finally:
        device.close()
        os.close(writefd)


def test_home_held_at_open_is_read_before_first_frame(monkeypatch):
    readfd, writefd = os.pipe()
    monkeypatch.setattr(g.os, 'open', lambda *a, **kw: readfd)
    monkeypatch.setattr(g.Gamepad, 'read_live_axes', lambda self: {})
    bits = bytearray(96); bits[0x13c//8] |= 1 << (0x13c%8)
    monkeypatch.setattr(g.fcntl, 'ioctl', lambda *a: bytes(bits))
    latch = StopLatch()
    device = g.Gamepad('fake', latch)
    try:
        assert device.held('HOME') and latch.latched
    finally:
        device.close(); os.close(writefd)


def test_home_wins_over_mode_and_disabled_arm_offboard_buttons():
    c, held = pad(('HOME', 'A', 'L1', 'B', 'PLUS', 'Y'))
    mapper = g.Mapper(c)
    channels = mapper.channels(held)
    assert channels[5:8] == [1000, 2000, 1000]
    assert mapper.mode_request is None
    _, idle = pad()
    assert mapper.channels(idle)[6] == 2000


@pytest.mark.parametrize('key,parameter', [('A', 2), ('L1', 1), ('B', 11)])
def test_button_sends_explicit_mode_even_with_same_slot(monkeypatch, key, parameter):
    link, clock, sent = peer(monkeypatch)
    c, pressed = pad((key,)); _, idle = pad()
    mapper = g.Mapper(c)
    for i in range(2):
        mapper.channels(pressed)
        assert mapper.mode_request == parameter
        assert link.request_mode(mapper.mode_request)
        message = sent[-1]
        main, sub = g.px4_modes.PARAM_TO_MODE[parameter]
        assert message.command == 176
        assert tuple(getattr(message, f'param{k}') for k in range(1, 8)) == (1, main, sub, 0, 0, 0, 0)
        link.observe_mode(fc_ack())
        assert link.mode_request['state'] == 'WAITING'
        link.observe_mode(fc_mode(parameter))
        assert link.mode_request['state'] == 'CONFIRMED'
        mapper.channels(idle); clock[0] += .1
    assert len(sent) == 2


def test_ack_and_mode_must_both_match_current_request(monkeypatch):
    link, clock, sent = peer(monkeypatch)
    assert link.request_mode(2)
    assert not link.request_mode(1)  # Do not overlap command 176 transactions.
    link.observe_mode(fc_mode(2))
    for msg in (fc_ack(system=2), fc_ack(target=251), fc_ack(component=220), fc_ack(result=5)):
        link.observe_mode(msg)
        assert link.mode_request['state'] == 'WAITING'
    clock[0] += 1.1  # Old matching heartbeat is insufficient when ACK arrives later.
    link.observe_mode(fc_ack())
    assert link.mode_request['state'] == 'WAITING'
    link.observe_mode(fc_mode(1))
    assert link.mode_request['state'] == 'WAITING'
    link.observe_mode(fc_mode(2))
    assert link.mode_request['state'] == 'CONFIRMED' and len(sent) == 1


@pytest.mark.parametrize('failure', ['reject', 'timeout', 'send_error'])
def test_failed_mode_request_never_resumes_mission(monkeypatch, failure):
    core, sink = airborne()
    link, clock, sent = peer(monkeypatch, core)
    if failure == 'send_error':
        def fail(*a): raise OSError('uncertain delivery')
        monkeypatch.setattr(link.mav.mav, 'command_long_send', fail)
        with pytest.raises(OSError): link.request_mode(2)
        assert link.mode_request['state'] == 'DELIVERY_UNCERTAIN'
    else:
        link.request_mode(2)
        if failure == 'reject': link.observe_mode(fc_ack(result=2))
        else:
            clock[0] += 3.1; link.check_mode_timeout()
        assert link.mode_request['state'] == ('REJECTED' if failure == 'reject' else 'TIMEOUT')
    before = len(sink.messages)
    core.tick(observation(5., armed=True, offboard=True), selector(), boot_ms=5000)
    core._abort('late_failure', observation(5.1, armed=True))
    core._land(observation(5.2, armed=True))
    assert not ack(core, 176, 5.3)
    assert len(sink.messages) == before and core.phase == 'PILOT_CONTROL'
    with pytest.raises(ValueError): core.start(observation(5.4), selector(), operator_confirmed=True)


def test_home_cancels_pending_mode_and_link_cannot_send_unkill(monkeypatch):
    link, clock, sent = peer(monkeypatch)
    link.request_mode(2)
    c, home = pad(('HOME',)); _, idle = pad()
    link.send_rc(g.Mapper(c).channels(home))
    assert link.mode_request['state'] == 'CANCELLED_BY_HOME'
    assert not link.request_mode(1)
    link.send_rc(g.Mapper(c).channels(idle))
    assert sent[-1].chan7_raw == 2000 and sent[-1].chan6_raw == 1000


@pytest.mark.parametrize('parameter', [1, 2])
def test_actual_manual_mode_yields_even_with_degraded_navigation(parameter):
    core, sink = airborne()
    state = LiveMissionInputs('offline-rigid')
    identity(state); telemetry(state, 4.5, armed=True, main_mode=6)
    state.ingest(fc_mode(parameter), 4.5)
    state.actuator = dict(at=4.5, timestamp_us=14500000, armed=True, lockdown=False)
    host = NS(inputs=state, pilot_source='gamepad', _fc_epoch_usable=lambda: True)
    o = ContinuousMissionRuntime._degraded_observation(host, 4.5)
    assert o.manual_control_active and not o.navigation_ok()
    before = len(sink.messages)
    core.cycle(o, {}, boot_ms=14500)
    assert core.phase == 'PILOT_CONTROL' and len(sink.messages) == before
    # Once yielded, late state changes and invalid navigation cannot take it back.
    core.tick(observation(5., armed=True, offboard=True), {}, boot_ms=15000)
    assert len(sink.messages) == before


def test_missing_or_stale_manual_evidence_keeps_fault_landing():
    state = LiveMissionInputs('offline-rigid'); identity(state)
    state.ingest(fc_mode(2), 1.)
    assert state.manual_mode_confirmed(1.501)
    assert not state.manual_mode_confirmed(3.)
    core, sink = airborne()
    core.tick(observation(4.5, armed=True, offboard=False), selector(), boot_ms=4500)
    assert core.phase == 'LAND' and sink.messages[-1].command == 21


def test_home_still_works_after_takeover_and_vision_support_is_preserved():
    core, sink = airborne()
    core.request_takeover()
    before = len(sink.messages)
    o = observation(4.5, armed=True, offboard=False, lockdown=False)
    core.cycle(o, selector(), boot_ms=4500, vision=(105000000, [1., -2., -3.], [1., 0., 0., 0.], 0.))
    assert [x.get_type() for x in sink.messages[before:]] == ['VISION_POSITION_ESTIMATE']
    with pytest.raises(PermissionError): core._emit(core._command(21, 4.5), 4.5)
    core.tick(observation(4.6, armed=True, home_pressed=True), {}, boot_ms=4600)
    assert core.stop_latched and sink.messages[-1].command == 185 and sink.messages[-1].param1 == 2


def test_main_exits_on_input_loss_without_reconnection(monkeypatch):
    monkeypatch.setattr(sys, 'argv', ['gamepad_rc.py', '--monitor', '--no-wait-stable'])
    monkeypatch.setattr(g, 'find_device', lambda *a: {'path':'offline'})
    calls = []
    monkeypatch.setattr(g, 'Gamepad', lambda *a, **kw: NS(poll=lambda t: False, close=lambda: calls.append('close')))
    monkeypatch.setattr(g, 'wait_stable', lambda *a: pytest.fail('Must not reconnect'))
    assert g.main() == 1 and calls == ['close']
