"""Only the new installed-prop USB-isolated branch. All devices are simulated."""
from dataclasses import replace
import json
from types import SimpleNamespace as NS

import pytest
import ev_delay_joint_capture as live
from ev_delay_maintenance import MaintenanceCapture
from test_ev_delay_sources import maintenance
from test_ev_delay_joint_device import execute, Port


INSTALLED = dict(confirm_props_off_battery_disconnected_usb_only=False,
    confirm_props_installed_battery_disconnected_usb_only=True,
    confirm_motor_power_isolated=True, confirm_independent_usb_cutoff=True,
    authorization_id='SYNTHETIC_ONLY')


@pytest.mark.parametrize('change', [
    {'confirm_props_installed_battery_disconnected_usb_only': False},
    {'confirm_props_off_battery_disconnected_usb_only': True},
    {'confirm_motor_power_isolated': False},
    {'confirm_independent_usb_cutoff': False},
    {'authorization_id': ''},
])
def test_missing_or_contradictory_confirmation_opens_nothing(tmp_path, monkeypatch, change):
    monkeypatch.setattr(live, 'RUNS', tmp_path)
    with pytest.raises(PermissionError):
        live.execute(NS(**(INSTALLED | change)), {}, {})
    assert not list(tmp_path.iterdir())


def installed_observation(obs):
    return replace(obs, props_removed=False, props_installed=True,
        battery_disconnected=True, usb_only=True, motor_power_isolated=True)


@pytest.mark.parametrize('field', [
    'battery_disconnected', 'usb_only', 'motor_power_isolated', 'independent_cutoff_ready',
])
def test_power_condition_loss_latches_during_capture(field):
    gate, obs = maintenance()
    obs = installed_observation(obs)
    assert gate.begin(obs)
    assert not gate.check(replace(obs, **{field: False}))
    assert gate.state == 'FAILED'
    assert not gate.check(obs)


@pytest.mark.parametrize('fault', ['HOME', 'armed', 'lock', 'stale_pose'])
def test_installed_props_do_not_bypass_existing_guard(fault):
    gate, obs = maintenance()
    obs = installed_observation(obs)
    if fault == 'HOME': gate.home.verified = False
    elif fault == 'armed': gate.lock.status.sample['bits'] |= 2
    elif fault == 'lock': gate.lock.status.sample['bits'] &= ~4
    elif fault == 'stale_pose': obs = replace(obs, vicon_received_ns=1)
    assert not gate.begin(obs)


def test_installed_props_flow_keeps_truthful_report_and_existing_whitelist(tmp_path, monkeypatch):
    seen = []
    original = MaintenanceCapture.begin
    def begin(self, obs):
        seen.append(obs)
        return original(self, obs)
    monkeypatch.setattr(MaintenanceCapture, 'begin', begin)
    sim, code, report = execute(tmp_path, monkeypatch, physical_overrides=INSTALLED)
    assert code == 0, report
    assert seen and all(o.props_installed and not o.props_removed for o in seen)
    for physical in (report['physical_conditions'], report['owner']['physical_conditions']):
        assert physical['props_removed'] is False and physical['props_installed'] is True
    raw = json.loads(next(tmp_path.glob('joint_ev_delay_*/capture_raw.json')).read_text())
    assert raw['capture_metadata']['physical_conditions']['props_removed'] is False
    messages = [msg for port in sim.ports for _, msg in port.sent]
    assert not any(msg.get_type() in ('PARAM_SET', 'PARAM_EXT_SET', 'RC_CHANNELS_OVERRIDE',
        'MANUAL_CONTROL', 'SET_MODE', 'SET_POSITION_TARGET_LOCAL_NED', 'SET_ATTITUDE_TARGET') for msg in messages)
    assert all(msg.command in (185, 510, 511, 512) for msg in messages if msg.get_type() == 'COMMAND_LONG')
    assert all(msg.param1 == 2 for msg in messages if msg.get_type() == 'COMMAND_LONG' and msg.command == 185)
    assert sim.on_count == sim.off_count == 1 and all(p.closed == 1 for p in sim.ports)
    assert not report['calibration_approved'] and not report['flight_authorized']


def test_detected_battery_voltage_stops_installed_props_capture(tmp_path, monkeypatch):
    original = Port.telemetry
    def telemetry(self):
        original(self)
        sim = self.sim
        if self.route == 1 and sim.first_pose and sim.now-sim.first_pose > 600_000_000 and not sim.injected:
            self.append(self.fc.sys_status_encode(0, 0, 0, 0, 12000, 0, 0, 0, 0, 0, 0, 0, 0))
            sim.injected = True
    monkeypatch.setattr(Port, 'telemetry', telemetry)
    sim, code, report = execute(tmp_path, monkeypatch, physical_overrides=INSTALLED)
    assert code == 2 and not report['hardware_capture_completed']
    assert 'Propulsion battery voltage detected' in report['pump_failure']
    assert sim.last_pose-sim.first_pose < 1_000_000_000
    assert sim.on_count == sim.off_count == 1 and all(p.closed == 1 for p in sim.ports)


def test_confirmation_without_run_remains_local_only(monkeypatch, capsys):
    monkeypatch.setattr(live, 'local_inputs', lambda: ({'revision': 'MOCK'}, {}))
    def no_execute(*args): raise AssertionError('Default CLI must not execute')
    monkeypatch.setattr(live, 'execute', no_execute)
    assert live.main(['--confirm-props-installed-battery-disconnected-usb-only']) == 0
    assert json.loads(capsys.readouterr().out)['mode'] == 'local_only_no_devices'


def test_cli_rejects_incomplete_run_before_local_setup(monkeypatch):
    def no_setup(): raise AssertionError('Incomplete confirmation must be rejected first')
    monkeypatch.setattr(live, 'local_inputs', no_setup)
    with pytest.raises(SystemExit) as exc:
        live.main(['--run', '--authorization-id', 'MOCK', '--confirm-props-installed-battery-disconnected-usb-only'])
    assert exc.value.code == 2
