"""Continuous OFFLINE integration, actual encoded bytes/parsers, synthetic state."""
from copy import deepcopy
from pathlib import Path
import hashlib
import json
import subprocess
import sys
from unittest.mock import patch

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from continuous_flight import SCENARIOS, simulate
from synthetic_mission_peer import SyntheticPeer, synthetic_baseline
from continuous_mission_runtime import ContinuousMissionRuntime
from mavlink_crc_stream import CrcResyncParser
from test_usb_combined_fusion_policy import rows
import usb_combined_fusion_policy as ground_policy


@pytest.fixture(scope='module')
def scenarios():
    return {s:simulate(s) for s in SCENARIOS}


@pytest.mark.parametrize('scenario',SCENARIOS)
def test_full_synthetic_fault_matrix(scenarios,scenario):
    result = scenarios[scenario]
    assert result['passed'], result
    assert result['devices_opened'] == result['actual_sent_to_fc'] == 0
    assert result['flight_authorized'] is False


def prepared():
    peer = SyntheticPeer()
    state = peer.prepare()
    assert not state['failure'] and peer.runtime.core.phase == 'WAIT_EXPLICIT_START', state
    return peer


def test_no_hardware_constructor_or_implicit_start():
    r = ContinuousMissionRuntime('r',body_origin_in_rigid=[0,0,0],body_orientation_in_rigid=[0,0,0,1])
    assert not hasattr(r,'port') and r.envelope.port is None and r.envelope.authorizer is None
    with pytest.raises(PermissionError):
        r.request_start(operator_confirmed=True)
    with pytest.raises(TypeError):
        ContinuousMissionRuntime('r',port=object(),body_origin_in_rigid=[0,0,0],
                                body_orientation_in_rigid=[0,0,0,1])


def test_preparation_and_wait_keep_one_home_no_auto_enable_arm_or_targets():
    p = prepared()
    for _ in range(1000):
        p.turn()
    r = p.runtime
    assert not r.failure and r.core.phase == 'WAIT_EXPLICIT_START'
    assert sum(msg.get_type()=='COMMAND_LONG' and msg.command==185 for _,msg in p.calls) == 1
    assert not any(msg.get_type()=='COMMAND_LONG' and msg.command in (176,400) for _,msg in p.calls)
    assert not any(msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED' for _,msg in p.calls)
    times = [t for t,msg in p.calls if t > 5 and msg.get_type()=='VISION_POSITION_ESTIMATE']
    assert 31 < (len(times)-1)/(times[-1]-times[0]) < 33
    assert max(b-a for a,b in zip(times,times[1:])) < .041
    assert r.coordinator.core.sink is r.coordinator.fusion.sink is r.writer
    messages = [CrcResyncParser().parse_buffer(bytes.fromhex(row['wire_hex']))[0] for row in r.frames]
    assert [msg.get_seq() for msg in messages] == [i%256 for i in range(len(messages))]
    assert all((msg.get_srcSystem(),msg.get_srcComponent())==(250,220) for msg in messages)


def test_explicit_start_requires_boolean_confirmation_and_one_current_binding():
    p = prepared()
    with pytest.raises(PermissionError):
        p.runtime.request_start()
    with pytest.raises(PermissionError):
        p.runtime.request_start(operator_confirmed='yes')
    p.runtime.request_start(operator_confirmed=True)
    with pytest.raises(PermissionError):
        p.runtime.request_start(operator_confirmed=True)


def test_enable_ack_without_new_observed_unlock_never_starts():
    p = prepared();r=p.runtime
    r.request_start(operator_confirmed=True)
    p.inspect_output()  # Synthetic peer ACKs the request.
    p.locked=True       # But no actual unlocked state follows it.
    for _ in range(350):
        p.turn()
        if r.closed:break
    assert r.enable_ack_at is not None and not r.core.arm_attempted
    assert r.failure=='output_enable_state_confirmation_timeout'
    assert not any(msg.get_type()=='SET_POSITION_TARGET_LOCAL_NED' for _,msg in p.calls)


def test_home_after_handoff_beats_malformed_rx_and_release_never_resumes():
    p=prepared();r=p.runtime
    previous=sum(r.sent_counts.values())
    r.turn(p.now+.01,rx=b'x'*2049,pad_generation='SYNTHETIC-PAD',
           pad_present=True,home_events=(True,))
    new=list(r.frames)[-(sum(r.sent_counts.values())-previous):]
    first=CrcResyncParser().parse_buffer(bytes.fromhex(new[0]['wire_hex']))[0]
    assert first.command==185 and first.param1==2
    assert r.core.stop_latched
    with pytest.raises(PermissionError):r.request_start(operator_confirmed=True)


def test_ground_only_evidence_defaults_unchanged_airborne_is_explicit():
    data=rows()
    for row in data:row['flags']['cs_in_air']=True
    original=deepcopy(data)
    assert not ground_policy.fusion_evidence_ok(data,1)
    assert ground_policy.fusion_evidence_ok(data,1,allow_in_air=True)
    assert data==original  # Never falsify the received airborne flags.
    data[-1]['flags']['cs_gps']=True
    assert not ground_policy.fusion_evidence_ok(data,1,allow_in_air=True)
    assert not ground_policy.fusion_evidence_ok(data,1,allow_in_air=1)


def test_no_restore_until_observed_landed_disarmed_and_relocked():
    p=prepared();r=p.runtime;r.request_start(operator_confirmed=True)
    for _ in range(2000):
        p.turn()
        if r.closed:break
    assert r.closed and r.core.phase=='COMPLETE' and p.params==synthetic_baseline()
    restores=[(t,msg) for t,msg in p.calls if msg.get_type()=='PARAM_SET'
              and msg.param_id=='EKF2_EV_CTRL' and msg.param_value != 0
              and t > r.start_requested_at]
    assert len(restores)==1
    at=restores[0][0]
    state=next(row for row in p.state_rows if row['at']==at)
    assert not state['armed'] and state['locked'] and state['phase']=='COMPLETE'
    assert r.cleanup_ack_at is not None and r.cleanup_ack_at < at
    assert not any(msg.get_type()=='COMMAND_LONG' and msg.command==400 and msg.param2!=0
                   for _,msg in p.calls)
    assert sum(msg.get_type()=='COMMAND_LONG' and msg.command==400 for _,msg in p.calls)==1


def test_land_timeout_is_not_success_and_does_not_restore_fusion(scenarios):
    case=scenarios['landing_timeout']
    assert case['final']['core']['phase']=='LAND_TIMEOUT'
    assert case['restoration_unconfirmed'] and not case['baseline_restored']
    assert not case['final']['core']['mission_sequence_passed_in_offline_sink']


def test_airborne_home_lock_does_not_invent_landing_or_restore(scenarios):
    case=scenarios['home_stop']
    assert case['injection_at'] is not None
    assert case['final']['core']['stop_latched']
    assert case['restoration_unconfirmed'] and not case['baseline_restored']


def test_gamepad_loss_aborts_but_does_not_disable_land_or_ground_restore(scenarios):
    case=scenarios['gamepad_loss']
    assert case['core_fault']=='gamepad_disconnected' and 21 in case['commands']
    assert case['baseline_restored']
    assert not case['final']['core']['mission_sequence_passed_in_offline_sink']
    assert case['final']['mission_inputs']['missing_conditions']


def test_late_fresh_pose_does_not_erase_already_exceeded_dispatch_gap():
    from test_flight_mission_core import airborne,observation
    core,sink=airborne()
    o=observation(core.last_vision+.16,armed=True,landed=False,lockdown=False,offboard=True)
    assert core.vision_message(o,900000000,[1,2,3],[1,0,0,0],0)
    assert core.phase=='LAND' and core.fault=='fresh_vision_dispatch_gap'
    assert sink.messages[-1].get_type()=='VISION_POSITION_ESTIMATE'


def test_actual_hardware_apis_never_called_and_incident_holds_unchanged():
    paths=list((Path(__file__).resolve().parents[1]/'known_incidents').glob('*.json'))
    before={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    with patch('serial.Serial') as serial,patch('socket.socket') as socket, \
            patch('subprocess.Popen') as process,patch('os.open') as device:
        assert simulate('nominal')['passed']
        serial.assert_not_called();socket.assert_not_called()
        process.assert_not_called();device.assert_not_called()
    assert before=={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


def test_second_thread_cannot_take_over_owner():
    from concurrent.futures import ThreadPoolExecutor
    p=prepared()
    with ThreadPoolExecutor(max_workers=1) as executor:
        future=executor.submit(p.runtime.request_cancel)
        with pytest.raises(RuntimeError,match='immutable thread owner'):
            future.result()


def test_cli_defaults_to_plan_and_has_no_hardware_execute_option():
    tool=Path(__file__).resolve().parents[1]/'continuous_flight.py'
    p=subprocess.run([sys.executable,str(tool)],capture_output=True,text=True,check=True)
    result=json.loads(p.stdout)
    assert result['mode']=='offline_plan' and result['devices_opened']==0
    assert not result['hardware_execution_available']
    for flag in ('--execute','--port','--run-battery'):
        p=subprocess.run([sys.executable,str(tool),flag],capture_output=True,text=True)
        assert p.returncode==2 and 'unrecognized arguments' in p.stderr
