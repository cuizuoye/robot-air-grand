import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vicon_fc_attitude_check import attitude_comparison, nose_up_observation, right_side_up_observation


def test_world_yaw_difference_is_not_tilt_error():
    result = attitude_comparison([1, 0, 0, 0], [math.sqrt(.5), 0, 0, math.sqrt(.5)])
    assert result['gravity_axis_difference_deg'] == pytest.approx(0)
    assert result['fc_minus_vicon_wrapped_deg'][2] == pytest.approx(90)


def test_roll_error_detected():
    angle = math.radians(10)
    result = attitude_comparison([1, 0, 0, 0], [math.cos(angle / 2), math.sin(angle / 2), 0, 0])
    assert result['gravity_axis_difference_deg'] == pytest.approx(10)
    assert np.allclose(result['fc_minus_vicon_wrapped_deg'], [10, 0, 0])


def test_antipodal_quaternion_same_attitude():
    assert attitude_comparison([1, 0, 0, 0], [-1, 0, 0, 0])['gravity_axis_difference_deg'] == pytest.approx(0)


def test_invalid_quaternion_rejected():
    with pytest.raises(ValueError):
        attitude_comparison([1, 0, 0, 0], [0, 0, 0, 0])


def test_yaw_wrap_uses_shortest_difference():
    def yaw(angle):
        return [math.cos(math.radians(angle) / 2), 0, 0, math.sin(math.radians(angle) / 2)]
    assert attitude_comparison(yaw(179), yaw(-179))['fc_minus_vicon_wrapped_deg'][2] == pytest.approx(2)


def capture(pitch, seconds=0, roll=0):
    return {'static_observation_passed': True, 'identity': {'uid2': 'same'}, 'heading_candidate': 'same',
            'captured_at_utc': f'2026-10-13T03:00:{seconds:02d}+00:00',
            'last_pair': {'fc_boot_ms': seconds * 1000},
            'pairs': [{'vicon_roll_pitch_yaw_deg': [roll, pitch, 0],
                       'fc_roll_pitch_yaw_deg': [roll, pitch, 80]}]}


def test_small_pitch_same_sign_is_not_full_calibration():
    result = nose_up_observation(capture(0), capture(3, 10))
    assert result['pitch_sign_observation_passed']
    assert not result['both_pitch_changes_at_least_5_deg']
    assert result['quality'] == 'limited_small_angle_observation'
    assert result['dynamic_axes_verified'] is result['ready_to_send_to_fc'] is False


@pytest.mark.parametrize('pitch,roll', [(0, 0), (-10, 0), (10, 15)])
def test_no_pitch_wrong_sign_or_roll_dominant_does_not_pass(pitch, roll):
    assert not nose_up_observation(capture(0), capture(pitch, 10, roll))['pitch_sign_observation_passed']


def test_fc_reboot_and_identity_change_rejected():
    raised = capture(10, 10)
    raised['identity']['uid2'] = 'different'
    with pytest.raises(ValueError):
        nose_up_observation(capture(0), raised)
    raised = capture(10, 10)
    raised['last_pair']['fc_boot_ms'] = 0
    with pytest.raises(ValueError):
        nose_up_observation(capture(0), raised)


def test_failed_capture_rejected():
    raised = capture(10, 10)
    raised['static_observation_passed'] = False
    with pytest.raises(ValueError):
        nose_up_observation(capture(0), raised)


def test_right_side_up_requires_negative_frd_roll_and_preserves_safety_flags():
    result = right_side_up_observation(capture(0), capture(1, 10, -8))
    assert result['roll_sign_observation_passed']
    assert result['both_roll_changes_at_least_5_deg']
    assert result['endpoint_translation_mm'] is None
    assert result['dynamic_axes_verified'] is result['pure_axis_rotation_verified'] is False
    assert result['extrinsics_verified'] is result['ready_to_send_to_fc'] is False
    assert result['actual_sent_to_fc'] == 0


@pytest.mark.parametrize('roll,pitch', [(8, 1), (0, 0), (-8, 12), (-40, 1)])
def test_wrong_sign_no_motion_pitch_dominance_or_excessive_roll_fails(roll,pitch):
    assert not right_side_up_observation(capture(0),capture(pitch,10,roll))['roll_sign_observation_passed']


def test_yaw_comotion_is_retained_without_invalidating_qualitative_roll_sign():
    baseline,raised=capture(0),capture(1,10,-8)
    raised['pairs'][0]['vicon_roll_pitch_yaw_deg'][2]=67
    raised['pairs'][0]['fc_roll_pitch_yaw_deg'][2]=147
    result=right_side_up_observation(baseline,raised)
    assert result['roll_sign_observation_passed']
    assert result['substantial_yaw_co_motion']
    assert result['yaw_change_deg']==pytest.approx({'vicon':67,'fc':67})
    assert result['warnings'] and not result['pure_axis_rotation_verified']


def test_right_side_observation_keeps_identity_reboot_and_capture_gates():
    for field,value in [('identity',{'uid2':'other'}),('static_observation_passed',False),
                        ('heading_candidate','other'),('last_pair',{'fc_boot_ms':0})]:
        raised=capture(0,10,-8);raised[field]=value
        with pytest.raises(ValueError): right_side_up_observation(capture(0),raised)
