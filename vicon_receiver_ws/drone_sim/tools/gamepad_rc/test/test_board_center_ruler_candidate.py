import math
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from board_center_ruler_candidate import estimate


def test_right_sign_subtraction_and_millimeters():
    result = estimate([100, 0, 10], [0, 0, 0, 1], [70, 80], [1, 2], 6)
    assert np.allclose(result['board_to_marker_midpoint_flu_mm'], [75, -1.5, 6])
    assert np.allclose(result['board_center_in_rigid_m_candidate'], [.025, .0015, .004])
    assert np.allclose(result['conditional_board_center_rigid_bounds_mm'], [[20, 30], [1, 2], [4, 4]])


def test_rotated_axes_and_explicit_uncertainty():
    result = estimate([100, 0, 10], [0, 0, math.sqrt(.5), math.sqrt(.5)], [70, 80], [1, 2], 6)
    assert np.allclose(result['board_center_in_rigid_m_candidate'], [.0985, -.075, .004])
    assert np.allclose(result['conditional_board_center_rigid_bounds_mm'], [[98, 99], [-80, -70], [4, 4]])
    assert result['height_uncertainty_mm'] is None
    assert result['bounds_are_conditional_on_nominal_height_and_mounting'] is True


@pytest.mark.parametrize('forward,right,height', [
    ([80, 70], [1, 2], 6), ([70, 80], [2, 1], 6),
    ([float('nan'), 80], [1, 2], 6), ([70, 80], [1, float('inf')], 6),
    ([70, 80], [1, 2], -1), ([70, 80], [1, 2], float('nan')),
    ([70, 80000], [1, 2], 6), ([70], [1, 2], 6),
])
def test_invalid_measurements_rejected(forward, right, height):
    with pytest.raises(ValueError):
        estimate([100, 0, 10], [0, 0, 0, 1], forward, right, height)


def test_no_imu_or_live_permission_inferred():
    result = estimate([100, 0, 10], [0, 0, 0, 1], [70, 80], [1, 2], 6)
    assert result['body_origin_in_rigid'] is None
    assert result['imu_offset_verified'] is result['extrinsics_verified'] is False
    assert result['ready_to_send_to_fc'] is False
    assert result['actual_sent_to_fc'] == 0


def test_invalid_rotation_or_marker_rejected():
    for marker, q in [([float('nan'), 0, 0], [0, 0, 0, 1]), ([0, 0, 0], [0, 0, 0, 0])]:
        with pytest.raises(ValueError):
            estimate(marker, q, [70, 80], [1, 2], 6)
