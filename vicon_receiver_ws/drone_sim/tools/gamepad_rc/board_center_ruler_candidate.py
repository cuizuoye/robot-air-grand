#!/usr/bin/env python3
"""Offline board-center candidate from a sphere-center ruler measurement.

No ROS, serial, flight-controller writes or automatic calibration adoption.
Board geometric center is deliberately NOT identified with any IMU center.
"""
import argparse
import datetime
import hashlib
import itertools
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.frames import quaternion_to_matrix
from uav_coord_bridge.mavlink_vision import checked_quaternion, finite_vector
from rigid_body_revision import inspect_current_calibration


def estimate(marker_rigid_mm, mounting_xyzw, forward_range_mm, right_range_mm,
             height_estimate_mm):
    marker = finite_vector(marker_rigid_mm, 3)
    rotation = quaternion_to_matrix(checked_quaternion(mounting_xyzw))
    forward = finite_vector(forward_range_mm, 2)
    right = finite_vector(right_range_mm, 2)
    height = float(height_estimate_mm)
    if (not np.isfinite(height) or not 0 <= height <= 500
            or forward[0] > forward[1] or right[0] > right[1]
            or np.max(np.abs(np.r_[forward, right])) > 500):
        raise ValueError('Finite ordered millimeter ranges within 500mm required')
    # The ruler describes board -> marker in FLU: right is NEGATIVE left.
    midpoint = np.array([forward.mean(), -right.mean(), height])
    center = marker - rotation @ midpoint
    corners = np.array([marker - rotation @ np.array([x, -r, height])
                        for x, r in itertools.product(forward, right)])
    return {
        'target_reference': 'flight_controller_board_geometric_center_on_upper_surface_plane',
        'measurement_axes': 'board_center_to_marker_sphere_center; forward, right, up',
        'measured_forward_range_mm': forward.tolist(),
        'measured_right_range_mm': right.tolist(),
        'measured_height_estimate_mm': height,
        'height_uncertainty_mm': None,
        'board_to_marker_midpoint_flu_mm': midpoint.tolist(),
        'reference_marker_rigid_mm': marker.tolist(),
        'body_orientation_in_rigid_xyzw_candidate': list(mounting_xyzw),
        'formula': 'board_rigid_mm = marker_rigid_mm - R_body_to_rigid @ board_to_marker_flu_mm',
        'board_center_in_rigid_m_candidate': (center / 1000).tolist(),
        'conditional_board_center_rigid_bounds_mm': np.column_stack((corners.min(0), corners.max(0))).tolist(),
        'bounds_are_conditional_on_nominal_height_and_mounting': True,
        'uncertainty_caveat': 'Ruler ranges are not confidence intervals; height, mounting, marker correspondence and Vicon systematic uncertainty are not quantified',
        'midpoint_is_only_a_nominal_candidate': True,
        'body_origin_in_rigid': None,
        'imu_offset_verified': False,
        'extrinsics_verified': False,
        'ready_to_send_to_fc': False,
        'actual_sent_to_fc': 0,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry', type=Path, required=True)
    parser.add_argument('--heading', type=Path, required=True)
    parser.add_argument('--previous-board', type=Path, required=True)
    parser.add_argument('--marker', required=True)
    parser.add_argument('--physical-marker-correspondence-confirmed', action='store_true')
    parser.add_argument('--forward-mm', nargs=2, type=float, required=True)
    parser.add_argument('--right-mm', nargs=2, type=float, required=True)
    parser.add_argument('--height-mm', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or not args.physical_marker_correspondence_confirmed:
        parser.error('New output and an explicitly identified physical marker required')
    registry = inspect_current_calibration(args.heading, args.previous_board)
    geometry = json.loads(args.geometry.read_text())
    heading = json.loads(args.heading.read_text())
    previous = json.loads(args.previous_board.read_text())
    if geometry.get('checks_passed') is not True or geometry.get('heading_candidate') != str(args.heading.resolve()):
        parser.error('Valid geometry captured using this exact heading candidate required')
    markers = [row for row in geometry['markers'] if row['name'] == args.marker]
    if len(markers) != 1:
        parser.error('One unambiguous marker model entry required')
    result = estimate(markers[0]['rigid_xyz_mm'], heading['body_orientation_in_rigid_xyzw_candidate'],
                      args.forward_mm, args.right_mm, args.height_mm)
    delta = 1000 * (np.array(result['board_center_in_rigid_m_candidate'])
                    - finite_vector(previous['board_center_in_rigid_m_candidate'], 3))
    result.update(scope='Offline approximate board-center estimate; no IMU calibration or live adoption',
                  captured_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  rigid_body_revision_id=registry['active_revision_id'], marker_name=args.marker,
                  marker_correspondence_source='Operator-identified previously moved forward/central sphere; provisional model name retained from geometric transfer',
                  new_minus_previous_board_center_rigid_mm=delta.tolist(),
                  difference_from_previous_candidate_mm=float(np.linalg.norm(delta)),
                  saved_candidates_and_guards_modified=False,
                  sources=[dict(path=str(path.resolve()), sha256=hashlib.sha256(path.read_bytes()).hexdigest())
                           for path in (args.geometry, args.heading, args.previous_board)])
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
