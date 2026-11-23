"""Offline same-body frame migration and an explicit live-injection revision gate.

Physical forward is inherited, not redefined. Four fixed marker positions and
the operator-described motion select a PROVISIONAL registration. This is not
an IMU calibration, dynamic pose validation, or permission for flight.
"""
import hashlib
import itertools
import json
import math
from pathlib import Path
import sys

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.frames import quaternion_to_matrix, matrix_to_quaternion
from uav_coord_bridge.mavlink_vision import checked_quaternion, finite_vector

REGISTRY = Path(__file__).resolve().parent / 'known_incidents/mircoair_rigid_body_revision.json'
MEASURED_CAPTURE_SCOPE = 'measured-layout-motor-isolated-joint-capture-only-v1'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def inspect_frame_transfer(ref, baseline, current):
    """Check an explicit rigid-frame rebase without rewriting historical proof.

    This preserves the old physical body reference. It is neither a fresh IMU
    calibration nor permission to send poses or fly.
    """
    if digest(ref['path']) != ref['sha256']:
        raise ValueError('Rigid-frame transfer evidence changed')
    proof = json.loads(Path(ref['path']).read_text())
    if (proof.get('scope') != 'same-body-rigid-frame-transfer-v1'
            or proof.get('flight_authorized') is not False
            or proof.get('old_revision') != baseline['revision']
            or proof.get('new_revision') != current['revision']):
        raise ValueError('Rigid-frame transfer scope/revision mismatch')
    for source in proof['sources'].values():
        if digest(source['path']) != source['sha256']:
            raise ValueError('Rigid-frame transfer source changed')
    rotation = np.asarray(proof['rotation_old_rigid_to_new_rigid'], dtype=float)
    translation = finite_vector(proof['translation_old_rigid_to_new_rigid_m'], 3)
    if (rotation.shape != (3, 3) or not np.isfinite(rotation).all()
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-9, rtol=0)
            or abs(float(np.linalg.det(rotation))-1) > 1e-9):
        raise ValueError('Rigid-frame transfer must be a proper rigid rotation')
    expected_origin = rotation @ finite_vector(baseline['origin'], 3) + translation
    expected_orientation = rotation @ quaternion_to_matrix(checked_quaternion(baseline['orientation']))
    if (not np.allclose(current['origin'], expected_origin, atol=1e-10, rtol=0)
            or not np.allclose(quaternion_to_matrix(checked_quaternion(current['orientation'])),
                               expected_orientation, atol=1e-10, rtol=0)):
        raise ValueError('Current extrinsics differ from recorded rigid-frame transfer')
    return proof


def measured_capture_inputs(evidence_path, *, registry_path=None):
    """Inspect historical installation evidence for the isolated collector ONLY.

    No registration transfer, global adoption, fusion proof or flight permission
    is manufactured. The live collector still requires current HOME/lock, fresh
    source/FC attitude, boot binding, isolated motor power and local GO.
    """
    from vicon_heading_calibration import solve_heading
    from vicon_fc_attitude_check import nose_up_observation, right_side_up_observation
    from board_center_ruler_candidate import estimate
    from receive_only_permit import UID

    evidence_path = Path(evidence_path).resolve()
    evidence = json.loads(evidence_path.read_text())
    registry = json.loads(Path(registry_path or REGISTRY).read_text())
    revision_id = registry['active_revision_id']
    if (evidence.get('scope') != MEASURED_CAPTURE_SCOPE
            or evidence.get('rigid_body_revision_id') != revision_id
            or evidence.get('flight_authorized') is not False
            or evidence.get('calibration_approved') is not False):
        raise ValueError('Measured-layout evidence scope/revision invalid')
    required = {'heading', 'board', 'geometry', 'measurement', 'start', 'end',
                'level', 'nose_up', 'right_up', 'level_rx', 'nose_up_rx', 'right_up_rx'}
    refs = evidence.get('files', {})
    if set(refs) != required:
        raise ValueError('Complete measured-layout evidence files required')
    paths = {}
    for name, ref in refs.items():
        path = Path(ref['path'])
        if not path.is_absolute() or digest(path) != ref['sha256']:
            raise ValueError('Measured-layout input hash/path mismatch: '+name)
        paths[name] = path.resolve()
    docs = {name: json.loads(path.read_text()) for name, path in paths.items()
            if not name.endswith('_rx')}
    h, b, measurement = (docs[n] for n in ('heading', 'board', 'measurement'))
    for name in ('heading', 'board'):
        if (Path(registry['pending_'+name+'_candidate_path']).resolve() != paths[name]
                or registry['pending_'+name+'_candidate_sha256'] != digest(paths[name])
                or docs[name].get('rigid_body_revision_id') != revision_id
                or docs[name].get('ready_to_send_to_fc') is not False):
            raise ValueError('Current pending candidate binding required')
    if h.get('manual_forward_horizontal_confirmed') is not True or b.get('imu_offset_verified') is not False:
        raise ValueError('Explicit measured-forward and approximate-origin labels required')
    if [Path(p).resolve() for p in h.get('source_files', [])] != [paths['start'], paths['end']]:
        raise ValueError('Heading endpoint binding mismatch')
    solved = solve_heading(docs['start'], docs['end'], forward_horizontal_confirmed=True)
    mount = checked_quaternion(h['body_orientation_in_rigid_xyzw_candidate'])
    solved_mount = checked_quaternion(solved['body_orientation_in_rigid_xyzw_candidate'])
    if (abs(float(mount @ solved_mount)) < 1-1e-10
            or h.get('orientation_candidate_quality') != solved['orientation_candidate_quality']):
        raise ValueError('Heading differs from recorded endpoints/quality')
    if measurement.get('board_still_centered_confirmed') is not True:
        raise ValueError('Board-center physical reference required')
    measured = finite_vector(measurement['forward_left_up_mm'], 3)
    markers = docs['geometry']['markers']
    selected = [m for m in markers if m['name'] == b.get('marker_name')]
    rotation = quaternion_to_matrix(mount)
    forwards = sorted((float(finite_vector(m['rigid_xyz_mm'], 3) @ rotation[:, 0]), m['name'])
                      for m in markers)
    if (len(markers) != 5 or len({m['name'] for m in markers}) != 5
            or any(m.get('occluded') is not False or m.get('result') != 2 for m in markers)
            or len(selected) != 1 or forwards[-1][1] != b.get('marker_name')
            or forwards[-1][0]-forwards[-2][0] < 10
            or measurement.get('marker_model_name_candidate') != b.get('marker_name')):
        raise ValueError('Unambiguous nose-marker correspondence required')
    nominal = estimate(selected[0]['rigid_xyz_mm'], mount,
        [measured[0]]*2, [-measured[1]]*2, measured[2])
    offset = finite_vector(b['board_center_in_rigid_m_candidate'], 3)
    if not np.allclose(offset, nominal['board_center_in_rigid_m_candidate'], atol=1e-9, rtol=0):
        raise ValueError('Board offset differs from measured geometry')
    identity = docs['level'].get('identity')
    for name in ('level', 'nose_up', 'right_up'):
        report = docs[name]
        if (report.get('static_observation_passed') is not True or report.get('error')
                or report.get('identity') != identity or not identity
                or identity.get('uid2') != UID or identity.get('firmware') != '1.14.3'
                or Path(report['heading_candidate']).resolve() != paths['heading']
                or report.get('actual_sent_to_fc') != 0 or not report.get('pairs')
                or not report.get('checks') or not all(v is True for v in report['checks'].values())
                or report.get('rx_evidence', {}).get('crc_bypass') is not False
                or Path(report['rx_evidence']['path']).resolve() != paths[name+'_rx']
                or not report.get('heartbeats') or any(r['armed'] for r in report['heartbeats'])
                or set(report.get('tx_messages', {})) - {'HEARTBEAT', 'AUTOPILOT_VERSION_REQUEST'}):
            raise ValueError('Passing read-only same-candidate attitude evidence required: '+name)
    pitch = nose_up_observation(docs['level'], docs['nose_up'])
    roll = right_side_up_observation(docs['level'], docs['right_up'])
    if not (pitch['pitch_sign_observation_passed'] and pitch['both_pitch_changes_at_least_5_deg']
            and roll['roll_sign_observation_passed'] and roll['both_roll_changes_at_least_5_deg']):
        raise ValueError('Measured-layout held-axis observations failed')
    return dict(heading=paths['heading'], board=paths['board'], proof=evidence_path,
        revision=revision_id, origin=offset.tolist(), orientation=mount.tolist(),
        measured_layout=True, evidence_sha256=digest(evidence_path),
        registry_sha256=digest(registry_path or REGISTRY),
        evidence_scope=MEASURED_CAPTURE_SCOPE, heading_quality=h['orientation_candidate_quality'],
        calibration_approved=False, flight_authorized=False)


def solve_registration(old_markers_mm, new_markers_mm, old_mount_xyzw, new_world_xyzw,
                       *, only_old_marker_four_moved_confirmed=False):
    if not only_old_marker_four_moved_confirmed:
        raise ValueError('Operator confirmation of exactly one moved physical marker required')
    old, new = np.asarray(old_markers_mm, dtype=float), np.asarray(new_markers_mm, dtype=float)
    if old.shape != (5, 3) or new.shape != (5, 3) or not np.isfinite([old, new]).all():
        raise ValueError('Two finite five-marker constellations required')
    mount = quaternion_to_matrix(checked_quaternion(old_mount_xyzw))
    world = quaternion_to_matrix(checked_quaternion(new_world_xyzw))
    fixed = [0, 1, 2, 4]  # old physical elevated marker was named MircoAir4.
    a = old[fixed]
    if np.linalg.svd(a - a.mean(0), compute_uv=False)[1] < 1:
        raise ValueError('Non-collinear fixed marker geometry required')
    all_rows, plausible = [], []
    for permutation in itertools.permutations(range(5), 4):
        b = new[list(permutation)]
        u, _, vt = np.linalg.svd((a - a.mean(0)).T @ (b - b.mean(0)))
        handedness = np.eye(3)
        handedness[2, 2] = np.linalg.det(vt.T @ u.T)
        rotation = vt.T @ handedness @ u.T
        translation = b.mean(0) - rotation @ a.mean(0)
        errors = np.linalg.norm(a @ rotation.T + translation - b, axis=1)
        moved = next(i for i in range(5) if i not in permutation)
        motion_flu = mount.T @ (rotation.T @ (new[moved] - translation) - old[3])
        up_dot = float((world @ rotation @ mount)[2, 2])
        row = {'old_fixed_names': [f'MircoAir{i+1}' for i in fixed],
               'new_matching_names': [f'MircoAir{i+1}' for i in permutation],
               'new_moved_marker_name_candidate': f'MircoAir{moved+1}',
               'rotation_old_rigid_to_new_rigid': rotation.tolist(),
               'translation_old_rigid_to_new_rigid_mm': translation.tolist(),
               'fixed_marker_fit_rms_mm': float(np.sqrt(np.mean(errors**2))),
               'fixed_marker_fit_max_mm': float(errors.max()),
               'moved_marker_forward_left_up_mm': motion_flu.tolist(),
               'body_up_world_up_dot': up_dot}
        all_rows.append(row)
        # Broad, labelled operator-approximation bounds, NOT precision tolerances.
        if (row['fixed_marker_fit_rms_mm'] <= 3 and up_dot > .95
                and 10 <= motion_flu[0] <= 70 and abs(motion_flu[1]) <= 15
                and -50 <= motion_flu[2] <= -10):
            plausible.append(row)
    if len(plausible) != 1:
        raise ValueError('No unique registration consistent with fixed points, upright body and reported forward/downward relocation')
    return {**plausible[0], 'hypotheses_evaluated': len(all_rows),
            'operator_motion_bounds_forward_left_up_mm': [[10, 70], [-15, 15], [-50, -10]],
            'operator_approximation_used_to_disambiguate': True,
            'registration_quality': 'provisional_geometric_transfer',
            'independently_verified': False, 'automatic_live_adoption_allowed': False}


def rebase(registration, old_heading, old_board, revision):
    rotation = np.asarray(registration['rotation_old_rigid_to_new_rigid'], dtype=float)
    translation = finite_vector(registration['translation_old_rigid_to_new_rigid_mm'], 3) / 1000
    mounting = quaternion_to_matrix(checked_quaternion(old_heading['body_orientation_in_rigid_xyzw_candidate']))
    center = finite_vector(old_board['board_center_in_rigid_m_candidate'], 3)
    new_mounting = rotation @ mounting
    heading = {
        'scope': 'Offline new-rigid-frame candidate; inherits physical nose, not a new forward movement calibration',
        'rigid_body_revision_id': revision,
        'body_orientation_in_rigid_xyzw_candidate': matrix_to_quaternion(new_mounting),
        'forward_heading_in_rigid_xy_deg': math.degrees(math.atan2(new_mounting[1, 0], new_mounting[0, 0])),
        'forward_heading_in_map_deg': None,
        'manual_forward_horizontal_confirmed': old_heading.get('manual_forward_horizontal_confirmed') is True,
        'physical_forward_definition_inherited': True,
        'new_manual_forward_translation_performed': False,
        'orientation_candidate_quality': 'provisional_geometric_transfer',
        'source_files': [], 'actual_sent_to_fc': 0,
        'body_origin_in_rigid': None, 'origin_offset_verified': False,
        'north_alignment_verified': False, 'time_sync_verified': False,
        'ready_to_send_to_fc': False, 'extrinsics_verified': False,
        'warnings': ['Rebuilt body: transferred candidate requires locked/static validation before live adoption']}
    # Do not retain old-layout marker height/name assumptions in the new record.
    board = {'scope': 'Transferred approximate board geometric center; NOT calibrated IMU center',
        'rigid_body_revision_id': revision,
        'target_reference': old_board['target_reference'],
        'board_center_in_rigid_m_candidate': (rotation @ center + translation).tolist(),
        'measurement_quality': 'old approximate board center plus ~millimeter rigid-registration uncertainty; not quantified',
        'old_elevated_marker_height_assumption_reused': False,
        'new_moved_marker_name_candidate': registration['new_moved_marker_name_candidate'],
        'marker_name': None, 'body_origin_in_rigid': None,
        'imu_offset_verified': False, 'extrinsics_verified': False,
        'ready_to_send_to_fc': False, 'actual_sent_to_fc': 0}
    return heading, board


def inspect_current_calibration(heading_path, board_path, *, registry_path=REGISTRY):
    """Verify version/hashes, WITHOUT granting a live-injection permission."""
    registry = json.loads(Path(registry_path).read_text())
    revision = registry['active_revision_id']
    heading = json.loads(Path(heading_path).read_text())
    board = json.loads(Path(board_path).read_text())
    if any(doc.get('rigid_body_revision_id') != revision for doc in (heading, board)):
        raise ValueError('MircoAir was rebuilt: old rigid-body extrinsics are invalid; use current revision candidates')
    if (digest(heading_path) != registry['heading_candidate_sha256']
            or digest(board_path) != registry['board_candidate_sha256']):
        raise ValueError('Current revision candidate hash mismatch')
    return registry


def require_current_calibration(heading_path, board_path, *, registry_path=REGISTRY):
    """Live senders fail closed; inspection alone never grants permission.

    This supplements existing incident/physical/one-use guards; it never clears
    them. Read-only and restore-only paths remain usable.
    """
    registry = inspect_current_calibration(heading_path, board_path, registry_path=registry_path)
    if registry.get('static_validation_completed') is not True:
        raise ValueError('Current rigid-body calibration remains provisional; full current-layout review incomplete; no live pose injection')
    return registry
