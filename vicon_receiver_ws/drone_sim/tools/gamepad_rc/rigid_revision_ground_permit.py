"""One-use unpowered, output-locked, pending-revision Offboard ground check.

Requires fresh attitude/parameter evidence and a passing same-revision fusion
AND local-input-loss test. Only frozen current FC position/yaw, one Offboard
request, and exact original-mode restoration are permitted. Never arm, unlock,
take off, increase altitude, adopt calibration globally or clear safety holds.
"""
import argparse
import datetime
import json
import os
from pathlib import Path
import time

import rigid_revision_static_permit as base
import trs_vision_reference_permit as vision
from receive_only_permit import digest, UID

SCOPE = 'battery-off-usb-observed-locked-rebuilt-frame-disarmed-offboard-once'


def validate(backup, output, heading, board, acceptance, prior):
    expected, checks = base.validate(backup, output, heading, board, acceptance)
    prior = Path(prior).resolve()
    proof = json.loads(prior.read_text())
    captured = datetime.datetime.fromisoformat(proof['captured_at_utc'])
    loss = proof.get('local_source_loss', {})
    if (captured.utcoffset() is None or not 0 <= time.time()-captured.timestamp() < 1800
            or proof.get('rigid_body_revision_id') != expected['rigid_body_revision_id']
            or proof.get('rigid_revision_static_validation') is not True
            or proof.get('ekf_stationary_xyz_yaw_fusion_verified') is not True
            or proof.get('source_loss_verified') is not True or loss.get('verified') is not True
            or loss.get('automatic_restart_allowed') is not False
            or loss.get('tx_count_when_blocked') != loss.get('tx_count_after_recovery')
            or not loss.get('checks') or not all(loss['checks'].values())
            or proof.get('control_readiness_during_fusion', {}).get('estimator_prerequisites_observed') is not True
            or proof.get('error') or proof.get('restore_errors')
            or proof.get('restored') != {'EKF2_EV_CTRL': True, 'EKF2_EV_DELAY': True}
            or proof.get('arming_mode_motion_or_unlock_commands_sent') is not False
            or proof.get('actuator_after', {}).get('armed') is not False
            or proof.get('actuator_after', {}).get('lockdown') is not True
            or any(proof.get('identities', {}).get(route, {}).get('uid2') != UID for route in ('usb', 'trs'))
            or Path(proof.get('heading_candidate_path', '')).resolve() != Path(heading).resolve()
            or Path(proof.get('board_center_candidate_path', '')).resolve() != Path(board).resolve()):
        raise ValueError('Fresh same-revision locked fusion, estimator and local-source-loss proof required')
    expected.update(vision.fields(output, backup, heading, board, acceptance, ground_check=True))
    expected.update(scope=SCOPE, revision_disarmed_ground_check=True,
                    permits_revision_registry_update=False, flight_authorized=False,
                    prior_fusion_loss_proof_path=str(prior), source_loss_test=False)
    checks.update(prior_fusion_loss_proof_sha256=prior,
                  revision_ground_permit_sha256=Path(__file__),
                  ground_estimator_settlement_sha256=Path(__file__).with_name('ground_estimator_settlement.py'),
                  selector_parser_sha256=Path(__file__).with_name('ekf_compare_readonly_policy.py'))
    return expected, checks


def consume(path, backup, output, heading, board):
    path = Path(path).resolve()
    record = json.loads(path.read_text())
    now = time.time()
    if (type(record.get('issued_unix')) not in (int, float)
            or type(record.get('expires_unix')) not in (int, float)
            or not record['issued_unix'] <= now < record['expires_unix']
            or not 0 < record['expires_unix']-record['issued_unix'] <= 900
            or Path(output).exists()):
        raise ValueError('Expired/invalid disarmed revision-ground permit')
    expected, checks = validate(backup, output, heading, board,
                                record['acceptance_path'], record['prior_fusion_loss_proof_path'])
    if any(record.get(k) != value for k, value in expected.items()):
        raise ValueError('Wrong frozen-target ground scope; no arming, flight, loss or guard expansion')
    if any(record.get(k) != digest(value) for k, value in checks.items()):
        raise ValueError('Revision-ground code/evidence hash mismatch')
    fd = os.open(str(path)+'.consumed.json', os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600)
    with os.fdopen(fd, 'w') as stream:
        json.dump(dict(scope=SCOPE, consumed_unix=now, permit_sha256=digest(path),
                       revision_registry_modified=False, flight_authorized=False), stream, indent=2)
        stream.write('\n');stream.flush();os.fsync(stream.fileno())
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--operator-authorized', action='store_true', required=True)
    for name in ('permit', 'backup', 'output', 'heading', 'board', 'acceptance', 'prior-fusion-loss-proof'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    if args.permit.exists() or args.output.exists():
        parser.error('New permit/output required')
    expected, checks = validate(args.backup, args.output, args.heading, args.board,
                                args.acceptance, args.prior_fusion_loss_proof)
    now = time.time()
    with args.permit.open('x') as stream:
        json.dump({**expected, 'issued_unix': now, 'expires_unix': now+600,
                   **{k: digest(v) for k, v in checks.items()}}, stream, indent=2)
        stream.write('\n')
    print('One-use DISARMED/LOCKED ground-check permit created; no device commands or global guard changes')


if __name__ == '__main__':
    main()
