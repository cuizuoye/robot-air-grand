"""One-use locked USB-only rebuilt-body fusion and LOCAL source-loss test.

Compose the existing fresh attitude/backup/revision checks, plus a prior
same-revision static fusion proof. This never approves flight, modes, targets,
global calibration adoption, radio disconnection or the props-off legacy path.
"""
import argparse
import json
import os
from pathlib import Path
import time

import rigid_revision_static_permit as base
from receive_only_permit import digest, UID

SCOPE = 'battery-off-usb-observed-locked-rebuilt-frame-local-source-loss-once'


def validate(backup, output, heading, board, acceptance, prior):
    expected, checks = base.validate(backup, output, heading, board, acceptance)
    prior = Path(prior).resolve()
    proof = json.loads(prior.read_text())
    if (proof.get('rigid_body_revision_id') != expected['rigid_body_revision_id']
            or proof.get('rigid_revision_static_validation') is not True
            or proof.get('ekf_stationary_xyz_yaw_fusion_verified') is not True
            or proof.get('error') or proof.get('restore_errors')
            or proof.get('restored') != {'EKF2_EV_CTRL': True, 'EKF2_EV_DELAY': True}
            or proof.get('arming_mode_motion_or_unlock_commands_sent') is not False
            or proof.get('actuator_after', {}).get('armed') is not False
            or proof.get('actuator_after', {}).get('lockdown') is not True
            or any(proof.get('identities', {}).get(route, {}).get('uid2') != UID
                   for route in ('usb', 'trs'))
            or Path(proof.get('heading_candidate_path', '')).resolve() != Path(heading).resolve()
            or Path(proof.get('board_center_candidate_path', '')).resolve() != Path(board).resolve()):
        raise ValueError('Prior passing same-revision non-actuating static fusion proof required')
    expected.update(scope=SCOPE, source_loss_test=True,
                    local_subscriber_mask_only=True, automatic_restart_allowed=False,
                    prior_static_proof_path=str(prior))
    checks.update(prior_static_proof_sha256=prior,
                  revision_loss_permit_sha256=Path(__file__),
                  combined_loss_policy_sha256=Path(__file__).with_name('combined_input_loss.py'))
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
        raise ValueError('Expired/invalid one-use local source-loss permit')
    expected, checks = validate(backup, output, heading, board,
                                record['acceptance_path'], record['prior_static_proof_path'])
    if any(record.get(k) != value for k, value in expected.items()):
        raise ValueError('Wrong local source-loss scope; no modes, flight or guard expansion')
    if any(record.get(k) != digest(value) for k, value in checks.items()):
        raise ValueError('Local source-loss code/evidence hash mismatch')
    fd = os.open(str(path)+'.consumed.json', os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600)
    with os.fdopen(fd, 'w') as stream:
        json.dump(dict(scope=SCOPE, consumed_unix=now, permit_sha256=digest(path),
                       revision_registry_modified=False, flight_authorized=False), stream, indent=2)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--operator-authorized', action='store_true', required=True)
    for name in ('permit', 'backup', 'output', 'heading', 'board', 'acceptance', 'prior-static-proof'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    if args.permit.exists() or args.output.exists():
        parser.error('New permit and output paths required')
    expected, checks = validate(args.backup, args.output, args.heading, args.board,
                                args.acceptance, args.prior_static_proof)
    now = time.time()
    with args.permit.open('x') as stream:
        json.dump({**expected, 'issued_unix': now, 'expires_unix': now+600,
                   **{k: digest(v) for k, v in checks.items()}}, stream, indent=2)
        stream.write('\n')
    print('One-use local-input-loss permit created; zero device commands; all general holds retained')


if __name__ == '__main__':
    main()
