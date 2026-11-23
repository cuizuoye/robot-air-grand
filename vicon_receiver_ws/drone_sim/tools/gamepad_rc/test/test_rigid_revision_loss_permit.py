import json
from pathlib import Path
import subprocess
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rigid_revision_loss_permit as p
from test_rigid_revision_static_permit import fixture as original_fixture


def fixture(tmp_path, monkeypatch):
    permit, backup, output, heading, board, old_record = original_fixture(tmp_path, monkeypatch)
    prior = tmp_path/'prior-static.json'
    prior.write_text(json.dumps(dict(rigid_body_revision_id='new', rigid_revision_static_validation=True,
        ekf_stationary_xyz_yaw_fusion_verified=True, error=None, restore_errors=[],
        restored={'EKF2_EV_CTRL': True, 'EKF2_EV_DELAY': True},
        arming_mode_motion_or_unlock_commands_sent=False,
        actuator_after={'armed': False, 'lockdown': True},
        identities={route: {'uid2': p.UID} for route in ('usb', 'trs')},
        heading_candidate_path=str(heading), board_center_candidate_path=str(board))))
    expected, checks = p.validate(backup, output, heading, board, old_record['acceptance_path'], prior)
    record = {**expected, 'issued_unix': time.time()-1, 'expires_unix': time.time()+600,
              **{k: p.digest(v) for k, v in checks.items()}}
    permit.write_text(json.dumps(record))
    return permit, backup, output, heading, board, record, prior


def test_one_use_loss_is_not_flight_or_global_calibration_permission(tmp_path, monkeypatch):
    f, backup, out, h, b, record, _ = fixture(tmp_path, monkeypatch)
    got = p.consume(f, backup, out, h, b)
    assert got['scope'] == p.SCOPE and got['local_subscriber_mask_only']
    assert not got['flight_authorized'] and not got['permits_revision_registry_update']
    assert not got['automatic_restart_allowed'] and not got['ground_command_scope']
    assert got['max_fusion_seconds'] == 10 and got['max_pose_packets'] == 480
    with pytest.raises(FileExistsError):
        p.consume(f, backup, out, h, b)


@pytest.mark.parametrize('key,value', [('scope', 'flight'), ('source_loss_test', False),
    ('local_subscriber_mask_only', False), ('automatic_restart_allowed', True),
    ('flight_authorized', True), ('max_fusion_seconds', 20)])
def test_changed_scope_rejected_before_consumption(tmp_path, monkeypatch, key, value):
    f, backup, out, h, b, record, _ = fixture(tmp_path, monkeypatch)
    f.write_text(json.dumps({**record, key: value}))
    with pytest.raises(ValueError):
        p.consume(f, backup, out, h, b)
    assert not Path(str(f)+'.consumed.json').exists()


@pytest.mark.parametrize('key,value', [('ekf_stationary_xyz_yaw_fusion_verified', False),
    ('rigid_body_revision_id', 'old'), ('arming_mode_motion_or_unlock_commands_sent', True),
    ('restore_errors', ['rollback failed'])])
def test_prior_static_failure_not_bypassed_by_rehashing(tmp_path, monkeypatch, key, value):
    f, backup, out, h, b, record, prior = fixture(tmp_path, monkeypatch)
    proof = json.loads(prior.read_text());proof[key] = value;prior.write_text(json.dumps(proof))
    record['prior_static_proof_sha256'] = p.digest(prior);f.write_text(json.dumps(record))
    with pytest.raises(ValueError, match='Prior passing'):
        p.consume(f, backup, out, h, b)


def test_cli_loss_requires_distinct_revision_profile_before_any_device(tmp_path):
    runner = Path(__file__).resolve().parents[1]/'trs_static_fusion_bench.py'
    cmd = [sys.executable, str(runner), '--baud','115200', '--backup', '/missing', '--heading-candidate', '/missing',
           '--board-center-candidate', '/missing', '--output', str(tmp_path/'never.json'),
           '--revision-source-loss-validation']
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
    assert result.returncode == 2 and 'distinct revision Vision permit' in result.stderr
    assert not (tmp_path/'never.json').exists()
