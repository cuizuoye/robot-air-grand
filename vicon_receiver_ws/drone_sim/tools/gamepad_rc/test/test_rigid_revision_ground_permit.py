import datetime
import json
from pathlib import Path
import subprocess
import sys
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rigid_revision_ground_permit as p
from test_rigid_revision_loss_permit import fixture as loss_fixture


def fixture(tmp_path, monkeypatch):
    permit, backup, output, heading, board, old, prior = loss_fixture(tmp_path, monkeypatch)
    proof = json.loads(prior.read_text())
    proof.update(captured_at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                 source_loss_verified=True, control_readiness_during_fusion={'estimator_prerequisites_observed': True},
                 local_source_loss={'verified': True, 'automatic_restart_allowed': False,
                     'tx_count_when_blocked': 308, 'tx_count_after_recovery': 308, 'checks': {'latched': True}})
    prior.write_text(json.dumps(proof))
    expected, checks = p.validate(backup, output, heading, board, old['acceptance_path'], prior)
    record = {**expected, 'issued_unix': time.time()-1, 'expires_unix': time.time()+600,
              **{k: p.digest(v) for k, v in checks.items()}}
    permit.write_text(json.dumps(record))
    return permit, backup, output, heading, board, record, prior


def test_new_scope_never_grants_flight_or_changes_global_revision(tmp_path, monkeypatch):
    f, backup, out, h, b, record, _ = fixture(tmp_path, monkeypatch)
    got = p.consume(f, backup, out, h, b)
    assert got['scope'] == p.SCOPE and got['disarmed_offboard_ground_check']
    assert not got['source_loss_test'] and not got['flight_authorized']
    assert not got['permits_revision_registry_update']
    assert got['max_fusion_seconds'] == 20 and got['max_pose_packets'] == 800
    assert got['ground_command_scope'] == ['frozen_current_position_yaw', 'one_disarmed_offboard_request', 'restore_initial_disarmed_mode']
    with pytest.raises(FileExistsError):
        p.consume(f, backup, out, h, b)


@pytest.mark.parametrize('key,value', [('scope', 'flight'), ('source_loss_test', True),
    ('flight_authorized', True), ('max_fusion_seconds', 60),
    ('ground_command_scope', ['arm']), ('permits_revision_registry_update', True)])
def test_scope_expansion_rejected(tmp_path, monkeypatch, key, value):
    f, backup, out, h, b, record, _ = fixture(tmp_path, monkeypatch)
    f.write_text(json.dumps({**record, key: value}))
    with pytest.raises(ValueError):
        p.consume(f, backup, out, h, b)
    assert not Path(str(f)+'.consumed.json').exists()


@pytest.mark.parametrize('key,value', [('source_loss_verified', False),
    ('rigid_body_revision_id', 'old'), ('arming_mode_motion_or_unlock_commands_sent', True),
    ('actuator_after', {'armed': True, 'lockdown': True}),
    ('control_readiness_during_fusion', {'estimator_prerequisites_observed': False})])
def test_failed_prior_proof_cannot_be_bypassed_by_rehash(tmp_path, monkeypatch, key, value):
    f, backup, out, h, b, record, prior = fixture(tmp_path, monkeypatch)
    proof = json.loads(prior.read_text());proof[key] = value;prior.write_text(json.dumps(proof))
    record['prior_fusion_loss_proof_sha256'] = p.digest(prior);f.write_text(json.dumps(record))
    with pytest.raises(ValueError, match='Fresh same-revision'):
        p.consume(f, backup, out, h, b)


def test_cli_new_flag_without_full_ground_scope_opens_no_device(tmp_path):
    runner = Path(__file__).resolve().parents[1]/'trs_static_fusion_bench.py'
    cmd = [sys.executable, str(runner), '--baud','115200', '--backup', '/missing', '--heading-candidate', '/missing',
           '--board-center-candidate', '/missing', '--output', str(tmp_path/'never.json'), '--revision-ground-check']
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
    assert result.returncode == 2 and 'distinct revision Vision ground permit' in result.stderr
    assert not (tmp_path/'never.json').exists()
