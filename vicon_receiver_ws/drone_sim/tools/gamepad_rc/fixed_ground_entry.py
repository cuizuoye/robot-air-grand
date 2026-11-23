#!/usr/bin/env python3
"""Fixed EKF ground entry: inspect by default; explicit one-use permit to run.

Reuses the existing single-owner radio/ROS/HOME/fusion loop. Never writes EKF
parameters, calibrates, arms, unlocks, changes mode, or emits motion targets.
File checks are not live fusion proof. A draft never approves its own inputs.
"""
import argparse
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace

import ekf_static_profile as profiles
import one_home_preflight_policy as legacy
from preparation_authorization import (
    EvidenceSeal, exclusive_json, load_fixed_backup, require_no_pending_recovery,
)
from rigid_body_revision import digest
from trs_uart_readonly_check import UID2

SCOPE = legacy.FIXED_LIVE_SCOPE
PHYSICAL = dict(props_removed=True, stationary=True, battery_only=True,
                independent_cutoff_ready=True, powered_scope_reviewed=True)


def evidence_paths(*, profile, backup, heading, board, proof, usb_proof):
    from drone_preflight import BINARY_CANDIDATE, BINARY_INSTALLATION
    paths = legacy.paths_for(heading, board, proof, backup, usb_proof)
    tools = Path(__file__).resolve().parent
    for name in ('fixed_ground_entry.py', 'ekf_static_profile.py'):
        paths['fixed/'+name] = tools/name
    paths.update(fixed_profile=Path(profile), installed_candidate=BINARY_CANDIDATE,
                 installation_receipt=BINARY_INSTALLATION)
    return paths


def inspect(*, profile, backup, heading, board, proof, usb_proof, now_unix=None):
    """Local evidence only; reuse successful USB evidence, never rerun it."""
    from drone_preflight import require_binary_installation, BINARY_CANDIDATE
    from trs_runtime_monitor import inspect_evidence
    now = time.time() if now_unix is None else now_unix
    document = profiles.load_json(profile)
    baseline = load_fixed_backup(backup, document, now)
    from low_bandwidth_vision import noise_evidence
    noise_evidence(baseline)
    if document['firmware']['uid2'] != UID2:
        raise ValueError('Fixed profile belongs to another aircraft')
    require_binary_installation()
    if document['firmware']['artifact_sha256'] != digest(BINARY_CANDIDATE):
        raise ValueError('Fixed profile and installed binary candidate differ')
    revision = inspect_evidence(heading, board, proof)['active_revision_id']
    old = profiles.load_json(usb_proof)
    if (old.get('ground_fusion_verified') is not True
            or old.get('runtime', {}).get('failure') is not None
            or old.get('parameter_restoration_verified') is not True
            or old.get('telemetry_restoration_verified') is not True
            or old.get('runtime', {}).get('mission_inputs', {}).get('rigid_revision') != revision):
        raise ValueError('Completed same-revision USB ground evidence required; not repeated automatically')
    paths = evidence_paths(profile=profile, backup=backup, heading=heading,
                           board=board, proof=proof, usb_proof=usb_proof)
    for key, field in (('incident', 'block_active_benches'),
                       ('physical_hold', 'block_props_off_benches')):
        if profiles.load_json(paths[key]).get(field) is not True:
            raise ValueError('Existing general holds must remain intact: '+key)
    # This entry has a distinct, explicit props-off scope. It never clears the
    # historical hold, or converts a USB-only approval into powered approval.
    return document, baseline, paths, revision


def draft(*, run_dir, trs_port, profile, backup, heading, board, proof, usb_proof,
          now_unix=None):
    inputs = dict(profile=profile, backup=backup, heading=heading, board=board,
                  proof=proof, usb_proof=usb_proof)
    document, _, paths, revision = inspect(**inputs, now_unix=now_unix)
    return dict(schema=1, scope=SCOPE, uid2=UID2, firmware='1.14.3', baud=115200,
        run_dir=str(Path(run_dir).resolve()), trs_port=str(Path(trs_port).absolute()),
        **{k: str(Path(v).resolve()) for k, v in inputs.items()},
        fixed_ekf_profile=str(Path(profile).resolve()),
        profile_sha256=profiles.fingerprint(document), rigid_revision=revision,
        hashes={k: digest(v) for k, v in paths.items()},
        receive_rate_only=False, compact_ground_fusion=True, short_status=True,
        binary_diagnostics=True, low_bandwidth_vision=True, frozen_target_stream=False,
        allowed_parameter_writes=[], flight_authorized=False, control_ready=False,
        limits=dict(stream_seconds=20, max_packets=640, fusion_seconds=12),
        user_authorized=False, physical_confirmed=False,
        physical_confirmations={k: False for k in PHYSICAL},
        issued_unix=None, expires_unix=None)


def consume(path, *, backup, output, heading, board, baud, profile, trs_port):
    """Consume only this exact ground scope; old permits remain incompatible."""
    record = profiles.load_json(path)
    if record.get('scope') != SCOPE:
        raise PermissionError('Dedicated fixed ground permit required; no legacy fallback')
    now = time.time()
    issued, expires = record.get('issued_unix'), record.get('expires_unix')
    if (record.get('user_authorized') is not True
            or record.get('physical_confirmed') is not True
            or record.get('physical_confirmations') != PHYSICAL
            or any(type(v) is not bool for v in record.get('physical_confirmations', {}).values())
            or any(type(t) not in (int, float) or not math.isfinite(t) for t in (issued, expires))
            or not issued <= now < expires or not 0 < expires-issued <= 600):
        raise PermissionError('Current explicit props-off powered-ground approval required')
    output = Path(output).resolve()
    if output.name != 'report.json' or output.exists() or baud != 115200:
        raise ValueError('New report.json and pinned 115200 baud required')
    expected = draft(run_dir=output.parent, trs_port=trs_port, profile=profile,
        backup=backup, heading=heading, board=board, proof=record['proof'],
        usb_proof=record['usb_proof'], now_unix=now)
    approvals = {'user_authorized', 'physical_confirmed', 'physical_confirmations',
                 'issued_unix', 'expires_unix'}
    if set(record) != set(expected):
        raise ValueError('Fixed ground permit fields changed')
    for key in set(expected)-approvals:
        if record[key] != expected[key]:
            raise ValueError('Fixed ground scope/code/evidence mismatch: '+key)
    require_no_pending_recovery(output.parent.parent)
    paths = evidence_paths(profile=profile, backup=backup, heading=heading,
        board=board, proof=record['proof'], usb_proof=record['usb_proof'])
    seal = EvidenceSeal(paths, record['hashes'])
    if not seal.refresh():
        raise ValueError(seal.fault)
    exclusive_json(str(path)+'.consumed.json', dict(scope=SCOPE, consumed_unix=now,
        permit_sha256=digest(path), flight_authorized=False))
    return {**record, '_seal': seal}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    for name in ('profile', 'backup', 'heading', 'board', 'proof', 'usb-proof', 'run-dir'):
        ap.add_argument('--'+name, type=Path, required=True)
    ap.add_argument('--trs-port', required=True)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument('--draft-permit', type=Path)
    mode.add_argument('--run', action='store_true')
    ap.add_argument('--permit', type=Path)
    ap.add_argument('--confirm-props-off-powered-ground', action='store_true')
    a = ap.parse_args(argv)
    try:
        if a.run and (a.permit is None or not a.confirm_props_off_powered_ground):
            raise PermissionError('Execution requires a reviewed permit and current physical confirmation')
        if not a.run and (a.permit or a.confirm_props_off_powered_ground):
            raise ValueError('Execution options require --run')
        record = draft(run_dir=a.run_dir, trs_port=a.trs_port, profile=a.profile,
            backup=a.backup, heading=a.heading, board=a.board, proof=a.proof,
            usb_proof=a.usb_proof)
        if not a.run:
            if a.draft_permit:
                exclusive_json(a.draft_permit, record)
            print(json.dumps(dict(mode='local_evidence_check', inputs_valid=True,
                draft_permit=str(a.draft_permit) if a.draft_permit else None,
                devices_opened=0, hardware_verified=False, flight_authorized=False), indent=2))
            return 0
        # Compare CLI evidence with the separately reviewed record before any
        # resource discovery; do not silently substitute a different proof.
        grant = profiles.load_json(a.permit)
        for key in ('profile', 'backup', 'heading', 'board', 'proof', 'usb_proof',
                    'run_dir', 'trs_port', 'hashes', 'scope'):
            if grant.get(key) != record[key]:
                raise ValueError('CLI/permit mismatch: '+key)
        from prepare_drone import RUNS, require_port_free
        from mavlink_readonly_probe import FC_USB
        if a.run_dir.resolve().parent != RUNS.resolve():
            raise ValueError('Run directory must be directly under preparation_runs')
        if FC_USB.exists():
            raise ValueError('This wireless ground scope requires FC USB absent; no automatic power switching')
        require_port_free(a.trs_port)
        # The review directory may already contain its permit, never a report.
        a.run_dir.mkdir(exist_ok=True)
        import trs_powered_fusion_bench as bench
        return bench.run(SimpleNamespace(fixed_ground_live=True, fixed_ekf_profile=a.profile,
            one_home_preflight=True, receive_rate_only=False, compact_ground_fusion=True,
            short_status=True, binary_diagnostics=True, low_bandwidth_vision=True,
            frozen_target_stream=False, powered_locked_static_confirmed=True,
            trs_port=a.trs_port, baud=115200, permit=a.permit, backup=a.backup,
            heading=a.heading, board=a.board, output=a.run_dir/'report.json',
            home_watch_seconds=0, home_wait_seconds=90.))
    except (ValueError, OSError, RuntimeError, KeyError, TypeError) as exc:
        print(json.dumps(dict(inputs_valid=False, error=str(exc), flight_authorized=False),
                         ensure_ascii=False, indent=2))
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
