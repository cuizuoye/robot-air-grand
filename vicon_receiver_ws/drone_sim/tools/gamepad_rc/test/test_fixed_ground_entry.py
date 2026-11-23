"""Synthetic files/transports only; actual consumer and ground runner wiring."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import fixed_ground_entry as entry
import drone_preflight as old_cli
from test_fixed_backup import backup
from test_trs_powered_fusion_reactor import reactor
from fixed_profile_fixtures import approved_profile


@pytest.fixture
def prepared(backup, tmp_path, monkeypatch):
    document, snapshot, manifest, _ = backup
    image = tmp_path/'candidate.px4'
    image.write_bytes(b'SYNTHETIC TEST IMAGE, NEVER FLASH')
    installation = tmp_path/'installation.json'
    installation.write_text(json.dumps(dict(firmware_flashed=True,
        application_crc_verified_by_uploader=True, candidate_sha256=entry.digest(image),
        error=None, bootloader={'uid': entry.UID2})))
    monkeypatch.setattr(old_cli, 'BINARY_CANDIDATE', image)
    monkeypatch.setattr(old_cli, 'BINARY_INSTALLATION', installation)
    document['firmware']['artifact_sha256'] = entry.digest(image)
    document['persistence']['configuration_sha256'] = entry.profiles.configuration_digest(document)
    profile = tmp_path/'profile.json'
    profile.write_text(json.dumps(document))
    files = {name: tmp_path/(name+'.json') for name in ('heading', 'board', 'proof', 'usb_proof', 'incident', 'physical_hold')}
    for file in files.values():
        file.write_text('{}')
    files['incident'].write_text('{"block_active_benches":true}')
    files['physical_hold'].write_text('{"block_props_off_benches":true}')
    files['usb_proof'].write_text(json.dumps(dict(ground_fusion_verified=True,
        parameter_restoration_verified=True, telemetry_restoration_verified=True,
        runtime=dict(failure=None, mission_inputs={'rigid_revision':'SYNTHETIC'}))))
    import trs_runtime_monitor
    monkeypatch.setattr(trs_runtime_monitor, 'inspect_evidence', lambda *args: {'active_revision_id':'SYNTHETIC'})
    monkeypatch.setattr(entry.legacy, 'paths_for', lambda *args: {
        **files, 'backup':snapshot, 'backup_manifest':manifest})
    monkeypatch.setattr(entry, 'require_no_pending_recovery', lambda *args: None)
    inputs = dict(profile=profile, backup=tmp_path,
                  **{k:files[k] for k in ('heading','board','proof','usb_proof')})
    context = dict(run_dir=tmp_path, trs_port='/fake/TRS', **inputs)
    token = tmp_path/'permit.json'
    record = entry.draft(**context)
    args = dict(backup=tmp_path, output=tmp_path/'report.json',
        heading=files['heading'], board=files['board'], baud=115200,
        profile=profile, trs_port='/fake/TRS')
    return SimpleNamespace(document=document, record=record, token=token,
        args=args, inputs=inputs, context=context, files=files, installation=installation)


def approve(p):
    now = entry.time.time()
    p.record.update(user_authorized=True, physical_confirmed=True,
        physical_confirmations=dict(entry.PHYSICAL), issued_unix=now, expires_unix=now+600)


def write(p):
    p.token.write_text(json.dumps(p.record))


def test_draft_never_grants_physical_access(prepared):
    p = prepared
    write(p)
    with pytest.raises(PermissionError, match='approval'):
        entry.consume(p.token, **p.args)
    assert not Path(str(p.token)+'.consumed.json').exists()


def test_real_consumer_is_one_use_and_seals_profile(prepared):
    p = prepared
    approve(p); write(p)
    result = entry.consume(p.token, **p.args)
    assert result['scope'] == entry.SCOPE and result['_seal'].fresh()
    assert not result['flight_authorized'] and result['allowed_parameter_writes'] == []
    with pytest.raises(FileExistsError):
        entry.consume(p.token, **p.args)
    p.inputs['profile'].write_text('{}')
    assert not result['_seal'].refresh()


@pytest.mark.parametrize('change', ['expired','nan','unconfirmed','physical_int','legacy','port',
    'profile_hash','code_hash','target','parameter_write','extra_field','output','baud'])
def test_bad_permit_is_rejected_before_consumption(prepared, change):
    p = prepared
    approve(p)
    if change == 'expired': p.record['expires_unix'] = p.record['issued_unix']-1
    elif change == 'nan': p.record['issued_unix'] = float('nan')
    elif change == 'unconfirmed': p.record['physical_confirmations']['powered_scope_reviewed'] = False
    elif change == 'physical_int': p.record['physical_confirmations']['props_removed'] = 1
    elif change == 'legacy': p.record['scope'] = entry.legacy.FIXED_SCOPE
    elif change == 'port': p.args['trs_port'] = '/fake/other'
    elif change == 'profile_hash': p.record['profile_sha256'] = 'f'*64
    elif change == 'code_hash': p.record['hashes']['fixed/fixed_ground_entry.py'] = 'f'*64
    elif change == 'target': p.record['frozen_target_stream'] = True
    elif change == 'parameter_write': p.record['allowed_parameter_writes'] = ['EKF2_EV_CTRL']
    elif change == 'extra_field': p.record['execute'] = True
    elif change == 'output': p.args['output'].write_text('{}')
    elif change == 'baud': p.args['baud'] = 57600
    write(p)
    with pytest.raises((ValueError, PermissionError)):
        entry.consume(p.token, **p.args)
    assert not Path(str(p.token)+'.consumed.json').exists()


@pytest.mark.parametrize('change', ['delay','firmware','installation','usb_failure','revision','hold'])
def test_incomplete_evidence_never_becomes_ready(prepared, change):
    p = prepared
    if change == 'delay': p.document['delay_calibration']['status'] = 'pending_calibration'
    elif change == 'firmware': p.document['firmware']['artifact_sha256'] = 'f'*64
    elif change == 'installation': p.installation.write_text('{}')
    elif change == 'usb_failure': p.files['usb_proof'].write_text('{}')
    elif change == 'revision':
        d = json.loads(p.files['usb_proof'].read_text())
        d['runtime']['mission_inputs']['rigid_revision'] = 'OTHER'
        p.files['usb_proof'].write_text(json.dumps(d))
    elif change == 'hold': p.files['physical_hold'].write_text('{}')
    p.inputs['profile'].write_text(json.dumps(p.document))
    with pytest.raises(ValueError): entry.draft(**p.context)


def cli_args(p):
    return [part for key, value in p.context.items()
            for part in ('--'+key.replace('_','-'), str(value))]


def test_default_cli_only_checks_local_files(prepared, capsys):
    assert entry.main(cli_args(prepared)) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['devices_opened'] == 0 and not report['hardware_verified']
    assert not prepared.token.exists()


def test_cli_draft_stays_unsigned(prepared):
    p = prepared
    assert entry.main(cli_args(p)+['--draft-permit',str(p.token)]) == 0
    assert json.loads(p.token.read_text())['user_authorized'] is False


def test_cli_run_without_confirmation_does_not_inspect_or_open(prepared, monkeypatch):
    monkeypatch.setattr(entry, 'draft', lambda **kw: pytest.fail('No evidence/device work before confirmation'))
    assert entry.main(cli_args(prepared)+['--run']) == 2


def test_cli_dispatches_to_existing_fixed_runner_only(prepared, monkeypatch):
    import prepare_drone
    import mavlink_readonly_probe
    import trs_powered_fusion_bench as bench
    p = prepared
    approve(p); write(p)
    monkeypatch.setattr(prepare_drone, 'RUNS', p.token.parent.parent)
    monkeypatch.setattr(prepare_drone, 'require_port_free', lambda path: None)
    monkeypatch.setattr(mavlink_readonly_probe, 'FC_USB', p.token.parent/'absent-usb')
    calls = []
    monkeypatch.setattr(bench, 'run', lambda args: calls.append(args) or 0)
    assert entry.main(cli_args(p)+['--run','--permit',str(p.token),
        '--confirm-props-off-powered-ground']) == 0
    assert len(calls) == 1 and calls[0].fixed_ground_live
    assert calls[0].fixed_ekf_profile == p.inputs['profile']
    assert not calls[0].frozen_target_stream and calls[0].low_bandwidth_vision


def test_cli_cannot_substitute_reviewed_source_evidence(prepared, monkeypatch):
    import prepare_drone
    p = prepared
    approve(p); p.record['proof'] = '/different-proof.json'; write(p)
    monkeypatch.setattr(prepare_drone, 'require_port_free', lambda path: pytest.fail('No discovery on mismatch'))
    assert entry.main(cli_args(p)+['--run','--permit',str(p.token),
        '--confirm-props-off-powered-ground']) == 2


@pytest.mark.parametrize('scenario', ['healthy','radio_lost','source_lost','short_clock_lost',
    'one_home_abort','fixed_ctrl_mismatch','binary_restore_echo_missing','permit_expires'])
def test_production_fixed_entry_uses_existing_loop_without_global_release(reactor, scenario):
    import trs_powered_fusion_bench as bench
    assert bench.FIXED_GROUND_LIVE_ENABLED is False
    code, report, sent = reactor(scenario=scenario, baud=115200,
        fixed_document=approved_profile(), fixed_live=True)
    assert code == (0 if scenario == 'healthy' else 2), report['error']
    assert report['scope'] == entry.SCOPE and report['fixed_live_enabled']
    assert not report['flight_authorized'] and not report['control_ready']
    assert not report['parameter_writes']
    assert not any(msg.get_type() in ('PARAM_SET','PARAM_EXT_SET','SET_POSITION_TARGET_LOCAL_NED') for _,msg in sent)
    if scenario == 'healthy':
        assert report['fixed_ground_acceptance_verified']
        assert any(msg.get_type() == 'VISION_POSITION_ESTIMATE' for _,msg in sent)
    if scenario == 'permit_expires':
        assert 'expired' in report['error']
        assert all(t < 6 for t,msg in sent if msg.get_type() == 'VISION_POSITION_ESTIMATE')
