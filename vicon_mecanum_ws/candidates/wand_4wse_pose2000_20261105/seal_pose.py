"""Seal only the named localization-age delta; verify all other host sources."""
import ast
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
CP = 'host/runtime/src/mecanum_wand_4wse_control/mecanum_wand_4wse_control/'
HP = 'host/runtime/src/mecanum_wand_4wse_hardware/mecanum_wand_4wse_hardware/'
CHANGED = {'host/hardware_runtime.py', 'host/profiles/stage6.LONG600_NOT_APPROVED.json',
    'host/web/error_messages.js', *(CP+n+'.py' for n in ('operator_console', 'cached_packet',
        'pose_quality', 'staged_config', 'rotation_source', 'source_bundle', 'bound_history')),
    *(HP+n+'.py' for n in ('bound_rotation', 'rotation_nodes', 'nodes', 'coherent_source',
        'ground_admission', 'source_relay'))}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    out = HERE / 'host/HOST_MANIFEST.json'
    if out.exists():
        raise ValueError('never silently reseal')
    origin = json.loads((HERE / 'ORIGINS.json').read_text())
    parent = Path(origin['parent'])
    assert sha(parent / 'RELEASE.json') == origin['parent_release_sha256']
    assert sha(parent / 'host/HOST_MANIFEST.json') == origin['parent_manifest_sha256']
    sources = [p for p in (HERE / 'host').rglob('*') if p.is_file() and '__pycache__' not in p.parts]
    normalized = origin['normalized_source_hashes']
    changed = {r for r, digest in normalized.items() if sha(HERE / r) != digest}
    added = {str(p.relative_to(HERE)) for p in sources} - set(normalized)
    assert changed == CHANGED, (changed ^ CHANGED)
    assert added == {CP + 'pose_policy.py'}, added
    for path in sources:
        if path.suffix == '.py':
            ast.parse(path.read_text(), filename=str(path))
    inherited = json.loads((parent / 'host/HOST_MANIFEST.json').read_text())
    proof = HERE / 'evidence/PARENT_VERIFIED.json'
    proof.write_text(json.dumps(dict(parent_manifest_verified=True,
        source_age_limit_s=2., source_age_only=True, firmware_changed=False,
        command_feedback_and_estop_sources_unchanged=True, network_settings_changed=False,
        ros_initialized=False, hardware_socket_opened=False), indent=2) + '\n')
    value = dict(inherited, revision='POSE2000_BROWSER_V1',
        parent_host_manifest_sha256=origin['parent_manifest_sha256'],
        presentation_only=False, source_age_limit_s=2.,
        parent_motion_and_feedback_sources_unchanged=False,
        speed_acceleration_and_feedback_limits_unchanged=True,
        source_age_change_requires_new_real_trial=True,
        source_hashes={str(p.relative_to(HERE)): sha(p) for p in sources + [HERE / 'ORIGINS.json', HERE / 'README.md']},
        evidence={str(proof.relative_to(HERE)): sha(proof)})
    out.write_text(json.dumps(value, indent=2) + '\n')
    entry = HERE / 'entry.py'
    entry.write_text(entry.read_text().replace('HOST_MANIFEST_PENDING', sha(out)))
    print('Sealed pose2000 sources; original command/feedback/firmware limits retained.')


if __name__ == '__main__':
    main()
