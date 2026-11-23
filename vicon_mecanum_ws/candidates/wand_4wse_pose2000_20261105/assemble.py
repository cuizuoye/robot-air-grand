"""Private 2000ms source-age candidate; no device, ROS or network access."""
import hashlib
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
PARENT = HERE.parent / 'wand_4wse_zh_errors_20261105'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if (HERE / 'host').exists():
        raise ValueError('refusing to overwrite an existing candidate')
    for name, groups in (('RELEASE.json', ('sources', 'evidence')),
                         ('host/HOST_MANIFEST.json', ('source_hashes', 'evidence'))):
        manifest = json.loads((PARENT / name).read_text())
        for group in groups:
            for relative, digest in manifest[group].items():
                assert sha(PARENT / relative) == digest, relative
    shutil.copytree(PARENT / 'host', HERE / 'host', ignore=shutil.ignore_patterns(
        'sessions', '__pycache__', '*.pyc', 'HOST_MANIFEST.json'))
    normalized = {}
    for path in (HERE / 'host').rglob('*'):
        if path.is_file():
            path.write_text(path.read_text().replace(str(PARENT), str(HERE)))
            normalized[str(path.relative_to(HERE))] = sha(path)
    for name in ('entry.py', 'receiver_manager.py', 'test_startup.py', 'test_domain_isolation.py',
                 'test_public_browser.cjs', 'test_error_messages.cjs', 'test_diagnostic_view.py', 'test_zh_browser.cjs'):
        value = (PARENT / name).read_text().replace(str(PARENT), str(HERE))
        if name == 'entry.py':
            value = value.replace(sha(PARENT / 'host/HOST_MANIFEST.json'), 'HOST_MANIFEST_PENDING')
            value = value.replace('4WSE_ZH_ERRORS_V1', '4WSE_POSE2000_V1')
        (HERE / name).write_text(value)
    path = HERE / 'host/release_binding.py'
    path.write_text(path.read_text().replace('ZH_ERRORS_BROWSER_V1', 'POSE2000_BROWSER_V1'))
    normalized['host/release_binding.py'] = sha(path)
    (HERE / 'evidence').mkdir()
    (HERE / 'ORIGINS.json').write_text(json.dumps(dict(parent=str(PARENT),
        parent_release_sha256=sha(PARENT / 'RELEASE.json'),
        parent_manifest_sha256=sha(PARENT / 'host/HOST_MANIFEST.json'),
        normalized_source_hashes=normalized), indent=2) + '\n')
    print('Independent pose2000 candidate copied; original releases unchanged.')


if __name__ == '__main__':
    main()
