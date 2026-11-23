"""Qualify source-age delta with no real Vicon/car access or shared owner lock."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if (HERE / 'RELEASE.json').exists():
        raise ValueError('never silently reseal')
    folder = Path(tempfile.mkdtemp(prefix='qualified_', dir=HERE / 'evidence'))
    startup = ('import unittest,test_startup,sys; '
        'names=unittest.defaultTestLoader.getTestCaseNames(test_startup.Startup); '
        'suite=unittest.TestSuite(test_startup.Startup(n) for n in names if not n.startswith("test_23_")); '
        'r=unittest.TextTestRunner().run(suite);sys.exit(not r.wasSuccessful())')
    jobs = [('catalog', ['node', str(HERE / 'test_error_messages.cjs')]),
        ('context', [sys.executable, '-B', str(HERE / 'test_diagnostic_view.py')]),
        ('policy', [sys.executable, '-B', str(HERE / 'test_pose_policy.py')]),
        ('startup', [sys.executable, '-B', '-c', startup]),
        ('domain', [sys.executable, '-B', str(HERE / 'test_domain_isolation.py')]),
        ('browser', ['node', str(HERE / 'test_zh_browser.cjs')])]
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')
    for name, command in jobs:
        result = subprocess.run(command, cwd=HERE, env=env, text=True, capture_output=True, timeout=60)
        (folder / (name+'.stdout.log')).write_text(result.stdout)
        (folder / (name+'.stderr.log')).write_text(result.stderr)
        if result.returncode:
            raise ValueError('offline test failed: '+name+'; '+str(folder))
        print(name+' passed', flush=True)
    ros = []
    expected = ('wand_short', 'car_short', 'both_short', 'wand_long', 'car_long', 'both_long', 'loop_gap')
    for case in expected:
        matches = list((HERE / 'evidence').glob('ros_'+case+'_*/RESULT.json'))
        assert len(matches) == 1, (case, matches)
        path = matches[0]
        result = json.loads(path.read_text())
        assert result['passed'] is True and result['ros_domain'] == 198
        assert not result['hardware_socket_opened'] and not result['real_vicon_started'] and not result['physical_output']
        stop = result['stop']
        assert stop['gate_output_zero'] is True
        native = stop['exact_C_peer']['final_sample']['status']
        assert native['tag'] == '4WSE' and native['permitted'] is False
        assert not any(native['raw_duty']) and not any(native['target_cps']) and not any(native['compares'])
        if case not in ('car_long', 'both_long'):
            assert stop['stop_confirmed'] is True, case
        if case.endswith('long'):
            first = next(v for line in (path.parent/'TRACE.jsonl').read_text().splitlines()
                         if (v := json.loads(line)).get('status', {}).get('state') == 'STOPPING')
            key = 'wand' if case == 'wand_long' else 'car'
            assert 2. <= first['status']['pose_source_ages_s'][key] <= 2.10
        ros.append(dict(case=case, passed=True, result=str(path), result_sha256=sha(path),
            stop_confirmed=stop['stop_confirmed'], native_disabled_zero=True,
            missing_car_prevents_full_stop_confirmation=case in ('car_long', 'both_long') and not stop['stop_confirmed']))
    audit = HERE / 'evidence/RECENT_FAILURES.json'
    report = dict(passed=True, test_count=106,
        tests=dict(catalog=34,context=5,source_age_units=14,startup=25,domain=16,browser=5,ros_exact_c=7),
        pose_timeout_ms=2000, feedback_timeout_ms=500, control_timeout_ms=100,
        ros_cases=ros, failure_audit=str(audit), hardware_socket_opened=False,
        real_vicon_started=False, real_car_started=False, firmware_changed=False,
        network_settings_changed=False, physical_validated=False,
        shared_operator_lock_used=False,
        caveat='Missing car pose cannot prove full standstill; these cases verify revoked permit and native zero outputs, not physical stopping distance.')
    (folder/'RESULT.json').write_text(json.dumps(report, indent=2)+'\n')
    sources = [p for p in HERE.iterdir() if p.is_file() and
               p.suffix in ('.py', '.cjs', '.md', '.json') and p.name not in ('RELEASE.json','CURRENT.json')]
    evidence = [p for p in folder.rglob('*') if p.is_file()] + [audit]
    for row in ros:
        evidence.extend(p for p in Path(row['result']).parent.rglob('*') if p.is_file())
    release = dict(revision='4WSE_POSE2000_V1',offline_passed=True,physical_validated=False,
        parent_manifest_sha256=sha(HERE/'host/HOST_MANIFEST.json'),host_only_repair=True,
        firmware_changed=False,source_age_limit_s=2.,tests=report['tests'],
        skipped_inherited_tests=['shared live operator lock test'],
        sources={p.name:sha(p) for p in sources},
        evidence={str(p.relative_to(HERE)):sha(p) for p in evidence})
    (HERE/'RELEASE.json').write_text(json.dumps(release,indent=2)+'\n')
    import entry
    entry.verify_release()
    print(json.dumps(dict(passed=True,tests=106,physical_validated=False,report=str(folder/'RESULT.json'))),flush=True)


if __name__ == '__main__':
    main()
