"""Release command has no hardware or approval capability."""
from pathlib import Path
import json
import sys
from unittest.mock import patch

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import offline_flight_release as release
from flight_execution_journal import FlightExecutionJournal
from test_continuous_execution import wired
from flight_start_confirmation import FlightRunController


def test_default_release_checks_sources_without_any_device_or_subprocess(capsys):
    with patch('serial.Serial') as serial,patch('socket.socket') as socket,patch('subprocess.run') as process:
        assert release.main([])==0
        serial.assert_not_called();socket.assert_not_called();process.assert_not_called()
    assert '未打开硬件' in capsys.readouterr().out


def test_json_manifest_marks_all_physical_authority_unavailable(capsys):
    assert release.main(['--json'])==0
    data=json.loads(capsys.readouterr().out)
    assert data['source_check_passed'] and data['offline_dependencies_available']
    assert not data['flight_authorized'] and not data['native_cli_flight_enabled']
    assert data['actual_sent_to_fc']==0 and not data['physical_authorizer_installed']
    assert len(data['source_sha256'])>=35


@pytest.mark.parametrize('flag',['--execute','--approve','--arm','--run-battery'])
def test_release_never_exposes_approval_or_execution_flags(flag):
    with pytest.raises(SystemExit) as exc:release.main([flag])
    assert exc.value.code==2


def test_inspection_never_reads_character_devices_or_fifos():
    with pytest.raises(ValueError,match='regular'):FlightExecutionJournal.inspect('/dev/null')


def test_self_test_requires_new_retained_result_directory():
    with pytest.raises(SystemExit):release.main(['--self-test'])


def test_controller_can_end_empty_failed_preparation_without_any_privileged_bytes(tmp_path):
    p,r,io,journal=wired(tmp_path,authorized=False)
    p.controller=FlightRunController(r,io)
    for _ in range(30):
        p.turn(home=(True,) if _==10 else ())
        if p.controller.done:break
    assert p.controller.done and not journal.record['events']
    assert p.controller.journal_finalized and not r.core.arm_attempted


def test_code_syntax_or_missing_dependency_never_becomes_green(tmp_path,monkeypatch,capsys):
    path=tmp_path/'broken.py';path.write_text('def invalid(')
    monkeypatch.setattr(release,'TOOLS',tmp_path)
    monkeypatch.setattr(release,'FILES',('broken.py',))
    assert release.main([])==2
    assert '未通过' in capsys.readouterr().out


def test_previous_recovery_remains_untouched_by_inspect(tmp_path,capsys):
    path=tmp_path/'flight_recovery.json'
    record=dict(needs_restore=True,needs_operator_review=True,verified_closed=False,events=[dict(kind='arm')])
    path.write_text(json.dumps(record));original=path.read_bytes()
    assert release.main(['--inspect-recovery',str(path),'--json'])==0
    report=json.loads(capsys.readouterr().out)
    assert report['recovery']['needs_restore'] and not report['recovery']['automatic_recovery_available']
    assert path.read_bytes()==original
