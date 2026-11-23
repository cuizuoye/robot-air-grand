"""Transport selection tests never access real serial or flight hardware."""
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace as NS

import pytest

TOOLS=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(TOOLS))
from trs_transport_config import require_baud


@pytest.mark.parametrize('value',[None,True,False,'57600',57600.,0,9600,230400])
def test_unknown_or_implicit_baud_never_selects_a_fallback(value):
    with pytest.raises(ValueError,match='Explicit TRS baud'):require_baud(value)


@pytest.mark.parametrize('value',[57600,115200])
def test_only_known_explicit_baud_values_allowed(value):
    assert require_baud(value)==value


@pytest.mark.parametrize('script',[
    'trs_runtime_monitor.py','trs_powered_fusion_bench.py',
    'trs_powered_fusion_permit.py'])
def test_cli_requires_baud_before_any_device_or_evidence_operation(script):
    result=subprocess.run([sys.executable,str(TOOLS/script)],capture_output=True,text=True,timeout=5)
    assert result.returncode==2 and '--baud' in result.stderr


def test_monitor_rejects_missing_baud_before_physical_or_serial_checks(monkeypatch):
    import trs_runtime_monitor as runner
    monkeypatch.setattr(runner,'check_physical',lambda *a:pytest.fail('Invalid config opened physical checks'))
    with pytest.raises(ValueError,match='Explicit TRS baud'):runner.run(NS())


def test_fusion_crc_configuration_is_validated_before_permit_consumption(tmp_path,monkeypatch):
    import trs_powered_fusion_bench as runner
    monkeypatch.setattr(runner,'FC_USB',tmp_path/'missing-usb')
    def bad_crc():raise ValueError('invalid pinned CRC definition')
    monkeypatch.setattr(runner,'CrcResyncParser',bad_crc)
    monkeypatch.setattr(runner.permit_policy,'consume',lambda *a,**kw:pytest.fail('Consumed permit before CRC validation'))
    with pytest.raises(ValueError,match='pinned CRC'):
        runner.run(NS(powered_locked_static_confirmed=True,output=tmp_path/'new-output',baud=57600))
