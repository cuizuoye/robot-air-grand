"""Offline suite hard stop: an accidental real serial/device open must fail.

Tests may inject in-memory transports; a fake ready event never authorizes a
physical device. This fixture is intentionally inherited by existing tests.
"""
import os
from pathlib import Path

import pytest
import serial.serialposix


@pytest.fixture(autouse=True)
def no_real_hardware(monkeypatch):
    def forbidden_serial(*args, **kwargs):
        raise AssertionError('OFFLINE TEST attempted to open a real serial port')
    monkeypatch.setattr(serial.serialposix.Serial, 'open', forbidden_serial)
    original_open = os.open

    def regular_or_mock_only(path, *args, **kwargs):
        if isinstance(path, (str, bytes, os.PathLike)):
            resolved = str(Path(os.fsdecode(path)).resolve())
            if resolved.startswith(('/dev/tty', '/dev/serial/', '/dev/input/', '/dev/hid', '/dev/bus/')):
                raise AssertionError('OFFLINE TEST attempted real device access: ' + resolved)
        return original_open(path, *args, **kwargs)
    monkeypatch.setattr(os, 'open', regular_or_mock_only)
