#!/usr/bin/env python3
"""Repository-local geometry/state tests. No hardware/network/process I/O."""
import os
from pathlib import Path
import sys
import pytest

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / 'vicon_receiver_ws/drone_sim/tools/gamepad_rc'
BRIDGE = ROOT / 'vicon_receiver_ws/drone_sim/ros2_ws/src/uav_coord_bridge'
sys.path[:0] = [str(TOOLS), str(BRIDGE), str(TOOLS/'test')]

class NoHardware:
    @pytest.fixture(autouse=True)
    def isolated(self, monkeypatch):
        import socket
        import subprocess
        import serial
        def deny(*args, **kwargs):
            raise AssertionError('Repository validation attempted device/network/process I/O')
        monkeypatch.setattr(socket, 'socket', deny)
        monkeypatch.setattr(subprocess, 'Popen', deny)
        monkeypatch.setattr(serial.Serial, 'open', deny)

if __name__ == '__main__':
    os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
    dest=ROOT/'validation/latest'
    dest.mkdir(parents=True,exist_ok=True)
    args=['-q','-p','no:cacheprovider',
          str(TOOLS/'test/test_follow_targets_offline.py'),
          str(TOOLS/'test/test_air_ground_sequence.py'),
          '--junitxml='+str(dest/'offline.xml')]
    raise SystemExit(pytest.main(args,plugins=[NoHardware()]))
