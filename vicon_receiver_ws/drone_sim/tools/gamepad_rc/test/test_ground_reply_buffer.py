from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from persistent_shell import reply_buffer_limit


def test_only_explicit_ground_48_frame_reply_gets_larger_bounded_buffer():
    command=b'listener vehicle_visual_odometry -n 48\n'
    assert reply_buffer_limit(command, ground_check=True)==65536
    assert reply_buffer_limit(command)==24000
    for other in (b'', b'listener actuator_armed\n', b'listener vehicle_visual_odometry -n 999\n'):
        assert reply_buffer_limit(other, ground_check=True)==24000
