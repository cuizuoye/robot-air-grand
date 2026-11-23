"""Offline compatibility audit, not implementation of a TIMESYNC safety gate."""
from pathlib import Path
import sys

from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mavlink_crc_stream import CrcResyncParser
from trs_message_interval_trial import allowed


def test_existing_trial_does_not_yet_authorize_time_probe():
    encoder = m.MAVLink(None, srcSystem=250, srcComponent=203)
    request = encoder.timesync_encode(0, 1_000_000_000)
    assert not allowed(request, safe=True, locked=True, short_status=True)


def test_crc_alone_does_not_reject_replayed_timesync_reply():
    encoder = m.MAVLink(None, srcSystem=1, srcComponent=1)
    packet = encoder.timesync_encode(5_000_000_000, 1_000_000_000).pack(encoder)
    parser = CrcResyncParser()
    replies = parser.parse_buffer(packet + packet)
    assert len(replies) == 2
    assert replies[0].ts1 == replies[1].ts1
    # A future caller must consume a matching outstanding request only once.


def test_firmware_probe_reply_returns_before_clock_filter_update():
    root = Path(__file__).resolve().parents[3]
    source = (root / 'micoair_shell_firmware_20261013_ikdyTI/source/src/modules/mavlink/mavlink_timesync.cpp').read_text()
    request_branch = source.split('if (tsync.tc1 == 0)', 1)[1].split('} else if (tsync.tc1 > 0)', 1)[0]
    assert 'rsync.tc1 = now * 1000ULL' in request_branch
    assert 'rsync.ts1 = tsync.ts1' in request_branch
    assert 'return;' in request_branch
    assert '_timesync.update' not in request_branch
