"""Real MAVLink encoding/CRC, synthetic clocks; no device or control API."""
from pathlib import Path
import sys
import pytest
from pymavlink.dialects.v20 import common as m
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from output_lock_status import OutputLockStatus, MAGIC
from mavlink_crc_stream import CrcResyncParser


def wire(bits=MAGIC | 5, stamp=10000, source=(1, 1), name=b'OUTLOCK1'):
    encoder=m.MAVLink(None,srcSystem=source[0],srcComponent=source[1])
    return bytes(encoder.named_value_int_encode(stamp,name,bits).pack(encoder))


def feed(status, raw, now):
    for msg in CrcResyncParser().parse_buffer(raw):status.ingest(msg, now)


def ready():
    status=OutputLockStatus();status.update_clock(10100, 1.)
    feed(status,wire(),1.)
    return status


def test_standard_wire_crc_and_explicit_identity_required():
    status=ready()
    assert not status.locked_disarmed(1.)
    assert status.locked_disarmed(1.,identity_verified=True)
    assert len(wire())<=30  # unsigned MAVLink2, 18-byte payload + framing


@pytest.mark.parametrize('bits',[MAGIC, MAGIC|1, MAGIC|5|2, MAGIC|5|8,
                                  MAGIC|5|16, MAGIC|5|32, 0, 0x4c530205])
def test_invalid_or_unsafe_state_never_proves_lock(bits):
    status=ready();feed(status,wire(bits),1.01)
    assert not status.locked_disarmed(1.01,identity_verified=True)


def test_corrupted_frame_does_not_create_or_refresh_state():
    status=ready();damaged=bytearray(wire(stamp=10400));damaged[-1]^=1
    feed(status,damaged,.49+1)
    assert status.sample['received']==1.
    status.update_clock(10600,1.5)
    assert not status.locked_disarmed(1.5,identity_verified=True)


def test_repeated_cached_source_cannot_hide_source_expiry():
    status=ready()
    for now in (1.2,1.4,1.6,1.8,2.,2.2,2.4,2.6,2.8,3.):
        status.update_clock(10100+int((now-1)*1000),now)
        feed(status,wire(),now)
    assert not status.locked_disarmed(3.,identity_verified=True)


def test_source_and_transport_boundaries_and_future_clock():
    status=ready();status.update_clock(12000,1.1)
    assert not status.locked_disarmed(1.1,identity_verified=True)
    status=ready();feed(status,wire(stamp=10200),1.)
    assert not status.locked_disarmed(1.,identity_verified=True)
    status.update_clock(10250,1.1)
    assert status.locked_disarmed(1.1,identity_verified=True)
    assert not status.locked_disarmed(1.7,identity_verified=True)


def test_reboot_latches_failure_no_automatic_recovery():
    status=ready();status.update_clock(20,1.1)
    feed(status,wire(stamp=20),1.1)
    status.update_clock(10300,1.2)
    feed(status,wire(stamp=10300),1.2)
    assert status.fault and not status.locked_disarmed(1.2,identity_verified=True)


@pytest.mark.parametrize('raw',[wire(source=(2,1)),wire(source=(1,2)),wire(name=b'OTHER')])
def test_wrong_source_or_name_is_not_lock_evidence(raw):
    status=OutputLockStatus();status.update_clock(10100,1.)
    feed(status,raw,1.)
    assert status.sample is None


def test_source_regression_latches_failure():
    status=ready();feed(status,wire(stamp=9999),1.1)
    assert status.fault and not status.locked_disarmed(1.1,identity_verified=True)


def test_assessment_explains_future_source_without_accepting_it():
    status=ready();feed(status,wire(stamp=10200),1.)
    detail=status.assess(1.,identity_verified=True)
    assert detail['source_age_ms']==-100
    assert detail['reasons']==['source_future'] and not detail['valid']
    status.update_clock(10250,1.1)
    assert status.assess(1.1,identity_verified=True)['valid']


def test_assessment_distinguishes_identity_from_source_expiry():
    status=ready()
    assert status.assess(1.)['reasons']==['identity_not_fresh']
    status.update_clock(12000,1.1)
    assert status.assess(1.1,identity_verified=True)['reasons']==['source_expired']


def test_source_timestamp_is_not_replaced_by_transmission_time_in_firmware():
    root=Path(__file__).resolve().parents[3]
    header=(root/'micoair_shell_firmware_20261013_ikdyTI/source/src/modules/mavlink/streams/OUTPUT_LOCK_STATUS.hpp').read_text()
    assert 'state.timestamp / 1000ULL' in header
    assert '0x4c530100u' in header and '"OUTLOCK1"' in header
    assert 'hrt_absolute_time' not in header
