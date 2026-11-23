"""Reviewed cached IMU-selection evidence, entirely SYNTHETIC and offline."""
from copy import deepcopy
import hashlib
import json

import pytest

from ev_delay_sources import ulog_events, merge_ulog_capture, _selection_brackets
from ev_delay_capture import analyze
from test_ev_delay_sources import fake_ulog, dataset

BOOT = 'MOCK BOOT+window binding'


def reply(timestamp=1, gyro=7, accel=8):
    return ('listener sensor_selection -n 1\nTOPIC: sensor_selection\n sensor_selection\n'
            f'    timestamp: {timestamp} (1.0 seconds ago)\n'
            f'    accel_device_id: {accel} (Type: 0x6A, SPI:2 (0x00))\n'
            f'    gyro_device_id: {gyro} (Type: 0x66, SPI:2 (0x00))\nnsh> \x1b[K')


def refresh_hash(row):
    row['reply_sha256'] = hashlib.sha256(row['raw_reply'].encode()).hexdigest()


def example(remove_native=True):
    document, ulog = fake_ulog('sensor_combined')
    if remove_native:
        ulog.data_list = [d for d in ulog.data_list if d.name != 'sensor_selection']
    ulog._capture_provenance = dict(sha256='a'*64, path='/MOCK/log.ulg', size=123)
    evidence = dict(schema=1, scope='same-boot-readonly-sensor-selection',
        review_status='reviewed_offline', reviewer='SYNTHETIC_REVIEWER_NOT_A_PERMIT',
        fc_boot_session=BOOT, identity=deepcopy(ulog.msg_info_dict), ulog_sha256='a'*64,
        boot_binding_evidence='SYNTHETIC identity and continuous native FC timeline',
        fc_clock_bounds_evidence='SYNTHETIC original clock exchanges, not RTT/2',
        selection_publish_semantics_evidence='SYNTHETIC pinned source review')
    for name, host, low, high in [('before', 10**12, 450000, 500000),
                                  ('after', 10**12+17000000000, 16100000, 16200000)]:
        row = dict(route='usb_readonly_serial_control', system_id=1, component_id=1,
            command='listener sensor_selection -n 1', crc_validated=True,
            raw_reply=reply(), host_received_ns=host,
            read_fc_lower_us=low, read_fc_upper_us=high)
        refresh_hash(row)
        evidence[name] = row
    return document, ulog, evidence


def imported(ulog, evidence=None):
    return ulog_events(ulog, boot_session=BOOT, expected_info=ulog.msg_info_dict,
                       import_ns=10**15, source='sensor_combined', selection_evidence=evidence)


def test_missing_native_selection_is_not_invented_and_no_evidence_still_fails():
    _, ulog, ev = example()
    original = deepcopy(ulog.data_list)
    with pytest.raises(ValueError, match='missing_pre_window_sensor_selection'):
        imported(ulog)
    report = imported(ulog, ev)
    assert report['selection_origin'] == 'reviewed_readonly_cache_brackets'
    assert report['sensor_id'] == 7
    assert report['external_selection_evidence']['not_injected_into_ULog']
    assert report['external_selection_evidence']['native_topic_missing']
    assert not any(e.get('topic') == 'sensor_selection' for e in report['events'])
    assert [(d.name,d.data) for d in ulog.data_list] == [(d.name,d.data) for d in original]
    assert not report['physical_authorization'] and not report['flight_authorized']


def test_explicit_merge_preserves_raw_evidence_and_reuses_calibrator_without_approval():
    doc, ulog, ev = example()
    doc['capture_metadata']['fc_boot_session'] = BOOT
    doc['capture_metadata']['ulog_boot_binding_evidence'] = 'SYNTHETIC independent review'
    doc['events'] = [e for e in doc['events'] if e['kind'] not in
                     ('calibration_imu','calibration_visual')]
    original = deepcopy(doc)
    merged, summary = merge_ulog_capture(doc, ulog, expected_info=ulog.msg_info_dict,
        import_ns=10**15, selection_evidence=ev)
    assert doc == original
    assert merged['events'][:len(doc['events'])] == doc['events']
    assert merged['capture_metadata']['external_sensor_selection_evidence']['evidence'] == ev
    fit = analyze(merged)
    assert fit['candidate_identified'], fit['reasons']
    assert fit['fit']['effective_delay_candidate_ms'] == pytest.approx(37, abs=1)
    assert fit['ev_delay_parameter_candidate_ms'] is None
    assert not fit['parameter_approved'] and not fit['control_ready']
    assert not summary['physical_authorization']


@pytest.mark.parametrize('fault', [
    'unreviewed', 'wrong_boot', 'wrong_identity', 'wrong_ulog', 'no_artifact',
    'hash_changed', 'wrong_route', 'wrong_system', 'wrong_command', 'CRC_unknown',
    'before_late', 'after_early', 'host_reversed', 'FC_bounds_reversed',
    'native_time_future', 'native_time_changed', 'gyro_changed', 'accel_changed',
    'duplicate_field', 'wrong_topic', 'zero_ID', 'out_of_range', 'boolean_time',
    'no_clock_evidence', 'no_boot_evidence', 'schema_bool',
])
def test_unbound_ambiguous_or_changed_selection_is_refused(fault):
    _, ulog, ev = example()
    if fault == 'unreviewed': ev['review_status'] = 'unreviewed'
    elif fault == 'wrong_boot': ev['fc_boot_session'] = 'OTHER BOOT'
    elif fault == 'wrong_identity': ev['identity']['sys_uuid'] = 'OTHER UID'
    elif fault == 'wrong_ulog': ev['ulog_sha256'] = 'b'*64
    elif fault == 'no_artifact': del ulog._capture_provenance
    elif fault == 'hash_changed': ev['after']['raw_reply'] += 'changed'
    elif fault == 'wrong_route': ev['after']['route'] = 'TRS'
    elif fault == 'wrong_system': ev['after']['system_id'] = True
    elif fault == 'wrong_command': ev['after']['command'] = 'sensors reset'
    elif fault == 'CRC_unknown': ev['after']['crc_validated'] = False
    elif fault == 'before_late': ev['before']['read_fc_upper_us'] = 1000000
    elif fault == 'after_early': ev['after']['read_fc_lower_us'] = 10000000
    elif fault == 'host_reversed': ev['after']['host_received_ns'] = 1
    elif fault == 'FC_bounds_reversed': ev['before']['read_fc_lower_us'] = 600000
    elif fault == 'native_time_future': ev['before']['raw_reply'] = reply(timestamp=600000)
    elif fault == 'native_time_changed': ev['after']['raw_reply'] = reply(timestamp=15000000)
    elif fault == 'gyro_changed': ev['after']['raw_reply'] = reply(gyro=9)
    elif fault == 'accel_changed': ev['after']['raw_reply'] = reply(accel=9)
    elif fault == 'duplicate_field': ev['after']['raw_reply'] += '\n gyro_device_id: 7\n'
    elif fault == 'wrong_topic': ev['after']['raw_reply'] = reply().replace('TOPIC: sensor_selection', 'TOPIC: vehicle_attitude')
    elif fault == 'zero_ID': ev['after']['raw_reply'] = reply(gyro=0)
    elif fault == 'out_of_range': ev['after']['raw_reply'] = reply(gyro=2**32)
    elif fault == 'boolean_time': ev['after']['read_fc_upper_us'] = True
    elif fault == 'no_clock_evidence': del ev['fc_clock_bounds_evidence']
    elif fault == 'no_boot_evidence': del ev['boot_binding_evidence']
    elif fault == 'schema_bool': ev['schema'] = True
    if fault != 'hash_changed':
        for name in ('before','after'): refresh_hash(ev[name])
    with pytest.raises(ValueError):
        imported(ulog, ev)


def test_native_selection_takes_precedence_and_external_cache_cannot_mask_switch():
    _, ulog, ev = example(remove_native=False)
    assert imported(ulog)['selection_origin'] == 'native_ULog'
    assert imported(ulog, ev)['selection_origin'] == 'native_ULog'
    ulog.data_list.append(dataset('sensor_selection', [
        dict(timestamp=3000000, gyro_device_id=7, accel_device_id=8)]))
    # Same IDs but new native publication -> out-and-back/republish unexplained.
    with pytest.raises(ValueError, match='conflicts_with_native'):
        imported(ulog, ev)


@pytest.mark.parametrize('fault', ['IMU_gap', 'calibration_change', 'clipping',
                                  'health_error', 'no_health', 'armed', 'no_lock', 'dropout'])
def test_external_selection_never_bypasses_original_raw_IMU_health_and_lock_checks(fault):
    _, ulog, ev = example()
    byname = {d.name: d.data for d in ulog.data_list}
    if fault == 'IMU_gap':
        data = byname['sensor_combined']
        for key in data: data[key] = data[key][::3]
    elif fault == 'calibration_change': byname['sensor_combined']['gyro_calibration_count'][20] = 1
    elif fault == 'clipping': byname['sensor_combined']['gyro_clipping'][20] = 1
    elif fault == 'health_error': byname['vehicle_imu_status']['gyro_error_count'][5] = 1
    elif fault == 'no_health': ulog.data_list = [d for d in ulog.data_list if d.name != 'vehicle_imu_status']
    elif fault == 'armed': byname['actuator_armed']['armed'][0] = True
    elif fault == 'no_lock': byname['actuator_armed']['lockdown'][0] = False
    else: ulog.dropouts = [object()]
    with pytest.raises(ValueError):
        imported(ulog, ev)


def test_native_integer_text_is_not_silently_truncated_from_decimal():
    _, ulog, ev = example()
    ev['after']['raw_reply'] = reply().replace('gyro_device_id: 7 ', 'gyro_device_id: 7.5 ')
    refresh_hash(ev['after'])
    with pytest.raises(ValueError, match='native_field'):
        imported(ulog, ev)


def test_native_uint64_timestamp_and_uint32_ID_survive_without_float_conversion():
    _, ulog, ev = example()
    start = 2**53+123
    for name, lo, hi in [('before',start-2000,start-1000),
                          ('after',start+16000000,start+16001000)]:
        ev[name].update(read_fc_lower_us=lo, read_fc_upper_us=hi,
                        raw_reply=reply(timestamp=start-3000, gyro=2**32-1, accel=2**32-2))
        refresh_hash(ev[name])
    native, brackets = _selection_brackets(ev, boot_session=BOOT,
        expected_info=ulog.msg_info_dict, artifact=ulog._capture_provenance,
        lo=start, hi=start+15000000)
    assert native['timestamp'] == start-3000
    assert native['gyro_device_id'] == 2**32-1
    assert brackets[0]['read_fc_upper_us'] == start-1000


def test_new_cli_option_is_offline_only_and_passes_evidence_explicitly(tmp_path, monkeypatch):
    import phase2_acceptance
    import ev_delay_sources
    doc, ulog, ev = example()
    doc['capture_metadata']['fc_boot_session'] = BOOT
    doc['capture_metadata']['ulog_boot_binding_evidence'] = 'SYNTHETIC independent review'
    doc['events'] = [e for e in doc['events'] if e['kind'] not in ('calibration_imu','calibration_visual')]
    paths = {}
    for name, value in [('capture', doc), ('identity', ulog.msg_info_dict), ('selection', ev)]:
        paths[name] = tmp_path/(name+'.json')
        paths[name].write_text(json.dumps(value))
    monkeypatch.setattr(ev_delay_sources, 'load_ulog_file', lambda path: ulog)
    output = tmp_path/'offline.json'
    assert phase2_acceptance.main(['--output', str(output), 'delay-ulog',
        '--capture', str(paths['capture']), '--ulog', str(tmp_path/'MOCK.ulg'),
        '--identity', str(paths['identity']), '--sensor-selection-evidence', str(paths['selection'])]) == 0
    report = json.loads(output.read_text())
    assert not report['physical_authorization'] and not report['flight_authorized']
    assert report['ulog_import']['selection_origin'] == 'reviewed_readonly_cache_brackets'
    assert report['ev_delay_parameter_candidate_ms'] is None
