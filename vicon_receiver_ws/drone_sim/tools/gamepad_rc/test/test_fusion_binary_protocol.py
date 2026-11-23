from copy import deepcopy
from pathlib import Path
import subprocess
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import fusion_binary_record as legacy
from fusion_binary_protocol import (FUSION_SIZE, RESET_FIELDS, VISUAL, encode_fusion,
    decode_fusion, encode_visual, decode_visual, message)
from fusion_binary_receiver import FusionBinaryReceiver, fusion_evidence


def row(stamp=1_000_000):
    flags = dict.fromkeys(legacy.FLAGS, False)
    flags.update(timestamp=stamp, timestamp_sample=stamp-100,
                 cs_tilt_align=True, cs_baro_hgt=True, cs_ev_pos=True, cs_ev_hgt=True, cs_ev_yaw=True)
    aid = dict(timestamp=stamp, timestamp_sample=stamp-100, time_last_fuse=stamp-10,
               estimator_instance=0, fusion_enabled=True, fused=True, innovation_rejected=False,
               test_ratio=.5, observation_variance=1.)
    return dict(flags=flags,
        selector=dict(timestamp=stamp, primary_instance=0, instances_available=1,
                      primary_healthy=True, healthy=[True]+[False]*8, gyro_fault_detected=False,
                      accel_fault_detected=False, instance_changed_count=2, accel_device_id=3, gyro_device_id=4),
        local=dict(timestamp=stamp, timestamp_sample=stamp-100, xy_valid=True, z_valid=True,
                   **dict.fromkeys(RESET_FIELDS, 0)),
        aid={**aid, 'test_ratio':[.5,.25], 'observation_variance':[1.,2.]},
        height_aid=dict(aid), yaw_aid=dict(aid))


def visual(stamp=1_000_000):
    return dict(timestamp=stamp, timestamp_sample=stamp, pose_frame=1, velocity_frame=0,
                position=[1.,2.,3.], q=[1.,0.,0.,0.], reset_counter=0, quality=0)


def wire(payload, source=(1,1)):
    e=m.MAVLink(None,srcSystem=source[0],srcComponent=source[1])
    return message(e,payload).pack(e)


def test_protocol_roundtrip_and_full_payload():
    data=row();b=encode_fusion(data,7)
    assert len(b)==FUSION_SIZE==249
    assert decode_fusion(b)==dict(data, sequence=7)
    b=encode_visual(visual(),8)
    assert len(b)==VISUAL.size==56
    assert decode_visual(b.ljust(249,b'\0'))==dict(visual(),sequence=8)


@pytest.mark.parametrize('change',['version','trailing','reserved_health','mixed_estimator'])
def test_schema_failures(change):
    data=row();b=encode_fusion(data,1)
    if change=='version':b=b'EVF9'+b[4:]
    if change=='trailing':b+=b'\0'
    if change=='reserved_health':b=b[:223]+bytes([0x80])+b[224:]
    if change=='mixed_estimator':
        data['yaw_aid']['estimator_instance']=1;b=encode_fusion(data,1)
    with pytest.raises(ValueError):decode_fusion(b)


def test_split_corrupted_and_intact_frames():
    receiver=FusionBinaryReceiver()
    corrupt=bytearray(wire(encode_fusion(row(),1)));corrupt[20]^=0x40
    intact=wire(encode_fusion(row(1_500_000),2))
    accepted=[]
    for i,b in enumerate(bytes(corrupt)+intact):
        accepted+=receiver.feed(bytes([b]),i/10000,identity_verified=True)
    assert len(accepted)==1 and accepted[0][1]['sequence']==2
    assert receiver.crc_rejections>=1 and receiver.fault is None


def test_wrong_sender_target_and_identity():
    r=FusionBinaryReceiver()
    assert not r.feed(wire(encode_fusion(row(),1),source=(2,1)),0,identity_verified=True)
    e=m.MAVLink(None,srcSystem=1,srcComponent=1);msg=message(e,encode_fusion(row(),1))
    msg.target_system=251
    assert not r.feed(msg.pack(e),.1,identity_verified=True)
    assert not r.feed(wire(encode_fusion(row(),1)),.2,identity_verified=False)
    assert not r.feed(wire(encode_fusion(row(),2)),.3,identity_verified=True)
    assert r.fault=='identity_or_boot_session_lost'


def test_stale_state_latches_and_cannot_resume():
    r=FusionBinaryReceiver();r.feed(wire(encode_fusion(row(),1)),0,identity_verified=True)
    # New packet with old uORB time is not new evidence.
    r.feed(wire(encode_fusion(row(),2)),.8,identity_verified=True)
    with pytest.raises(RuntimeError):r.snapshot('fusion',1.01,fc_now_us=1_500_000,identity_verified=True)
    assert not r.feed(wire(encode_fusion(row(1_500_000),3)),1.1,identity_verified=True)


def test_one_hz_flags_dont_hide_progressing_aid_and_local_sources():
    r=FusionBinaryReceiver()
    first=row();r.feed(wire(encode_fusion(first,1)),0,identity_verified=True)
    for i in (1,2):
        current=row(1_000_000+i*500_000)
        current['flags']=deepcopy(first['flags'])  # PX4 publishes unchanged flags at ~1Hz.
        r.feed(wire(encode_fusion(current,i+1)),i*.5,identity_verified=True)
        assert r.snapshot('fusion',i*.5,fc_now_us=current['local']['timestamp'],identity_verified=True)['aid']['timestamp']==current['aid']['timestamp']


@pytest.mark.parametrize('change',['switch','reset','imu','source_regression','sequence_regression'])
def test_frozen_state_rejects_changes(change):
    r=FusionBinaryReceiver();r.feed(wire(encode_fusion(row(),10)),0,identity_verified=True)
    r.freeze((0,2));next_row=row(1_500_000);seq=11
    if change=='switch':next_row['selector']['instance_changed_count']+=2  # switched away and back
    if change=='reset':next_row['local']['xy_reset_counter']=1
    if change=='imu':next_row['selector']['gyro_device_id']=5
    if change=='source_regression':next_row['flags']['timestamp']=900_000
    if change=='sequence_regression':seq=9
    assert not r.feed(wire(encode_fusion(next_row,seq)),.5,identity_verified=True)
    assert r.fault


@pytest.mark.parametrize('stamp',[0,2_000_001])
def test_source_time_rejected(stamp):
    r=FusionBinaryReceiver();data=row();data['flags']['timestamp']=stamp
    r.feed(wire(encode_fusion(data,1)),0,identity_verified=True)
    with pytest.raises(RuntimeError):r.snapshot('fusion',.1,fc_now_us=2_000_000,identity_verified=True)


def test_decoded_evidence_uses_existing_fusion_policy():
    rows=[decode_fusion(encode_fusion(row(t),i)) for i,t in enumerate((1_000_000,1_500_000,2_000_000))]
    yaw=[dict(fresh=True,fc_minus_vicon_wrapped_deg=.2)]*3
    assert fusion_evidence(rows,yaw)
    for key in ('aid','height_aid','yaw_aid'):
        bad=deepcopy(rows);bad[1][key]['fused']=False
        assert not fusion_evidence(bad,yaw)
    bad=deepcopy(rows);bad[1]['selector']['instance_changed_count']+=2
    assert not fusion_evidence(bad,yaw)
    assert not fusion_evidence(rows,[dict(fresh=False)]*3)


def test_archived_aid_values_and_policy_are_preserved():
    from fusion_binary_offline import archive_core_check
    root=Path(__file__).resolve().parents[3]
    result=archive_core_check(root/'mircoair_combined_aiding_20261013_5Q5FHe/combined_aiding_live.json')
    assert result['rows']==3 and result['core_decision_unchanged'] and result['aid_fields_lossless']
    assert not result['EVF2_revalidation_possible']


def test_receipt_and_snapshot_are_not_mutable_aliases():
    r=FusionBinaryReceiver()
    accepted=r.feed(wire(encode_visual(visual(),1)),0,identity_verified=True)
    assert accepted[0][0]=='visual'
    accepted[0][1]['timestamp']=0
    result=r.snapshot('visual',.1,fc_now_us=1_000_001,identity_verified=True)
    assert result['timestamp']==1_000_000


def test_cpp_layout_matches_python_encoder(tmp_path):
    root=Path(__file__).resolve().parents[3]
    includes=root/'micoair_shell_firmware_20261013_ikdyTI/source/src/modules/mavlink/streams'
    source=Path(__file__).with_name('fusion_binary_golden.cpp')
    binary=tmp_path/'golden'
    subprocess.run(['g++','-std=c++14','-Wall','-Werror','-I',str(includes),str(source),'-o',str(binary)],check=True,capture_output=True)
    output=subprocess.run([str(binary)],check=True,capture_output=True).stdout
    assert output[:FUSION_SIZE]==encode_fusion(row(),7)
    assert output[FUSION_SIZE:]==encode_visual(visual(),8)


def inactive_infinite_payload(stamp=1_000_000):
    data=row(stamp)
    for key,flag in (('aid','cs_ev_pos'),('yaw_aid','cs_ev_yaw')):
        data['flags'][flag]=False
        data[key].update(fusion_enabled=False,fused=False,innovation_rejected=True,time_last_fuse=0)
    payload=bytearray(encode_fusion(data,1))
    for index in (0,2):
        offset=legacy.HEAD.size+index*legacy.AID.size
        fields=list(legacy.AID.unpack_from(payload,offset))
        fields[5]=float('inf')
        if index==0:fields[6]=float('inf')
        legacy.AID.pack_into(payload,offset,*fields)
    return bytes(payload)


def test_inactive_rejected_infinity_is_not_promoted_or_a_fatal_startup_fault():
    raw=wire(inactive_infinite_payload())
    r=FusionBinaryReceiver()
    assert r.feed(raw,0,identity_verified=True)==[]
    assert r.fault is None and r.latest=={} and r.received_at=={}
    assert r.not_ready_records==1 and r.last_not_ready['fields']==['aid','yaw_aid']
    # Genuine finite evidence is still required; no automatic success.
    r.feed(wire(encode_fusion(row(1_500_000),2)),.5,identity_verified=True)
    assert r.snapshot('fusion',.5,fc_now_us=1_500_001,identity_verified=True)['sequence']==2


@pytest.mark.parametrize('change',['enabled_flag','enabled_aid','fused','nan','negative_inf','variance_inf','tail_reserved','mixed_estimator'])
def test_inactive_classification_never_hides_invalid_evidence(change):
    payload=bytearray(inactive_infinite_payload())
    head=list(legacy.HEAD.unpack_from(payload))
    fields=list(legacy.AID.unpack_from(payload,legacy.HEAD.size))
    if change=='enabled_flag':head[3]|=1<<legacy.FLAGS.index('cs_ev_pos')
    if change=='enabled_aid':fields[4]|=1
    if change=='fused':fields[4]|=2
    if change=='nan':fields[5]=float('nan')
    if change=='negative_inf':fields[5]=float('-inf')
    if change=='variance_inf':fields[7]=float('inf')
    if change=='mixed_estimator':fields[3]=1
    legacy.HEAD.pack_into(payload,0,*head)
    legacy.AID.pack_into(payload,legacy.HEAD.size,*fields)
    if change=='tail_reserved':payload[223]|=0x80
    r=FusionBinaryReceiver()
    assert not r.feed(wire(bytes(payload)),0,identity_verified=True)
    assert r.fault and r.not_ready_records==0


def test_unready_after_freezing_remains_a_latched_failure():
    r=FusionBinaryReceiver()
    r.feed(wire(encode_fusion(row(),0)),0,identity_verified=True)
    r.freeze((0,2))
    r.feed(wire(inactive_infinite_payload(1_500_000)),.5,identity_verified=True)
    assert r.fault=='binary_fusion_became_unavailable'


def test_recorded_battery_only_payloads_replay_without_accepting_unready_fusion():
    # CRC-valid unedited TRS frame captured 2026-10-20, battery-only, inactive
    # horizontal/yaw aids. Embed it so another checkout needs no local logs.
    raw=bytes.fromhex(
        'fdfb00003a0101f80000f2fd00fad545564632456a980e00000000a635970e00000000'
        '030000000002d3a0990e000000000201499c100700000000bf0b0f0700000000000000'
        '00000000000004000000000000f07f000000000000f07f00000060e17a843f00000060'
        'e17a843f489c100700000000bf0b0f070000000054150f07000000000003000000405a'
        '7c823e000000000000000000000060e17a843f00000000000000004b9c100700000000'
        'bf0b0f070000000000000000000000000004000000000000f07f0000000000000000'
        '000000a09999b93f00000000000000000100000012006a001200660003000102010100'
        '26a9950e00000000679f990e00000000014383')
    r=FusionBinaryReceiver()
    r.feed(raw,0,identity_verified=True)
    assert r.parser.summary()['counts']['crc_valid_frames']==1
    assert r.fault is None and r.not_ready_records>0
    assert r.latest=={} and r.received_at=={}
