#!/usr/bin/env python3
"""No-hardware replay of the binary diagnostic candidate and its fault handling."""
import argparse
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import shutil

from pymavlink.dialects.v20 import common as m
from fusion_binary_protocol import RESET_FIELDS, encode_fusion, message
from fusion_binary_receiver import FusionBinaryReceiver, fusion_evidence
from fusion_binary_record import FLAGS
import fusion_binary_record as legacy
import usb_combined_fusion_policy as combined


def synthetic_row(stamp):
    """Artificial diagnostic data, never archived/live flight evidence."""
    flags = dict.fromkeys(FLAGS, False)
    flags.update(timestamp=stamp, timestamp_sample=stamp-100, cs_tilt_align=True,
                 cs_baro_hgt=True, cs_ev_pos=True, cs_ev_hgt=True, cs_ev_yaw=True)
    aid = dict(timestamp=stamp, timestamp_sample=stamp-100, time_last_fuse=stamp-10,
               estimator_instance=0, fusion_enabled=True, fused=True, innovation_rejected=False,
               test_ratio=.1, observation_variance=.2)
    return dict(flags=flags, selector=dict(timestamp=stamp, primary_instance=0,
        instances_available=1, primary_healthy=True, healthy=[True]+[False]*8,
        gyro_fault_detected=False, accel_fault_detected=False,
        instance_changed_count=0, accel_device_id=3, gyro_device_id=4),
        local=dict(timestamp=stamp, timestamp_sample=stamp-100, xy_valid=True, z_valid=True,
                   **dict.fromkeys(RESET_FIELDS,0)),
        aid={**aid, 'test_ratio':[.1,.1], 'observation_variance':[.2,.2]},
        height_aid=dict(aid), yaw_aid=dict(aid))


def packet(row, seq, source=(1,1)):
    encoder=m.MAVLink(None,srcSystem=source[0],srcComponent=source[1])
    return message(encoder,encode_fusion(row,seq)).pack(encoder)


def scenario(name):
    receiver=FusionBinaryReceiver();rows=[];fault=None;wire_bytes=0
    for i in range(3):
        now=i*.5;stamp=1_000_000+i*500_000;row=synthetic_row(stamp)
        identity=True;source=(1,1);seq=i
        if i and name=='cached_publications':row=synthetic_row(1_000_000)
        if i and name=='duplicate_sequence':seq=0
        if i and name=='selector_change':row['selector']['instance_changed_count']=2
        if i and name=='position_reset':row['local']['xy_reset_counter']=1
        if i and name=='imu_change':row['selector']['gyro_device_id']=5
        if i and name=='identity_lost':identity=False
        if name=='wrong_sender':source=(2,1)
        if name=='mixed_estimator':row['yaw_aid']['estimator_instance']=1
        if name=='fusion_rejected':row['yaw_aid']['innovation_rejected']=True
        if name=='future_source':row['flags']['timestamp']=stamp+100_000
        wire=packet(row,seq,source)
        if name=='crc_stream_lost' and i:
            wire=bytearray(wire);wire[20]^=0x40;wire=bytes(wire)
        if name=='one_crc_then_recovery' and i==1:
            broken=bytearray(wire);broken[20]^=0x40
            receiver.feed(bytes(broken),now-.01,identity_verified=True)
        wire_bytes+=len(wire)
        receiver.feed(wire,now,identity_verified=identity)
        try:
            snapshot=receiver.snapshot('fusion',now,fc_now_us=stamp,identity_verified=identity)
            if receiver.signature is None:
                receiver.freeze((snapshot['selector']['primary_instance'],snapshot['selector']['instance_changed_count']))
            rows.append(deepcopy(snapshot))
        except RuntimeError as exc:
            fault=str(exc)
    yaw=[dict(fresh=True,fc_minus_vicon_wrapped_deg=0.) for _ in rows]
    healthy=not fault and fusion_evidence(rows,yaw)
    expected=name in ('normal','one_crc_then_recovery')
    return dict(scenario=name,passed=healthy==expected,expected_healthy=expected,
                fusion_evidence_accepted=healthy,receiver_fault=receiver.fault,
                last_observation_error=fault,crc_rejections=receiver.crc_rejections,
                generated_wire_bytes=wire_bytes,synthetic_rows=len(rows),flight_authorized=False)


SCENARIOS=('normal','one_crc_then_recovery','crc_stream_lost','cached_publications',
           'duplicate_sequence','selector_change','position_reset','imu_change',
           'identity_lost','wrong_sender','mixed_estimator','fusion_rejected','future_source')


def archive_core_check(path):
    """Compare recorded fields only; missing health/reset data never becomes EVF2 proof."""
    record=json.loads(path.read_text());original=record['fusion_snapshots']
    decoded=[]
    for row in original:
        projection=deepcopy(row)
        # Older capture did not record selector health. Encode UNKNOWN as false,
        # report the missing evidence, and never use it as new health proof.
        projection['selector']['primary_healthy']=False
        new=legacy.decode(legacy.encode(projection))
        new['yaw_comparison']=deepcopy(row['yaw_comparison'])
        decoded.append(new)
        for key in ('aid','height_aid','yaw_aid'):
            if any(new[key][field]!=row[key][field] for field in new[key]):
                raise RuntimeError('Archived aid values changed')
    primary=record['primary_estimator']
    before=combined.fusion_evidence_ok(original,primary)
    after=combined.fusion_evidence_ok(decoded,primary)
    return dict(source=str(path),rows=len(original),aid_fields_lossless=True,
                original_core_policy_result=before,decoded_core_policy_result=after,
                core_decision_unchanged=before==after,EVF2_revalidation_possible=False,
                missing_recorded_fields=['selector.healthy','local reset counters'],
                scope='Historical core-value preservation only; no new hardware/session or health proof')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--candidate-package',type=Path)
    parser.add_argument('--archive',type=Path)
    args=parser.parse_args()
    args.output_dir.mkdir(parents=True,exist_ok=False)
    cases=[scenario(name) for name in SCENARIOS]
    report=dict(scope='Synthetic bytes -> strict MAVLink CRC -> EVF2 -> existing fusion policy; no hardware',
                passed=all(c['passed'] for c in cases),cases=cases,devices_opened=0,
                physical_tx_bytes=0,flashed=False,flight_authorized=False,
                runtime_runner_integration_complete=False)
    if args.archive:
        report['archived_core_replay']=archive_core_check(args.archive)
        report['passed'] &= report['archived_core_replay']['core_decision_unchanged']
    if args.candidate_package:
        with args.candidate_package.open('rb') as source, (args.output_dir/'candidate_NOT_FLASHED.px4').open('xb') as dest:
            shutil.copyfileobj(source,dest)
        report['candidate_sha256']=hashlib.sha256((args.output_dir/'candidate_NOT_FLASHED.px4').read_bytes()).hexdigest()
    with (args.output_dir/'report.json').open('x') as stream:
        json.dump(report,stream,indent=2,allow_nan=False);stream.write('\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))
    return 0 if report['passed'] else 2


if __name__=='__main__':
    raise SystemExit(main())
