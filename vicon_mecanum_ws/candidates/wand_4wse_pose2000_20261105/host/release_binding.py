"""Exact new-image deployment, read-only proof, and private host snapshot."""
import json
from pathlib import Path
from flash_receipt import verify_flash_record,require,sha,RECORD,RECORD_SHA,RELEASE_SHA,IMAGE_SHA

HOST=Path(__file__).resolve().parent
ROOT=HOST.parent
ZERO=Path('/home/czy/vicon_mecanum_ws/candidates/wand_4wse_admission_20261104/evidence/live_zero_119ojqfn/LIVE_RESULT.json')
ZERO_SHA='869b6ecfbd3a8b768c2d5b5441c35020f81e8972ef481c88b3f1e1b3d403ac10'
MIGRATION=dict(revision='CONTINUITY_V1',image_sha256=IMAGE_SHA,
    record=str(RECORD),record_sha256=RECORD_SHA,parent_release_sha256=RELEASE_SHA,
    disabled_handshake=str(ZERO),disabled_handshake_sha256=ZERO_SHA,
    inherited_physical_qualification=False,new_revision_physical_qualified=False)

def verify_zero_proof():
    require(sha(ZERO)==ZERO_SHA,'disabled handshake proof changed')
    r=json.loads(ZERO.read_text());s=r['status']
    require(r.get('passed') is True and r.get('mode')=='DISABLED_ZERO_ONLY' and r.get('identity')=='4WSE'
        and r.get('hardware_socket_opened') is True and r.get('software_loopback_only') is False
        and r.get('flash_record_sha256')==RECORD_SHA and r.get('parent_release_sha256')==RELEASE_SHA
        and r.get('fault_bits')==0 and r.get('clock',{}).get('ready') is True,'real new-image zero handshake required')
    for key in ('enable_sent','select_sent','clear_fault_sent','motion_started','physical_motion_authority'):
        require(r.get(key) is False,'zero proof must not contain motion authority')
    require(s.get('tag')=='4WSE' and s.get('coupled_shape_revision')==3 and s.get('output_authority_version')==8
        and s.get('selected') is False and s.get('permitted') is False
        and all(not any(s[k]) for k in ('compares','raw_duty','target_cps','body_milli','effective_cps','granted_direction')),
        'zero proof output/identity mismatch')
    return r

def verify_migration(profile):
    require(profile.get('firmware_deployment')==MIGRATION,'exact new-image migration binding required')
    verify_flash_record(RECORD);verify_zero_proof()
    return True

def verify_host_snapshot(root=HOST):
    require(Path(root).resolve()==HOST,'private deployment host required')
    m=json.loads((HOST/'HOST_MANIFEST.json').read_text())
    require(m.get('identity')=='4WSE' and m.get('revision')=='POSE2000_BROWSER_V1'
        and m.get('host_offline_passed') is True and m.get('hardware_launch_enabled') is True
        and m.get('deployment_sha256')==RECORD_SHA and m.get('parent_release_sha256')==RELEASE_SHA
        and m.get('motion_requires_explicit_start') is True and m.get('physical_qualified') is False,
        'qualified new-image controlled-test host required')
    for group in ('source_hashes','evidence'):
        require(isinstance(m.get(group),dict) and bool(m[group]),'missing private host seal')
        for relative,digest in m[group].items():
            p=(ROOT/relative).resolve()
            require(p.is_relative_to(ROOT) and sha(p)==digest,'private host changed: '+relative)
    verify_flash_record(RECORD);verify_zero_proof()
    return m
