"""File-only binding to the independently verified 4WSE deployment.

No sockets, ROS initialization, probe access, flashing or motion authority.
The sealed offline candidate and the original demonstration remain unchanged.
"""
import hashlib
import json
from pathlib import Path

PARENT=Path('/home/czy/vicon_mecanum_ws/candidates/wand_continuity_20261104')
RELEASE=PARENT/'RELEASE.json'
RELEASE_SHA='f4444263796973cfdfe765bd71cc30735c5982b86c029872b82bd493c9b006ec'
BACKUP=Path('/home/czy/vicon_mecanum_ws/stm32/mecanum_f407/backups/pre_4wse_20261104_P9wp89')
RECORD=BACKUP/'DEPLOYMENT_4WSE_2026-11-04.json'
RECORD_SHA='46dd2cdb91d013ba3037e0e25a53e85f4a435b62d57429d0311b30ee99555327'
IMAGE_SHA='cd71c7cc4ee2fcb629f4a7ba47efb618fca15ef63be8e03fca950ff126eaab9d'
READBACK_SHA='0f42140027f2bb7c81dba18d90ebb5738dfafef8447828d73e565f48546e9b1e'
OLD_FLASH_SHA='9c0a1139b4da4bf61a3a3bc4d751c0a60189542a17aedea47240b8730783994e'

def require(condition,message):
    if not condition:raise ValueError(message)
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def validate_metadata(r):
    require(isinstance(r,dict),'deployment record must be an object')
    expected=dict(schema='4wse-deployment-v1',identity='4WSE',firmware_revision='CONTINUITY_V1',
        state='FLASH_VERIFIED_CORE_HALTED_MOTION_NOT_AUTHORIZED',
        image_sha256=IMAGE_SHA,image_bytes=31720,full_backup_bytes=1048576,
        full_readback_sha256=READBACK_SHA,backup_sha256=OLD_FLASH_SHA,
        protocol_authority_version=8,diagnostic_payload_bytes=240,coupled_shape_revision=3,
        stlink_sn='E1007200D0D2139393740544',chip_uid_words=['004D0051','33355113','31323636'],
        device_id='0x413',candidate_release_sha256=RELEASE_SHA,erased_sectors=[0,1],
        erased_padding_bytes_verified_ff=1048,flash_unchanged_from_address='0x08008000')
    for k,v in expected.items():require(type(r.get(k)) is type(v) and r[k]==v,'deployment field mismatch: '+k)
    for k in ('backup_verified','two_full_backups_identical','flashed','independent_readback_verified',
              'image_prefix_matches_byte_for_byte','option_bytes_unchanged','final_core_halted',
              'user_authorized_flash','user_confirmed_battery_and_motor_power_disconnected'):
        require(r.get(k) is True,'deployment proof missing: '+k)
    for k in ('application_run_command_issued','motor_motion_command_sent','esp32_modified','network_changed',
              'motion_authorized','physical_performance_qualified','host_hardware_migration_complete'):
        require(r.get(k) is False,'cannot convert flash receipt to motion authority: '+k)

def artifact_path(r,key,name):
    p=Path(r[key]).resolve()
    require(p==(BACKUP/name).resolve(),'unexpected artifact path: '+key)
    return p

def verify_parent():
    require(sha(RELEASE)==RELEASE_SHA,'sealed parent release changed')
    r=json.loads(RELEASE.read_text())
    require(r['identity']=='4WSE' and r['revision']=='CONTINUITY_V1','wrong parent candidate')
    for group in ('source_hashes','artifact_hashes','evidence'):
        for relative,digest in r[group].items():
            p=(PARENT/relative).resolve()
            require(p.is_relative_to(PARENT) and sha(p)==digest,'parent seal changed: '+relative)
    return r

def verify_flash_record(record_path=RECORD,build_manifest=None):
    require(Path(record_path).resolve()==RECORD.resolve(),'only the pinned new deployment is admitted')
    require(build_manifest is None or Path(build_manifest).resolve()==RELEASE.resolve(),'wrong build manifest')
    require(sha(RECORD)==RECORD_SHA,'deployment receipt changed')
    r=json.loads(RECORD.read_text());validate_metadata(r);verify_parent()
    require(Path(r['candidate_release']).resolve()==RELEASE.resolve(),'wrong candidate release path')
    image=artifact_path(r,'archived_programmed_image','4wse_continuity_v1_programmed_image.bin')
    a=artifact_path(r,'backup_pass1_path','flash_before_pass1_1MiB.bin')
    b=artifact_path(r,'backup_pass2_path','flash_before_pass2_1MiB.bin')
    after=artifact_path(r,'full_readback_path','flash_after_4wse_1MiB.bin')
    require(sha(image)==IMAGE_SHA and image.stat().st_size==31720,'wrong archived firmware')
    require(a.stat().st_size==1048576 and sha(a)==sha(b)==OLD_FLASH_SHA,'complete double backup differs')
    require(after.stat().st_size==1048576 and sha(after)==READBACK_SHA,'independent full readback changed')
    raw=after.read_bytes();old=a.read_bytes();code=image.read_bytes()
    require(raw[:len(code)]==code,'board readback prefix differs from new image')
    require(raw[len(code):32768]==b'\xff'*1048 and raw[32768:]==old[32768:],'flash erase boundary changed')
    for name in ('option_bytes_before_pass1.bin','option_bytes_before_pass2.bin','option_bytes_after.bin'):
        p=BACKUP/name
        require(p.stat().st_size==16 and sha(p)==r['option_bytes_sha256'],'option bytes differ')
    for name,digest in r['log_sha256'].items():
        p=(BACKUP/name).resolve()
        require(p.is_relative_to(BACKUP) and p.suffix=='.log' and sha(p)==digest,'flash log changed')
    return r
