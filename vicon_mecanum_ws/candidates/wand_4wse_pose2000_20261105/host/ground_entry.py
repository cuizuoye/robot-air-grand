"""Bounded private hardware profile or dedicated localhost C rehearsal."""
import hashlib
import json
from pathlib import Path
import secrets
import socket
import time

HERE=Path(__file__).resolve().parent
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def free_port():
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1',0));return s.getsockname()[1]
def profile(ready,folder,software,site_ready=False,operator_trial=False,seconds=600.):
    if Path(ready).resolve()!=HERE or type(software) is not bool:raise ValueError('private explicit runtime required')
    if type(seconds) not in (int,float) or not 1<=seconds<=600:raise ValueError('bounded session required')
    if not software and (site_ready is not True or operator_trial is not True):raise ValueError('fresh controlled-test site declaration required')
    p=json.loads((HERE/'profiles/stage6.LONG600_NOT_APPROVED.json').read_text())
    p.update(identity='4WSE',software_rehearsal=software,namespace='/wand_4wse_test_'+secrets.token_hex(5),
        baseline_observation={},firmware_deployment={},commissioning_review={},operator_duration_trial={},physical_qualified=False)
    if software:
        ports=[free_port(),free_port()]
        while ports[0]==ports[1] or any(v in (46000,46001) for v in ports):ports=[free_port(),free_port()]
        p['base_yaw_offset_rad']=0.
        p['hardware'].update(esp_ip='127.0.0.1',network_interface='lo',command_port=ports[0],telemetry_port=ports[1])
        consent='LOOPBACK_ONLY_C_REHEARSAL'
    else:
        from release_binding import MIGRATION,verify_flash_record,verify_zero_proof
        from mecanum_wand_4wse_hardware.ground_admission import KIND,CONSENT
        verify_flash_record();verify_zero_proof()
        p['hardware'].update(command_port=46000,telemetry_port=46001)
        p['firmware_deployment']=dict(MIGRATION)
        p['confirmations'].update(wiring_secured=True,power_cut_ready=True)
        p['commissioning_review']=dict(kind=KIND,issued_utc_s=time.time(),human_authorized_stage=6,
            operator_declared_site_ready_now=True,controlled_new_image_test=True,
            measurements_still_unknown=True,no_physical_qualification_claim=True,max_active_s=seconds)
        consent=CONSENT
    path=Path(folder)/'PROFILE.json';path.write_text(json.dumps(p,indent=2)+'\n')
    return path,consent
