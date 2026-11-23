"""One-use separate powered LOCKED static exception; global holds stay intact."""
import argparse
import datetime
import json
import os
from pathlib import Path
import time

from receive_only_permit import digest, UID, TOOLS
from rigid_body_revision import inspect_current_calibration
from trs_static_fusion_bench import load_backup
from trs_transport_config import add_baud_argument, require_baud

SCOPE='battery-only-single-trs-locked-static-fusion-once-no-flight'
HOLDS=TOOLS/'known_incidents'


def recent(record, maximum=1800):
    captured=datetime.datetime.fromisoformat(record['captured_at_utc'])
    if captured.utcoffset() is None or not 0<=time.time()-captured.timestamp()<maximum:
        raise ValueError('Fresh timezone-qualified evidence required')
    return captured.timestamp()


def validate(backup, output, heading, board, attitude, home, static_proof, *, baud):
    baud=require_baud(baud)
    from trs_runtime_monitor import inspect_evidence
    registry=inspect_current_calibration(heading,board)
    inspect_evidence(heading,board,static_proof)
    load_backup(Path(backup),vision_reference_static=True)
    a,h=[json.loads(Path(p).read_text()) for p in (attitude,home)]
    at,ht=recent(a),recent(h)
    if (a.get('static_observation_passed') is not True or a.get('error')
        or a.get('identity',{}).get('uid2')!=UID or a.get('identity',{}).get('firmware')!='1.14.3'
        or a.get('actual_sent_to_fc')!=0 or not 0<=a.get('median_gravity_axis_difference_deg',999)<=5
        or not a.get('checks') or not all(a['checks'].values())
        or Path(a['heading_candidate']).resolve()!=Path(heading).resolve()
        or h.get('lockdown_after_release_verified') is not True or h.get('error')
        or h.get('guarded_props_installed_stop_only_confirmed') is not True
        or h.get('fc_usb_present_at_end') is not False
        or h.get('evidence',{}).get('identity',{}).get('uid2')!=UID
        or h.get('evidence',{}).get('identity',{}).get('firmware')!='1.14.3'
        or h.get('passive_power',{}).get('reboot_observed') is not False
        or not h.get('home_events') or not h.get('acks')
        or any(ack['result']!=0 or ack.get('source')!=[1,1] or ack.get('target')!=[250,199] for ack in h['acks'])
        or not any(event.get('value')==1 for event in h['home_events'])
        or not any(event.get('value')==0 for event in h['home_events'])):
        raise ValueError('Fresh same-revision attitude and battery-only HOME lock evidence required')
    state=h['snapshots'][-1]
    if state['armed'] or not state['lockdown'] or state['force_failsafe'] or state['manual_lockdown']:
        raise ValueError('Latest retained actuator evidence must be unarmed and locked')
    if abs((state['timestamp_us']/1e6-a['last_pair']['fc_boot_ms']/1000)-(ht-at))>4:
        raise ValueError('Attitude and HOME evidence do not share a consistent FC boot clock')
    expected={'schema':2,'scope':SCOPE,'user_authorized':True,'uid2':UID,
        'trs_baud':baud,'rx_parser':'strict_crc_bytewise_resynchronization',
        'output_path':str(Path(output).resolve()),'backup_path':str(Path(backup).resolve()),
        'heading_path':str(Path(heading).resolve()),'board_path':str(Path(board).resolve()),
        'attitude_path':str(Path(attitude).resolve()),'home_path':str(Path(home).resolve()),
        'static_proof_path':str(Path(static_proof).resolve()),
        'rigid_body_revision_id':registry['active_revision_id'],
        'max_stream_seconds':20,'max_packets':640,'max_fusion_seconds':12,
        'max_post_test_stop_only_watch_seconds':120,
        'allowed_parameter_values':{'EKF2_EV_CTRL':[2,11],'EKF2_EV_DELAY':[0,65]},
        'physical_conditions':['battery_only','fc_usb_absent','props_installed_protected','stationary','output_locked'],
        'commands_forbidden':['arm','unlock','mode','target','takeoff','land','reboot'],
        'flight_authorized':False,'global_holds_modified':False,
        'reference_boot_s':state['timestamp_us']/1e6,'reference_utc_unix':ht}
    paths={'backup':Path(backup)/'parameters_verified.json','manifest':Path(backup)/'parameters_manifest.json',
        'heading':Path(heading),'board':Path(board),'attitude':Path(attitude),'home':Path(home),
        'static_proof':Path(static_proof),'permit_policy':Path(__file__)}
    for name in ('trs_powered_fusion_policy.py','trs_powered_fusion_bench.py','compact_vision.py',
                 'vicon_fc_attitude_check.py','persistent_shell.py','ground_estimator_settlement.py',
                 'ekf_compare_readonly_policy.py','rigid_body_revision.py','trs_runtime_monitor.py',
                 'estop_only_policy.py','mavlink_readonly_probe.py','trs_vision_reference_policy.py',
                 'trs_static_fusion_policy.py','vision_reference_xyz_policy.py','usb_combined_fusion_policy.py',
                 'usb_yaw_fusion_policy.py','usb_height_fusion_policy.py','usb_fusion_policy.py',
                 'control_readonly_policy.py','trs_static_fusion_bench.py',
                 'diagnostic_transaction.py','actuator_query.py','trs_only_estop_bench.py',
                 'preflight_session_status.py','readiness_console.py','live_attitude_check.py',
                 'mavlink_crc_stream.py','micoair_crc_definitions.json','trs_transport_config.py'):
        paths[name]=TOOLS/name
    for name in ('frames.py','mavlink_vision.py'):
        paths[name]=TOOLS.parents[1]/'ros2_ws/src/uav_coord_bridge/uav_coord_bridge'/name
    from rigid_body_revision import REGISTRY
    paths['revision_registry']=REGISTRY
    for name in ('physical_safety_hold.json','micoair743_v1143_shell_crash.json'):
        paths[name]=HOLDS/name
    if (json.loads(paths['physical_safety_hold.json'].read_text()).get('block_props_off_benches') is not True
        or json.loads(paths['micoair743_v1143_shell_crash.json'].read_text()).get('block_active_benches') is not True):
        raise ValueError('Global safety holds must remain intact')
    return expected,paths


def consume(path,**request):
    require_baud(request.get('baud'))
    path=Path(path);record=json.loads(path.read_text());now=time.time()
    if (Path(request['output']).exists() or not record['issued_unix']<=now<record['expires_unix']
        or not 0<record['expires_unix']-record['issued_unix']<=600):
        raise ValueError('New output and current one-use powered-static permit required')
    expected,paths=validate(**request,attitude=record['attitude_path'],home=record['home_path'],static_proof=record['static_proof_path'])
    if any(record.get(k)!=v for k,v in expected.items()) or record.get('sha256')!={k:digest(v) for k,v in paths.items()}:
        raise ValueError('Powered-static scope/code/evidence mismatch')
    fd=os.open(str(path)+'.consumed.json',os.O_CREAT|os.O_EXCL|os.O_WRONLY,0o600)
    with os.fdopen(fd,'w') as stream:
        json.dump({'scope':SCOPE,'consumed_unix':now,'permit_sha256':digest(path),'flight_authorized':False},stream)
        stream.flush();os.fsync(stream.fileno())
    return record


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    add_baud_argument(ap)
    ap.add_argument('--operator-authorized-powered-static',action='store_true',required=True)
    for name in ('permit','backup','output','heading','board','attitude','home','static-proof'):
        ap.add_argument('--'+name,type=Path,required=True)
    a=ap.parse_args()
    if a.permit.exists() or a.output.exists():ap.error('New permit and output required')
    expected,paths=validate(a.backup,a.output,a.heading,a.board,a.attitude,a.home,a.static_proof,baud=a.baud)
    now=time.time()
    with a.permit.open('x') as out:
        json.dump({**expected,'issued_unix':now,'expires_unix':now+600,'sha256':{k:digest(v) for k,v in paths.items()}},out,indent=2)
        out.write('\n')
    print('Separate one-use powered LOCKED static permit prepared; no device opened or commands sent')


if __name__=='__main__':main()
