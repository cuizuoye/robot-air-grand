"""PX4 1.14.3紧凑位姿的纯数学：显式FC航向对齐、协方差、台架时间测量。"""

import math
import re
import sys
from pathlib import Path
import numpy as np
from pymavlink.dialects.v20 import common as m

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.frames import quaternion_to_matrix, matrix_to_quaternion
from uav_coord_bridge.mavlink_vision import checked_quaternion, finite_vector, pose_covariance

SERIES_COMMAND=b'listener vehicle_visual_odometry -n 12\n'
RATE_COMMAND=b'uorb top -1 vehicle_visual_odometry\n'


def publication_counter_rate(text):
    """One-shot rounded uORB publication counter rate; diagnostic ONLY.

    Not per-packet receipt, source matching, timestamp synchronization, or a
    maximum-gap measurement. Never use it alone to approve fusion/flight.
    """
    plain=re.sub(r'\x1b\[[0-?]*[ -/]*[@-~]','',text)
    if not re.search(r'TOPIC NAME\s+INST\s+#SUB\s+RATE\s+#Q\s+SIZE',plain):return None
    rows=re.findall(r'^\s*vehicle_visual_odometry\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s*$',plain,re.M)
    if len(rows)!=1 or int(rows[0][0])!=0:return None
    instance,subscribers,rate,queue,size=map(int,rows[0])
    return dict(instance=instance,subscribers=subscribers,rounded_publication_rate_hz=rate,
        queue_size=queue,topic_size_bytes=size,maximum_gap_verified=False,flight_authorized=False,
        scope='One short rounded publication-counter window; diagnostic only, not reception/flight acceptance')


def visual_series(text, samples=12):
    from usb_vision_policy import parse_visual_snapshot
    blocks=re.split(r'TOPIC:\s*vehicle_visual_odometry[^\n]*\n',text)[1:]
    rows=[parse_visual_snapshot('TOPIC: vehicle_visual_odometry\n'+block) for block in blocks]
    rows=[r for r in rows if r]
    return rows if len(rows)==samples else None


def series_timing(rows):
    stamps=[r['timestamp'] for r in rows]
    if len(stamps)<2 or any(b<=a for a,b in zip(stamps,stamps[1:])):
        raise ValueError('Fresh distinct increasing publication timestamps required')
    return {'samples':len(stamps),'rate_hz':(len(stamps)-1)*1e6/(stamps[-1]-stamps[0]),
            'maximum_publication_gap_ms':max(b-a for a,b in zip(stamps,stamps[1:]))/1000,
            'scope':'Short internal publication window; not an all-packets receipt count or camera synchronization'}


def ground_receipt_evidence(observations,sent,*,sample_budget_us=1000,native_rc=False):
    """Three request-bounded snapshots, NOT a receiver cadence/flight proof.

    Caller holds its same-boot stationary preparation conditions continuously.
    Mapped sample time uses the existing 150ms budget only when explicitly set;
    the default receive-time path remains at 1ms.
    Do not infer publication gaps from a print-limited NSH listener. Native
    RC preparation uses the mission's existing dispatch-gap budget; the 32Hz
    sender remains unchanged, and measured average rate is diagnostic. Legacy
    bench acceptance retains its original 30Hz/50ms requirement.
    """
    if sample_budget_us not in (1000,150000):raise ValueError('Pinned sample-time budget required')
    if type(native_rc) is not bool:raise ValueError('Explicit native RC receipt policy required')
    if len(observations)!=3 or len(sent)<12:
        raise ValueError('Three snapshots and at least twelve actual sends required')
    stamps=[]
    for item in observations:
        row=item['snapshot'];stamp=row['timestamp'];sample=row['timestamp_sample']
        before=item['requested_boot_us'];after=item['completed_boot_us']
        at=item['at']
        if (any(type(x) is not int for x in (stamp,sample,before,after))
                or any(not math.isfinite(x) for x in (at,item['requested_at']))
                or not 0<=item['requested_at']<=at or not 0<before<=after
                or not before-100_000<=stamp<=after
                or not 0<=after-stamp<=500_000 or not 0<=stamp-sample<=sample_budget_us):
            raise ValueError('Fresh nonfuture request-bounded receive-time snapshot required')
        recent=[s for s in sent if 0<=at-s['elapsed_s']<=.5]
        if not match(row,recent):
            raise ValueError('Snapshot must match a recent actually sent pose')
        stamps.append(stamp)
    if any(b<=a for a,b in zip(stamps,stamps[1:])) or stamps[-1]-stamps[0]<100_000:
        raise ValueError('Distinct progressing FC receipts over at least 100ms required')
    times=[s['elapsed_s'] for s in sent]
    if any(not math.isfinite(t) for t in times) or any(b<=a for a,b in zip(times,times[1:])):
        raise ValueError('Finite increasing actual send times required')
    rate=(len(times)-1)/(times[-1]-times[0]);gap=max(b-a for a,b in zip(times,times[1:]))
    from rc_watch import NATIVE_DISPATCH_MAX_GAP_S
    gap_limit=NATIVE_DISPATCH_MAX_GAP_S if native_rc else .05
    if gap>gap_limit or not native_rc and rate<30:
        raise ValueError(f'Host send cadence rejected: rate={rate:.3f}Hz, '
            f'max_gap={gap*1000:.3f}ms; gap_limit={gap_limit*1000:.0f}ms, '
            f'min_rate={"diagnostic_only" if native_rc else "30Hz"}')
    return dict(snapshot_count=3,fc_snapshot_progress_us=stamps[-1]-stamps[0],
        host_send_rate_hz=rate,host_maximum_send_gap_ms=gap*1000,
        cadence_policy='native_rc_mission_continuity' if native_rc else 'strict_bench',
        host_maximum_send_gap_limit_ms=gap_limit*1000,
        host_send_rate_diagnostic_only=native_rc,
        host_nominal_cadence_met=rate>=30 and gap<=.05,
        recent_matching_receipts_verified=True,maximum_fc_publication_gap_verified=False,
        all_packet_reception_rate_verified=False,source_clock_synchronization_verified=False,
        scope='Locked stationary ground fusion only; not strict preflight or mission handoff',
        flight_authorized=False)


def rotation(q_wxyz):
    w,x,y,z=checked_quaternion(q_wxyz)
    return quaternion_to_matrix([x,y,z,w])


def euler(q_wxyz):
    r=rotation(q_wxyz)
    pitch=math.asin(float(np.clip(-r[2,0],-1,1)))
    if abs(pitch)>math.radians(85):
        raise ValueError('Euler singularity proximity; no compact encoding')
    return [math.atan2(r[2,1],r[2,2]),pitch,math.atan2(r[1,0],r[0,0])]


def yaw_alignment(source_body_q,fc_body_q,*,tilt_tolerance_deg=5.):
    """只对齐静止机身航向，不冒充地理真北或跨重启持久外参。"""
    if (type(tilt_tolerance_deg) not in (int,float) or
            not math.isfinite(tilt_tolerance_deg) or not 0<tilt_tolerance_deg<=6.):
        raise ValueError('Finite tilt tolerance in (0, 6] degrees required')
    sr,sp,sy=euler(source_body_q)
    fr,fp,fy=euler(fc_body_q)
    error_deg=math.degrees(max(abs(sr-fr),abs(sp-fp)))
    if error_deg>tilt_tolerance_deg:
        raise ValueError('Source/FC tilt disagreement; no NED candidate '
                         f'(difference={error_deg:.3f} deg, limit={tilt_tolerance_deg:.3f} deg)')
    return math.atan2(math.sin(fy-sy),math.cos(fy-sy))


def aligned_pose(position,q_wxyz,alignment_yaw):
    if not math.isfinite(alignment_yaw):
        raise ValueError('Explicit finite yaw alignment required')
    c,s=math.cos(alignment_yaw),math.sin(alignment_yaw)
    a=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
    p=(a@finite_vector(position,3)).tolist()
    x,y,z,w=matrix_to_quaternion(a@rotation(q_wxyz))
    return p,[w,x,y,z]


def message(enc,stamp_us,position,q_wxyz,alignment_yaw):
    if stamp_us<=0:
        raise ValueError('Positive source timestamp required')
    p,q=aligned_pose(position,q_wxyz,alignment_yaw)
    return enc.vision_position_estimate_encode(stamp_us,*p,*euler(q),pose_covariance(),0),p,q


def pose_fields_allowed(msg):
    return (msg.get_type()=='VISION_POSITION_ESTIMATE' and msg.usec>0 and msg.reset_counter==0
            and all(math.isfinite(v) for v in (msg.x,msg.y,msg.z,msg.roll,msg.pitch,msg.yaw))
            and abs(msg.pitch)<=math.radians(85))


def pose_allowed(msg):
    return (pose_fields_allowed(msg) and all(math.isfinite(v) for v in msg.covariance)
            and list(msg.covariance)==pose_covariance())


def match(snapshot,sent):
    if snapshot.get('pose_frame')!=1 or snapshot.get('reset_counter')!=0:
        return None
    q=checked_quaternion(snapshot['q'])
    for row in reversed(sent):
        angle=math.degrees(2*math.acos(float(np.clip(abs(q@np.asarray(row['q'])),0,1))))
        distance=float(np.linalg.norm(np.asarray(snapshot['position'])-row['position']))
        if distance<=.002 and angle<=.5:
            return {'source_stamp_ns':row['source_stamp_ns'],'position_error_m':distance,
                    'quaternion_difference_deg':angle}
    return None


def reference_usable(ref,pc_now_us):
    return bool(ref and 0<=pc_now_us-ref['remote_timestamp']<1_000_000
                and 0<=ref['round_trip_time']<10_000
                and abs(ref['observed_offset']-ref['estimated_offset'])<=2000)


def arrival_measurement(row,snapshot,before,after):
    """USB时钟参考仅地面诊断；不改TRS同步器，不把发布时间说成曝光时间。"""
    send=row['pc_sent_us']
    a,b=before['remote_timestamp'],after['remote_timestamp']
    if not a<=send<=b or b<=a or not match(snapshot,[row]):
        raise ValueError('No bracketing reference or matching single real pose')
    if any(not 0<=ref['round_trip_time']<10_000 or abs(ref['observed_offset']-ref['estimated_offset'])>2000
           for ref in (before,after)):
        raise ValueError('USB reference clock outside original timing gate')
    ratio=(send-a)/(b-a)
    offset=before['estimated_offset']+ratio*(after['estimated_offset']-before['estimated_offset'])
    sent_age=send-row['wire_time_usec']
    radio_delay=snapshot['timestamp']-offset-send
    if not 0<=sent_age<=150_000 or not 0<=radio_delay<=150_000:
        raise ValueError('Unexpected source age or measured transport delay')
    return {'age_at_send_ms':sent_age/1000,'send_to_fc_publication_ms':radio_delay/1000,
            'source_ros_publication_to_fc_ms':(radio_delay+sent_age)/1000,
            'fc_publication_minus_sample_us':snapshot['timestamp']-snapshot['timestamp_sample'],
            'interpolated_usb_reference_offset_us':offset,
            'camera_exposure_latency_measured':False,'wireless_clock_conversion_verified':False,
            'scope':'USB-referenced same-source single-packet bench observation, not flight synchronization or independent accuracy'}
def next_parameter_read(names, received_at, requested_at, now, retry_seconds=.4,
                        minimum_reply_age=0.):
    """Oldest reply first, with bounded retries; requests never refresh evidence."""
    eligible = [n for n in names if now-requested_at.get(n, -1e9) >= retry_seconds
                and now-received_at.get(n, -1e9) >= minimum_reply_age]
    if not eligible:
        return None
    return min(eligible, key=lambda n: (received_at.get(n, -1e9), requested_at.get(n, -1e9)))


def parameter_freshness_details(names, received_at, requested_at, now):
    """JSON-safe diagnostic, preserving the existing strict <4s acceptance."""
    return {n: dict(reply_age_s=now-received_at[n] if n in received_at else None,
                    request_age_s=now-requested_at[n] if n in requested_at else None,
                    fresh=n in received_at and 0 <= now-received_at[n] < 4)
            for n in names}

