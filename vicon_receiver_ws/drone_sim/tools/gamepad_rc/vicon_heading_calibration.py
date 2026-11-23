#!/usr/bin/env python3
"""从两次静止Vicon端点估计实际前向；支持只读START/END采集，不修改控制配置。"""

import argparse
import datetime
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/uav_coord_bridge'))
from uav_coord_bridge.frames import matrix_to_quaternion, quaternion_to_matrix  # noqa: E402
from uav_coord_bridge.mavlink_vision import checked_quaternion, finite_vector  # noqa: E402


def endpoint(record, *, subject='MircoAir'):
    """只读采集通过且结束时新鲜、静止，才用最后一帧作端点；不伪造曝光时间。"""
    if record.get('source_checks_passed') is not True or record.get('frame_ids') != ['map']:
        raise ValueError('invalid_source_capture')
    if subject not in ('MircoAir', 'car01') or record.get('pose_topic') not in (
            f'/vicon/{subject}/{subject}', f'/vicon_b3_check/{subject}/{subject}'):
        raise ValueError('wrong_rigid_body')
    timing = record['source_timing']
    if timing['samples'] < 100 or not math.isfinite(timing['rate_hz']) or timing['rate_hz'] < 30:
        raise ValueError('insufficient_live_samples')
    gap = timing['maximum_arrival_gap_ms']
    if not math.isfinite(gap) or not 0 <= gap <= 150:
        raise ValueError('capture_continuity_exceeds_150_ms')
    std = finite_vector(record['position_std_mm'], 3)
    if np.any(std < 0) or np.any(std > 3):
        raise ValueError('endpoint_not_stationary')
    pose = record['last_pose']
    captured = datetime.datetime.fromisoformat(record['captured_at_utc'])
    if captured.utcoffset() is None:
        raise ValueError('capture_timezone_missing')
    age = captured.timestamp() - pose['stamp_ns'] / 1e9
    if not 0 <= age <= 0.15:
        raise ValueError('stale_endpoint')
    return (finite_vector(pose['position_m'], 3),
            checked_quaternion(pose['quaternion_xyzw']), pose['stamp_ns'])


def solve_heading(start, end, forward_horizontal_confirmed=False, *, subject='MircoAir'):
    if not forward_horizontal_confirmed:
        raise ValueError('manual_forward_and_level_confirmation_required')
    p0, q0, t0 = endpoint(start, subject=subject)
    p1, q1, t1 = endpoint(end, subject=subject)
    if not 0 < (t1 - t0) / 1e9 <= 1800:
        raise ValueError('endpoint_order_or_session_mismatch')
    delta = p1 - p0
    distance = float(np.linalg.norm(delta[:2]))
    # 小车沿已确认直线人工前移，距离更长不会降低方向分辨率；
    # 保留最短基线和高度约束，不沿用无人机手持动作的80cm上限。
    if distance < 0.15 or (subject != 'car01' and distance > 0.8) or abs(delta[2]) > 0.05:
        raise ValueError('translation_outside_calibration_bounds')
    rotation_change = math.degrees(2 * math.acos(float(np.clip(abs(q0 @ q1), 0, 1))))
    if rotation_change > 5:
        raise ValueError('body_turned_more_than_5_degrees')

    # 平移只观测前向；机体z向上来自用户确认水平的约束，不是IMU滚俯测量。
    x = np.array([delta[0] / distance, delta[1] / distance, 0.0])
    z = np.array([0.0, 0.0, 1.0])
    body_in_world = np.column_stack((x, np.cross(z, x), z))
    if q0 @ q1 < 0:
        q1 = -q1
    mean_q = (q0 + q1) / np.linalg.norm(q0 + q1)
    body_in_rigid = quaternion_to_matrix(mean_q).T @ body_in_world
    mount_q = matrix_to_quaternion(body_in_rigid)
    heading_world = math.degrees(math.atan2(x[1], x[0]))
    heading_rigid = math.degrees(math.atan2(body_in_rigid[1, 0], body_in_rigid[0, 0]))
    warnings = []
    if rotation_change > 2:
        warnings.append('Endpoint orientation changed >2 degrees: include this uncertainty when interpreting the candidate')
    preferred_min, preferred_max = (0.4, 0.6) if subject == 'car01' else (0.2, 0.3)
    if not preferred_min <= distance <= preferred_max:
        warnings.append(f'Observed distance differs from requested {preferred_min}..{preferred_max} m')
    return {
        'subject': subject,
        'scope': 'Offline mounting-direction candidate from real ROS captures; no flight-controller send',
        'manual_forward_horizontal_confirmed': True,
        'endpoint_position_source': 'last valid pose in each stationary capture; not an averaged pose',
        'start_stamp_ns': t0, 'end_stamp_ns': t1,
        'delta_map_m': delta.tolist(), 'horizontal_distance_m': distance,
        'endpoint_rotation_change_deg': rotation_change,
        'forward_heading_in_map_deg': heading_world,
        'forward_heading_in_rigid_xy_deg': heading_rigid,
        'body_orientation_in_rigid_xyzw_candidate': mount_q,
        'body_origin_in_rigid': None,
        'origin_offset_verified': False,
        'orientation_candidate_quality': 'coarse' if rotation_change > 2 else 'initial_estimate',
        'extrinsics_verified': False, 'north_alignment_verified': False,
        'time_sync_verified': False, 'ready_to_send_to_fc': False,
        'actual_sent_to_fc': 0, 'warnings': warnings,
    }


def capture_record(rows, subject, captured_at):
    """把当前接收器FrameSample转换为原端点格式；原始帧另存，不要求帧号连续。"""
    valid = [r for r in rows if r['subject'] == subject and r['segment'] == subject
             and r['sdk_ok'] and not r['occluded'] and r['pose_published']
             and np.isfinite(r['world_pose']).all()
             and abs(np.linalg.norm(r['world_pose'][3:]) - 1) < .05]
    if len(valid) < 2:
        raise ValueError('insufficient_live_samples')
    poses = np.array([r['world_pose'] for r in valid])
    arrivals = np.array([r['callback_ns'] for r in valid], dtype=np.int64)
    gaps = np.diff(arrivals) / 1e9
    stamps = [r['ros_stamp_ns'] for r in valid]
    return dict(
        captured_at_utc=captured_at, pose_topic=f'/vicon_b3_check/{subject}/{subject}',
        capture_source='Dedicated FrameSample world_pose; no PoseStamped subscription',
        subject=subject, frame_ids=sorted(set(r['frame_id'] for r in valid)),
        source_checks_passed=bool(np.all(gaps > 0) and all(b > a for a, b in zip(stamps, stamps[1:]))),
        rejected_samples=len(rows) - len(valid),
        source_timing=dict(samples=len(valid), rate_hz=float((len(valid)-1) / gaps.sum())
                           if gaps.sum() > 0 else 0.0,
                           maximum_arrival_gap_ms=float(gaps.max() * 1000)),
        position_std_mm=(poses[:, :3].std(axis=0)*1000).tolist(),
        last_pose=dict(stamp_ns=stamps[-1], position_m=poses[-1, :3].tolist(),
                       quaternion_xyzw=poses[-1, 3:].tolist()))


def capture_heading(run, subject):
    """复用已有接收器与手动START/END交互；只生成候选，没有任何车机控制出口。"""
    import select
    import time
    from ev_delay_vicon_source import ViconSource
    from vicon_receiver_provenance import verify_current

    if not sys.stdin.isatty():
        raise RuntimeError('Local operator terminal required')
    run.mkdir(parents=True, exist_ok=False)
    source = None
    report = dict(subject=subject, commands=[], raw_endpoints={}, records={}, candidate=None,
                  error=None, cleanup_error=None, car_commands_sent=0, MAVLink_sent=0,
                  parameter_writes=0, automatic_adoption_allowed=False)

    def save(name, data):
        with (run/name).open('x') as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)

    def cue(word, prompt):
        print(prompt, flush=True)
        while True:  # 操作人员决定窗口开始时间，等待动作期间不设倒计时。
            source.poll()
            if select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if not line or line.strip().upper() in ('STOP', 'QUIT'):
                    raise RuntimeError('Operator stopped or terminal closed')
                if line.strip().upper() == word:
                    report['commands'].append(dict(command=word, at_ns=time.monotonic_ns()))
                    return
                print(f'请输入 {word}，取消请输入 STOP。', flush=True)
            time.sleep(.001)

    def capture(label):
        print(f'{label}：保持静止3秒……', flush=True)
        rows = report['raw_endpoints'][label] = []
        source.on_frame = lambda row, at: rows.append(dict(row))
        until = time.monotonic() + 3
        try:
            while time.monotonic() < until:
                source.poll()
                time.sleep(.001)
        finally:
            source.on_frame = lambda row, at: None
        record = capture_record(rows, subject, datetime.datetime.now(datetime.timezone.utc).isoformat())
        report['records'][label] = record
        save(label+'.json', record)
        endpoint(record, subject=subject)
        print(f'{label}已保存。', flush=True)
        return record

    try:
        source = ViconSource(run, verify_current(), latest_only=True, subject=subject)
        until = time.monotonic() + 10
        while True:
            source.poll()
            row = source.latest
            if (row and row['subject'] == subject and row['segment'] == subject
                    and row['sdk_ok'] and row['pose_published'] and not row['occluded']
                    and 0 <= source.now_ros_ns() - row['ros_stamp_ns'] <= 150_000_000):
                break
            if time.monotonic() > until:
                raise RuntimeError('No fresh valid Vicon frame')
            time.sleep(.001)
        cue('START', f'{subject}只读采集已就绪。动力保持断开，车身/机身沿直线参照放稳；'
                    '输入 START 回车，随后保持静止3秒。无需赶时间。')
        start = capture('start')
        distance = '约50 cm' if subject == 'car01' else '20–30 cm'
        cue('END', f'现在沿实际车头/机头方向前移{distance}，保持原朝向与高度，不转弯；'
                  '到新位置放稳，不移回。输入 END 回车（确认已按此方向平移），再静止3秒。')
        end = capture('end')
        candidate = solve_heading(start, end, True, subject=subject)
        candidate.update(source_files=[str(run/'start.json'), str(run/'end.json')],
                         operator_confirmation_source='Local END acknowledgement of physical-forward prompt')
        report['candidate'] = candidate
        save('heading_candidate.json', candidate)
        print('方向候选已保存，未修改运行配置。请保持放稳，等待分析。', flush=True)
    except BaseException as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
        print('采集停止：'+report['error'], flush=True)
    finally:
        if source:
            try:
                source.close()
            except BaseException as exc:
                report['cleanup_error'] = str(exc)
            report['source'] = source.report
        save('report.json', report)
        print('记录目录：'+str(run), flush=True)
    return 1 if report['error'] or report['cleanup_error'] else 0


def compare_candidates(first, second, tolerance_deg=5.0):
    """同一安装的两次机头估计应一致；单次动作质量通过不代表标定可采纳。"""
    if not math.isfinite(tolerance_deg) or not 0 < tolerance_deg <= 15:
        raise ValueError('invalid_consistency_tolerance')
    q0 = checked_quaternion(first['body_orientation_in_rigid_xyzw_candidate'])
    q1 = checked_quaternion(second['body_orientation_in_rigid_xyzw_candidate'])
    difference = math.degrees(2 * math.acos(float(np.clip(abs(q0 @ q1), 0, 1))))
    consistent = difference <= tolerance_deg
    return {'mounting_candidate_difference_deg': difference,
            'consistency_tolerance_deg': tolerance_deg,
            'mounting_candidates_consistent': consistent,
            'automatic_adoption_allowed': False,
            'extrinsics_verified': False, 'ready_to_send_to_fc': False,
            'action': 'Confirm physical nose and movement labels before adoption' if not consistent
                      else 'Still confirm mounting origin and independent body-axis observations'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', type=Path)
    parser.add_argument('--end', type=Path)
    parser.add_argument('--subject', choices=('MircoAir', 'car01'), default='MircoAir')
    parser.add_argument('--capture', action='store_true', help='只读START/END采集；output指定新目录')
    parser.add_argument('--forward-horizontal-confirmed', action='store_true')
    parser.add_argument('--compare-with', type=Path,
                        help='Optional previous candidate; inconsistency is never auto-resolved')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Refusing to overwrite calibration evidence')
    if args.capture:
        if args.start or args.end or args.compare_with:
            parser.error('--capture cannot be combined with offline input files')
        return capture_heading(args.output.resolve(), args.subject)
    if args.start is None or args.end is None:
        parser.error('Offline solve requires --start and --end')
    with args.start.open() as stream:
        start = json.load(stream)
    with args.end.open() as stream:
        end = json.load(stream)
    result = solve_heading(start, end, args.forward_horizontal_confirmed, subject=args.subject)
    if args.compare_with is not None:
        with args.compare_with.open() as stream:
            previous = json.load(stream)
        result['repeat_consistency'] = compare_candidates(previous, result)
    result.update({'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                   'source_files': [str(args.start.resolve()), str(args.end.resolve())]})
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    sys.exit(main())
