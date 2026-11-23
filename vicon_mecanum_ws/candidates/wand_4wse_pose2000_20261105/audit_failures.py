"""Read-only audit of today's real-demo records; never inspects hardware."""
from datetime import datetime
import hashlib
import json
from pathlib import Path
from zoneinfo import ZoneInfo

HERE = Path(__file__).resolve().parent
WS = Path('/home/czy/vicon_mecanum_ws')


def main():
    rows = []
    for path in (WS / 'candidates').glob('*/host/sessions/*/result.json'):
        if datetime.fromtimestamp(path.stat().st_mtime, ZoneInfo('Asia/Shanghai')).date().isoformat() != '2026-11-05':
            continue
        result = json.loads(path.read_text())
        if result.get('mode') != 'hardware' or result.get('passed') is True:
            continue
        session = path.parent
        first = None
        trace = session / 'TRACE.jsonl'
        if trace.exists():
            for line in trace.read_text().splitlines():
                try:
                    value = json.loads(line)
                except json.JSONDecodeError:
                    continue
                s = value.get('status', {})
                if s.get('state') in ('STOPPING', 'FAULT') and s.get('first_stop_reason'):
                    first = value
                    break
        status = (first or {}).get('status', {})
        transport = (first or {}).get('transport', {})
        cause = status.get('first_stop_reason') or result.get('reason')
        rows.append(dict(session=str(session), saved_at_local=datetime.fromtimestamp(
            path.stat().st_mtime, ZoneInfo('Asia/Shanghai')).isoformat(),
            result_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest() if trace.exists() else None,
            root_reason=cause, final_reason=result.get('reason'),
            pose_source_ages_ms={k: v*1000 for k, v in status.get('pose_source_ages_s', {}).items()},
            native_fault_bits=transport.get('native_fault_bits'),
            transport_fault=transport.get('fault'),
            first_gate_fault=(first or {}).get('gate', {}).get('fault'),
            max_control_gap_ms=1000*result.get('max_control_gap_s', 0.),
            motion_started=result.get('physical_motion_started'),
            stop_confirmed=result.get('stop_evidence', {}).get('stop_confirmed'),
            addressed_by_pose2000=cause in ('CAR_POSE_TIMEOUT', 'WAND_POSE_TIMEOUT'),
            upstream_gap_origin_proven=False))
    rows.sort(key=lambda r: r['saved_at_local'])
    report = dict(date='2026-11-05', timezone='Asia/Shanghai', failed_sessions=rows,
        count=len(rows), pose_timeout_sessions=sum(r['addressed_by_pose2000'] for r in rows),
        other_sessions=sum(not r['addressed_by_pose2000'] for r in rows),
        hardware_socket_opened=False, motion_started=False,
        caveat='Timeout labels prove which software check fired, not whether the camera, transport or ROS scheduler caused the upstream gap.')
    target = HERE / 'evidence/RECENT_FAILURES.json'
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(dict(count=report['count'], pose_timeout_sessions=report['pose_timeout_sessions'],
                         other_sessions=report['other_sessions'], report=str(target))))


if __name__ == '__main__':
    main()
