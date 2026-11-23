#!/usr/bin/env python3
"""One command, one HOME, battery-only static acceptance. Default is offline."""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import time

from prepare_drone import configuration,RUNS,TRS,require_port_free
from preparation_authorization import load_backup,require_no_pending_recovery,exclusive_json
from mavlink_readonly_probe import FC_USB
import one_home_preflight_policy as policy
from trs_uart_readonly_check import UID2

BINARY_CANDIDATE=RUNS/'binary_fusion_candidate_20261020_final/candidate_NOT_FLASHED.px4'
BINARY_INSTALLATION=RUNS/'binary_fusion_candidate_20261020_final/installation.json'


def require_binary_installation():
    """Local verified-uploader receipt; runtime still requires actual EVF2/EVR1."""
    if not BINARY_INSTALLATION.is_file():
        raise ValueError('新二进制诊断固件尚无安装凭据；保持当前连接，暂不能启用该实机模式')
    record=json.loads(BINARY_INSTALLATION.read_text())
    expected=hashlib.sha256(BINARY_CANDIDATE.read_bytes()).hexdigest()
    if (record.get('firmware_flashed') is not True
            or record.get('application_crc_verified_by_uploader') is not True
            or record.get('candidate_sha256')!=expected or record.get('error') is not None
            or record.get('bootloader',{}).get('uid')!=UID2):
        raise ValueError('二进制诊断候选的安装/校验/身份凭据不匹配')


def prerequisites(backup=None,usb_proof=None):
    c=configuration();require_no_pending_recovery(RUNS)
    if backup is None:
        candidates=sorted(RUNS.glob('backup_*/parameters_verified.json'),key=lambda p:p.stat().st_mtime,reverse=True)
        for p in candidates:
            try:load_backup(p.parent,time.time())
            except (ValueError,OSError,KeyError):continue
            backup=p.parent;break
    if backup is None:raise ValueError('需要一小时内的完整参数备份；可通过TRS只读获取，不要求切换USB供电')
    load_backup(Path(backup),time.time())
    if usb_proof is None:
        candidates=sorted(RUNS.glob('ground_fusion_usb_*/report.json'),key=lambda p:p.stat().st_mtime,reverse=True)
        for p in candidates:
            d=json.loads(p.read_text())
            if (d.get('ground_fusion_verified') is True and d.get('parameter_restoration_verified') is True
                    and d.get('telemetry_restoration_verified') is True
                    and d.get('runtime',{}).get('mission_inputs',{}).get('rigid_revision')==c['revision']):
                usb_proof=p;break
    if usb_proof is None:raise ValueError('缺少当前刚体版本已通过的USB静态融合记录；不自动重做标定')
    return c,Path(backup),Path(usb_proof)


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-battery',action='store_true')
    configuration_mode=ap.add_mutually_exclusive_group()
    configuration_mode.add_argument('--fixed-ekf-profile',type=Path,
        help='Plan B OFFLINE fixed profile; no hardware release in this phase')
    configuration_mode.add_argument('--legacy-dynamic-ekf',action='store_true',
        help='Explicit old experimental PARAM_SET scope; never fallback from plan B')
    ap.add_argument('--snapshot',type=Path,help='Existing typed JSON snapshot for fixed-profile offline comparison')
    modes=ap.add_mutually_exclusive_group()
    modes.add_argument('--receive-rate-only',action='store_true',help='Separate no-parameter-write receive diagnostic; never fusion acceptance')
    modes.add_argument('--compact-ground-fusion',action='store_true',help='Three fresh matching snapshots plus full EKF audits; ground only, never flight/handoff proof')
    ap.add_argument('--home-wait-seconds',type=float,default=90.,help='Wait for initial HOME, 10..300s; default 90s, never a flight timeout')
    ap.add_argument('--short-status',action='store_true',help='Opt-in compact static candidate with bounded clock and restored 5Hz lock stream; never flight')
    ap.add_argument('--binary-diagnostics',action='store_true',help='Opt-in EVF2/EVR1 backend; requires matching installed candidate, restores 2Hz stream, no text fallback')
    ap.add_argument('--low-bandwidth-vision',action='store_true',help='Opt-in ground-only 48-byte unknown-covariance profile; requires identical effective PX4 noise')
    ap.add_argument('--frozen-target-stream',action='store_true',help='Distinct locked ground load test: fixed actual FC XYZ/yaw, 16Hz/3s, NO mode change')
    ap.add_argument('--wireless-proof',type=Path,help='Completed same-calibration low-bandwidth wireless report to reuse')
    ap.add_argument('--confirm-protected-stationary-battery-only',action='store_true')
    ap.add_argument('--backup',type=Path);ap.add_argument('--usb-proof',type=Path)
    ap.add_argument('--run-dir',type=Path);ap.add_argument('--trs-port',default=str(TRS))
    a=ap.parse_args(argv)
    if a.fixed_ekf_profile is not None:
        if a.run_battery:
            print('此入口保留离线比较；固定参数真实地面入口为 fixed_ground_entry.py，需要独立许可和已批准配置。不会退回旧动态改参流程。',flush=True)
            return 2
        if a.snapshot is None:
            ap.error('--fixed-ekf-profile requires an existing --snapshot; no device discovery')
        from ekf_static_profile import load_json,plan_changes
        try:
            report=plan_changes(load_json(a.fixed_ekf_profile),load_json(a.snapshot))
        except (ValueError,OSError,TypeError,KeyError) as exc:
            print('固定配置离线校验失败：'+str(exc),flush=True);return 2
        report.update(mode='fixed-ekf-ground-offline-candidate',live_enabled=False,
            stages=['CONFIG_VERIFYING','CONFIG_VERIFIED','HOME_VERIFIED','VICON_VERIFIED',
                    'POSITION_AUTHORIZED','EKF_FUSION_VERIFIED','READY'],
            required_current_evidence='No current-session proof can be issued by a file comparison',
            control_ready=False)
        print(json.dumps(report,ensure_ascii=False,indent=2))
        return 0 if report['validation']['configuration_approved'] else 2
    if a.snapshot is not None:
        ap.error('--snapshot only belongs to --fixed-ekf-profile')
    if a.run_battery and not a.legacy_dynamic_ekf:
        print('没有选择参数模式：旧动态写参须显式 --legacy-dynamic-ekf；方案B尚未开放硬件。',flush=True)
        return 2
    if not 10<=a.home_wait_seconds<=300:ap.error('HOME waiting budget must be 10..300 seconds')
    if a.short_status and not a.compact_ground_fusion:ap.error('--short-status requires --compact-ground-fusion')
    if a.binary_diagnostics and not a.short_status:ap.error('--binary-diagnostics requires --short-status')
    if a.low_bandwidth_vision and not a.binary_diagnostics:ap.error('--low-bandwidth-vision requires --binary-diagnostics')
    if a.frozen_target_stream and not a.low_bandwidth_vision:ap.error('--frozen-target-stream requires --low-bandwidth-vision')
    if a.wireless_proof and not a.frozen_target_stream:ap.error('--wireless-proof requires distinct --frozen-target-stream scope')
    if not a.run_battery:
        print(json.dumps(dict(mode='offline_plan_no_hardware',
            backup=str(a.backup) if a.backup else None,usb_proof=str(a.usb_proof) if a.usb_proof else None,
            prerequisites_checked=False,hardware_home_presses_required=1,
            binary_diagnostics=a.binary_diagnostics,short_status=a.short_status,
            low_bandwidth_vision=a.low_bandwidth_vision,
            frozen_target_stream=a.frozen_target_stream,
            target_rate_hz=16 if a.frozen_target_stream else 0,
            target_duration_s=3 if a.frozen_target_stream else 0,
            pose_wire_bytes=48 if a.low_bandwidth_vision else 128,
            compact_ground_fusion=a.compact_ground_fusion,receive_rate_only=a.receive_rate_only,
            requires_matching_installed_binary_candidate=a.binary_diagnostics,
            temporary_intervals_us={'252':200000,'248':500000,'32':50000} if a.frozen_target_stream else {'252':200000,'248':500000} if a.binary_diagnostics else {'252':200000} if a.short_status else {},
            stages=['本次会话参数核对','一次HOME输出锁定证明','定位接收与三份融合证据','停发并逐项恢复参数和原回传频率'],
            devices_opened=0,physical_tx_bytes=0,flight_authorized=False),ensure_ascii=False,indent=2))
        return 0
    try:
        if a.binary_diagnostics:require_binary_installation()
        c,backup,proof=prerequisites(a.backup,a.usb_proof)
        if a.frozen_target_stream:
            from frozen_target_stream import require_prior
            if a.wireless_proof is None:raise ValueError('请指定已通过的 --wireless-proof；不自动重复历史试验')
            require_prior(a.wireless_proof,c['heading'],c['board'])
        if a.low_bandwidth_vision:
            from low_bandwidth_vision import noise_evidence
            noise_evidence(load_backup(backup,time.time()))
        if not a.confirm_protected_stationary_battery_only or FC_USB.exists():
            raise ValueError('仅限明确确认的保护架完整、机身静止、飞控USB拔掉的电池地面验收；不会自动切换供电')
        require_port_free(a.trs_port)
        run=a.run_dir or RUNS/('one_home_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        if run.resolve().parent!=RUNS.resolve():raise ValueError('结果目录必须直接位于preparation_runs且尚不存在')
        run.mkdir(exist_ok=False)
        record=policy.draft(run,c['heading'],c['board'],c['proof'],backup,proof,
            receive_rate_only=a.receive_rate_only,compact_ground_fusion=a.compact_ground_fusion,short_status=a.short_status,
            binary_diagnostics=a.binary_diagnostics,low_bandwidth_vision=a.low_bandwidth_vision,
            frozen_target_stream=a.frozen_target_stream,wireless_proof=a.wireless_proof)
        now=time.time();record.update(user_authorized=True,physical_confirmed=True,issued_unix=now,expires_unix=now+600)
        token=run/'permit.json';exclusive_json(token,record)
        import trs_powered_fusion_bench as bench
        from types import SimpleNamespace
        print('统一无线地面验收：只需一次HOME；不解锁、不切模式。'+
            ('新增冻结飞控当前位置/航向目标16Hz、3秒，绝不改变高度。' if a.frozen_target_stream else '不发送目标。'),flush=True)
        if a.receive_rate_only:print('独立接收诊断：禁止全部参数写入，不运行长篇多样本打印，不验收融合或飞行。',flush=True)
        if a.compact_ground_fusion:print('简洁地面融合：三份新鲜匹配回读＋完整EKF验收；不证明全包接收率，不进入起飞交接。',flush=True)
        print(f'等待首次HOME上限{a.home_wait_seconds:g}秒；短按可排队，识别到正确未解锁飞控后才发送锁定。',flush=True)
        print('报告目录：'+str(run),flush=True)
        code=bench.run(SimpleNamespace(one_home_preflight=True,receive_rate_only=a.receive_rate_only,compact_ground_fusion=a.compact_ground_fusion,short_status=a.short_status,binary_diagnostics=a.binary_diagnostics,low_bandwidth_vision=a.low_bandwidth_vision,frozen_target_stream=a.frozen_target_stream,powered_locked_static_confirmed=True,
            trs_port=a.trs_port,baud=115200,permit=token,backup=backup,heading=c['heading'],board=c['board'],
            output=run/'report.json',home_watch_seconds=0,home_wait_seconds=a.home_wait_seconds))
        report=json.loads((run/'report.json').read_text())
        print('分段计时：'+json.dumps(report.get('timing',{}),ensure_ascii=False),flush=True)
        success=('接收诊断完成；不是融合或飞行验收。' if a.receive_rate_only else
            '简洁地面融合及恢复通过；不是严格起飞验收。' if a.compact_ground_fusion else '通过：无线地面检查和恢复完成。')
        print((success if code==0 else '未通过：'+str(report.get('error') or report.get('restore_errors')))
            +' HOME监听已结束；不会进入起飞流程。',flush=True)
        return code
    except (ValueError,OSError,RuntimeError,KeyError,KeyboardInterrupt) as exc:
        print('未启动或未完成：'+str(exc)+'；不自动改网、重连、解锁或起飞。',flush=True);return 2


if __name__=='__main__':raise SystemExit(main())
