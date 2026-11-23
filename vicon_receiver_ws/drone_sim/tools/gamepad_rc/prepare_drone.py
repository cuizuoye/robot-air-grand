#!/usr/bin/env python3
"""Default: OFFLINE config inspection. Live modes are USB-ground ONLY.

--monitor-ground: explicitly confirmed read/HOME only.
--prepare-ground: additionally requires a reviewed, unused ground permit and
fresh full backup. Includes the existing temporary 10/5/5Hz telemetry profile
and exact restoration. Localization preparation only; never unlock/arm/fly.
No radio discovery, automatic reconnection or network configuration changes.
"""
import argparse
from contextlib import ExitStack
import json
from pathlib import Path
import subprocess
import tempfile
import time

from mission_preparation_runtime import PreparationRuntime
from mission_preparation_io import PreparationIO
from mission_fusion_service import context_from_inputs
from preparation_authorization import (ROOT,draft,consume,RecoveryJournal,RateRecoveryJournal,exclusive_json,
    require_no_pending_recovery,CODE_NAMES,TOOLS)
from rigid_body_revision import REGISTRY
from trs_runtime_monitor import inspect_evidence
from mavlink_readonly_probe import FC_USB
from trs_transport_config import require_baud
from readiness_console import LatestStatusPrinter
from estop_only_policy import DurableStopLatch
from trs_uart_readonly_check import UID2

TRS=Path('/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
PROOF=ROOT/'mircoair_calibration_20261016_LRnswK/wireless_fusion_and_local_loss_01.json'
RUNS=ROOT/'preparation_runs'


def require_port_free(path):
    path=Path(path).resolve(strict=True)
    result=subprocess.run(['fuser',str(path)],capture_output=True,timeout=2)
    if result.returncode==0:raise ValueError('TRS occupied; close QGC/web serial/other monitor yourself; no process killed')
    if result.returncode!=1:raise ValueError('Cannot establish exclusive TRS ownership')


def configuration(proof=PROOF):
    registry=json.loads(REGISTRY.read_text())
    heading,board=Path(registry['heading_candidate_path']),Path(registry['board_candidate_path'])
    inspected=inspect_evidence(heading,board,proof)
    h,b=json.loads(heading.read_text()),json.loads(board.read_text())
    return dict(heading=heading,board=board,proof=Path(proof),revision=inspected['active_revision_id'],
        origin=b['board_center_in_rigid_m_candidate'],orientation=h['body_orientation_in_rigid_xyzw_candidate'])


class HostSession:
    """Testable host lifecycle, no device constructors or blocking operations."""
    def __init__(self,runtime,io,*,prepare=False,seconds=90,observe_seconds=3):
        self.runtime,self.io=runtime,io
        self.prepare=prepare;self.deadline=seconds;self.observe_seconds=observe_seconds
        self.ready_since=None;self.close_deadline=None;self.done=False;self.reason=None
        self.monitor_observed=False
        self.active_started_at=None

    def _close(self,now,reason):
        self.reason=self.reason or reason
        if self.close_deadline is not None:return
        self.close_deadline=now+(35 if self.runtime.rates else 8)
        try:self.runtime.request_close()
        except (ValueError,PermissionError):pass  # Keep receiving fresh ground state for bounded retry.

    def monitor_missing(self,now):
        r=self.runtime;c=r.context(now)
        checks=(('飞控身份或心跳',r._identity_ok()),
            ('本会话HOME响应',r.inputs.home_proven),
            ('新鲜的输出锁定状态',r.coordinator.audit.live_lockdown(now) is True),
            ('Vicon实时位姿',c.source_age<=.15),
            ('手柄当前输入',c.gamepad_current))
        return [name for name,passed in checks if not passed]

    def ground_missing(self,now):
        r=self.runtime
        checks=(('已识别飞控及实时锁定状态',r._identity_ok() and r.context(now).locked_ground()),
            ('本会话HOME响应',r.inputs.home_proven),
            ('飞控内部定位接收证据',r.coordinator.fusion.receive_verified),
            ('三份静态XYZ/航向融合证据',r.coordinator.ground_verified(now)))
        return [name for name,passed in checks if not passed]

    def turn(self,now,ros_ns):
        if self.done:return self.status()
        self.io.step(now,ros_ns)
        r=self.runtime;c=r.context(now)
        if self.prepare and self.active_started_at is None and r.home_check_requested:
            self.active_started_at=now
        if self.close_deadline is not None:
            if not r.closing and not r.closed and c.restore_ground():
                try:r.request_close()
                except (ValueError,PermissionError):pass
            if (r.closed or now>=self.close_deadline or r.rates is not None
                    and r.rates.phase=='RESTORE_FAILED' and r.coordinator.fusion.phase=='CLOSED'):
                self.done=True  # Retain failed recovery journal; no useful writes remain.
            return self.status()
        if self.io.failure or r.failure or r.stop_latched:self._close(now,'ground_fault_or_home_stop')
        elif (now>=self.deadline if not self.prepare else
                now>=300 if self.active_started_at is None else now-self.active_started_at>=self.deadline):
            self._close(now,'bounded_session_end')
        elif not self.prepare:
            self.monitor_observed=not self.monitor_missing(now)
            if self.monitor_observed:self._close(now,'ground_monitor_conditions_observed')
        elif self.prepare:
            rates_ready=r.rates is None or r.rates.summary()['measured_profile_verified']
            if not r.prepare_requested and r.inputs.home_proven and c.locked_ground() and rates_ready:
                r.request_prepare(operator_confirmed=True)  # Operator explicitly selected PREPARE mode beforehand.
            if r.ground_fusion_only and r.coordinator.ground_verified(now):
                self.ready_since=now
                self._close(now,'ground_fusion_acceptance_complete_no_mission_handoff')
            elif r.coordinator.fusion.phase=='HANDED_OFF':
                if self.ready_since is None:self.ready_since=now
                if now-self.ready_since>=self.observe_seconds:self._close(now,'preparation_observation_complete')
        return self.status()

    def status(self):
        return dict(done=self.done,reason=self.reason,prepared=self.ready_since is not None and not self.runtime.ground_fusion_only,
            ground_fusion_verified=self.ready_since is not None and self.runtime.ground_fusion_only,
            active_started_at=self.active_started_at,
            monitor_conditions_observed=self.monitor_observed,
            runtime=self.runtime.status(),io=self.io.status(),flight_authorized=False,
            note='USB-ground preparation; no flight start, unlock or arming entry')


def run_live(args,config):
    # Called ONLY after a separate explicit CLI physical confirmation.
    if (getattr(args,'operator_session_confirmed',False) is not True
            or not args.confirm_battery_disconnected or not args.confirm_usb_only_stationary):
        raise ValueError('No current explicit ground session confirmation; no device opens')
    import serial
    from preparation_host_bindings import LinuxGamepad,RosLatestPose
    if not FC_USB.exists():raise ValueError('FC USB required; this host has no battery-only flight scope')
    if args.baud is None:raise ValueError('Explicit --baud required for live mode')
    require_baud(args.baud)
    require_no_pending_recovery(RUNS)
    require_port_free(args.trs_port)
    if getattr(args,'ground_fusion_only',False):require_port_free(FC_USB)
    if args.prepare_ground:
        if args.permit is None or args.backup is None or args.run_dir is None:
            raise ValueError('Preparation requires --permit, --backup and --run-dir')
        run_dir=args.run_dir.resolve()
        if run_dir.parent!=RUNS.resolve():raise ValueError('New run directory must be directly inside preparation_runs')
    else:run_dir=None
    RUNS.mkdir(exist_ok=True)
    if run_dir is None:run_dir=Path(tempfile.mkdtemp(prefix='monitor_',dir=RUNS))
    else:run_dir.mkdir(exist_ok=False)
    origin=time.monotonic()
    rates_journal=RateRecoveryJournal(run_dir/'rates_recovery.json') if args.prepare_ground else None
    r=PreparationRuntime(config['revision'],body_origin_in_rigid=config['origin'],
        body_orientation_in_rigid=config['orientation'],enable_offline_fusion=args.prepare_ground,
        enable_temporary_rates=args.prepare_ground,rates_persist=rates_journal,
        ground_fusion_only=getattr(args,'ground_fusion_only',False),
        usb_observer=getattr(args,'ground_fusion_only',False))
    journal=RecoveryJournal(run_dir/'recovery.json')
    grant=None
    if args.prepare_ground:
        grant=consume(args.permit,r,run_dir,config['heading'],config['board'],config['proof'],args.backup,origin=origin)
    with ExitStack() as stack:
        if grant:
            grant.seal.start();stack.callback(grant.seal.close)
        # Never auto-probe baud, reset a port, close someone else's handle or reopen on error.
        port=stack.enter_context(serial.Serial(str(args.trs_port),args.baud,timeout=0,write_timeout=0,exclusive=True))
        observer=stack.enter_context(serial.Serial(str(FC_USB),115200,timeout=0,write_timeout=0,exclusive=True)) if r.observer_required else None
        pad=LinuxGamepad();stack.callback(pad.close)
        pose=RosLatestPose();stack.callback(pose.close)
        console=LatestStatusPrinter();stack.callback(console.close)
        stop_record=DurableStopLatch(run_dir/'stop.latch.json',UID2)
        io=PreparationIO(r,port,poll_gamepad=pad.poll,poll_pose=pose.poll,usb_present=FC_USB.exists,
            authorize=grant,journal=journal if grant else None,rates_journal=rates_journal,observer_port=observer)
        session=HostSession(r,io,prepare=args.prepare_ground,seconds=args.seconds,observe_seconds=args.observe_seconds)
        next_notice=0.;interrupted=False
        print('USB地面会话开始；不会解锁或启动电机。报告：'+str(run_dir),flush=True)
        if r.ground_fusion_only:
            print('独立断动力台架模式：1秒锁定状态窗口；禁止任务READY或交接，不是起飞许可。',flush=True)
            print('TRS仅定位/HOME/基础回传；详细诊断及临时融合参数走独立验明身份的USB。不是脱USB验收。',flush=True)
        while not session.done:
            now=time.monotonic()-origin
            try:
                session.turn(now,pose.now_ns)
                if r.stop_latched:stop_record.fail('home_during_preparation')
                elif r.failure:stop_record.fail(r.failure)
                if now>=next_notice:
                    current=r.status()
                    missing=(current['mission_inputs']['missing_conditions'] if args.prepare_ground
                        else session.monitor_missing(now))
                    if r.ground_fusion_only:missing=session.ground_missing(now)
                    label='地面准备' if args.prepare_ground else '断动力联合监听'
                    if r.ground_fusion_only:label='断动力静态融合验收'
                    if args.prepare_ground and not r.home_check_requested and not (r.closing or r.closed or r.failure):
                        console.publish('等待本会话一次HOME输入；尚未提频、发送定位或改参。'
                            +(' HOME监听已就绪。' if r._identity_ok() else ' 正在识别飞控。'))
                    else:
                        console.publish(label+'：'+current['coordinator']['fusion']['phase']+'；缺少：'+(', '.join(missing) or '无')
                            +'；不是起飞许可。'+(' HOME监听已就绪，可短按验证。' if r._identity_ok()
                                and not (r.home_check_requested or r.closing or r.closed or r.failure) else ''))
                    next_notice=now+1
                time.sleep(.001)  # Scheduling only; no waiting for NSH replies.
            except KeyboardInterrupt:
                if interrupted:
                    session.reason='operator_forced_exit_recovery_may_be_required';break
                interrupted=True;session._close(now,'operator_cancelled')
            except (OSError,RuntimeError,ValueError,TypeError,KeyError) as exc:
                io.failure=io.failure or 'host_exception: '+str(exc)
                r._fail(io.failure);stop_record.fail(io.failure)
                session.reason='host_exception_recovery_may_be_required'
                break  # Report uncertainty; do not pretend a rollback occurred.
        restored=journal.finalize(r,io)
        rates_restored=rates_journal.finalize(r,io) if rates_journal else True
        result=session.status()
        result.update(parameter_restoration_verified=restored,resident_monitor_running_after_exit=False,
            telemetry_restoration_verified=rates_restored,
            physical_mode='operator-confirmed-battery-off-usb-only',permit_consumed=grant is not None)
        exclusive_json(run_dir/'report.json',result)
    print('会话已结束，HOME监听已停止；'+('参数与回传频率无待恢复项。' if restored and rates_restored else '恢复未确认：保持动力断开，先处理恢复日志。'),flush=True)
    return 0 if restored and rates_restored and not io.failure and not r.failure and (
        session.ready_since is not None if args.prepare_ground else session.monitor_observed) else 2


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--check-config',action='store_true',help='Default: offline files/config only; no hardware opens')
    mode.add_argument('--monitor-ground',action='store_true')
    mode.add_argument('--prepare-ground',action='store_true')
    mode.add_argument('--draft-permit',type=Path,help='Write an UNAUTHORIZED draft; no device access')
    parser.add_argument('--ground-fusion-only',action='store_true',
        help='Separately scoped battery-off fusion-only acceptance: 1s ground status, no mission handoff')
    parser.add_argument('--static-proof',type=Path,default=PROOF)
    parser.add_argument('--trs-port',type=Path,default=TRS)
    parser.add_argument('--baud',type=int,choices=(57600,115200))
    parser.add_argument('--seconds',type=float,default=90)
    parser.add_argument('--observe-seconds',type=float,default=3)
    parser.add_argument('--confirm-battery-disconnected',action='store_true')
    parser.add_argument('--confirm-usb-only-stationary',action='store_true')
    for name in ('permit','backup','run-dir'):parser.add_argument('--'+name,type=Path)
    args=parser.parse_args(argv)
    try:
        config=configuration(args.static_proof)
        if args.draft_permit:
            if args.backup is None or args.run_dir is None:raise ValueError('Draft requires backup and future run-dir paths')
            exclusive_json(args.draft_permit,draft(args.run_dir,config['heading'],config['board'],config['proof'],args.backup,
                ground_fusion_only=args.ground_fusion_only,usb_observer=args.ground_fusion_only))
            print('仅保存未授权草稿；未连接硬件，不能用于启动。');return 0
        if not (args.monitor_ground or args.prepare_ground):
            print(json.dumps(dict(mode='offline_configuration_only',revision=config['revision'],
                heading=str(config['heading']),board=str(config['board']),
                evidence_current=True,source_files_present=all((TOOLS/name).is_file() for name in CODE_NAMES),
                historical_static_proof_is_not_live_readiness=True,
                live_authorization_installed=False,hardware_opened=0,flight_authorized=False,
                next='Physical connection and a NEW reviewed ground scope are required for live validation'),ensure_ascii=False,indent=2))
            return 0
        if args.ground_fusion_only and not args.prepare_ground:
            raise ValueError('Ground fusion-only profile is not a monitoring or flight mode')
        # Read/HOME-only monitoring can wait for a human longer. Active
        # localization preparation retains its original 120-second ceiling.
        limit=300 if args.monitor_ground else 120
        if (not args.confirm_battery_disconnected or not args.confirm_usb_only_stationary
                or not 10<=args.seconds<=limit or not 1<=args.observe_seconds<=10):
            raise ValueError('Explicit battery-OFF USB-stationary confirmation and bounded timing required')
        word=('FUSION' if args.ground_fusion_only else 'PREPARE') if args.prepare_ground else 'MONITOR'
        print('仅USB供电、动力电池拔掉、桨按已安装处理、机身放稳。不会解锁或起飞。',flush=True)
        if input('确认当前条件，输入 '+word+' 开始（否则取消）：').strip()!=word:
            print('已取消，未打开硬件。');return 2
        args.operator_session_confirmed=True
        return run_live(args,config)
    except (OSError,ValueError,RuntimeError,KeyError,PermissionError,EOFError,KeyboardInterrupt) as exc:
        print('未完成：'+str(exc)+'；不自动重连、改网络或解除锁定。');return 2


if __name__=='__main__':raise SystemExit(main())
