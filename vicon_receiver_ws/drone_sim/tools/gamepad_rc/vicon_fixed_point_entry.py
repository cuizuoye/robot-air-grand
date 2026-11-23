#!/usr/bin/env python3
"""Fixed Vicon entry; default CLI is an offline plan.

Reuses SerialOwner, LinuxGamepad, RosLatestPose and build_guarded_session.
Opening a session does not install reviews, step the owner or start flight.
Explicit --run services the existing controller and nonblocking operator console.
No automatic profile approval, parameter writes, reconnect or flight start.
Pocket/WiFi ground observation and the reviewed flight console share this entry.
Neither opens a gamepad or emits overrides/HOME/output-enable commands.
The legacy gamepad flight route must be explicitly selected.
"""
import json
from contextlib import ExitStack
from pathlib import Path
import time

from ekf_static_profile import load_json, require_approved_profile
from flight_start_confirmation import build_guarded_session


def open_session(*, profile_path, recovery_path, trs_port, open_devices=False,
                 height=.3, hover_seconds=3.):
    """Explicit future native entry. Caller services step on this same thread.

    Install the existing ground review in the current session BEFORE HOME,
    then a distinct flight review at WAIT_EXPLICIT_START. The existing
    start.create()/confirm() remains a separate, one-use operator action.
    Call close() only after the controller reports confirmed safe closure;
    it refuses to release resources after arm unless ground is confirmed.
    """
    if open_devices is not True:
        raise PermissionError('Device opens require explicit open_devices=True; default is offline')
    profile=load_json(profile_path)
    require_approved_profile(profile)  # Reject current unapproved candidate before I/O.
    from continuous_device_host import SerialOwner, configuration
    from preparation_host_bindings import LinuxGamepad, RosLatestPose
    from preparation_authorization import require_no_pending_recovery
    from mavlink_readonly_probe import FC_USB
    from flight_mission_core import MissionCore
    # Validate the existing envelope before opening resources too.
    MissionCore(lambda msg: None,height=height,hover_seconds=hover_seconds)
    require_no_pending_recovery(Path(recovery_path).parent.parent)
    config=configuration()
    calibration=load_json(Path(__file__).parent/'gamepad_calib.json')
    if FC_USB.exists():raise PermissionError('Native execution has no FC USB flight route')
    resources=ExitStack()
    try:
        owner=SerialOwner(trs_port);resources.callback(owner.close)
        pad=LinuxGamepad(controls=True);resources.callback(pad.close)
        pose=RosLatestPose();resources.callback(pose.close)
        origin_monotonic=time.monotonic()
        controller=build_guarded_session(**config,port=owner.port,
            poll_gamepad=pad.poll,poll_pose=pose.poll,now_ros_ns=pose.now_ns,
            check_port_identity=owner.check,usb_present=FC_USB.exists,
            origin_monotonic=origin_monotonic,recovery_path=recovery_path,
            height=height,hover_seconds=hover_seconds,fixed_profile=profile,gamepad_calibration=calibration)
        controller.origin_monotonic=origin_monotonic
        controller.start.native_start_check=controller.require_native_start
        controller.resources=resources.pop_all()
        return controller
    except BaseException:
        resources.close()  # No step/send has occurred during construction.
        raise


def plan():
    proposal=load_json(Path(__file__).parent/'plan_b/fixed_point/vicon_parameter_switch.json')
    progress_path=Path(__file__).parent/'plan_b/fixed_point/FIRST_FLIGHT_20261028.json'
    progress=load_json(progress_path)
    from ekf_static_profile import compare
    profile=load_json(Path(__file__).parent/'plan_b/fixed_point/vicon_profile.initial_trial.json')
    from rigid_body_revision import REGISTRY
    registry=load_json(REGISTRY)
    # Self-comparison explains approval gaps only; it is NOT FC readback.
    review=compare(profile,profile['parameters'])
    return dict(mode='offline_plan',devices_opened=0,physical_tx_bytes=0,
        native_entry='vicon_fixed_point_entry --handset pocket --ground-observe',
        legacy_gamepad_entry='vicon_fixed_point_entry.open_session',
        recommended_hardware='RadioMaster Pocket native RC + MicoAir-WiFi-Link UART4',
        pocket_entry='--handset pocket --ground-observe',
        pocket_flight_runtime_integrated=True,
        flight_timestamp_integration=dict(mode='local_ros_timesync',
            same_owner_reply_offline_implemented=True,
            receipt_contract_accepts_px4_fallback_and_mapping=True,
            temporary_sync_rate_lifecycle_integrated=True,
            setup_requires_bootstrap_review=True,restoration_survives_vicon_loss=True,
            restore_readback_required=True,unchanged_rate_skips_set=True,
            ground_rate_lifecycle_hardware_validated=True,
            battery_static_evidence='preparation_runs/wifi_battery_fusion_20261105_attempt01/analysis.json',
            hardware_scope='GroundObserver plus shared TemporaryTelemetryRates; not the full flight state machine',
            hardware_validated=False,flight_authorized=False),
        pocket_flight_entry='--handset pocket --flight-console --profile plan_b/fixed_point/vicon_profile.initial_trial.json (separate current reviews and TAKEOFF required)',
        pocket_observation_sends='GCS heartbeat, version/parameter reads, TIMESYNC, fixed read-only shell queries',
        pocket_observation_workflow=dict(start='--ground-observe --wait-for-go; GO only after prompt',
            reports=['parameter_readbacks','parameter_review','observation_summary','diagnostic_gaps'],
            no_receiver_required_for_offline_work=True,airborne_validation=False),
        dynamic_calibration_status='2026-11-04 resumed: complete 15 s WiFi capture, 453 exact visual matches; constant-delay review blocked by receipt jitter, no parameter approval',
        dynamic_capture_evidence='preparation_runs/joint_ev_delay_20261104_142205_506397/analysis_summary.json',
        wifi_timestamp_check=dict(same_channel_exchange_verified=True,sample_time_mapping_verified=True,
            clock_accuracy_candidate_pass=False,
            diagnostic_gaps=['wifi_clock_accuracy_not_verified'],
            observed_behavior='Continuous 180 s test: prior 31.5 ms residual did not recur; final residual -2.674 ms, last-minute range -7.114..+2.679 ms; real sample mapping passed, 2 ms stability candidate did not',
            evidence='preparation_runs/wifi_sync_continuous_20261104_170304/analysis.json',
            prior_receive_time_fallback_evidence='preparation_runs/wifi_timestamp_probe_20261104_151145/analysis.json',
            continuous_startup_fix='offline tested: one bounded 30..180 s session, early exit on stable real mapping; reply count is not FC accepted count',
            continuous_startup_hardware_verified=True,
            clock_stability_candidate_pass=False,
            repeat_stationary_sync_required=False,
            residual_precision_tuning_deferred=True,
            deferral_basis='User explicitly deferred clock residual tuning after the static fusion-effect observation; not a flight approval',
            static_fusion_effect_evidence='preparation_runs/wifi_sync_fusion_effect_20261104_171242/analysis.json',
            static_fusion_no_adverse_effect_observed=True,
            airborne_control_effect_verified=False,
            temporary_rate_restored=True,repeat_tilt_required=False,flight_authorized=False),
        rc_specialist_work_status='Paused by user; reuse basic RC and ground loss/recovery evidence',
        last_position_prepare_evidence='preparation_runs/wifi_position_prepare_20261104_140031/analysis.json',
        rigid_layout_reference=dict(revision=profile['initial_trial']['revision'],
            historical_measured_revision=registry['active_revision_id'],
            runtime_profile='plan_b/fixed_point/vicon_profile.initial_trial.json',
            frame_transfer=profile['initial_trial'].get('rigid_frame_transfer'),
            heading=registry['heading_candidate_path'],board=registry['board_candidate_path'],
            provisional=not registry['static_validation_completed']),
        vicon_delivery_policy=dict(flight='latest_only',isolated_fusion='latest_only',
            calibration='capture_history',flight_subscription_depth=1,flight_sdk_buffer_size=1,
            static_hardware_evidence='preparation_runs/wifi_vicon_latest_20261104_135550/analysis.json',
            combined_target_load_validation_pending=False,
            combined_target_load_evidence='preparation_runs/wifi_fixed_target_20261104_140649/analysis.json',
            actual_offboard_entry_exit_validation_pending=False,
            disarmed_offboard_evidence='preparation_runs/wifi_offboard_check_20261104_attempt02/analysis.json',
            powered_flight_validation_pending=True),
        pocket_localization_sends=False,
        pocket_flight_localization_requires_ground_review=True,
        legacy_gamepad_scope='The flight console and full_gamepad_takeover fields below describe only --handset legacy-gamepad',
        profile='plan_b/fixed_point/vicon_profile.initial_trial.json',
        changes=proposal['changes'],
        changes_scope='Historical switch completed and reboot-verified on 2026-10-28; not a request to write again',
        preparation_record=str(progress_path),historical_evidence=progress['evidence'],
        current_session_ready=False,default_cli_executes_flight=False,
        battery_static_observation=dict(completed=True,
            evidence='preparation_runs/wifi_battery_fusion_20261105_attempt01/analysis.json',
            position_preflight_passed=True,ev_xyz_yaw_fused=True,rate_restored=True,
            residual_clock_tuning_deferred=True,repeat_stationary_observation_required=False,
            formal_flight_runtime_validated=False),
        native_console_available=True,execution_requires_explicit_run_reviews_and_start=True,
        first_sortie=progress['first_sortie'],
        legacy_gamepad_release_conditions=progress['release_conditions'],
        release_conditions=[
            dict(status='SOFTWARE_READY',condition='Scoped native-RC initial trial uses retained baseline: fixed origin, 0.2..0.3 m, hover <=3 s. Exact extrinsics/delay remain uncalibrated; no flight permission implied'),
            dict(status='PENDING_HARDWARE',condition='Battery-static fusion and shared sync-rate lifecycle passed on 2026-11-05; full flight state machine remains unvalidated on hardware'),
            dict(status='PENDING',condition='Current WiFi/Vicon session, fresh preflight state and separate ground/flight reviews')],
        operation_order=[
            'Reuse saved RC and static fusion evidence; no repeated stick or tilt exercise',
            'Use vicon_profile.initial_trial.json; retain approximate layout/untuned delay without repeating precision calibration',
            'Use existing motor-isolated diagnostics only for concrete missing current-state evidence; the flight console remains the separately authorized wireless route',
            'After separate flight authorization use Pocket + WiFi flight-console, ground review, disarmed Position',
            'WAIT_EXPLICIT_START -> execution review -> preview -> one-use TAKEOFF token',
            'Fixed-origin warmup -> Offboard -> arm -> 0.3 m ascent -> 3 s hover -> Land',
            'Physical takeover yields irreversibly; continue fresh Vicon until confirmed landed/disarmed'],
        runbook=str(Path(__file__).parent/'plan_b/fixed_point/FLIGHT_RELEASE.txt'),
        profile_review=dict(scope='offline profile approval gaps, not current readback',
            reusable_archive=str(Path(__file__).parent/'plan_b/fixed_point/reconciled_parameter_archive_20261105'),
            archive_scope='Historical complete catalog plus saved task-parameter observations; not a current or simultaneous full snapshot',
            archive_file_age_requires_recapture=False,
            host_timestamp_change_requires_fc_reboot=False,
            current_typed_task_readback_still_required=True,
            approved=review['configuration_approved'],
            initial_trial_eligible=review['initial_trial_eligible'],
            precision_calibration_required_for_initial_trial=False,
            pending_calibration=review['pending_calibration'],
            unreviewed_parameters=review['unapproved'],
            persistence_bound=review['persistence_evidence_present']),
        default_height_m=.3,default_hover_seconds=3,
        full_gamepad_takeover_connected=True,hardware_takeover_verified=False,flight_authorized=False,
        hardware_takeover_scope='2026-11-04 axes, SA Position/Land, SC Kill and receiver loss/recovery observed disarmed; active-mission takeover/airborne failsafe not verified',
        legacy_gamepad_remaining=progress['remaining'],
        remaining=['Current-session preparation and physical review for the bounded initial trial',
            'Hardware validation of the integrated flight owner; no repeat of passed standalone RC/mode checks',
            'Separate authorized first real takeoff-hover-land'],
        deferred_by_user=progress['deferred_by_user'])


def open_pocket_session(*,profile_path,output,peer,bind,open_devices=False,height=None,hover_seconds=3.,
                        cooperative_plan=None,site_ready=False):
    """Same reviewed controller over WiFi; no gamepad, no implicit reviews/start."""
    if open_devices is not True:raise PermissionError('Explicit open_devices=True required')
    if height is None:height=cooperative_plan['clearance_m'] if cooperative_plan is not None else .3
    profile=load_json(profile_path)
    from ekf_static_profile import require_mission_profile, inspect_initial_trial_evidence
    require_mission_profile(profile,pilot_source='native_rc',height=height,hover_seconds=hover_seconds)
    if profile['parameters']['COM_RC_IN_MODE']['value']!=0:
        raise PermissionError('Pocket flight requires reviewed COM_RC_IN_MODE=0; no automatic parameter switch')
    from flight_mission_core import MissionCore
    MissionCore(lambda msg:None,height=height,hover_seconds=hover_seconds,pilot_source='native_rc',
                horizontal_motion=profile.get('horizontal_motion'))
    from continuous_device_host import configuration
    from rigid_body_revision import REGISTRY,require_current_calibration
    registry=load_json(REGISTRY)
    if 'initial_trial' in profile:
        trial=profile['initial_trial']
        config={key:trial[key] for key in ('revision','origin','orientation')}
        inspect_initial_trial_evidence(profile,**config)  # Before any device/network access.
    else:
        require_current_calibration(registry['heading_candidate_path'],registry['board_candidate_path'])
        config=configuration()
    if cooperative_plan is not None:
        from air_ground_resources import validate_bindings
        validate_bindings(cooperative_plan,config,live=True)
        if site_ready is not True:raise PermissionError('Joint console requires current --site-ready')
        if profile.get('horizontal_motion') is not None or abs(height-cooperative_plan['clearance_m'])>1e-9:
            raise ValueError('Joint task requires fixed-point profile with matching deck clearance')
    from preparation_authorization import require_no_pending_recovery
    from mavlink_readonly_probe import FC_USB
    from mavlink_link import FixedPeerUDP
    from ev_delay_vicon_source import ViconSource
    from vicon_receiver_provenance import verify_current
    output=Path(output)
    require_no_pending_recovery(output.parent)
    if FC_USB.exists():raise PermissionError('Flight console requires the wireless, battery-only route')
    provenance=verify_current()
    output.mkdir(parents=True,exist_ok=False)
    resources=ExitStack()
    try:
        log=resources.enter_context((output/'wifi_raw.jsonl').open('x'))
        def record(data,address,accepted):
            log.write(json.dumps(dict(at_ns=time.monotonic_ns(),peer=address,accepted=accepted,
                wire_hex=data.hex()))+'\n')
        source=ViconSource(output,provenance,latest_only=True);resources.callback(source.close)
        frames=resources.enter_context((output/'vicon_frames.jsonl').open('x'))
        source.on_frame=lambda row,at:frames.write(json.dumps(dict(at_ns=at,frame=row))+'\n')
        port=FixedPeerUDP(peer,bind,on_datagram=record);resources.callback(port.close)
        origin=time.monotonic();joint=None
        if cooperative_plan is not None:
            from air_ground_resources import JointResources
            joint=JointResources(plan=cooperative_plan,uav_config=config,source=source,output=output,
                origin_monotonic=origin,open_devices=True,site_ready=site_ready)
            resources.callback(joint.close)
        def poll_pose():
            # A car-side failure is recorded by joint.poll(), never allowed to
            # interrupt continuing UAV localization during Land/RC takeover.
            if joint:joint.poll()
            source.poll();row=source.latest
            if row is None:return None
            if joint:joint.cache.receive_uav(row)
            if (not row['sdk_ok'] or row['occluded'] or not row['pose_published'] or row['queue_dropped']
                    or (row['subject'],row['segment'],row['frame_id'])!=('MircoAir','MircoAir','map')):
                return None  # Existing pose freshness watchdog determines loss.
            return row['ros_stamp_ns'],'map',row['world_pose'][:3],row['world_pose'][3:]
        controller=build_guarded_session(**config,port=port,poll_gamepad=None,
            poll_pose=poll_pose,now_ros_ns=source.now_ros_ns,check_port_identity=port.check,
            usb_present=FC_USB.exists,origin_monotonic=origin,recovery_path=output/'flight_recovery.json',
            fixed_profile=profile,pilot_source='native_rc',height=height,hover_seconds=hover_seconds,
            cooperative_execution=joint.execution if joint else None)
        controller.origin_monotonic=origin
        controller.joint_resources=joint
        controller.start.native_start_check=controller.require_native_start
        controller.resources=resources.pop_all()
        return controller
    except BaseException as exc:
        try:resources.close()
        except Exception as cleanup:exc.add_note('Resource cleanup: '+str(cleanup))
        raise


def open_car_route_session(*,plan,output,open_devices=False,site_ready=False):
    """Existing Vicon/bridge resources only; no drone port, profile or flight core."""
    from air_ground_resources import validate_bindings,JointResources
    alignment_only=plan.get('alignment_only') is True
    uav_config=plan.get('alignment_uav_config') if alignment_only else None
    if alignment_only:
        if not uav_config:raise ValueError('alignment_only requires measured UAV body configuration')
        validate_bindings(dict(plan,alignment_only=False),uav_config,sources_only=True)
    validate_bindings(plan,uav_config,live=open_devices,car_only=True)
    if open_devices is not True or site_ready is not True:
        raise PermissionError('仅小车实测需要 --run --site-ready；默认只看计划')
    from ev_delay_vicon_source import ViconSource
    from vicon_receiver_provenance import verify_current
    provenance=verify_current()
    output=Path(output);output.mkdir(parents=True,exist_ok=False)
    (output/'car_plan.json').write_text(json.dumps(plan,ensure_ascii=False,indent=2)+'\n')
    resources=ExitStack()
    try:
        source=ViconSource(output,provenance,latest_only=True,subject='MircoAir' if alignment_only else 'car01');resources.callback(source.close)
        owner=JointResources(plan=plan,uav_config=uav_config,source=source,output=output,
            origin_monotonic=time.monotonic(),open_devices=True,site_ready=True,car_only=True)
        resources.callback(owner.close)
        owner.session_resources=resources.pop_all()
        return owner
    except BaseException as exc:
        try:resources.close()
        except Exception as cleanup:exc.add_note('Car resource cleanup: '+str(cleanup))
        raise


def serve_car_route_console(owner,read_line,*,emit=print,clock=time.monotonic,idle=None):
    """One explicit START; STOP/EOF/Ctrl-C close through measured zero output."""
    idle=idle or (lambda:time.sleep(.01))
    previous=None;status=None;fault_since=None
    emit('仅小车路线实测：就绪后输入 START；STOP、cancel 或 Ctrl-C 停车。没有无人机控制。')
    try:
        while True:
            command=read_line()
            if command is not None:
                if command=='' or command.strip().upper() in ('STOP','CANCEL'):break
                command=command.strip().upper()
                if command=='START' and (status is None or status['phase']!='WAIT_START'):
                    emit('当前不接受 START；等待就绪后重新输入，不会自动执行排队命令。');command=None
                elif command and command not in ('START','STATUS'):
                    emit('可用命令：START / STOP / STATUS');command=None
            status=owner.step_car(start=command=='START')
            key=(status['phase'],status.get('fault'),status.get('route_index'),status.get('waiting_for'))
            if key!=previous or command=='STATUS':
                emit(json.dumps(dict(phase=key[0],fault=key[1],route_index=key[2],waiting_for=key[3],
                    car_stopped=status.get('car_stopped')),ensure_ascii=False));previous=key
            if status.get('fault'):
                if fault_since is None:fault_since=clock()
                if clock()-fault_since>=3.:break
            if status['phase'] in ('COMPLETE','ABORTED'):break
            idle()
    except KeyboardInterrupt:
        emit('已请求停车，正在保留通信等待停车反馈。')
    finally:
        owner.session_resources.close()
    return dict(status=status,stop=owner.stop_result,actual_sent_to_fc=0)


def serve_console(controller, read_line, *, emit=print, clock=time.monotonic,
                  idle=lambda: time.sleep(.002)):
    """Nonblocking console over the existing controller; no automatic reviews/start.

    read_line returns None when idle and '' on EOF. EOF/Ctrl-C request the
    controller's existing cancellation, then keep serving until safe closure.
    A failed flight turn must not drop the localization/handset resource owner.
    """
    last_status=None;input_closed=False;last_fault=None
    while not controller.done:
        try:
            status=controller.step(clock()-controller.origin_monotonic)
            joint=getattr(controller,'joint_resources',None)
            if joint:joint.record(controller.runtime,status)
            confirmation=status.get('command_confirmation')
            landing=status.get('landing_confirmation')
            motion=status.get('horizontal_motion')
            cooperative=status.get('cooperative')
            key=(status['phase'],tuple(status['missing_conditions']),status['intervention_required'],
                 json.dumps(confirmation,sort_keys=True),status.get('failure'),json.dumps(landing,sort_keys=True),
                 status.get('operator_action'),motion.get('stage') if motion else None,
                 (cooperative.get('phase'),cooperative.get('car_task_phase'),
                  cooperative.get('car_state'),cooperative.get('fault')) if cooperative else None)
            if key!=last_status:
                emit(json.dumps(dict(phase=key[0],missing=key[1],intervention_required=key[2],
                    command_confirmation=confirmation,landing_confirmation=landing,failure=key[4],
                    operator_action=status.get('operator_action'),
                    horizontal_motion=motion,
                    cooperative=cooperative,
                    failure_snapshot=status.get('failure_snapshot')),ensure_ascii=False))
                last_status=key
            if controller.done:break
            line=None if input_closed else read_line()
            if line=='':
                input_closed=True;controller.cancel()
                emit('Console closed: cancellation requested; retain resources until confirmed safe closure.')
            elif line is not None:
                command=line.strip()
                try:
                    if command.startswith('ground '):controller.install_bootstrap_review(command[7:])
                    elif command.startswith('review '):controller.install_execution_review(command[7:])
                    elif command=='CAR_START':emit(json.dumps(controller.start_car(),ensure_ascii=False))
                    elif command=='preview':emit(json.dumps(controller.start.create(),ensure_ascii=False))
                    elif command.startswith('TAKEOFF '):controller.start.confirm(command)
                    elif command=='cancel':controller.cancel()
                    elif command:emit('Commands: ground <review-file>, review <review-file>, CAR_START, preview, TAKEOFF <token>, cancel')
                except (OSError,ValueError,PermissionError) as exc:
                    emit('Command rejected: '+str(exc))
        except KeyboardInterrupt:
            controller.cancel()
            emit('Cancellation requested; still servicing localization and handset until safe closure.')
        except Exception as exc:
            controller.cancel()
            fault=type(exc).__name__+': '+str(exc)
            if fault!=last_fault:emit('Service fault; cancellation requested: '+fault);last_fault=fault
        idle()
    try:
        controller.close()  # Existing close refuses unknown/airborne cleanup.
    except Exception as exc:
        emit('资源收尾未完全确认：'+str(exc))
        raise
    return controller.status()


def terminal_reader(fd):
    """Existing console line reader, shared by flight console and ground GO."""
    import os,select
    pending=bytearray();eof=False
    def read_line():
        nonlocal eof
        if b'\n' in pending:
            line,_,rest=pending.partition(b'\n');pending[:]=rest
            return line.decode('utf8',errors='strict')+'\n'
        if eof:
            pending.clear();return ''
        if select.select([fd],[],[],0)[0]:
            chunk=os.read(fd,1024)
            if chunk:pending.extend(chunk)
            else:eof=True
            if len(pending)>8192:raise ValueError('Console input limit exceeded')
        return None
    return read_line


def ground_go_ready(read_line):
    line=read_line()
    if line is None:return False
    if line.strip()!='GO':raise RuntimeError('Pocket observation cancelled; no window started')
    return True


def paired_preparation_plan(output, *, car_seconds=600):
    """Two existing preparation entries, not a coordinated motion runner."""
    import shlex
    if type(car_seconds) is not int or not 1 <= car_seconds <= 600:
        raise ValueError('Car session duration must be 1..600 seconds')
    root=Path(__file__).resolve().parent
    output=Path(output).resolve()
    car='/home/czy/vicon_mecanum_ws/candidates/wand_4wse_pose2000_20261105/entry.py'
    ros='/opt/ros/jazzy/setup.bash'
    vicon='/home/czy/vicon_receiver_ws/drone_sim/isolated/vicon_b3_recovery_20261022_135653/install/local_setup.bash'
    car_args=['/usr/bin/python3','-B',car,'--launch','--site-ready','--seconds',str(car_seconds)]
    uav_args=['/usr/bin/python3','-u','-B',str(root/'vicon_fixed_point_entry.py'),
              '--run','--handset','pocket','--flight-console',
              '--profile',str(root/'plan_b/fixed_point/vicon_profile.initial_trial.json'),
              '--wifi-peer','192.168.4.1','--wifi-bind','192.168.4.2','--wifi-port','14550',
              '--height','0.3','--hover-seconds','3','--output',str(output/'uav')]
    # shlex, not string interpolation of commands from profiles. Neither entry
    # is given a start token or input to its motion button/flight console.
    terminals=[['gnome-terminal','--window','--title=小车准备 — 原 pose2000 入口',
                '--','bash','-c',shlex.join(car_args)],
               ['gnome-terminal','--window','--title=无人机准备 — 原 Pocket WiFi 入口',
                '--','bash','-c',f'source {shlex.quote(ros)} && source {shlex.quote(vicon)} && exec '+shlex.join(uav_args)]]
    return dict(schema='paired_existing_entries_v1',car_entry=car,car_argv=car_args,
                uav_argv=uav_args,terminal_argvs=terminals,output=str(output),
                automatic_motion=False,ready=False,live_cooperation_connected=False,
                note='仅同时打开原准备入口。小车仍是标定杆模式；不要把两套各自就绪当作自动跟随和车上降落已经接通。')


def main(argv=None):
    import argparse
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run',action='store_true',help='Explicitly open the selected route; no automatic reviews or takeoff')
    ap.add_argument('--prepare-pair',action='store_true',help='Plan/open both existing preparation entries; no motion or automatic coupling; default is file-only')
    ap.add_argument('--cooperative-plan',type=Path,help='Use the existing flight console for the car/UAV task; default is offline plan only')
    ap.add_argument('--car-route-plan',type=Path,help='Existing car-only route console; default is offline, no UAV connection or flight handoffs')
    ap.add_argument('--site-ready',action='store_true',help='Current joint site prepared; required for explicit pair/cooperative resource opens')
    ap.add_argument('--car-seconds',type=int,default=600)
    ap.add_argument('--profile',type=Path)
    ap.add_argument('--recovery',type=Path)
    ap.add_argument('--trs-port')
    ap.add_argument('--handset',choices=('pocket','legacy-gamepad'),default='pocket')
    ap.add_argument('--ground-observe',action='store_true',help='Pocket ground diagnostics only; no gamepad/HOME/RC override or localization sends')
    ap.add_argument('--fusion-observe',action='store_true',help='Motor-isolated Vicon injection; targets/modes require explicit target-load/offboard-check; never arm or write parameters')
    ap.add_argument('--target-load',action='store_true',help='With isolated fusion: frozen ground XYZ/yaw for 3 s; offboard-check extends the bounded window to 15 s')
    ap.add_argument('--offboard-check',action='store_true',help='With target-load and >=30 s window: one disarmed Offboard entry and Position return; no arm')
    ap.add_argument('--wifi-timesync-check',action='store_true',help='Isolated static same-channel time check; temporary TIMESYNC rate restored; no targets/modes')
    ap.add_argument('--fusion-clock-check',action='store_true',help='Continuous static fusion with same-link timesync and three EKF snapshots; 60..180 s; no targets/modes')
    ap.add_argument('--flight-console',action='store_true',help='Pocket reviewed mission controller; horizontal task comes only from its selected profile; no automatic takeoff')
    ap.add_argument('--wifi-peer',default='192.168.4.1')
    ap.add_argument('--wifi-bind',default='192.168.4.2',help='Dedicated drone adapter IPv4; never change the office/car route')
    ap.add_argument('--wifi-port',type=int,default=14550)
    ap.add_argument('--output',type=Path)
    ap.add_argument('--motor-power-isolated',action='store_true',help='Battery disconnected, ESC/motors have no other supply; props may remain installed')
    ap.add_argument('--battery-static',action='store_true',help='Battery-only protected stationary fusion, USB absent; 60 s only, no mode/target/arm messages')
    ap.add_argument('--observe-seconds',type=float,default=15.,help='Observation duration; WiFi timesync uses this as a 30..180 s deadline and exits early on stability')
    ap.add_argument('--wait-for-go',action='store_true',help='Pocket ground window starts only after local terminal GO; no HOME')
    ap.add_argument('--layout-evidence',type=Path,help='Optionally receive and log Vicon using the measured layout; never send poses or approve flight')
    ap.add_argument('--height',type=float,default=None,help='Default: cooperative plan clearance, otherwise 0.3 m')
    ap.add_argument('--hover-seconds',type=float,default=3.)
    a=ap.parse_args(argv)
    if a.car_route_plan:
        if (any((a.cooperative_plan,a.prepare_pair,a.flight_console,a.ground_observe,a.fusion_observe,
                 a.target_load,a.offboard_check,a.wifi_timesync_check,a.fusion_clock_check,
                 a.profile,a.recovery,a.trs_port,a.motor_power_isolated,a.battery_static,a.wait_for_go,a.layout_evidence))
                or a.car_seconds!=600):
            ap.error('--car-route-plan cannot mix drone or independent launcher modes')
        car_plan=load_json(a.car_route_plan)
        from air_ground_resources import validate_bindings
        result=validate_bindings(car_plan,car_only=True)
        if a.run:
            if not a.site_ready or not a.output:ap.error('小车实测需要 --site-ready 和新的 --output 目录')
            owner=open_car_route_session(plan=car_plan,output=a.output,open_devices=True,site_ready=True)
            result=serve_car_route_console(owner,terminal_reader(0))
        print(json.dumps(result,ensure_ascii=False,indent=2))
        if not a.run:return 0
        return 0 if result['status'] and result['status'].get('complete') else 2
    if a.prepare_pair:
        if (not a.output or a.handset!='pocket' or any((a.flight_console,a.ground_observe,a.fusion_observe,
                a.target_load,a.offboard_check,a.wifi_timesync_check,a.fusion_clock_check,a.profile,a.recovery,
                a.trs_port,a.motor_power_isolated,a.battery_static,a.wait_for_go,a.layout_evidence,a.cooperative_plan))):
            ap.error('--prepare-pair requires its own --output and cannot mix other device modes')
        result=paired_preparation_plan(a.output,car_seconds=a.car_seconds)
        if a.run:
            if not a.site_ready:ap.error('--prepare-pair --run requires current --site-ready; no terminal opened')
            if a.output.exists():ap.error('Choose a new output directory')
            import shutil,subprocess
            if not shutil.which('gnome-terminal'):ap.error('gnome-terminal unavailable; use the printed existing entry commands')
            a.output.mkdir(parents=True)
            (a.output/'paired_start.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
            for command in result['terminal_argvs']:
                subprocess.Popen(command,start_new_session=True)
            result['launch_requested']=True  # Not proof either backend is ready.
        print(json.dumps(result,ensure_ascii=False,indent=2));return 0
    cooperative=load_json(a.cooperative_plan) if a.cooperative_plan else None
    if a.height is None:a.height=cooperative['clearance_m'] if cooperative is not None else .3
    if cooperative is not None:
        if a.handset!='pocket' or any((a.ground_observe,a.fusion_observe,a.trs_port,a.recovery,
                a.motor_power_isolated,a.battery_static,a.wait_for_go)):
            ap.error('--cooperative-plan is exclusive to the Pocket flight console')
        if a.run and (not a.flight_console or not a.site_ready):
            ap.error('--cooperative-plan --run requires --flight-console --site-ready')
    if (a.site_ready and cooperative is None) or a.car_seconds!=600:
        ap.error('--site-ready/--car-seconds apply only to --prepare-pair')
    if a.fusion_clock_check and (not a.fusion_observe or a.handset!='pocket' or a.wifi_timesync_check or a.flight_console or a.target_load or a.offboard_check or not 60<=a.observe_seconds<=180):
        ap.error('--fusion-clock-check requires only Pocket --fusion-observe with 60..180 seconds')
    if a.wifi_timesync_check and (not a.fusion_observe or a.handset!='pocket' or a.flight_console or a.target_load or a.offboard_check or not 30<=a.observe_seconds<=180):
        ap.error('--wifi-timesync-check requires only Pocket --fusion-observe with 30..180 seconds')
    if a.offboard_check and (not a.target_load or a.observe_seconds<30):
        ap.error('--offboard-check requires --target-load and --observe-seconds >=30')
    if a.target_load and (not a.fusion_observe or a.handset!='pocket' or a.flight_console or a.ground_observe):
        ap.error('--target-load requires only Pocket isolated --fusion-observe')
    if a.wait_for_go and (a.handset!='pocket' or not a.ground_observe or a.flight_console or a.fusion_observe
                          or not 0<a.observe_seconds<=60):
        ap.error('--wait-for-go requires Pocket --ground-observe and 0 < --observe-seconds <= 60')
    if a.battery_static and (a.handset!='pocket' or a.flight_console or a.ground_observe
            or not a.fusion_observe or not a.fusion_clock_check or a.motor_power_isolated
            or a.target_load or a.offboard_check or a.wifi_timesync_check or a.wait_for_go
            or a.observe_seconds!=60):
        ap.error('--battery-static permits only --fusion-observe --fusion-clock-check --observe-seconds 60')
    if not a.run:
        # The joint plan should not repeat old single-flight preparation
        # history or label already completed flights as pending work.
        result=plan() if cooperative is None else dict(mode='offline_cooperative_plan',
            devices_opened=0,physical_tx_bytes=0,flight_authorized=False)
        if cooperative is not None:
            from air_ground_resources import validate_bindings
            result['cooperative']=validate_bindings(cooperative)
            result['cooperative']['plan_path']=str(a.cooperative_plan.resolve())
        if a.profile:
            profile=load_json(a.profile)
            from ekf_static_profile import require_mission_profile
            require_mission_profile(profile,pilot_source='native_rc' if a.handset=='pocket' else 'gamepad',
                                    height=a.height,hover_seconds=a.hover_seconds)
            if cooperative is not None and profile.get('horizontal_motion') is not None:
                ap.error('Cooperative task requires a fixed-point profile, not a second horizontal task')
            if profile.get('horizontal_motion') is not None:
                from follow_target_generator import HorizontalMotion
                motion=HorizontalMotion(profile['horizontal_motion'],hover_seconds=a.hover_seconds)
                result['horizontal_motion']=motion.describe()
                result['selected_profile']=str(a.profile.resolve())
                result['next_step']='Offline horizontal motion preview; current physical review and TAKEOFF still required'
        print(json.dumps(result,ensure_ascii=False,indent=2));return 0
    if a.handset=='pocket' and not a.flight_console:
        if not ((a.ground_observe != a.fusion_observe) and a.output and (a.motor_power_isolated != a.battery_static)):
            ap.error('Choose observation with --output DIR and exactly one explicit power condition')
        if a.fusion_observe and not a.layout_evidence:
            ap.error('--fusion-observe requires --layout-evidence for the current measured revision')
        if any((a.trs_port,a.profile,a.recovery)):
            ap.error('Pocket ground observation cannot mix legacy gamepad/TRS flight arguments')
        from native_rc_ground import run
        start_ready=None
        if a.wait_for_go:
            import sys
            if not sys.stdin.isatty():ap.error('--wait-for-go requires a local terminal')
            reader=terminal_reader(sys.stdin.fileno())
            start_ready=lambda:ground_go_ready(reader)
        result=run(output=a.output,peer=(a.wifi_peer,a.wifi_port),bind=(a.wifi_bind,a.wifi_port),
            motor_power_isolated=a.motor_power_isolated,battery_static=a.battery_static,
            observe_seconds=a.observe_seconds,layout_evidence=a.layout_evidence,
            fusion_observe=a.fusion_observe,start_ready=start_ready,target_load=a.target_load,offboard_check=a.offboard_check,
            timesync_check=a.wifi_timesync_check,fusion_clock_check=a.fusion_clock_check)
        print(json.dumps(dict(report=str(a.output/'report.json'),error=result['error'],
            observation_window_complete=result.get('observation_window_complete',False),
            diagnostic_gaps=result.get('diagnostic_gaps',[]),
            parameter_review=result.get('parameter_review'),
            observation_summary=result.get('observation_summary'),
            flight_authorized=False),ensure_ascii=False))
        return 2 if result['error'] else 0
    if a.handset=='pocket':
        if not (a.output and a.profile) or any((a.ground_observe,a.fusion_observe,a.motor_power_isolated,a.layout_evidence,a.trs_port,a.recovery)):
            ap.error('Pocket flight console requires --profile and --output, without ground/legacy arguments')
        controller=open_pocket_session(profile_path=a.profile,output=a.output,
            peer=(a.wifi_peer,a.wifi_port),bind=(a.wifi_bind,a.wifi_port),open_devices=True,
            height=a.height,hover_seconds=a.hover_seconds,
            **(dict(cooperative_plan=cooperative,site_ready=a.site_ready) if cooperative is not None else {}))
    else:
        if any((a.ground_observe,a.fusion_observe,a.flight_console,a.output,a.motor_power_isolated,a.layout_evidence)):
            ap.error('Pocket options cannot be used with legacy-gamepad')
        if not all((a.profile,a.recovery,a.trs_port)):
            ap.error('--run requires --profile, --recovery and --trs-port')
        controller=open_session(profile_path=a.profile,recovery_path=a.recovery,
            trs_port=a.trs_port,open_devices=True,height=a.height,hover_seconds=a.hover_seconds)
    import sys
    # 镜像已有终端事件供图形窗口读取；图形运行按钮仍走本入口标准输入。
    # 复用非阻塞输出队列，磁盘/窗口卡顿不能阻塞飞行服务。
    mirror=None
    if a.handset=='pocket':
        try:
            from readiness_console import LatestStatusPrinter
            mirror=LatestStatusPrinter((Path(a.output)/'console_events.jsonl').open('x'),
                capacity=128,close_stream=True)
        except OSError as exc:
            print('图形控制台日志不可用，原终端继续：'+str(exc))
    def emit(text):
        if mirror:
            mirror.publish(json.dumps(dict(at_ns=time.monotonic_ns(),text=text),ensure_ascii=False))
        print(text)
    try:
        serve_console(controller,terminal_reader(sys.stdin.fileno()),emit=emit)
    finally:
        if mirror:mirror.close(drain=True)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
