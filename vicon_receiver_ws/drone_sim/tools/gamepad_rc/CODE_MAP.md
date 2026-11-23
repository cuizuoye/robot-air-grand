# 无人机与协同代码地图

更新：2026-11-20。先看 [日常入口](README.md)，需要修改功能时再按下表找模块。

调用关系概览：

```text
vicon_fixed_point_entry.py
  ├─ 准备、遥控/预飞状态 → 连续飞行执行 → MAVLink → 飞控
  ├─ Vicon 接收与坐标变换 → 定位发送 / 融合观察
  └─ air_ground_execution / resources
        ├─ follow_target_generator → 本轮起点对齐、侧向跟随/历史平台目标
        └─ car_route_adapter → 保留的标定杆版控制核与硬件桥

drone_console.py ← 已有会话日志与轨迹
  └─ joint_console_run.py → 显式按钮 → 原 vicon_fixed_point_entry.py 标准输入
ev_delay_joint_capture.py → 需要时单独补采集
```

配置在 `plan_b/`，离线测试在 `test/`，原始实机记录在工作空间 `preparation_runs/`。以下同一组内的文件是职责分工，不是多个替代版本。

## 日常入口和显示

操作者主要使用这些入口；模块清单不是需要依次执行的步骤。

[vicon_fixed_point_entry.py](vicon_fixed_point_entry.py) | [drone_console.py](drone_console.py) | [readiness_console.py](readiness_console.py) | [replay_follow_targets.py](replay_follow_targets.py)

[joint_console_run.py](joint_console_run.py) 仅负责图形按钮与原入口之间的进程、确认及报文解释，不包含另一个车机控制器。启动窗口本身不连接硬件。

## 空地协同

当前地面自动对齐、侧向跟随、地面降落与停车交接；兼容历史平台目标。小车路线与资源仍由同一入口持有。

[air_ground_execution.py](air_ground_execution.py) | [air_ground_resources.py](air_ground_resources.py) | [car_route_adapter.py](car_route_adapter.py) | [follow_target_generator.py](follow_target_generator.py)

## 飞行任务和执行

同一飞行链路的状态机、起飞确认、轨迹发送、退出和记录。

[continuous_device_host.py](continuous_device_host.py) | [continuous_device_io.py](continuous_device_io.py) | [continuous_execution_io.py](continuous_execution_io.py) | [continuous_mission_runtime.py](continuous_mission_runtime.py) | [flight_execution_journal.py](flight_execution_journal.py) | [flight_execution_review.py](flight_execution_review.py) | [flight_mission_core.py](flight_mission_core.py) | [flight_start_confirmation.py](flight_start_confirmation.py) | [frozen_target_stream.py](frozen_target_stream.py) | [mission_live_inputs.py](mission_live_inputs.py) | [mission_wire_transport.py](mission_wire_transport.py) | [mission_audit_collector.py](mission_audit_collector.py)

## 准备阶段和状态

入口编排、设备绑定、状态汇总及既有准备条件。

[drone_preflight.py](drone_preflight.py) | [mission_preparation_coordinator.py](mission_preparation_coordinator.py) | [mission_preparation_io.py](mission_preparation_io.py) | [mission_preparation_runtime.py](mission_preparation_runtime.py) | [preparation_authorization.py](preparation_authorization.py) | [preparation_host_bindings.py](preparation_host_bindings.py) | [preflight_session_status.py](preflight_session_status.py) | [ground_estimator_settlement.py](ground_estimator_settlement.py) | [ground_link_diagnostics.py](ground_link_diagnostics.py)

## 通信与飞控协议

MAVLink 严格校验、UDP/串口、飞控 shell、状态查询和消息编码。

[actuator_query.py](actuator_query.py) | [diagnostic_transaction.py](diagnostic_transaction.py) | [lock_clock_probe.py](lock_clock_probe.py) | [mavlink_crc_stream.py](mavlink_crc_stream.py) | [mavlink_link.py](mavlink_link.py) | [mavlink_readonly_probe.py](mavlink_readonly_probe.py) | [persistent_shell.py](persistent_shell.py) | [px4_modes.py](px4_modes.py) | [px4_shell.py](px4_shell.py) | [short_status_transport.py](short_status_transport.py) | [trs_transport_config.py](trs_transport_config.py)

## 遥控输入和输出状态

Pocket 原生遥控与历史游戏手柄接口；当前主线仍复用其中部分状态和策略。

[combined_input_loss.py](combined_input_loss.py) | [gamepad_estop_monitor.py](gamepad_estop_monitor.py) | [gamepad_rc.py](gamepad_rc.py) | [native_rc_ground.py](native_rc_ground.py) | [output_lock_status.py](output_lock_status.py) | [rc_watch.py](rc_watch.py)

## Vicon 输入和坐标

刚体身份、机头方向、原点换算、接收来源以及人工轴向核对。

[board_center_ruler_candidate.py](board_center_ruler_candidate.py) | [live_attitude_check.py](live_attitude_check.py) | [manual_translation.py](manual_translation.py) | [manual_vertical.py](manual_vertical.py) | [rigid_body_revision.py](rigid_body_revision.py) | [vicon_fc_attitude_check.py](vicon_fc_attitude_check.py) | [vicon_frame_sample_capture.py](vicon_frame_sample_capture.py) | [vicon_heading_calibration.py](vicon_heading_calibration.py) | [vicon_metadata_audit_process.py](vicon_metadata_audit_process.py) | [vicon_receiver_provenance.py](vicon_receiver_provenance.py)

## 定位发送和融合观察

定位压缩/发送及飞控 EKF 融合状态解码，不是另一个飞行控制器。

[compact_vision.py](compact_vision.py) | [low_bandwidth_vision.py](low_bandwidth_vision.py) | [fixed_mission_fusion.py](fixed_mission_fusion.py) | [mission_fusion_service.py](mission_fusion_service.py) | [fusion_binary_protocol.py](fusion_binary_protocol.py) | [fusion_binary_receiver.py](fusion_binary_receiver.py) | [fusion_binary_record.py](fusion_binary_record.py) | [ekf_ground_acceptance.py](ekf_ground_acceptance.py) | [ekf_static_profile.py](ekf_static_profile.py)

## 联合采集与时延标定

现有联合采集入口及数据源、时间配对和采集生命周期；仅缺数据时使用。

[ev_delay_calibration.py](ev_delay_calibration.py) | [ev_delay_capture.py](ev_delay_capture.py) | [ev_delay_device_io.py](ev_delay_device_io.py) | [ev_delay_joint_capture.py](ev_delay_joint_capture.py) | [ev_delay_live_owner.py](ev_delay_live_owner.py) | [ev_delay_maintenance.py](ev_delay_maintenance.py) | [ev_delay_maintenance_host.py](ev_delay_maintenance_host.py) | [ev_delay_sources.py](ev_delay_sources.py) | [ev_delay_vicon_source.py](ev_delay_vicon_source.py)

## 参数、日志与恢复

参数备份、临时频率及其恢复、原始证据保存；参数工具并非日常飞行入口。

[fault_archive_service.py](fault_archive_service.py) | [mission_telemetry_rates.py](mission_telemetry_rates.py) | [parameter_snapshot.py](parameter_snapshot.py) | [px4_params.py](px4_params.py) | [px4_readonly_backup.py](px4_readonly_backup.py) | [restore_preparation_rates.py](restore_preparation_rates.py) | [static_ulog_capture.py](static_ulog_capture.py) | [static_ulog_capture_actions.py](static_ulog_capture_actions.py) | [static_ulog_capture_policy.py](static_ulog_capture_policy.py)

## 沿用的地面诊断组件

旧命名组件仍在导入或文件校验依赖中；保留不代表要求重跑这些台架流程。

[fc_health_readonly.py](fc_health_readonly.py) | [fixed_ground_entry.py](fixed_ground_entry.py) | [prepare_drone.py](prepare_drone.py) | [trs_battery_readiness.py](trs_battery_readiness.py) | [trs_fusion_restore_only.py](trs_fusion_restore_only.py) | [trs_message_interval_trial.py](trs_message_interval_trial.py) | [trs_only_estop_bench.py](trs_only_estop_bench.py) | [trs_powered_fusion_bench.py](trs_powered_fusion_bench.py) | [trs_runtime_monitor.py](trs_runtime_monitor.py) | [trs_static_fusion_bench.py](trs_static_fusion_bench.py) | [trs_uart_readonly_check.py](trs_uart_readonly_check.py) | [trs_vicon_receive_bench.py](trs_vicon_receive_bench.py) | [usb_vicon_receive_bench.py](usb_vicon_receive_bench.py)

## 沿用的策略与证据接口

为现有入口提供结构化状态和条件判定；本轮只清理孤立旧工具，没有调整这些行为。

[control_readonly_policy.py](control_readonly_policy.py) | [disarmed_offboard_policy.py](disarmed_offboard_policy.py) | [ekf_compare_readonly_policy.py](ekf_compare_readonly_policy.py) | [estop_only_policy.py](estop_only_policy.py) | [height_readonly_policy.py](height_readonly_policy.py) | [horizontal_fusion_permit.py](horizontal_fusion_permit.py) | [incident_guard.py](incident_guard.py) | [one_home_preflight_policy.py](one_home_preflight_policy.py) | [receive_only_permit.py](receive_only_permit.py) | [rigid_revision_ground_permit.py](rigid_revision_ground_permit.py) | [rigid_revision_loss_permit.py](rigid_revision_loss_permit.py) | [rigid_revision_static_permit.py](rigid_revision_static_permit.py) | [static_fusion_permit.py](static_fusion_permit.py) | [static_lock_gate.py](static_lock_gate.py) | [trs_powered_fusion_permit.py](trs_powered_fusion_permit.py) | [trs_powered_fusion_policy.py](trs_powered_fusion_policy.py) | [trs_range_static_policy.py](trs_range_static_policy.py) | [trs_static_fusion_policy.py](trs_static_fusion_policy.py) | [trs_vision_reference_permit.py](trs_vision_reference_permit.py) | [trs_vision_reference_policy.py](trs_vision_reference_policy.py) | [usb_combined_fusion_policy.py](usb_combined_fusion_policy.py) | [usb_fusion_policy.py](usb_fusion_policy.py) | [usb_height_fusion_policy.py](usb_height_fusion_policy.py) | [usb_vision_policy.py](usb_vision_policy.py) | [usb_yaw_fusion_policy.py](usb_yaw_fusion_policy.py) | [vision_reference_fusion_permit.py](vision_reference_fusion_permit.py) | [vision_reference_xyz_policy.py](vision_reference_xyz_policy.py)

## 离线验证辅助

合成设备、离线状态机和已有记录审核；结果不能代替实机完成。

[continuous_flight.py](continuous_flight.py) | [fusion_binary_offline.py](fusion_binary_offline.py) | [offline_flight_release.py](offline_flight_release.py) | [phase2_acceptance.py](phase2_acceptance.py) | [synthetic_mission_peer.py](synthetic_mission_peer.py)

## 其他工作空间依赖

- Vicon 接收器及 ROS 2 包位于 `/home/czy/vicon_receiver_ws`；没有因清理无人机而修改共享接收器。
- 当前小车版本为 `/home/czy/vicon_mecanum_ws/candidates/wand_4wse_pose2000_20261105`；协同复用它，不启动旧候选版本。
- 保留的旧名称文件中有校验绑定；若以后要继续合并模块，应单独调整调用和校验，不能只按文件名删除。
