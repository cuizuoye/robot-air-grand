# 当前代码入口地图

所有路径相对本仓库。先读主入口和四个协同模块，再沿导入关系阅读协议细节。

## 无人机与协同

根目录：`vicon_receiver_ws/drone_sim/tools/gamepad_rc/`。

| 文件 | 主要职责 |
|---|---|
| `vicon_fixed_point_entry.py` | 统一 CLI；准备、单机任务、车机联合、小车独立路线 |
| `drone_console.py` / `run_drone_console.sh` | 图形操作台、轨迹、速度、位置差、RViz、回放 |
| `joint_console_run.py` | 专用网卡准备、后端进程、界面按钮、阶段与错误解释 |
| `follow_target_generator.py` | 相对目标、外框避让、参考速度/加速度约束、矩形/圆周几何、联合阶段 |
| `air_ground_execution.py` | 将联合目标接入既有飞行状态机与小车运动任务 |
| `air_ground_resources.py` | Vicon 成对样本、资源独占、机体参考点变换与实机桥 |
| `car_route_adapter.py` | 协同路线接入保留的 4WSE 控制核；连续圆周路径跟踪 |
| `flight_mission_core.py` / `continuous_*` | 单机任务执行、起飞与收尾状态机 |
| `compact_vision.py` / `mavlink_*` | 外部视觉消息、WiFi/串口传输、CRC 与遥测 |
| `mission_fusion_service.py` / `fixed_mission_fusion.py` | 飞控实际融合观察与定位服务 |
| `ev_delay_*` / `rigid_body_revision.py` | 已有联合标定、时间配对、刚体与外参版本 |
| `native_rc_ground.py` / `rc_watch.py` | Pocket 原生遥控状态读取与接管处理 |
| `plan_b/follow/*.json` | 短线、矩形、圆周及离线示例计划 |
| `plan_b/fixed_point/vicon_profile.*.json` | 原现场飞行配置快照，带历史路径绑定 |
| `test/` | 离线单元与回归测试；含部分依赖本机场地证据的历史测试 |

更细的函数分组见该目录中的 `CODE_MAP.md`。本次上传复用源码，不通过删文件或修改阈值改变现场控制行为。

## 小车

`vicon_mecanum_ws/candidates/wand_4wse_pose2000_20261105/entry.py` 是保留的标定杆遥操作入口。`host/runtime/src/` 内的两个 4WSE 包提供控制与硬件桥；协同复用其中的控制契约、发送器与反馈，并未同时启动第二个标定杆控制环。

`vicon_mecanum_ws/src/mecanum_vicon_control/` 提供共享协议与基础控制，`mecanum_description/` 提供模型。浏览器小车操作台位于保留版本的 `host/web/`。

## ROS 2 与 Vicon

- `vicon_receiver_ws/src/ros2-vicon-receiver/vicon_receiver/`：C++ 采集、PoseStamped/FrameSample 与帧队列。
- `vicon_receiver_ws/drone_sim/ros2_ws/src/uav_coord_bridge/`：坐标变换、MAVLink 编码、目标配准与可视化桥。
- `uav_bringup/`：既有 ROS 启动与配置资源。

桥包保留部分 Gazebo/PX4 ROS 2 实验接口；实际 MicoAir 飞行链路使用 MAVLink/UDP，阅读时以统一入口的调用链为准。

## 代码与现场资料的边界

`source-manifest.json` 记录本次每个复制文件的源工作空间位置与 SHA-256。部署授权、设备签名密钥、临时 token、私有采集目录、历史原始日志、厂商 SDK 和构建产物均留在本地。历史固件刷写回执/哈希仍可在源码中被引用，它们属于部署条件，不随源码复制自动成立。
