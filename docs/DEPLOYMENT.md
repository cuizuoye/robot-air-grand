# 环境、依赖与部署

## 发布形式

这是当前实验室工程的**源码与配置快照**，不是自动刷机或免标定安装器。本次没有改现场控制代码、网络、参数或硬件。仓库保留原工作空间目录结构和来源哈希，便于对照。

源码含现场绝对路径、文件哈希与部署证据引用。现有设备继续使用已经配置的 `/home/czy/vicon_receiver_ws` 和 `/home/czy/vicon_mecanum_ws`；不要用仓库覆盖正在工作的目录。新电脑先离线验证，再按以下步骤建立自己的部署绑定。

## 环境清单

| 项目 | 要求/说明 |
|---|---|
| 系统 | 原现场为 Linux、ROS 2 Jazzy；新部署推荐 Jazzy 对应的 Ubuntu 24.04 |
| Python | Python 3；离线依赖见根目录 `requirements-offline.txt` |
| ROS 2 | `rclcpp`、`rclpy`、`geometry_msgs`、`nav_msgs`、`std_msgs`、`std_srvs`、`tf2*`、`visualization_msgs`、`rosidl_default_generators` |
| 可视化 | `python3-tk`、RViz2；小车原浏览器操作台还需 Node.js |
| 构建 | colcon、ament、CMake、C++17 编译器 |
| Vicon | Tracker 与授权的 DataStream SDK；接收器 CMake 使用 Boost 1.75 相关动态库 |
| 无人机 | MicoAir743v2 / PX4 1.14.3，MAVLink/UDP 链路；无需机载电脑 |
| 小车 | 已匹配的 ESP32-S3 桥与 STM32 4WSE 固件；主机与固件身份/协议须一致 |

`px4_msgs` 与 `motion_capture_tracking_interfaces` 是桥包内部分实验节点的依赖；实际 Pocket/WiFi 实飞入口使用 MAVLink。需要构建整个桥包时，应按 `package.xml` 补齐依赖；不把 Gazebo/uXRCE-DDS 路径当作本次实机数传路径。

## 先做无硬件验证

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements-offline.txt
python scripts/validate_offline.py
```

该脚本运行受限的目标生成与联合阶段测试，使用仓库内源码，拒绝联网、串口及子进程启动。结果在 `validation/latest/`，不会进入 Git。其它历史测试可能依赖现场证据文件；不把缺失现场记录当作可忽略的真实运行条件。

## Vicon 接收器依赖

接收器源码位于 `vicon_receiver_ws/src/ros2-vicon-receiver/vicon_receiver/`。保留其许可证和本地改动。当前 CMake 期望以下布局，SDK 与 Boost 未随仓库上传：

```text
vicon_receiver/third_party/x86_64/
  vicon_sdk/include/...
  vicon_sdk/lib/libViconDataStreamSDK_CPP.so
  boost/include/...
  boost/lib/libboost_{system,thread,chrono,timer}*.so*
```

ARM64 对应 `aarch64/`。具体名称以 `CMakeLists.txt` 为准。通过具备使用许可的 Vicon 安装包取得 SDK，按实际 SDK 版本匹配 Boost 与动态链接依赖；不要复制不明版本的共享库。

依赖齐全后，可在单独工作空间构建接收器：

```bash
source /opt/ros/jazzy/setup.bash
cd vicon_receiver_ws
colcon build --base-paths src/ros2-vicon-receiver --packages-select vicon_receiver
source install/setup.bash
```

构建步骤不连接设备。需要启动时再依据现场 Tracker 地址、对象和坐标系设置参数。无人机桥包和小车包同样用各自 `package.xml` / `setup.py` 构建；发布快照不包含 `build/install`。

## 网络与接线基线

| 通道 | 现场基线 | 用途 |
|---|---|---|
| Vicon 有线 | Tracker `10.10.10.2` | SDK 位姿数据；地址按现场调整 |
| 无人机专用 Wi-Fi | 电脑 `192.168.4.2` ↔ WiFi-Link `192.168.4.1:14550` | MAVLink 双向通信 |
| 小车专用 Wi-Fi | 电脑 `192.168.50.2` ↔ ESP32 `192.168.50.1:46000` | MC-Link；主机反馈端口 46001 |
| 办公网 | 独立网卡/连接 | 保持日常互联网访问 |
| 机载数传 | WiFi-Link TX→飞控 RX4，RX→TX4，共地并按板卡电气规格供电 | 使用原 UART4 插座 |
| 原生遥控 | Pocket / TRS / SBUS → 飞控 | 独立接管链路 |
| 小车内部 | ESP32-S3 UART → STM32 → 四轮驱动/编码器 | 本地执行反馈 |

现有源码的网卡名称、热点名、连接 UUID 是实验室绑定数据，不是通用硬件身份。新机器需先用 NetworkManager 建立连接，再修改准备逻辑的对应身份映射。无线密码保存在本机网络配置，不放进仓库。不要让 QGroundControl 与任务进程争用同一 UDP 端口。

## 必须完成的移植项

1. **路径**：核对 `car_route_adapter.CAR_ENTRY`、小车 `entry.py` / `host/bootstrap.py`、`joint_console_run.py` 的 ROS overlay source 行和配置中的绝对路径。当前代码保留 `/home/czy/...`，在新机器修改后属于新的部署版本。
2. **外参和世界系**：建立 `car01` / `MircoAir`，填写原点偏移、实体机头方向、地面参考、边界与版本。现有数值仅作为数据格式示例。
3. **飞控**：核对实装版本与外部视觉融合参数、坐标和时间处理、遥控接管及降落设置。参数方案按目标硬件重新读回，上传不写参数。
4. **小车部署**：保留版本含 `HOST_MANIFEST.json`、`RELEASE.json` 和历史刷写回执绑定。凭据/私有证据未公开，新部署要建立自身文件与固件一致性记录，不删除校验来跳过。
5. **源码审查绑定**：飞行准备会记录源码、配置及证据哈希。迁移、重构或升级依赖后，用目标现场的验证结果建立新记录，旧授权不复制为新授权。
6. **验证**：离线→静止连接→小车低速→单机起降→联合短线→完整路线，复用同一现场已通过的结果。

因此，仓库中的历史 `physical_validated`、`measured` 或哈希仅保留原记录语义，移植时需与目标设备对应；本发布没有自动生成新硬件授权。

## 固件边界

此仓库上传的是空地协同主机与 ROS 代码。MicoAir/PX4、WiFi-Link、ESP32-S3、STM32 已烧录镜像、厂商 SDK 和完整固件工具链未包含。小车主机包内保留 MC-Link 编解码、握手与故障状态实现，便于理解接口；生产固件的刷写、签名回执和版本匹配由设备部署流程管理。需要替换下位机时，先按现有协议做独立适配，再接入协同状态机。

## 依赖和来源

来源与许可证见根目录 `THIRD_PARTY_NOTICES.md`。外部资料用于接口参考，实机结论取自本仓库的实测摘要。发布没有对软件依赖做一次完整新机器装机验收；本轮验证范围见 `VALIDATION.md`。
