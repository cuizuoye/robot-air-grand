# 来源、依赖与许可证

本仓库为现有工程的整理快照。原代码中的版权和包许可证声明保留；本次发布未额外授予一个覆盖全部内容的新许可证。

| 组件 | 来源 / 管理方式 |
|---|---|
| Vicon ROS 2 接收器 | 上游 `OPT4SMART/ros2-vicon-receiver`，本地派生自 `cuizuoye/ros2-vicon-receiver`；包含 FrameSample 等本地修改。保留目录内 GPL-3.0 许可证文件 |
| Vicon DataStream SDK | Vicon 提供的专有依赖，由使用方按厂商条款取得；未上传 SDK 头文件、二进制或 Boost vendor 树 |
| PX4 | 外部飞控固件依赖，仓库未复制 PX4 固件工程或固件镜像 |
| MAVLink / pymavlink | 外部协议与 Python 依赖，保留使用方原有许可，未复制依赖包 |
| ROS 2 / RViz / NumPy / pytest / pyserial | 外部运行/开发依赖，按各自项目许可证使用 |
| UAV / mecanum ROS 包 | 保留各包 `package.xml` 中既有声明；项目自有应用未在本次代为设置新许可证 |
| ESP32-S3 / STM32 固件与库 | 接口在主机源码中可查；本次不分发整套固件、SDK/HAL、镜像或刷写凭据 |

参考入口（用于查阅接口，不作为本项目实机测试证据）：

- [Vicon Tracker](https://www.vicon.com/software/tracker/)
- [Vicon DataStream SDK](https://www.vicon.com/software/datastream-sdk/)
- [接收器上游](https://github.com/OPT4SMART/ros2-vicon-receiver)
- [接收器项目派生仓库](https://github.com/cuizuoye/ros2-vicon-receiver)
- [PX4 v1.14 外部位置估计](https://docs.px4.io/v1.14/en/ros/external_position_estimation.html)
- [PX4 v1.14 Offboard](https://docs.px4.io/v1.14/en/flight_modes/offboard.html)
- [MAVLink 消息定义](https://mavlink.io/en/messages/common.html)
- [MicoAir WiFi-Link](https://micoair.com/zh/docs/telemetry/micoair-wifi-link/micoair-wifi-link-wifi6-telemetry)

公开目录不含现场网络密码、运行签名密钥、私有授权文件或大体积原始日志。配置中的私有网段、已知设备名和原本机路径保留为工程结构说明，移植时按部署文档替换。
