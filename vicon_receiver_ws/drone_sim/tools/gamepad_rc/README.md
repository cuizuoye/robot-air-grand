# 无人机与空地协同主机源码

本目录是现行入口的源码快照。请从仓库根目录 [README](../../../../README.md) 阅读功能、技术报告与最新状态，部署看 [DEPLOYMENT](../../../../docs/DEPLOYMENT.md)。

主要入口为 `vicon_fixed_point_entry.py` 和 `run_drone_console.sh`，协同核心为 `follow_target_generator.py`、`air_ground_execution.py`、`car_route_adapter.py`、`air_ground_resources.py`。详细分组见 [CODE_MAP](CODE_MAP.md)。

`gamepad_rc` 是历史目录名，实机主线使用 Pocket。源码保留依赖模块与原现场路径；私有授权/证据不随仓库发布。不要直接运行旧配置中的实机指令来替代新部署准备。
