#!/usr/bin/env bash
# 只打开操作台；显式点击准备/就位/起飞按钮后才运行原联合入口。
set -e
source /opt/ros/jazzy/setup.bash
console_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$console_dir/drone_console.py" "$@"
