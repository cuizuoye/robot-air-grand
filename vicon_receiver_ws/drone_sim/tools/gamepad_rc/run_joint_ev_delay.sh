#!/bin/bash
# Process-local ROS environment; no daemon, hardware or network changes here.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/czy/vicon_receiver_ws/drone_sim/isolated/vicon_b3_recovery_20261022_135653/install/local_setup.bash
exec /usr/bin/python3 -B /home/czy/vicon_receiver_ws/drone_sim/tools/gamepad_rc/ev_delay_joint_capture.py "$@"
