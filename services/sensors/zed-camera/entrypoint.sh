#!/bin/bash

set -eo pipefail

source /opt/ros/jazzy/setup.bash
source /workspace/ros2_ws/install/setup.bash

config_file="${ZED_CAMERA_CONFIG_FILE:-/workspace/config_zed_camera.yml}"
if [[ ! -r "${config_file}" ]]; then
    echo "ZED camera config is not readable: ${config_file}" >&2
    exit 1
fi

echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-unset}"
echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-unset}"
echo "ZED camera config: ${config_file}"

exec /workspace/ros2_ws/install/zed_open_capture_ros/lib/zed_open_capture_ros/zed_open_capture_node \
    --ros-args \
    --params-file "${config_file}"
