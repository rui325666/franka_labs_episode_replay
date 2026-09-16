#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
for trajectory in 1 2 3 4; do
  bash "$root/replay.sh" "$trajectory" inspect >/dev/null
  echo "轨迹 $trajectory 文件校验通过"
done
exec docker run --rm --pull never --init --network none \
  -e ROS_DOMAIN_ID=93 -e ROS_LOCALHOST_ONLY=1 -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v "$root/data/datasets/tools:/tests:ro" -w /tests \
  --entrypoint /bin/bash registry.localhost/labs/controller-coordinator:latest \
  -c 'source /opt/ros/humble/setup.bash; exec python3 test_replay_labs.py'
