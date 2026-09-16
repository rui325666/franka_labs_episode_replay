#!/usr/bin/env bash
set -euo pipefail
tools_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
labs_root="$(cd -- "$tools_dir/../../.." && pwd)"
dataset_name="${1:?Expected dataset name}"
shift
[[ "$dataset_name" =~ ^[a-zA-Z0-9_-]+$ ]] || { echo 'Invalid dataset name.' >&2; exit 2; }
dataset="$labs_root/data/datasets/lerobot/$dataset_name"
bundle="$labs_root/data/datasets/replay/$dataset_name"
mode="${1:-inspect}"
if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in
  prepare)
    mkdir -p "$bundle"
    exec docker run --rm --pull never --init --network none --user "$(id -u):$(id -g)" \
      -v "$tools_dir:$tools_dir:ro" -v "$dataset:$dataset:ro" -v "$bundle:$bundle" \
      --entrypoint /workspace/.venv/bin/python registry.localhost/labs/dataset-builder:latest \
      "$tools_dir/prepare_replay.py" --dataset "$dataset" --output "$bundle" "$@"
    ;;
  inspect)
    exec python3 "$tools_dir/inspect_bundle.py" "$bundle" "$@"
    ;;
  check|gripper-check|home|run)
    [[ -f "$bundle/manifest.json" ]] || { echo 'Run prepare first.' >&2; exit 1; }
    exec docker run --rm --pull never --init --name labs-dataset-replay --network host --stop-timeout 30 \
      --user "$(id -u):$(id -g)" \
      -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
      -e CYCLONEDDS_URI=file:///replay-cyclonedds.xml -e ROS_LOG_DIR=/tmp/replay-ros-logs \
      -v "$labs_root/deployments/example_station/cyclonedds.xml:/replay-cyclonedds.xml:ro" \
      -v "$tools_dir:$tools_dir:ro" -v "$bundle:$bundle" \
      --entrypoint /bin/bash registry.localhost/labs/controller-coordinator:latest \
      -c 'source /opt/ros/humble/setup.bash; exec python3 -u "$@"' bash \
      "$tools_dir/replay_labs.py" "$mode" --bundle "$bundle" "$@"
    ;;
  *) echo 'Usage: bash replay_dataset.sh DATASET_NAME prepare|inspect|check|gripper-check|home|run [--speed 1.0]' >&2; exit 2 ;;
esac
