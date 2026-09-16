#!/usr/bin/env bash
set -euo pipefail
tools_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd -- "$tools_dir/../../.." && pwd)"
image=registry.localhost/labs/replay-gui:latest
headless=false
for arg in "$@"; do
  case "$arg" in --headless|-h|--help) headless=true ;; esac
done
if ! docker image inspect "$image" >/dev/null 2>&1; then
  echo '缺少校准镜像。先运行 bash scripts/images.sh import 镜像目录，或 bash scripts/build_images.sh gui。' >&2
  exit 1
fi

display_args=()
if [[ "$headless" == false ]]; then
  [[ -n "${DISPLAY:-}" ]] || { echo '请在本机 Linux 桌面终端打开 cali；无窗口诊断可加 --headless --seconds 10。' >&2; exit 1; }
  display_args+=(-e "DISPLAY=$DISPLAY" -e QT_X11_NO_MITSHM=1 -e QT_QPA_PLATFORM=xcb
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro)
  auth_file="${XAUTHORITY:-$HOME/.Xauthority}"
  if [[ -f "$auth_file" ]]; then
    display_args+=(-e XAUTHORITY=/tmp/replay.xauth -v "$auth_file:/tmp/replay.xauth:ro")
  fi
fi
# Keep the checkout path inside Docker; custom --ref/--out must be inside it.
exec docker run --rm --pull never --init --network host --user "$(id -u):$(id -g)" \
  -e ROS_DOMAIN_ID=0 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e CYCLONEDDS_URI=file:///replay-cyclonedds.xml -e ROS_LOG_DIR=/tmp/replay-ros-logs \
  "${display_args[@]}" -v "$root:$root" -w "$root" \
  -v "$root/deployments/example_station/cyclonedds.xml:/replay-cyclonedds.xml:ro" \
  --entrypoint /bin/bash "$image" \
  -c 'source /opt/ros/jazzy/setup.bash; exec python3 -u "$@"' bash \
  "$tools_dir/live_scene_overlay.py" "$@"
