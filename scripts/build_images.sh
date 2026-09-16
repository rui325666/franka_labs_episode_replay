#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
selection="${1:-all}"
if [[ $# -gt 1 ]]; then echo '只接受一个目标参数。' >&2; exit 2; fi
case "$selection" in
  all) services=(franka-robot robotiq-gripper controller-coordinator zed-camera gui) ;;
  franka-robot|robotiq-gripper|controller-coordinator|zed-camera|gui) services=("$selection") ;;
  -h|--help) echo '用法：bash scripts/build_images.sh [all|franka-robot|robotiq-gripper|controller-coordinator|zed-camera|gui]'; exit 0 ;;
  *) echo '无效构建目标。' >&2; exit 2 ;;
esac
for service in "${services[@]}"; do
  echo "构建 $service（需要联网；不启动硬件）"
  if [[ "$service" == gui ]]; then
    docker build --pull=false -t registry.localhost/labs/replay-gui:latest \
      -f "$root/docker/replay-gui.dockerfile" "$root/docker"
  else
    category=actuators
    if [[ "$service" == zed-camera ]]; then category=sensors; fi
    context="$root/services/$category/$service"
    docker build --pull=false --target dev -t "registry.localhost/labs/$service:latest" \
      -f "$context/$service.dockerfile" "$context"
  fi
done
