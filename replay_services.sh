#!/usr/bin/env bash
# Manage the existing example_station services required by replay/cali.
set -euo pipefail
labs_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
station_dir="$labs_root/deployments/example_station"
compose=(docker compose --project-directory "$station_dir" -p example_station -f "$station_dir/docker-compose.yml")
arm_services=(franka-robot robotiq-gripper controller-coordinator)
all_services=("${arm_services[@]}" zed-camera-head)

usage() {
  cat <<'EOF'
LABS 回放底层服务
用法：bash replay_services.sh 命令 [参数]

  start [--no-camera] [--dry-run]  启动底层服务，随后进行只读就绪检查
  camera [--dry-run]             只启动头部相机，供 cali 使用
  check [1|2|3|4] [--dry-run]    只检查双臂、夹爪、控制器和指令冲突，默认轨迹 3
  status [--dry-run]            显示四个底层服务的容器状态
  logs [服务名] [--dry-run]     显示最近 100 行日志
  stop [--dry-run]              停止上述四个服务；应先退出回放/遥操作

start 复用已存在的容器，不重建运行中的服务。新启动夹爪驱动可能初始化开合。
start 不播放轨迹；检查通过后运行 bash replay.sh 选择 cali 或数字。
--dry-run 仅打印命令，不调用 Docker，也不连接或控制硬件。
详细中文说明：REPLAY_README_ZH.md
EOF
}

mode="${1:---help}"
if [[ $# -gt 0 ]]; then shift; fi
dry_run=false
with_camera=true
selection=3
log_service=
selection_given=false
for arg in "$@"; do
  case "$arg" in
    --dry-run) dry_run=true ;;
    --no-camera)
      [[ "$mode" == start ]] || { echo '--no-camera 仅用于 start。' >&2; exit 2; }
      with_camera=false
      ;;
    1|2|3|4)
      [[ "$mode" == check && "$selection_given" == false ]] || { echo '轨迹数字仅用于 check，且只能指定一次。' >&2; exit 2; }
      selection="$arg"
      selection_given=true
      ;;
    franka-robot|robotiq-gripper|controller-coordinator|zed-camera-head)
      [[ "$mode" == logs && -z "$log_service" ]] || { echo '服务名仅用于 logs，且只能指定一个。' >&2; exit 2; }
      log_service="$arg"
      ;;
    *) echo "未知参数：$arg" >&2; usage >&2; exit 2 ;;
  esac
done
case "$mode" in
  -h|--help|help) usage; exit 0 ;;
  start|camera|check|status|logs|stop) ;;
  *) echo "未知命令：$mode" >&2; usage >&2; exit 2 ;;
esac

run() {
  printf '+ '
  printf '%q ' "$@"
  printf '\n'
  if [[ "$dry_run" == false ]]; then "$@"; fi
}

if [[ "$dry_run" == false ]]; then
  command -v docker >/dev/null || { echo '找不到 docker。' >&2; exit 1; }
  docker info >/dev/null || { echo 'Docker 服务不可用，或当前用户没有访问权限。' >&2; exit 1; }
  docker compose version >/dev/null
  # Do not start/stop dependencies under an active replay, or collide with
  # the fixed container name used by the read-only check.
  if [[ "$mode" == start || "$mode" == stop || "$mode" == check ]]; then
    if [[ -n "$(docker ps --filter 'name=^/labs-dataset-replay$' --format '{{.Names}}')" ]]; then
      echo 'labs-dataset-replay 正在运行。先在原终端退出回放/检查，并等待停止核验结束。' >&2
      exit 1
    fi
  fi
fi

check_arms() {
  run bash "$labs_root/replay.sh" "$selection" check
}

case "$mode" in
  start)
    services=("${arm_services[@]}")
    if [[ "$with_camera" == true ]]; then services+=(zed-camera-head); fi
    echo '启动回放底层服务；新启动夹爪驱动可能执行初始化开合。'
    run "${compose[@]}" up -d --no-deps --no-recreate --no-build --pull never "${services[@]}"
    run "${compose[@]}" ps -a "${services[@]}"
    if [[ "$dry_run" == true ]]; then
      check_arms
      echo '以上仅为命令预览；实际启动后将进行最多 6 次只读就绪检查。'
      exit 0
    fi
    echo '等待双臂/夹爪反馈和控制器就绪（只读检查，最多 6 次）。'
    for attempt in {1..6}; do
      if check_arms; then
        echo '底层回放检查通过。ready at starting pose: False 仅表示尚未归位，run 会自动归位。'
        echo '下一步：bash replay.sh（选择 cali 校准，或输入数字执行回放）。'
        exit 0
      fi
      if (( attempt < 6 )); then
        echo "第 $attempt 次未通过，3 秒后重试；可按 Ctrl+C 结束等待。"
        sleep 3
      fi
    done
    echo '服务启动命令已执行，但实际反馈/控制器检查未通过；服务保留运行，未执行回放。' >&2
    echo '请检查两台机械臂 FCI、夹爪 USB/供电、网页遥操作状态；查看 bash replay_services.sh logs。' >&2
    exit 1
    ;;
  camera)
    run "${compose[@]}" up -d --no-deps --no-recreate --no-build --pull never zed-camera-head
    echo '下一步：bash replay.sh cali。相机实际出图情况在校准窗口查看。'
    ;;
  check) check_arms ;;
  status) run "${compose[@]}" ps -a "${all_services[@]}" ;;
  logs)
    services=("${all_services[@]}")
    if [[ -n "$log_service" ]]; then services=("$log_service"); fi
    run "${compose[@]}" logs --tail 100 "${services[@]}"
    ;;
  stop)
    run "${compose[@]}" stop controller-coordinator franka-robot robotiq-gripper zed-camera-head
    if [[ "$dry_run" == false ]]; then
      echo '回放底层服务已停止。下次回放先执行 bash replay_services.sh start。'
    fi
    ;;
esac
