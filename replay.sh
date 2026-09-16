#!/usr/bin/env bash
set -euo pipefail
labs_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ $# -eq 0 ]]; then
  echo '选择回放轨迹或场景校准：'
  echo '  1  最新录制（2026-09-15 19:52，默认半速 111.2 秒）'
  echo '  2  倒数第二段（2026-09-15 19:25，70.1 秒）'
  echo '  3  2026-09-15 19:18，87.4 秒'
  echo '  4  2026-09-15 17:47，114.2 秒'
  echo '  cali  实时叠图校准（参考：轨迹 3 起始画面）'
  echo '  start  启动回放底层服务和头部相机，并检查就绪状态'
  echo '  status  查看底层服务容器状态'
  echo '  q  退出'
  read -r -p '请输入 1、2、3、4、cali、start 或 status：' selection || exit 0
else
  selection="$1"
  shift
fi

case "$selection" in
  1) runner="$labs_root/data/datasets/tools/replay_tower_1.sh" ;;
  2) runner="$labs_root/data/datasets/tools/replay_tower_2.sh" ;;
  4) runner="$labs_root/data/datasets/tools/replay_tower.sh" ;;
  3) runner="$labs_root/data/datasets/tools/replay_tower_3.sh" ;;
  cali) exec bash "$labs_root/data/datasets/tools/calibrate_scene.sh" "$@" ;;
  start|status|stop) exec bash "$labs_root/replay_services.sh" "$selection" "$@" ;;
  q|Q) exit 0 ;;
  -h|--help)
    echo '用法：bash replay.sh [1|2|3|4] [run|inspect|check|home|gripper-check] [--speed 1.0]'
    echo '不带参数时输入数字选择；自动初始化夹爪和双臂。1 默认半速，2/3/4 默认原速。'
    echo '场景校准：bash replay.sh cali（实时画面与轨迹 3 起始画面叠加，关闭窗口退出）'
    echo '校准参数：bash replay.sh cali --help'
    echo '底层服务：bash replay.sh start | status | stop（start 后进行只读就绪检查）'
    echo '完整中文说明：REPLAY_README_ZH.md；服务脚本帮助：bash replay_services.sh --help'
    exit 0
    ;;
  *) echo '可选择 1、2、3、4、cali、start、status、stop 或 q（退出）。' >&2; exit 2 ;;
esac

mode=run
if [[ $# -gt 0 && "$1" != --* ]]; then
  mode="$1"
  shift
fi
[[ -f "$runner" ]] || { echo '该轨迹的回放脚本尚未准备完成。' >&2; exit 1; }
echo "选择轨迹 $selection，模式 $mode"
exec bash "$runner" "$mode" "$@"
