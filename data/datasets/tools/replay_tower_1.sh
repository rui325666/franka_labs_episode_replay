#!/usr/bin/env bash
set -euo pipefail
tools_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mode="${1:-inspect}"
if [[ $# -gt 0 ]]; then shift; fi
if [[ "$mode" == run ]]; then
  # This recording has a fast wrist turn with 0.627 rad recorded tracking lag.
  # Keep the live 0.50 rad ceiling; start trials at half speed by default.
  exec bash "$tools_dir/replay_dataset.sh" tower_of_babel_20260915_195242 "$mode" --speed 0.5 "$@"
fi
exec bash "$tools_dir/replay_dataset.sh" tower_of_babel_20260915_195242 "$mode" "$@"
