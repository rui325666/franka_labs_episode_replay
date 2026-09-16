#!/usr/bin/env bash
set -euo pipefail
tools_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$tools_dir/replay_dataset.sh" tower_of_babel_20260915_192532 "$@"
