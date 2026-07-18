#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
setup_file="$root/install/setup.bash"

if [[ ! -f "$setup_file" ]]; then
  echo "Workspace is not built. Run 'pixi run build' first." >&2
  exit 1
fi

# Colcon setup scripts probe optional variables such as COLCON_TRACE.
set +u
# shellcheck disable=SC1090
source "$setup_file"
set -u
exec ros2 "$@"
