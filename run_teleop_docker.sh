#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

if ! command -v docker-compose >/dev/null 2>&1; then
  echo "docker-compose is required but was not found in PATH." >&2
  exit 1
fi

if ! command -v xhost >/dev/null 2>&1; then
  echo "xhost is required for GUI forwarding but was not found in PATH." >&2
  exit 1
fi

xhost +local:root >/dev/null
exec docker-compose run --rm teleop bash -lc '
  set -e
  cd /ws
  colcon build --packages-select nr_dual_arm_description nr_dual_arm_moveit_config arm_teleop
  source /ws/install/setup.bash
  exec bash
'
