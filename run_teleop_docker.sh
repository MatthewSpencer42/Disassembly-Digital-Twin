#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

if docker compose version >/dev/null 2>&1; then
  DOCKER_COMPOSE_CMD=(docker compose)
  DOCKER_COMPOSE_ENV=()
elif command -v docker-compose >/dev/null 2>&1; then
  DOCKER_COMPOSE_CMD=(docker-compose)
  # Legacy docker-compose is a Python entrypoint and can break if user-site
  # packages override the distro docker/requests stack.
  DOCKER_COMPOSE_ENV=(env -u PYTHONPATH PYTHONNOUSERSITE=1)
else
  echo "Docker Compose is required but neither 'docker compose' nor 'docker-compose' was found in PATH." >&2
  exit 1
fi

if ! command -v xhost >/dev/null 2>&1; then
  echo "xhost is required for GUI forwarding but was not found in PATH." >&2
  exit 1
fi

xhost +local:root >/dev/null
exec "${DOCKER_COMPOSE_ENV[@]}" "${DOCKER_COMPOSE_CMD[@]}" run --rm teleop bash -lc '
  set -e
  cd /ws
  colcon build --packages-select ros_tcp_endpoint --executor sequential
  source /ws/install/setup.bash
  colcon build --packages-select nr_dual_arm_description nr_dual_arm_moveit_config arm_teleop --executor sequential
  source /ws/install/setup.bash
  exec bash
'
