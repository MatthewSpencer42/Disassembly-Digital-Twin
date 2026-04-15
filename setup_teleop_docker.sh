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
  echo "Install Docker Compose and rerun this script." >&2
  echo "Use either the Docker Compose v2 plugin ('docker compose') or the legacy 'docker-compose' binary." >&2
  exit 1
fi

exec "${DOCKER_COMPOSE_ENV[@]}" "${DOCKER_COMPOSE_CMD[@]}" build teleop
