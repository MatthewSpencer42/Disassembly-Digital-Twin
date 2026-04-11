#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

if docker compose version >/dev/null 2>&1; then
  DOCKER_COMPOSE_CMD=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
  DOCKER_COMPOSE_CMD=(docker-compose)
else
  echo "Docker Compose is required but neither 'docker compose' nor 'docker-compose' was found in PATH." >&2
  echo "Install Docker Compose and rerun this script." >&2
  echo "Use either the Docker Compose v2 plugin ('docker compose') or the legacy 'docker-compose' binary." >&2
  exit 1
fi

exec "${DOCKER_COMPOSE_CMD[@]}" build teleop
