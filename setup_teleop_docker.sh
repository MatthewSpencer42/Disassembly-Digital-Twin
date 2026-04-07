#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

if ! command -v docker-compose >/dev/null 2>&1; then
  echo "docker-compose is required but was not found in PATH." >&2
  exit 1
fi

exec docker-compose build teleop
