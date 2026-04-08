#!/usr/bin/env bash
set -euo pipefail

CONTAINER_ID=$(docker ps --filter "ancestor=agentic-disassembly-teleop:humble" --format '{{.ID}}' | head -1)

if [[ -z "${CONTAINER_ID}" ]]; then
  echo "No running agentic-disassembly-teleop container found." >&2
  exit 1
fi

echo "Attaching to container ${CONTAINER_ID}..."
exec docker exec -it "${CONTAINER_ID}" bash -lc 'source /ws/install/setup.bash 2>/dev/null && exec bash'
