#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIST_DIR="$ROOT_DIR/viewer/dist"

if [ ! -d "$DIST_DIR" ]; then
  echo "[INFO] Building Three.js frontend..."
  (cd "$ROOT_DIR/viewer" && bun install && bun run build)
fi

exec python3 "$ROOT_DIR/viewer/server.py" --open "$@"
