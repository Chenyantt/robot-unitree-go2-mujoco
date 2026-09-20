#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
exec "${HOST_PYTHON:-/usr/bin/python3}" "$ROOT/scripts/named-action.py" "$@"
