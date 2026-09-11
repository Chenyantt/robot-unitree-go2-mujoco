#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"
PYTHON="${GO2_SIM_PYTHON:-$ROOT/.venv/bin/python}"
"$PYTHON" "$ROOT/scripts/prepare_assets.py"
"$PYTHON" "$ROOT/scripts/export_description.py"
rbnx build -f "$ROOT/robonix_manifest.yaml"
