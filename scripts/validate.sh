#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PYTHON="${GO2_SIM_PYTHON:-$ROOT/.venv/bin/python}"
export PYTHONPATH="$ROOT${PYTHONPATH:+:$PYTHONPATH}"
"$PYTHON" "$ROOT/scripts/prepare_assets.py"
"$PYTHON" "$ROOT/scripts/export_description.py"
"$PYTHON" -m unittest discover -s "$ROOT/tests" -v
"$PYTHON" "$ROOT/scripts/test_locomotion.py" --report "$ROOT/.runtime/locomotion.json"
while IFS= read -r -d '' file; do bash -n "$file"; done < <(find "$ROOT/scripts" "$ROOT/sim" -name '*.sh' -print0)
bash -n "$ROOT/start.sh" "$ROOT/build.sh"
