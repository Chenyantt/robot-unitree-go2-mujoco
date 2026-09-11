#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
exec "${GO2_SIM_PYTHON:-$ROOT/.venv/bin/python}" "$ROOT/scripts/stop_sim.py"
