#!/usr/bin/env bash
set -e
exec bash "$(dirname "$0")/sim/start.sh" "$@"
