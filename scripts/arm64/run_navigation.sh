#!/usr/bin/env bash
set -euo pipefail
exec /usr/bin/python3 "$(dirname "$(readlink -f "$0")")/workspace.py" navigation "$@"
