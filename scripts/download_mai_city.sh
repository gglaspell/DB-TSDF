#!/usr/bin/env bash
set -euo pipefail
exec python3 "$(dirname "${BASH_SOURCE[0]}")/download_dataset.py" --sequences 00 01 02 "$@"
