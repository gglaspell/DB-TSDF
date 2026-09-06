#!/usr/bin/env bash
set -euo pipefail
# Downloads the FULL archive. Use db_tsdf_offline --synthetic for a small demo.
exec python3 "$(dirname "${BASH_SOURCE[0]}")/download_dataset.py" --sequences 01 "$@"
