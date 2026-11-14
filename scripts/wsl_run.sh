#!/bin/bash
# Helper: run a project script with the drone311 env from anywhere.
# Usage: bash scripts/wsl_run.sh <python_script> [args...]
cd "$(dirname "$0")/.."
export PATH="$HOME/miniconda3/envs/drone311/bin:$PATH"
if [ -n "$1" ]; then
    python "$@"
fi
