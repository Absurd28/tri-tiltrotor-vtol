#!/bin/bash
# Start the VTOL ground station (run inside WSL), then open http://localhost:8765.
# Works without activating conda: uses the project's env Python directly.
cd "$(dirname "$0")/.."
export MUJOCO_GL=${MUJOCO_GL:-egl}
ENV_PY="$HOME/miniconda3/envs/drone311/bin/python"
if [ -n "$PYTHON" ]; then PY="$PYTHON"
elif [ -x "$ENV_PY" ]; then PY="$ENV_PY"
else PY="$(command -v python3 || command -v python)"
fi
exec "$PY" gcs/server.py "$@"
