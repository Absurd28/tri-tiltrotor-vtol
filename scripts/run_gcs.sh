#!/bin/bash
# Start the VTOL ground station (run inside WSL, conda env drone311).
# Open http://localhost:8765 in a browser afterwards.
cd "$(dirname "$0")/.."
export MUJOCO_GL=${MUJOCO_GL:-egl}
PY=${PYTHON:-python}
exec "$PY" gcs/server.py "$@"
