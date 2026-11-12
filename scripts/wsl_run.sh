#!/bin/bash
# Helper: activate drone311 env and change to project directory
# Usage: source wsl_run.sh OR: bash wsl_run.sh <python_script>
source ~/miniconda3/bin/activate drone311
PROJ_WIN="C:/Users/Aryan's_WorkSpace/Desktop/Drone_7th_sem"
cd "/mnt/c/Users/Aryan's_WorkSpace/Desktop/Drone_7th_sem" 2>/dev/null || cd ~
if [ -n "$1" ]; then
    python "$@"
fi