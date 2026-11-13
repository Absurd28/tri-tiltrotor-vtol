#!/bin/bash
# Regenerate eval/rl_metrics.csv: 50 seeded episodes (seeds 10000-10049, with
# domain randomization) per task for the baseline and the trained policy.
set -e
cd "$(dirname "$0")/.."
rm -f eval/rl_metrics.csv
for spec in "hover s1_hover 0.3" "waypoint s2_waypoint 0.3" "obstacle s5_obstacle 0.4"; do
  set -- $spec
  python rl/evaluate.py --task $1 --baseline --dr -n 50 --difficulty $3 --label baseline
  # best.zip = checkpoint chosen on validation seeds 20000+ (rl/select_checkpoint.py)
  python rl/evaluate.py --task $1 --policy runs/$2/best.zip --dr -n 50 --difficulty $3 --label $2/best
  python rl/evaluate.py --task $1 --policy runs/$2/final.zip --dr -n 50 --difficulty $3 --label $2/final
done
