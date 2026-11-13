#!/bin/bash
# Regenerate eval/rl_metrics.csv (report §5 table): 50 TEST episodes per row,
# seeds 10000-10049, domain randomisation on. best.zip = checkpoint chosen on
# VALIDATION seeds 20000+ by rl/select_checkpoint.py (D-17).
set -e
cd "$(dirname "$0")/.."
E="python rl/evaluate.py --dr -n 50"
rm -f eval/rl_metrics.csv
$E --task hover --baseline --label baseline
$E --task hover --policy runs/s1_hover/best.zip --label s1_hover/best
$E --task hover --policy runs/s1_hover/final.zip --label s1_hover/final
$E --task waypoint --baseline --label baseline
$E --task waypoint --policy runs/s2_waypoint/best.zip --label s2_waypoint/best
$E --task waypoint --policy runs/s2_waypoint/final.zip --label s2_waypoint/final
$E --task obstacle --difficulty 0.4 --mode G --baseline --label baseline_guidance
$E --task obstacle --difficulty 0.4 --baseline --label baseline
$E --task obstacle --difficulty 0.4 --mode G --policy runs/s5_obstacle_G/best.zip --label s5_obstacle_G/best
$E --task obstacle --difficulty 0.4 --policy runs/s5_obstacle/best.zip --label s5_obstacle_B/best
$E --task obstacle --difficulty 0.4 --mode G --policy runs/s5_obstacle_G/ckpt_999960_steps.zip --label s5_obstacle_G/last
