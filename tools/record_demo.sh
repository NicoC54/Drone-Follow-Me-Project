#!/usr/bin/env bash
# Records a demo run as an MCAP file (opens in Foxglove with docs/foxglove/follow_me_layout.json).
#
#   terminal 1:  ros2 launch follow_me_gazebo intercept.launch.py        (or follow_me.launch.py)
#   terminal 2:  tools/record_demo.sh [--with-images] [output_dir]
#   Ctrl+C in terminal 2 to stop, BEFORE stopping the simulation (with sim time, the recorder
#   waits for /clock and hangs if Gazebo is gone).
#
# Without images: markers, TF, estimates, metrics (a few MB per minute, fine for a GitHub release).
# --with-images adds the camera HUD (/follower/hud_image, ~20 MB/s raw, zstd-compressed).
set -euo pipefail
WITH_IMAGES=0
if [[ "${1:-}" == "--with-images" ]]; then WITH_IMAGES=1; shift; fi
OUT="${1:-out/demo_$(date +%Y%m%d_%H%M%S)}"
TOPICS=(
  /clock /tf
  /radar/scan /radar/markers
  /target/estimate /tracker/markers /follower/detection /follower/detector/roi
  /mission/phase /follow/markers /follower/fc/accel_yaw_setpoint
  /follower/odom /follower/odom_truth /leader/odom_truth /follower/fc/status
  /monitor/markers /monitor/distance /monitor/estimation_error /monitor/nees /monitor/navigation_error
)
if [[ $WITH_IMAGES == 1 ]]; then TOPICS+=(/follower/hud_image); fi
mkdir -p "$(dirname "$OUT")"
echo "Recording ${#TOPICS[@]} topics to $OUT (MCAP, zstd). Ctrl+C to stop."
exec ros2 bag record -s mcap --storage-preset-profile zstd_fast --use-sim-time -o "$OUT" "${TOPICS[@]}"
