#!/usr/bin/env bash
# Starts ONLY one patrol node. Existing RBM / ROS / SLAM / Nav2 untouched.
set -Eeuo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
if ! command -v ros2 >/dev/null 2>&1; then
  echo 'ERROR: run INSIDE the existing ROS Docker (docker exec -it ros bash)' >&2
  exit 2
fi
if ros2 node list 2>/dev/null | grep -Eq '^/(patrol_classroom_v5|patrol_classroom_v4|route_manager|leg_detector|foot_follower|person_verifier)$'; then
  echo 'ERROR: v3/v4/v5 patrol or legacy YOLO already running. STOP it first.' >&2
  exit 3
fi
PY="${PATROL_PYTHON:-/root/venv/bin/python3}"
[[ -x "$PY" ]] || PY=python3
export PATROL_QR_FPS="${PATROL_QR_FPS:-2}"
export PATROL_QR_SCAN_WIDTH="${PATROL_QR_SCAN_WIDTH:-960}"
export PATROL_CAMERA_VIEW_WIDTH="${PATROL_CAMERA_VIEW_WIDTH:-720}"
export PATROL_JPEG_QUALITY="${PATROL_JPEG_QUALITY:-60}"
export PATROL_RTSP_URL="${PATROL_RTSP_URL:-rtsp://localhost:8554/cam}"
export PATROL_ALSA_DEVICE="${PATROL_ALSA_DEVICE:-plughw:0,0}"
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
PID=''
cleanup() {
  trap - INT TERM EXIT
  echo 'Stopping manual QR patrol; requesting navigation cancellation...'
  timeout 4 ros2 topic pub --once /robot_commands std_msgs/msg/String "{data: 'stop'}" >/dev/null 2>&1 || true
  sleep 3
  if [[ -n "$PID" ]]; then
    kill -INT "$PID" 2>/dev/null || true
    wait "$PID" 2>/dev/null || true
  fi
  echo 'If the robot is still moving, use the PHYSICAL emergency stop.'
}
trap cleanup INT TERM EXIT
cd "$HERE"
"$PY" -u patrol_node.py & PID=$!
echo 'PATROL v5 started: NO YOLO. Use CHECK / CAMERA ON in Lichtblick.'
wait "$PID"
