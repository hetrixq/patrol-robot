#!/usr/bin/env bash
# Existing RBM Docker only. Does not touch the running ROS/SLAM/Nav2 stack.
set -Eeuo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
if ! command -v ros2 >/dev/null 2>&1; then
  echo 'ERROR: run this INSIDE existing ros Docker container (docker exec -it ros bash)' >&2
  exit 2
fi
if ros2 node list 2>/dev/null | grep -Eq '^/(route_manager|leg_detector|patrol_classroom_v4|foot_follower|person_verifier)$'; then
  echo 'ERROR: old patrol/YOLO node or another v4 instance is already running.' >&2
  echo 'Stop previous start_classroom.sh (Ctrl+C) before starting v4.' >&2
  exit 3
fi
PY="${PATROL_PYTHON:-/root/venv/bin/python3}"
if [[ ! -x "$PY" ]]; then PY=python3; fi
MODEL="${PATROL_MODEL:-/root/weights/yolo11n_ncnn_model}"
if [[ ! -d "$MODEL" ]]; then
  echo "ERROR: NCNN model not found: $MODEL" >&2
  exit 4
fi
export PATROL_MODEL="$MODEL"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-2}"
export PATROL_YOLO_FPS="${PATROL_YOLO_FPS:-1.5}"
export PATROL_QR_FPS="${PATROL_QR_FPS:-3}"
export PATROL_IMGSZ="${PATROL_IMGSZ:-320}"
export PATROL_RTSP_URL="${PATROL_RTSP_URL:-rtsp://localhost:8554/cam}"
export PATROL_ALSA_DEVICE="${PATROL_ALSA_DEVICE:-plughw:0,0}"
PID=''
cleanup() {
  trap - INT TERM EXIT
  echo 'Stopping patrol v4; sending STOP to cancel Nav2 goal...'
  timeout 4 ros2 topic pub --once /robot_commands std_msgs/msg/String "{data: 'stop'}" >/dev/null 2>&1 || true
  sleep 2
  if [[ -n "$PID" ]]; then
    kill -INT "$PID" 2>/dev/null || true
    wait "$PID" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT
cd "$HERE"
"$PY" -u patrol_node.py & PID=$!
echo 'PATROL v4 started. Existing RBM pipeline untouched.'
echo 'Click Point on /clicked_point, then use /robot_commands START.'
wait "$PID"
