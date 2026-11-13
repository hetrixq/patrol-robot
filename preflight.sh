#!/usr/bin/env bash
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
PY="${PATROL_PYTHON:-/root/venv/bin/python3}"
[[ -x "$PY" ]] || PY=python3
err=0
ok() { echo "[OK] $*"; }
warn() { echo "[WARN] $*"; }
fail() { echo "[FAIL] $*"; err=$((err+1)); }
command -v ros2 >/dev/null && ok 'ros2 found' || fail 'run inside ROS Docker'
[[ -d /root/weights/yolo11n_ncnn_model || -d "${PATROL_MODEL:-/root/weights/yolo11n_ncnn_model}" ]] && ok 'YOLO11n NCNN model directory' || fail 'YOLO model missing'
"$PY" - <<'PY' && echo '[OK] Python ROS2/Nav2/OpenCV/Ultralytics available' || { echo '[FAIL] Python dependencies'; err=$((err+1)); }
import rclpy, cv2, ultralytics
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import MarkerArray
assert hasattr(cv2, 'QRCodeDetector')
PY
if command -v ros2 >/dev/null; then
  ros2 action list -t 2>/dev/null | grep -Eq '/navigate_to_pose \[nav2_msgs/action/NavigateToPose\]' && ok 'Nav2 NavigateToPose action ready' || fail 'Nav2 action not visible'
  ros2 topic list -t 2>/dev/null | grep -Eq '/map \[nav_msgs/msg/OccupancyGrid\]' && ok '/map available' || warn '/map not visible yet: move/scan with SLAM'
  ros2 node list 2>/dev/null | grep -qx '/goal_proxy' && ok 'existing goal_proxy is running (v4 does not use its cancel service)' || warn 'goal_proxy not shown; v4 uses Nav2 action directly'
  if ros2 node list 2>/dev/null | grep -Eq '^/(route_manager|leg_detector|foot_follower|person_verifier)$'; then
    warn 'old patrol running; stop old start_classroom.sh before starting v4'
  fi
fi
command -v aplay >/dev/null && ok 'aplay installed' || warn 'aplay not installed: visual alarm only'
[[ -e /dev/snd/controlC0 ]] && ok 'audio device exposed in container' || warn '/dev/snd not visible'
echo "Result: $err errors"
exit "$err"
