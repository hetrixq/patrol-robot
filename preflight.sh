#!/usr/bin/env bash
set -u
PY="${PATROL_PYTHON:-/root/venv/bin/python3}"
[[ -x "$PY" ]] || PY=python3
err=0
ok() { echo "[OK] $*"; }
warn() { echo "[WARN] $*"; }
fail() { echo "[FAIL] $*"; err=$((err+1)); }
command -v ros2 >/dev/null 2>&1 && ok 'ROS2 CLI' || fail 'Run inside ros Docker'
"$PY" - <<'PY' && ok 'Python: ROS2/Nav2/OpenCV/CompressedImage + QRCodeDetector' || { fail 'Python dependencies'; }
import rclpy, cv2
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import CompressedImage
from visualization_msgs.msg import MarkerArray
assert hasattr(cv2, 'QRCodeDetector')
PY
if command -v ros2 >/dev/null 2>&1; then
  ros2 action list -t 2>/dev/null | grep -Eq '/navigate_to_pose \[nav2_msgs/action/NavigateToPose\]' && ok 'Existing Nav2 action ready' || fail 'Nav2 action unavailable'
  ros2 topic list -t 2>/dev/null | grep -Eq '/map \[nav_msgs/msg/OccupancyGrid\]' && ok 'SLAM /map exists' || warn 'SLAM map not yet present'
  if ros2 node list 2>/dev/null | grep -Eq '^/(route_manager|leg_detector|patrol_classroom_v4|patrol_classroom_v5)$'; then
    warn 'Old/duplicate patrol node detected, stop it before starting v5'
  fi
fi
command -v aplay >/dev/null && ok 'ALSA aplay' || warn 'aplay absent: no sound'
[[ -e /dev/snd/controlC0 ]] && ok 'Audio device accessible' || warn 'No /dev/snd/controlC0'
[[ -f "$(dirname "$0")/tools/PERSON_001.png" ]] && ok 'Demo QR file present' || fail 'Missing QR image'
echo "Result: $err errors; YOLO is neither required nor imported"
exit "$err"
