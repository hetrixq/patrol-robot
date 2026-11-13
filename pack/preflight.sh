#!/usr/bin/env bash
# Запустить внутри исходного ROS2-контейнера. Никаких установок не делает.
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
ERRORS=0
WARNINGS=0
ok()   { printf '  [OK]   %s\n' "$*"; }
err()  { printf '  [FAIL] %s\n' "$*"; ERRORS=$((ERRORS+1)); }
warn() { printf '  [WARN] %s\n' "$*"; WARNINGS=$((WARNINGS+1)); }
check_file() { if [[ -f "$1" ]]; then ok "Файл $1"; else err "Нет $1"; fi; }
echo '=== PATROL PRE-FLIGHT: ничего не меняет ==='
if command -v ros2 >/dev/null 2>&1; then ok 'ros2 найден'; else err 'Нет ros2 в PATH'; fi
check_file "$HERE/scripts/route_manager.py"
check_file "$HERE/scripts/leg_detector.py"
check_file "$HERE/scripts/classroom_patrol.py"
check_file "$ROOT/robot_recognition/qr_only.py"
if python3 -c 'import rclpy, cv2, ultralytics; from visualization_msgs.msg import MarkerArray' >/dev/null 2>&1; then
  ok 'Python: ROS2 + OpenCV + Ultralytics + MarkerArray'
else
  err 'Ошибка импортов ROS2/OpenCV/Ultralytics/MarkerArray: проверьте базовый контейнер'
fi
qr_python="${PATROL_QR_PYTHON:-$ROOT/robot_recognition/venv/bin/python3}"
if [[ ! -x "$qr_python" ]]; then qr_python="$(command -v python3)"; warn "venv QR не найден, проверяю $qr_python"; fi
if "$qr_python" -c 'import cv2; from pyzbar import pyzbar' >/dev/null 2>&1; then
  ok "Python QR + ZBar: $qr_python"
else
  err "QR Python не может импортировать cv2/pyzbar/ZBar: $qr_python"
fi
if command -v aplay >/dev/null 2>&1; then ok 'aplay имеется (наличие устройства не проверено)'; else warn 'aplay не найден — тревога только в Lichtblick'; fi
if command -v ros2 >/dev/null 2>&1; then
  stype="$(ros2 service type /goal/cancel 2>/dev/null || true)"
  if [[ "$stype" == *'slam_toolbox/srv/Reset'* ]]; then ok "/goal/cancel: $stype"; else err "/goal/cancel недоступен или тип неожиданен: $stype"; fi
  for spec in '/goal|geometry_msgs/msg/PoseStamped' '/goal/status|std_msgs/msg/String' '/leg_detection/status|std_msgs/msg/Bool'; do
    topic="${spec%%|*}"; expected="${spec#*|}"
    found="$(ros2 topic type "$topic" 2>/dev/null || true)"
    if [[ "$found" == *"$expected"* ]]; then ok "$topic: $expected"; else warn "$topic пока не виден или другого типа ($found). Это нормально, если нода не запущена"; fi
  done
fi
printf '\nРезультат: %s ошибок, %s предупреждений.\n' "$ERRORS" "$WARNINGS"
if (( ERRORS > 0 )); then exit 1; fi
