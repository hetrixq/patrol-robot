#!/usr/bin/env bash
set -Eeuo pipefail
PACK_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPTS="$PACK_DIR/scripts"

if ! command -v ros2 >/dev/null 2>&1; then
  echo 'ОШИБКА: нет команды ros2. Откройте shell внутри настроенного ROS2-контейнера.' >&2
  exit 1
fi
for file in "$SCRIPTS/route_manager.py" "$SCRIPTS/leg_detector.py" "$SCRIPTS/classroom_patrol.py" "$PACK_DIR/../robot_recognition/qr_only.py"; do
  if [[ ! -f "$file" ]]; then echo "Нет файла: $file" >&2; exit 1; fi
done
for node in route_manager classroom_patrol person_verifier foot_follower leg_detector; do
  if ros2 node list 2>/dev/null | grep -qx "/$node"; then
    echo "ОШИБКА: уже запущена ROS2-нода /$node. Остановите старый патруль, не запускайте два стека одновременно." >&2
    exit 1
  fi
done

# Оба процесса живут в одном ROS-окружении и видят RTSP так же, как старый проект.
python3 -u "$SCRIPTS/leg_detector.py" & DET_PID=$!
python3 -u "$SCRIPTS/classroom_patrol.py" & PATROL_PID=$!

cleanup() {
  trap - EXIT INT TERM
  echo 'Останавливаю маршрут и отменяю цель...'
  timeout 3 ros2 service call /goal/cancel slam_toolbox/srv/Reset '{}' >/dev/null 2>&1 || true
  kill "$DET_PID" "$PATROL_PID" 2>/dev/null || true
  wait "$DET_PID" "$PATROL_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo 'Патруль запущен. В Lichtblick отметьте 2–10 точек (Click Point -> /clicked_point).'
echo "Команда: ros2 topic pub --once /robot_commands std_msgs/msg/String \"{data: 'start'}\""
echo "Сброс:   ros2 topic pub --once /robot_commands std_msgs/msg/String \"{data: 'reset'}\""
wait -n "$DET_PID" "$PATROL_PID"
echo 'Один из процессов завершился. Останавливаю весь патруль.' >&2
exit 1
