#!/usr/bin/env bash
set -Eeuo pipefail
SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET="${1:-}"
if [[ -z "$TARGET" ]]; then
  echo "Использование: ./install.sh /путь/к/КОПИИ/robomarv_26" >&2
  echo 'Создайте отдельную копию репозитория, не применяйте поверх работающих файлов!' >&2
  exit 2
fi
if [[ ! -f "$TARGET/pack/scripts/route_manager.py" || ! -f "$TARGET/pack/scripts/leg_detector.py" || ! -f "$TARGET/robot_recognition/card_recognizer.py" ]]; then
  echo "Это не корень robomarv_26 (не найдены исходные скрипты): $TARGET" >&2
  exit 2
fi
for item in \
  pack/scripts/classroom_patrol.py \
  pack/start_classroom.sh \
  pack/patrolctl.sh \
  pack/preflight.sh \
  robot_recognition/qr_only.py \
  robot_recognition/sounds/patrol_start.wav \
  robot_recognition/sounds/patrol_success.wav \
  robot_recognition/sounds/patrol_siren.wav; do
  mkdir -p "$(dirname "$TARGET/$item")"
  if [[ -e "$TARGET/$item" ]]; then
    echo "ОШИБКА: $TARGET/$item уже есть. Ничего не перезаписываю." >&2
    exit 3
  fi
done
for item in \
  pack/scripts/classroom_patrol.py \
  pack/start_classroom.sh \
  pack/patrolctl.sh \
  pack/preflight.sh \
  robot_recognition/qr_only.py \
  robot_recognition/sounds/patrol_start.wav \
  robot_recognition/sounds/patrol_success.wav \
  robot_recognition/sounds/patrol_siren.wav; do
  cp -n "$SOURCE_DIR/$item" "$TARGET/$item"
done
chmod +x "$TARGET/pack/patrolctl.sh" "$TARGET/pack/preflight.sh" "$TARGET/pack/start_classroom.sh" "$TARGET/pack/scripts/classroom_patrol.py" "$TARGET/robot_recognition/qr_only.py"
echo "Готово. Исходные файлы не изменены: $TARGET"
echo "Внутри ROS2 Docker: cd '$TARGET/pack' && ./start_classroom.sh"
