#!/usr/bin/env bash
# Простое управление патрулём из ROS2-контейнера (без запоминания YAML-команд).
set -Eeuo pipefail
COMMAND="${1:-}"
case "$COMMAND" in
  start|resume|stop|reset|clear|undo|loop|random) ;;
  status)
    exec timeout 8 ros2 topic echo --once --qos-durability transient_local /patrol/state
    ;;
  stats)
    exec timeout 8 ros2 topic echo --once --qos-durability transient_local /patrol/stats
    ;;
  events)
    exec ros2 topic echo /patrol/events
    ;;
  *)
    echo 'Использование: ./patrolctl.sh start|stop|reset|loop|random|undo|clear|status|stats|events' >&2
    exit 2
    ;;
esac
exec ros2 topic pub --once /robot_commands std_msgs/msg/String "{data: '$COMMAND'}"
