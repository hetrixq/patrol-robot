#!/usr/bin/env bash
set -Eeuo pipefail
case "${1:-}" in
  start|stop|reset|loop|random|undo|clear)
    ros2 topic pub --once /robot_commands std_msgs/msg/String "{data: '$1'}" ;;
  status)
    ros2 topic echo /patrol/state --once --qos-durability transient_local --qos-reliability reliable ;;
  stats)
    ros2 topic echo /patrol/stats --once --qos-durability transient_local --qos-reliability reliable ;;
  events)
    ros2 topic echo /patrol/events ;;
  *) echo 'Usage: ./patrolctl.sh {start|stop|reset|loop|random|undo|clear|status|stats|events}' >&2; exit 2;;
esac
