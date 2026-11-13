#!/usr/bin/env bash
set -Eeuo pipefail
case "${1:-}" in
  start|stop|reset|check|loop|random|undo|clear|camera_on|camera_off)
    ros2 topic pub --once /robot_commands std_msgs/msg/String "{data: '$1'}" ;;
  status)
    ros2 topic echo /patrol/state --once --qos-durability transient_local --qos-reliability reliable ;;
  qr)
    ros2 topic echo /patrol/qr/status --once --qos-durability transient_local --qos-reliability reliable ;;
  stats)
    ros2 topic echo /patrol/stats --once --qos-durability transient_local --qos-reliability reliable ;;
  events) ros2 topic echo /patrol/events ;;
  *) echo 'Usage: ./patrolctl.sh {check|camera_on|camera_off|start|stop|reset|loop|random|undo|clear|status|qr|stats|events}' >&2; exit 2 ;;
esac
