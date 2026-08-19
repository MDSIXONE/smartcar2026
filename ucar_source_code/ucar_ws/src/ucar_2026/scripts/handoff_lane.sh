#!/usr/bin/env bash
# Wait for the 2026 production launch to fully exit, then start the
# lane-follow launch.  lane_proto owns the chassis serial port and the
# camera, so it must never run while 2026.launch is still alive.
#
# Usage: handoff_lane.sh [MASTER_IP]
#   MASTER_IP falls back to $MASTER_IP, then to the host parsed from
#   $ROS_MASTER_URI.
set -u

requested_master_ip="${1:-${MASTER_IP:-}}"
if [[ -z "$requested_master_ip" ]]; then
  master_uri="${ROS_MASTER_URI:-}"
  requested_master_ip="${master_uri#http://}"
  requested_master_ip="${requested_master_ip%%:*}"
fi
if [[ -z "$requested_master_ip" || "$requested_master_ip" == "127.0.0.1" ||
      "$requested_master_ip" == "localhost" ]]; then
  echo "HANDOFF_ERROR cannot determine WSL Master address" >&2
  exit 2
fi

source /opt/ros/melodic/setup.bash
source "$HOME/ucar_ws/devel/setup.bash"

# Wait for 2026.launch to exit.  production_task_2026 is a required=true
# node, so when the mission finishes the whole launch dies and releases the
# chassis serial port and the camera.
# 2026-08-11：轮询粒度 1s→0.2s，缩短终点→巡线交接延迟（方案3）。
for attempt in $(seq 1 150); do
  if ! pgrep -f "roslaunch ucar_2026 2026.launch" >/dev/null 2>&1; then
    echo "HANDOFF_WAIT_2026_EXIT done after ${attempt}x0.2s"
    break
  fi
  if [[ "$attempt" -eq 150 ]]; then
    echo "HANDOFF_ERROR 2026.launch still running after 30s" >&2
    exit 3
  fi
  sleep 0.2
done

# Wait for the chassis serial port to be openable (driver released it).
serial_ready=false
for attempt in $(seq 1 50); do
  if [[ -e /dev/ttyUSB0 ]] && exec 3<>/dev/ttyUSB0 2>/dev/null; then
    exec 3>&-
    serial_ready=true
    echo "HANDOFF_SERIAL_READY after ${attempt}x0.2s"
    break
  fi
  sleep 0.2
done
if [[ "$serial_ready" != true ]]; then
  echo "HANDOFF_ERROR chassis serial port did not become available" >&2
  exit 4
fi
if [[ ! -e /dev/ucar_camera ]]; then
  echo "HANDOFF_WARNING camera /dev/ucar_camera not present" >&2
fi

unset ROS_HOSTNAME
export ROS_MASTER_URI="http://${requested_master_ip}:11311"
route_output="$(ip -4 route get "$requested_master_ip" 2>/dev/null || true)"
ROS_IP="$(awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }' \
  <<<"$route_output")"
if [[ -z "$ROS_IP" ]]; then
  echo "HANDOFF_ERROR cannot derive vehicle IP from route to $requested_master_ip" >&2
  exit 2
fi
export ROS_IP

lane_template="$(rospack find lane_proto)/config/red_template_band.png"
echo "HANDOFF_LANE_STARTED master=$ROS_MASTER_URI ip=$ROS_IP template=$lane_template"
roslaunch --screen lane_proto lane_proto.launch dry_run:=false take_cam_on_start:=true \
  linear_speed:=0.2 gain:=1.2 \
  "template:=$lane_template" is_fork:=yolo yellow_target:=0.90 \
  align_offset:=0.14 start_offset:=0.23 goal_y_lo:=0.75 rate:=20 dump_every:=5 \
  goal_pause:=1.0 goal_half:=40 use_lidar:=self goal_mode:=visual \
  board_in_lane:=false go_around:=false board_stop_dist:=0.321 \
  go_around_keepout:=0.08 board_arc_lat_scale:=0.3
lane_status=$?
if [[ "$lane_status" -ne 0 ]]; then
  echo "HANDOFF_LANE_FAILED exit=$lane_status" >&2
fi
exit "$lane_status"
