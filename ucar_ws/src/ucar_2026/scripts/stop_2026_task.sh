#!/usr/bin/env bash
# Stop the real-vehicle ucar_2026 launch task, including its legacy yolo2025
# wrapper.  This script never starts or stops roscore.
set -eo pipefail

configured_master_uri="${ROS_MASTER_URI:-}"
configured_master_ip="${MASTER_IP:-}"

source /opt/ros/melodic/setup.bash
source "$HOME/ucar_ws/devel/setup.bash"
set -u

unset ROS_HOSTNAME

if [[ -n "$configured_master_uri" ]]; then
  export ROS_MASTER_URI="$configured_master_uri"
  master_host="${ROS_MASTER_URI#*://}"
  master_host="${master_host%%/*}"
  master_host="${master_host%%:*}"
  if [[ -z "$master_host" ]]; then
    echo "ERROR: cannot extract the Master host from ROS_MASTER_URI=$ROS_MASTER_URI" >&2
    exit 2
  fi
  if [[ "$master_host" =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    MASTER_IP="$master_host"
  else
    MASTER_IP="$(getent ahostsv4 "$master_host" 2>/dev/null | awk 'NR == 1 { print $1 }' || true)"
    if [[ -z "$MASTER_IP" ]]; then
      echo "ERROR: cannot resolve ROS Master host '$master_host' to IPv4." >&2
      exit 2
    fi
  fi
elif [[ -n "$configured_master_ip" ]]; then
  MASTER_IP="$configured_master_ip"
  export ROS_MASTER_URI="http://${MASTER_IP}:11311"
else
  echo "ERROR: set ROS_MASTER_URI or MASTER_IP before running this script." >&2
  echo "The vehicle must use the WSL ROS Master; this script will not start roscore." >&2
  exit 2
fi
export MASTER_IP

ROS_IP="$(ip -4 route get "$MASTER_IP" 2>/dev/null | awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }' || true)"
if [[ -z "$ROS_IP" ]]; then
  echo "ERROR: cannot determine the vehicle ROS_IP for Master $MASTER_IP." >&2
  exit 2
fi
export ROS_IP

printf 'ROS_IP=%s\nROS_MASTER_URI=%s\n' "$ROS_IP" "$ROS_MASTER_URI"

# Safety first: stop a still-running base driver before terminating the task.
python2 /opt/ros/melodic/bin/rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' || true

mapfile -t launch_pids < <(pgrep -f \
  'roslaunch[[:space:]]+(ucar_2026|yolo2025)[[:space:]]+2026\.launch' || true)

if ((${#launch_pids[@]} == 0)); then
  echo "No running ucar_2026 2026.launch task or yolo2025 wrapper found."
  exit 0
fi

for launch_pid in "${launch_pids[@]}"; do
  # Stop the supervisor first, so respawn:=true cannot create a new child while
  # the remaining task children are being terminated.
  kill -STOP "$launch_pid" 2>/dev/null || true
  child_pids="$(pgrep -P "$launch_pid" || true)"

  if [[ -n "$child_pids" ]]; then
    kill -TERM $child_pids 2>/dev/null || true
    sleep 3
    for child_pid in $child_pids; do
      if kill -0 "$child_pid" 2>/dev/null; then
        kill -KILL "$child_pid" 2>/dev/null || true
      fi
    done
  fi

  # The parent was deliberately stopped above, so SIGKILL is needed to remove
  # the now-childless supervisor without affecting the ROS Master.
  kill -KILL "$launch_pid" 2>/dev/null || true
done

echo "ucar_2026 2026.launch task stopped; ROS Master remains running."
