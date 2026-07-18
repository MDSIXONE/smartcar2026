#!/usr/bin/env bash
# Stop only the real-vehicle yolo2025 2026.launch task when its launch
# terminal is no longer available.  It never starts or stops roscore.
set -euo pipefail

source /opt/ros/melodic/setup.bash
export ROS_MASTER_URI=http://192.168.8.197:11311
source "$HOME/ucar_ws/devel/setup.bash"
export ROS_MASTER_URI=http://192.168.8.197:11311
unset ROS_HOSTNAME
export ROS_IP=192.168.8.231

# Safety first: stop a still-running base driver before terminating the task.
python2 /opt/ros/melodic/bin/rostopic pub -1 /cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' || true

mapfile -t launch_pids < <(pgrep -f \
  'python2 /opt/ros/melodic/bin/roslaunch yolo2025 2026.launch' || true)

if ((${#launch_pids[@]} == 0)); then
  echo "No running yolo2025 2026.launch process found."
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

echo "yolo2025 2026.launch task stopped; ROS Master remains running."
