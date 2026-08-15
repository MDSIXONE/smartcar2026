#!/usr/bin/env bash
# Stop the locally supervised real-vehicle launch.  start_2026.sh receives
# roslaunch exit and then stops the ROS Master it owns.
set -eo pipefail

source /opt/ros/melodic/setup.bash
source "$HOME/ucar_ws/devel/setup.bash"
set -u

mapfile -t launch_pids < <(pgrep -f \
  'roslaunch[[:space:]]+(ucar_2026_extra|yolo2025)[[:space:]]+2026\.launch' || true)

if ((${#launch_pids[@]} == 0)); then
  echo "没有运行中的 ucar_2026_extra/yolo2025 2026.launch。"
  exit 0
fi

for launch_pid in "${launch_pids[@]}"; do
  kill -INT "$launch_pid"
done

echo "已请求停止 roslaunch；start_2026.sh 将同步停止它启动的小车 ROS Master。"
