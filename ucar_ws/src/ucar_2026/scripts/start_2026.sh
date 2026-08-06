#!/usr/bin/env bash
# Beginner-safe real-vehicle launcher. The vehicle always connects to the WSL
# ROS Master supplied by the user; this script never starts a local roscore.
set -eo pipefail

requested_master_ip="${1:-}"
launch_mode="${2:-manual}"

if [[ -z "$requested_master_ip" ]]; then
  read -r -p '请输入 WSL Master 地址（例如 192.168.8.199）: ' requested_master_ip
fi

if [[ -z "$requested_master_ip" || "$requested_master_ip" == "127.0.0.1" ||
      "$requested_master_ip" == "localhost" ]]; then
  echo "错误：WSL Master 地址不能为空，也不能是 localhost/127.0.0.1。" >&2
  exit 2
fi

source /opt/ros/melodic/setup.bash
source "$HOME/ucar_ws/devel/setup.bash"
set -u

python2_runner="$HOME/ucar_ws/src/ucar_2026/scripts/run_melodic_python2.sh"
if [[ ! -x "$python2_runner" ]]; then
  echo "错误：找不到 Melodic Python 2 启动器：$python2_runner" >&2
  exit 4
fi
if ! "$python2_runner" -c \
  'import rospy; import tf; from sensor_msgs.msg import LaserScan' \
  >/dev/null 2>&1; then
  echo "错误：ROS Melodic 的 Python 2 环境异常，已停止启动。" >&2
  echo "请按 docs/operations.md 的“Python 2 环境重建”处理。" >&2
  exit 4
fi

unset ROS_HOSTNAME
export MASTER_IP="$requested_master_ip"
export ROS_MASTER_URI="http://${MASTER_IP}:11311"

route_output="$(ip -4 route get "$MASTER_IP" 2>/dev/null || true)"
ROS_IP="$(awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }' \
  <<<"$route_output")"
if [[ -z "$ROS_IP" ]]; then
  echo "错误：无法自动获得小车 IP。请确认小车和电脑连接同一个网络。" >&2
  exit 2
fi
export ROS_IP

printf '\n网络检查结果：\n  WSL Master = %s\n  小车 ROS_IP = %s\n\n' \
  "$ROS_MASTER_URI" "$ROS_IP"

master_ready=false
for attempt in $(seq 1 10); do
  if timeout 2 python2 /opt/ros/melodic/bin/rosnode list >/dev/null 2>&1; then
    master_ready=true
    break
  fi
  if [[ "$attempt" -eq 1 ]]; then
    echo "WSL Master 尚未就绪，正在自动等待（最多约 30 秒）……"
  fi
  sleep 1
done
if [[ "$master_ready" != true ]]; then
  echo "错误：小车无法连接 WSL Master。" >&2
  echo "请确认 WSL Master 显示的不是 localhost/127.0.0.1，并检查两台设备网络。" >&2
  exit 3
fi

case "$launch_mode" in
  check)
    echo "Master 连接成功。仅检查网络，不启动导航。"
    exit 0
    ;;
  manual)
    echo "Master 连接成功。正在启动无自动目标的导航任务……"
    exec "$python2_runner" /opt/ros/melodic/bin/roslaunch \
      ucar_2026 2026.launch
    ;;
  mission)
    echo "任务前确认：lidar_loc 定位初值固定为起点 (-0.25, 2.75, 0)，"
    echo "车辆不在起点时启动会产生错误的地图位姿。"
    read -r -p '是否已把车放回起点？输入 yes 继续，其他输入将取消启动: ' \
      confirm_start
    case "$confirm_start" in
      yes|Yes|YES|y|Y)
        ;;
      *)
        echo "已取消启动。请先把车物理放回起点 (-0.25, 2.75, 0)，再重新运行本脚本。"
        exit 2
        ;;
    esac
    echo "Master 连接成功。正在启动 ucar_2026 自动生产任务（无 RViz）……"
    exec "$python2_runner" /opt/ros/melodic/bin/roslaunch \
      ucar_2026 2026.launch task_enabled:=true
    ;;
  full)
    echo "Master 连接成功。正在启动完整自动任务……"
    exec "$python2_runner" /opt/ros/melodic/bin/roslaunch \
      yolo2025 2026.launch full_task_enabled:=true
    ;;
  *)
    echo "错误：模式只能是 check、manual、mission 或 full，当前为 '$launch_mode'。" >&2
    exit 2
    ;;
esac
