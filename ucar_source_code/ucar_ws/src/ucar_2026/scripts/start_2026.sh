#!/usr/bin/env bash
# Real-vehicle launcher.  The ROS Master is owned by this vehicle process and
# is stopped with its roslaunch child; the PC is only the HTTP simulation host.
set -eo pipefail

simulation_host="${1:-}"
launch_mode="${2:-manual}"

if [[ -z "$simulation_host" ]]; then
  read -r -p '请输入电脑仿真服务地址（例如 192.168.8.199）: ' simulation_host
fi
if [[ -z "$simulation_host" || "$simulation_host" == "127.0.0.1" ||
      "$simulation_host" == "localhost" ]]; then
  echo "错误：电脑仿真服务地址不能为空，也不能是 localhost/127.0.0.1。" >&2
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
  exit 4
fi

unset ROS_HOSTNAME
route_output="$(ip -4 route get "$simulation_host" 2>/dev/null || true)"
ROS_IP="$(awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }' \
  <<<"$route_output")"
if [[ -z "$ROS_IP" ]]; then
  echo "错误：无法从到电脑 $simulation_host 的路由获得小车 ROS_IP。" >&2
  exit 2
fi
export ROS_IP
export ROS_MASTER_URI="http://${ROS_IP}:11311"
export SIMULATION_HOST="$simulation_host"

printf '\n网络检查结果：\n  小车 ROS Master = %s\n  电脑仿真服务 = %s\n\n' \
  "$ROS_MASTER_URI" "$SIMULATION_HOST"

# 预检电脑仿真 bridge 的 TCP 11313 是否可达，避免任务中途才发现网络错误
if timeout 2 bash -c "exec 3<>/dev/tcp/${simulation_host}/11313" 2>/dev/null; then
  echo "电脑仿真服务 TCP 11313 可达。"
else
  if [[ "$launch_mode" == "mission" ]]; then
    echo "错误：电脑仿真服务 $simulation_host:11313 不可达。" >&2
    echo "  请确认：" >&2
    echo "  1. 电脑上仿真 bridge 已启动，终端显示 listening on 0.0.0.0:11313；" >&2
    echo "  2. 填的是电脑的 Windows 局域网 IP（在电脑上运行 ipconfig 查 Wi-Fi 适配器 IPv4，不是 WSL 里的 ip addr）；" >&2
    echo "  3. WSL2 为 mirrored 网络模式，或在 Windows 上对 11313 配置了 netsh portproxy 且防火墙放行（RemoteAddress LocalSubnet）。" >&2
    exit 2
  else
    echo "警告：电脑仿真服务 $simulation_host:11313 当前不可达；非任务模式继续启动。" >&2
  fi
fi

cleanup_orphan_roscore() {
  local orphan_pids
  local master_nodes
  local orphan_pid
  local still_running

  orphan_pids="$(ps -eo pid=,ppid=,args= | awk '
    $2 == 1 &&
    $0 ~ /\/opt\/ros\/melodic\/bin\/roscore([[:space:]]|$)/ {
      print $1
    }')"
  if [[ -z "$orphan_pids" ]]; then
    return 0
  fi

  if ! master_nodes="$(timeout 2 "$python2_runner" \
      /opt/ros/melodic/bin/rosnode list 2>/dev/null)"; then
    echo "错误：发现孤儿 ROS Master（PID: $(tr '\n' ' ' <<<"$orphan_pids")），" \
      "但无法读取节点列表；拒绝自动清理。" >&2
    exit 3
  fi
  master_nodes="$(sed '/^[[:space:]]*$/d' <<<"$master_nodes")"
  if [[ "$master_nodes" != "/rosout" ]]; then
    echo "错误：发现孤儿 ROS Master（PID: $(tr '\n' ' ' <<<"$orphan_pids")），" \
      "但其中存在活动节点：$master_nodes；拒绝自动清理。" >&2
    exit 3
  fi

  echo "发现仅剩 /rosout 的孤儿 ROS Master（PID: $(tr '\n' ' ' <<<"$orphan_pids")），发送 SIGINT 清理。"
  while read -r orphan_pid; do
    if [[ -n "$orphan_pid" ]] && kill -0 "$orphan_pid" 2>/dev/null; then
      kill -INT "$orphan_pid"
    fi
  done <<<"$orphan_pids"
  for attempt in $(seq 1 10); do
    still_running=0
    while read -r orphan_pid; do
      if [[ -n "$orphan_pid" ]] && kill -0 "$orphan_pid" 2>/dev/null; then
        still_running=1
        break
      fi
    done <<<"$orphan_pids"
    if [[ "$still_running" -eq 0 ]]; then
      echo "孤儿 ROS Master 已退出。"
      return 0
    fi
    sleep 1
  done
  echo "错误：孤儿 ROS Master 在 10 秒内未退出。" >&2
  exit 3
}
cleanup_orphan_roscore

if timeout 2 "$python2_runner" /opt/ros/melodic/bin/rosnode list \
  >/dev/null 2>&1; then
  echo "错误：$ROS_MASTER_URI 已有 ROS Master；本脚本只管理自己启动的 Master。" >&2
  exit 3
fi
if pgrep -f '[l]ane_follow.py' >/dev/null 2>&1; then
  echo "错误：检测到旧版独立 lane_follow 正在运行；必须先让它结束，才能启动主流程。" >&2
  exit 3
fi

roscore_pid=""
roslaunch_pid=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$roslaunch_pid" ]] && kill -0 "$roslaunch_pid" 2>/dev/null; then
    kill -INT "$roslaunch_pid" 2>/dev/null || true
    wait "$roslaunch_pid" 2>/dev/null || true
  fi
  if [[ -n "$roscore_pid" ]] && kill -0 "$roscore_pid" 2>/dev/null; then
    kill -INT "$roscore_pid" 2>/dev/null || true
    wait "$roscore_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

"$python2_runner" /opt/ros/melodic/bin/roscore \
  >"$HOME/.ros/ucar_2026_roscore.log" 2>&1 &
roscore_pid=$!
for attempt in $(seq 1 10); do
  if timeout 2 "$python2_runner" /opt/ros/melodic/bin/rosnode list \
    >/dev/null 2>&1; then
    break
  fi
  if [[ "$attempt" -eq 10 ]]; then
    echo "错误：小车本机 ROS Master 未在 10 秒内就绪。" >&2
    exit 3
  fi
  sleep 1
done

run_launch() {
  "$python2_runner" /opt/ros/melodic/bin/roslaunch "$@" &
  roslaunch_pid=$!
  wait "$roslaunch_pid"
}

case "$launch_mode" in
  check)
    echo "小车本机 ROS Master 已就绪；仅检查网络，不启动导航。"
    ;;
  manual)
    echo "正在启动无自动目标的导航任务（小车本机 ROS Master）……"
    run_launch ucar_2026 2026.launch
    ;;
  mission|mission_alt1|mission_alt2)
    echo "任务前确认：lidar_loc 定位初值固定为起点 (-0.25, 2.75, 0)，"
    echo "车辆不在起点时启动会产生错误的地图位姿。"
    read -r -p '是否已把车放回起点？输入 yes 继续，其他输入将取消启动: ' \
      confirm_start
    case "$confirm_start" in
      yes|Yes|YES|y|Y) ;;
      *)
        echo "已取消启动。请先把车物理放回起点 (-0.25, 2.75, 0)，再重新运行本脚本。"
        exit 2
        ;;
    esac
    launch_file="2026.launch"
    launch_note="省赛主流程"
    if [[ "$launch_mode" == "mission_alt1" ]]; then
      launch_file="2026_alt1.launch"
      launch_note="省赛备用方案一（超时连续对准版）"
    elif [[ "$launch_mode" == "mission_alt2" ]]; then
      launch_file="2026_alt2.launch"
      launch_note="省赛备用方案二（8 方位定点扫描版）"
    fi
    echo "正在启动自动生产任务（$launch_note）：ROS Master 在小车，电脑只提供仿真 HTTP 服务……"
    echo "任务节点启动后会等待语音指令：请说“小飞小飞”，再完整说出任务命令。"
    run_launch ucar_2026 "$launch_file" task_enabled:=true \
      simulation_host:="$SIMULATION_HOST"
    ;;
  full)
    echo "正在启动 legacy 完整自动任务（小车本机 ROS Master）……"
    run_launch yolo2025 2026.launch full_task_enabled:=true
    ;;
  *)
    echo "错误：模式只能是 check、manual、mission、mission_alt1 或 full，当前为 '$launch_mode'。" >&2
    exit 2
    ;;
esac
