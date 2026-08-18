#!/usr/bin/env bash
# Start the isolated simulation ROS Master, Gazebo/RViz and HTTP bridge.
# Run this script in WSL from the cloned simulation workspace.

set -Ee -o pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SIM_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

GUI=true
RVIZ=true
MASTER_PORT=11312
BRIDGE_PORT=11313
READY_TIMEOUT=300

usage() {
  cat <<'EOF'
用法：
  bash scripts/start_simulation_stack.sh [选项]

默认启动 Gazebo + RViz；按 Ctrl-C 会依次停止 bridge、仿真和仿真 roscore。

选项：
  --headless             无界面启动 Gazebo，不启动 RViz
  --no-rviz              启动 Gazebo GUI，但不启动 RViz
  --master-port PORT     仿真 ROS Master 端口，默认 11312
  --bridge-port PORT     HTTP bridge 端口，默认 11313
  --ready-timeout SEC    等待 /map 的最长秒数，默认 300
  -h, --help             显示帮助
EOF
}

die() {
  echo "错误：$*" >&2
  exit 1
}

while (($# > 0)); do
  case "$1" in
    --headless)
      GUI=false
      RVIZ=false
      ;;
    --no-rviz)
      RVIZ=false
      ;;
    --master-port)
      (($# >= 2)) || die "--master-port 缺少端口"
      MASTER_PORT="$2"
      shift
      ;;
    --bridge-port)
      (($# >= 2)) || die "--bridge-port 缺少端口"
      BRIDGE_PORT="$2"
      shift
      ;;
    --ready-timeout)
      (($# >= 2)) || die "--ready-timeout 缺少秒数"
      READY_TIMEOUT="$2"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      die "未知选项：$1"
      ;;
  esac
  shift
done

[[ -f /opt/ros/noetic/setup.bash ]] || die "找不到 ROS Noetic：/opt/ros/noetic/setup.bash"
[[ -f "$SIM_ROOT/devel/setup.bash" ]] || die "找不到仿真工作区：$SIM_ROOT/devel/setup.bash，请先 catkin_make"
[[ -f "$SIM_ROOT/bridge/sim_bridge.py" ]] || die "找不到 bridge：$SIM_ROOT/bridge/sim_bridge.py"
command -v curl >/dev/null 2>&1 || die "找不到 curl，无法确认 bridge 的 HTTP 就绪状态"
command -v ss >/dev/null 2>&1 || die "找不到 ss，无法确认 HTTP bridge 端口所有权"

# ROS setup scripts may reference unset variables, so source them before
# enabling the environment variables used by this launcher.
source /opt/ros/noetic/setup.bash
source "$SIM_ROOT/devel/setup.bash"
unset ROS_HOSTNAME
export ROS_MASTER_URI="http://127.0.0.1:${MASTER_PORT}"
export ROS_IP=127.0.0.1

MASTER_PID=""
PREPARE_PID=""
BRIDGE_PID=""

process_alive() {
  local pid="$1"
  local state

  state="$(ps -o stat= -p "$pid" 2>/dev/null | tr -d '[:space:]')"
  [[ -n "$state" && "$state" != Z* ]]
}

bridge_port_owners() {
  local listeners
  local owners

  listeners="$(ss -H -ltnp "sport = :${BRIDGE_PORT}" 2>/dev/null)"
  [[ -n "$listeners" ]] || return 1
  owners="$(printf '%s\n' "$listeners" | grep -oE 'pid=[0-9]+' | cut -d= -f2 | sort -u)"
  [[ -n "$owners" ]] || die "无法识别 ${BRIDGE_PORT} 的监听进程；请手工检查 ss -ltnp"
  printf '%s\n' "$owners"
}

bridge_port_owned_by_child() {
  local owners

  owners="$(bridge_port_owners)" || return 1
  grep -Fxq "$BRIDGE_PID" <<<"$owners"
}

stop_process() {
  local name="$1"
  local pid="$2"
  local signal=INT
  local deadline

  [[ -n "$pid" ]] || return 0
  if process_alive "$pid"; then
    echo "停止 ${name}（PID ${pid}）"
    kill -INT "$pid" 2>/dev/null || :
    deadline=$((SECONDS + 5))
    while process_alive "$pid"; do
      if ((SECONDS >= deadline)); then
        if [[ "$signal" == INT ]]; then
          signal=TERM
          echo "${name} 未响应 SIGINT，发送 SIGTERM（PID ${pid}）"
          kill -TERM "$pid" 2>/dev/null || :
          deadline=$((SECONDS + 5))
        else
          echo "${name} 未响应 SIGTERM，发送 SIGKILL（PID ${pid}）"
          kill -KILL "$pid" 2>/dev/null || :
          break
        fi
      fi
      sleep 0.2
    done
  fi
  if ! wait "$pid" 2>/dev/null; then
    :
  fi
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  stop_process "HTTP bridge" "$BRIDGE_PID"
  stop_process "task3_prepare.launch" "$PREPARE_PID"
  stop_process "仿真 roscore" "$MASTER_PID"
  exit "$exit_code"
}

trap cleanup EXIT INT TERM

check_copy_mode() {
  [[ "$GUI" == true ]] || return 0

  local weston_log=/mnt/wslg/weston.log
  local shared_fs
  local gfxredir_count=0

  [[ -r "$weston_log" ]] || die "找不到 WSLg 日志 $weston_log，不能启动 GUI 仿真"
  if grep -q 'use_gfxredir = 0' "$weston_log"; then
    gfxredir_count="$(grep -c 'use_gfxredir = 0' "$weston_log")"
  fi
  if ! shared_fs="$(findmnt -no FSTYPE /mnt/shared_memory)"; then
    die "WSLg /mnt/shared_memory 未挂载，不能启动 GUI 仿真"
  fi
  [[ "$gfxredir_count" == 0 && "$shared_fs" == tmpfs ]] || die \
    "WSLg COPY MODE 预检失败：use_gfxredir = 0 次数=${gfxredir_count}，/mnt/shared_memory=${shared_fs}；请按操作文档执行 wsl --terminate 或 wsl --shutdown 后复检"
}

wait_for_master() {
  local deadline=$((SECONDS + 60))
  echo "等待仿真 ROS Master ${ROS_MASTER_URI} 就绪……"
  while ((SECONDS < deadline)); do
    if ! kill -0 "$MASTER_PID" 2>/dev/null; then
      wait "$MASTER_PID" 2>/dev/null || :
      die "仿真 roscore 已退出，无法继续"
    fi
    if rosnode list >/dev/null 2>&1; then
      echo "仿真 ROS Master 已就绪"
      return 0
    fi
    sleep 1
  done
  die "等待仿真 ROS Master 超时"
}

wait_for_map() {
  local deadline=$((SECONDS + READY_TIMEOUT))
  echo "等待仿真 /map 就绪（最长 ${READY_TIMEOUT}s）……"
  while ((SECONDS < deadline)); do
    if ! kill -0 "$PREPARE_PID" 2>/dev/null; then
      wait "$PREPARE_PID" 2>/dev/null || :
      die "task3_prepare.launch 已退出，无法继续"
    fi
    if timeout 5 rostopic echo -n 1 /map >/dev/null 2>&1; then
      echo "仿真 /map 已就绪"
      return 0
    fi
    sleep 2
  done
  die "等待仿真 /map 超时"
}

wait_for_bridge() {
  local deadline=$((SECONDS + READY_TIMEOUT))
  local owners
  local response

  echo "等待 HTTP bridge state=waiting（最长 ${READY_TIMEOUT}s）……"
  while ((SECONDS < deadline)); do
    if ! process_alive "$BRIDGE_PID"; then
      wait "$BRIDGE_PID" 2>/dev/null || :
      die "HTTP bridge 已退出，无法继续"
    fi

    if ! bridge_port_owned_by_child; then
      if owners="$(bridge_port_owners)"; then
        die "HTTP bridge 子进程 PID ${BRIDGE_PID} 未拥有端口 ${BRIDGE_PORT}；当前监听 PID：${owners//$'\n'/、}"
      fi
      sleep 1
      continue
    fi

    if response="$(curl -fsS --max-time 5 "http://127.0.0.1:${BRIDGE_PORT}/status")" \
      && grep -q '"state"[[:space:]]*:[[:space:]]*"waiting"' <<<"$response" \
      && process_alive "$BRIDGE_PID" \
      && bridge_port_owned_by_child; then
      echo "HTTP bridge 已就绪：state=waiting"
      return 0
    fi
    sleep 1
  done
  die "等待 HTTP bridge 就绪超时"
}

cd "$SIM_ROOT"

if owners="$(bridge_port_owners)"; then
  owners_for_command="${owners//$'\n'/ }"
  echo "错误：HTTP bridge 端口 ${BRIDGE_PORT} 已被 PID ${owners_for_command} 占用。" >&2
  echo "先核对进程：ps -o pid=,ppid=,stat=,args= -p ${owners_for_command}" >&2
  echo "确认是本项目 sim_bridge.py 后，直接终止：kill -TERM ${owners_for_command}" >&2
  echo "若仍未退出，再执行：kill -KILL ${owners_for_command}" >&2
  exit 1
fi

if [[ "$GUI" == true ]]; then
  echo "执行 GUI 启动前 WSLg COPY MODE 预检……"
  check_copy_mode
fi

echo "启动仿真 roscore -p ${MASTER_PORT}……"
roscore -p "$MASTER_PORT" &
MASTER_PID=$!
wait_for_master

echo "启动 task3_prepare.launch（gui:=${GUI}, rviz:=${RVIZ}）……"
roslaunch car3 task3_prepare.launch "gui:=${GUI}" "rviz:=${RVIZ}" &
PREPARE_PID=$!
wait_for_map

if [[ "$GUI" == true ]]; then
  echo "执行 Gazebo/RViz 启动后的 WSLg COPY MODE 复检……"
  check_copy_mode
fi

echo "启动 HTTP bridge（端口 ${BRIDGE_PORT}）……"
python3 "$SIM_ROOT/bridge/sim_bridge.py" \
  --port "$BRIDGE_PORT" \
  --wait-ready-timeout "$READY_TIMEOUT" &
BRIDGE_PID=$!
wait_for_bridge

echo "OK"
echo "仿真栈已启动：ROS Master=${ROS_MASTER_URI}，bridge=0.0.0.0:${BRIDGE_PORT}"
echo "看到 SIMULATION_BRIDGE_READY 和 state=waiting 后，再启动小车主流程。"
echo "本终端保持运行；结束时按 Ctrl-C。"

wait "$BRIDGE_PID"
