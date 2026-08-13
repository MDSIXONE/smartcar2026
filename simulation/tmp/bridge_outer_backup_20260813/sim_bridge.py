#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_bridge.py — 任务三仿真 HTTP 桥接服务（WSL 侧）

小车通过局域网 HTTP 单向访问本服务：
  POST /start  启动 task3_execute.launch（物品名透传：cargo_category + cargo_name）
  GET  /status 查询任务状态（完成状态回读：/sim_task3/done）

本脚本在任务开始前还会临时降低 Gazebo 的物理更新频率，以减轻待机阶段
桌面 UI 的负载；收到 POST /start 后恢复启动时读取的原值，再启动任务。
不修改规划器 / URDF / world 等仿真工程文件。运行前需已人工启动
task3_prepare.launch，且本终端已 source ROS 环境并指向仿真 Master
（ROS_MASTER_URI=http://127.0.0.1:11312）。

兼容 Python 3.8+；物理频率控制使用 ROS Noetic 已安装的 `rospy` 与
`gazebo_msgs` 服务类型。
"""

import argparse
import datetime
import json
import os
import shutil
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

BRIDGE_DIR = os.path.dirname(os.path.abspath(__file__))
ROS_NOETIC_BIN = "/opt/ros/noetic/bin"
# The arm-ready topic is published only once by set_arm_initial_pose, whose
# node exits right after publishing (hold_initial_arm_pose=false in the
# prepare launch), so the topic disappears from the master.  The map_server
# /map topic stays alive for the whole prepare launch and is the reliable
# readiness signal.
READY_TOPIC = "/map"
DONE_TOPIC = "/sim_task3/done"


def log(msg):
    """带时间戳的 stdout 日志。"""
    line = "%s %s" % (datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), msg)
    print(line, flush=True)


class BridgeState:
    """状态机：waiting ->(POST /start)-> running ->(done True)-> done
    running 中 roslaunch 非零退出 -> failed；轮询超时 -> failed。
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.state = "waiting"
        self.detail = "ready"
        self.item_name = ""
        self.category = "auto"
        self.start_time = None
        self.done_timeout = 1800.0
        self.proc = None
        self.logf = None
        self.watcher = None
        self.physics_rate_controller = None


BRIDGE = BridgeState()


def ensure_ros_bin_in_path():
    """启动 bridge 的终端已 source 过 ROS 环境；必要时用 Noetic bin 兜底。"""
    if shutil.which("rostopic") is not None:
        return
    if os.path.isdir(ROS_NOETIC_BIN):
        os.environ["PATH"] = ROS_NOETIC_BIN + os.pathsep + os.environ.get("PATH", "")


def run_rostopic(topic, timeout):
    """单次 rostopic echo -n 1，超时由 subprocess 强杀。"""
    return subprocess.run(
        ["rostopic", "echo", "-n", "1", topic],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        timeout=timeout,
    )


def wait_sim_ready(timeout, interval=2.0, sub_timeout=10.0):
    """Poll /map until the map_server publishes data (prepare launch ready).

    The map topic carries a latched OccupancyGrid; any non-empty output
    means the environment (Gazebo + map_server + move_base) is up.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            result = run_rostopic(READY_TOPIC, sub_timeout)
            if (result.stdout or "").strip():
                return True
        except subprocess.TimeoutExpired:
            pass
        except OSError as exc:
            log("ready check rostopic failed: %s" % exc)
        time.sleep(interval)
    return False


class GazeboPhysicsRateController:
    """在等待小车开始信号时降低 Gazebo 的物理更新频率。"""

    def __init__(self, idle_update_rate):
        if idle_update_rate <= 0.0:
            raise ValueError("idle_update_rate must be greater than zero")
        self.idle_update_rate = idle_update_rate
        self.original_properties = None
        self._get_physics = None
        self._set_physics = None

    def _connect(self):
        import rospy
        from gazebo_msgs.srv import GetPhysicsProperties, SetPhysicsProperties

        if not rospy.core.is_initialized():
            rospy.init_node("sim_bridge_physics", anonymous=True, disable_signals=True)
        rospy.wait_for_service("/gazebo/get_physics_properties", timeout=10.0)
        rospy.wait_for_service("/gazebo/set_physics_properties", timeout=10.0)
        self._get_physics = rospy.ServiceProxy(
            "/gazebo/get_physics_properties", GetPhysicsProperties
        )
        self._set_physics = rospy.ServiceProxy(
            "/gazebo/set_physics_properties", SetPhysicsProperties
        )

    def _set_update_rate(self, properties, update_rate):
        response = self._set_physics(
            properties.time_step,
            update_rate,
            properties.gravity,
            properties.ode_config,
        )
        if not response.success:
            raise RuntimeError(
                "Gazebo rejected max_update_rate=%.3f: %s"
                % (update_rate, response.status_message)
            )

    def reduce_for_idle(self):
        self._connect()
        self.original_properties = self._get_physics()
        self._set_update_rate(self.original_properties, self.idle_update_rate)
        log(
            "Gazebo max_update_rate saved %.3f Hz; idle rate set to %.3f Hz"
            % (self.original_properties.max_update_rate, self.idle_update_rate)
        )

    def restore_for_task(self):
        if self.original_properties is None:
            raise RuntimeError("Gazebo physics properties were not recorded")
        self._set_update_rate(
            self.original_properties, self.original_properties.max_update_rate
        )
        log(
            "Gazebo max_update_rate restored to %.3f Hz before task start"
            % self.original_properties.max_update_rate
        )


def start_task(item_name, category):
    """非阻塞启动 roslaunch；stdout/stderr 重定向到 bridge 目录下的运行日志。

    参数列表形式（不用 shell=True），中文类别/名称经列表参数 + 子进程转义天然安全。
    """
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join(BRIDGE_DIR, "task3_run_%s.log" % timestamp)
    logf = open(log_path, "wb")
    cmd = [
        "roslaunch", "car3", "task3_execute.launch",
        "cargo_category:=%s" % category,
        "cargo_name:=%s" % item_name,
    ]
    proc = subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT)
    log("roslaunch started pid=%d -> %s" % (proc.pid, log_path))
    return proc, logf


def finish_run(failed, detail):
    """结束一次运行：落定状态、关闭日志句柄、打印结果行。"""
    with BRIDGE.lock:
        BRIDGE.state = "failed" if failed else "done"
        BRIDGE.detail = detail
        logf = BRIDGE.logf
        BRIDGE.logf = None
        item_name, category = BRIDGE.item_name, BRIDGE.category
    if logf is not None:
        try:
            logf.close()
        except OSError:
            pass
    if failed:
        log("SIMULATION_BRIDGE_FAILED item=%s category=%s reason=%s"
            % (item_name, category, detail))
    else:
        log("SIMULATION_BRIDGE_DONE item=%s category=%s" % (item_name, category))


def watch_task():
    """后台线程：轮询 done 话题 + 监控 roslaunch 进程。"""
    interval = 1.0
    sub_timeout = 10.0
    with BRIDGE.lock:
        deadline = BRIDGE.start_time + BRIDGE.done_timeout
    while True:
        with BRIDGE.lock:
            proc = BRIDGE.proc
        if proc is not None:
            ret = proc.poll()
            if ret is not None and ret != 0:
                # roslaunch 非零退出且未收到 done -> failed
                finish_run(True, "roslaunch exited with code %d" % ret)
                return
            # 退出码 0 且未收到 done：roslaunch 可能先退，继续等 latched done
        try:
            result = run_rostopic(DONE_TOPIC, sub_timeout)
            if "data: True" in (result.stdout or ""):
                finish_run(False, "done")
                return
            # "data: False" 或其它输出：继续轮询
        except subprocess.TimeoutExpired:
            pass
        except OSError as exc:
            log("done poll rostopic failed: %s" % exc)
        if time.time() > deadline:
            finish_run(True, "timeout waiting %s" % DONE_TOPIC)
            return
        time.sleep(interval)


class BridgeHandler(BaseHTTPRequestHandler):
    server_version = "sim_bridge/1.0"

    def log_message(self, fmt, *args):
        log("http %s" % (fmt % args))

    def _send_json(self, code, payload):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _status_payload(self):
        with BRIDGE.lock:
            return {
                "state": BRIDGE.state,
                "detail": BRIDGE.detail,
                "item_name": BRIDGE.item_name,
                "category": BRIDGE.category,
            }

    def do_GET(self):
        if self.path != "/status":
            self._send_json(404, {"error": "not found"})
            return
        self._send_json(200, self._status_payload())

    def do_POST(self):
        if self.path != "/start":
            self._send_json(404, {"error": "not found"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0") or "0")
        except ValueError:
            length = 0
        raw = self.rfile.read(length) if length > 0 else b""
        try:
            data = json.loads(raw.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            self._send_json(400, {"error": "invalid JSON body"})
            return
        if not isinstance(data, dict):
            self._send_json(400, {"error": "invalid JSON body"})
            return
        item_name = data.get("item_name", "")
        category = data.get("category", "auto")
        if not isinstance(item_name, str) or not item_name.strip():
            self._send_json(400, {"error": "missing item_name"})
            return
        if not isinstance(category, str) or not category.strip():
            category = "auto"
        with BRIDGE.lock:
            if BRIDGE.state == "waiting":
                physics_rate_controller = BRIDGE.physics_rate_controller
                if physics_rate_controller is None:
                    self._send_json(503, {"error": "Gazebo physics rate controller is not ready"})
                    return
                try:
                    physics_rate_controller.restore_for_task()
                except RuntimeError as exc:
                    log("failed to restore Gazebo max_update_rate: %s" % exc)
                    self._send_json(503, {"error": str(exc)})
                    return
                try:
                    proc, logf = start_task(item_name, category)
                except OSError as exc:
                    self._send_json(500, {"error": "failed to start roslaunch: %s" % exc})
                    return
                BRIDGE.state = "running"
                BRIDGE.detail = "task running"
                BRIDGE.item_name = item_name
                BRIDGE.category = category
                BRIDGE.start_time = time.time()
                BRIDGE.proc = proc
                BRIDGE.logf = logf
                BRIDGE.watcher = threading.Thread(target=watch_task, daemon=True)
                BRIDGE.watcher.start()
                log("POST /start accepted item=%s category=%s" % (item_name, category))
                self._send_json(200, {"accepted": True, "state": "running"})
            elif BRIDGE.state == "running":
                self._send_json(409, {"error": "already running"})
            else:
                self._send_json(409, {"error": "already finished, restart bridge for another run"})


def cleanup():
    """退出前尽量终止 roslaunch 子进程。"""
    with BRIDGE.lock:
        proc = BRIDGE.proc
        logf = BRIDGE.logf
        BRIDGE.logf = None
    if logf is not None:
        try:
            logf.close()
        except OSError:
            pass
    if proc is not None and proc.poll() is None:
        log("terminating roslaunch pid %d" % proc.pid)
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            log("roslaunch pid %d did not stop, sending SIGKILL" % proc.pid)
            proc.kill()
            proc.wait()


def main():
    parser = argparse.ArgumentParser(description="Simulation task3 HTTP bridge (WSL side)")
    parser.add_argument("--port", type=int, default=11313,
                        help="listen port (default 11313)")
    parser.add_argument("--done-timeout", type=float, default=1800.0,
                        help="seconds to wait for /sim_task3/done while running (default 1800)")
    parser.add_argument("--wait-ready-timeout", type=float, default=300.0,
                        help="seconds to wait for ready topic; 0 to skip (default 300)")
    parser.add_argument("--idle-update-rate", type=float, default=100.0,
                        help="Gazebo max_update_rate while waiting for POST /start (default 100 Hz)")
    args = parser.parse_args()

    ensure_ros_bin_in_path()

    if args.wait_ready_timeout > 0:
        log("waiting for %s (timeout %.0fs) ..." % (READY_TOPIC, args.wait_ready_timeout))
        if not wait_sim_ready(args.wait_ready_timeout):
            log("ERROR: simulation environment not ready within %.0fs, exiting"
                % args.wait_ready_timeout)
            sys.exit(1)
        log("SIMULATION_BRIDGE_READY")
    else:
        log("SIMULATION_BRIDGE_READY (ready check skipped, wait-ready-timeout=0)")

    physics_rate_controller = GazeboPhysicsRateController(args.idle_update_rate)
    physics_rate_controller.reduce_for_idle()

    BRIDGE.done_timeout = args.done_timeout
    BRIDGE.physics_rate_controller = physics_rate_controller

    server = ThreadingHTTPServer(("0.0.0.0", args.port), BridgeHandler)
    server.daemon_threads = True
    log("simulation bridge listening on 0.0.0.0:%d (state=waiting)" % args.port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        log("interrupted, shutting down")
    finally:
        server.server_close()
        cleanup()
    log("bridge exited")


if __name__ == "__main__":
    main()
