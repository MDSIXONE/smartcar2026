#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""HTTP bridge from the real vehicle to the task-3 simulation.

The real vehicle POSTs ``/start`` with the collected item and polls
``/status``.  This process starts ``task3_execute.launch`` only after the
separate simulation environment is ready on ROS Master 127.0.0.1:11312.
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
READY_TOPIC = "/map"
DONE_TOPIC = "/sim_task3/done"


def log(msg):
    """Print one timestamped status line."""
    line = "%s %s" % (datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"), msg)
    print(line, flush=True)


class BridgeState:
    """waiting -> running -> done/failed state held by the HTTP service."""

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


BRIDGE = BridgeState()


def ensure_ros_bin_in_path():
    """Use the sourced Noetic environment, with a conservative bin fallback."""
    if shutil.which("rostopic") is not None:
        return
    if os.path.isdir(ROS_NOETIC_BIN):
        os.environ["PATH"] = ROS_NOETIC_BIN + os.pathsep + os.environ.get("PATH", "")


def run_rostopic(topic, timeout):
    """Return one topic sample; subprocess timeout stops the probe."""
    return subprocess.run(
        ["rostopic", "echo", "-n", "1", topic],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        timeout=timeout,
    )


def wait_sim_ready(timeout, interval=2.0, sub_timeout=10.0):
    """Wait for the persistent map topic published by prepare.launch."""
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


def start_task(item_name, category):
    """Start the simulation task asynchronously, recording its full log."""
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
    """Persist a terminal status and close the task log handle."""
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
    """Monitor the started launch and its latched /sim_task3/done result."""
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
                finish_run(True, "roslaunch exited with code %d" % ret)
                return
        try:
            result = run_rostopic(DONE_TOPIC, sub_timeout)
            if "data: True" in (result.stdout or ""):
                finish_run(False, "done")
                return
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
    """Stop only the roslaunch process created by this bridge."""
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
    args = parser.parse_args()

    ensure_ros_bin_in_path()
    if args.wait_ready_timeout > 0:
        log("waiting for %s (timeout %.0fs) ..." % (READY_TOPIC, args.wait_ready_timeout))
        if not wait_sim_ready(args.wait_ready_timeout):
            log("ERROR: simulation environment not ready within %.0fs, exiting"
                % args.wait_ready_timeout)
            sys.exit(1)
        log("SIMULATION_ENVIRONMENT_READY")
    else:
        log("SIMULATION_ENVIRONMENT_READY (ready check skipped, wait-ready-timeout=0)")

    BRIDGE.done_timeout = args.done_timeout
    server = ThreadingHTTPServer(("0.0.0.0", args.port), BridgeHandler)
    server.daemon_threads = True
    log("SIMULATION_BRIDGE_READY")
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
