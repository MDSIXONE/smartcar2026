#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
yolo_client.py — yolo_tiny_cuda 子进程客户端 (python2 / python3 都能跑)
====================================================================
为什么不直接用工程自带的 detect.py:
  detect.py 是 **python3 only** 的 (`#!/usr/bin/env python3`、
  `except BrokenPipeError`、`p.wait(timeout=5)`), 而 Melodic 下
  lane_follow.py 跑在 python2 上, import 进来直接语法/属性报错。
  协议本身(yolo-pipe/1)和语言无关, 所以这里按 README 重写一份
  py2/py3 兼容的最小客户端 —— C++ 那边一个字都不用改。

用法:
    from yolo_client import YoloProc
    with YoloProc(exe, weights, backend="cuda") as y:
        print(y.hello["classes"])          # ['left','right','stop','straight']
        r = y.detect_bgr(img_bgr)          # OpenCV 的 BGR ndarray, 零拷贝转换
        d = best_det(r)                    # 置信度最高的那个框, 没有则 None

设计要点:
  * 所有阻塞读都带 select 超时 —— 子进程要是卡死(CUDA 初始化失败、显存
    不够被内核 OOM 掉一半), 调用方必须能拿到异常而不是永远挂住。
  * close() 先发 BYE 再 terminate 再 kill, 保证"识别完就杀掉", 不给
    Nano 那 4GB 显存留占用。
"""
from __future__ import print_function

import json
import os
import select
import struct
import subprocess
import sys
import time

MAGIC = b'YV'
T_HELLO, T_CONFIG, T_READY = 0x01, 0x02, 0x03
T_IMAGE, T_RESULT, T_ERROR, T_BYE = 0x10, 0x11, 0x1F, 0x7F
ENC_AUTO, ENC_RGB8, ENC_BGR8 = 0, 1, 2

HDR = struct.Struct('<2sBBII')          # magic, type, flags, id, length

if sys.version_info[0] < 3:
    def _norm(o):
        """py2: json.loads 出来的全是 unicode, 一旦拿去格式化含中文的
        普通字符串("后端=%s" % u"cuda"), py2 会试着用 ascii 解码那个模板,
        直接 UnicodeDecodeError。所以解出来就全转成 utf-8 str。"""
        if isinstance(o, unicode):        # noqa: F821 (py2 only)
            return o.encode('utf-8')
        if isinstance(o, list):
            return [_norm(x) for x in o]
        if isinstance(o, dict):
            return dict((_norm(k), _norm(v)) for k, v in o.items())
        return o
else:
    def _norm(o):
        return o


def _loads(payload):
    return _norm(json.loads(payload.decode('utf-8')))


def find_exe(d):
    """在 yolo 工程目录里找可执行文件"""
    for name in ("yolo_tiny_cuda", "yolo_tiny", "yolo"):
        p = os.path.join(d, name)
        if os.path.isfile(p) and os.access(p, os.X_OK):
            return p
    return None


def find_weights(d):
    """找 .weights: 优先带 final / traffic 字样的, 否则取最新的一个"""
    cand = [os.path.join(d, f) for f in os.listdir(d)
            if f.lower().endswith(".weights")] if os.path.isdir(d) else []
    if not cand:
        return None
    cand.sort(key=lambda p: (("final" in os.path.basename(p).lower()) * 2 +
                             ("traffic" in os.path.basename(p).lower()),
                             os.path.getmtime(p)))
    return cand[-1]


def best_det(result, names=None):
    """结果里置信度最高的一个框; names 给了就只在这些类里挑。
    返回 (name, conf, box) 或 None"""
    best = None
    for d in result.get("det", []):
        if names and d["name"] not in names:
            continue
        if best is None or d["conf"] > best[1]:
            best = (d["name"], float(d["conf"]), d["box"])
    return best


class YoloProc(object):
    def __init__(self, exe, weights, backend="auto", device=0,
                 stderr=None, timeout=20.0):
        self.timeout = float(timeout)
        cmd = [exe, "--weights", weights, "--backend", backend,
               "--device", str(device)]
        # ⚠ close_fds=True 不能少。py2 的 Popen 默认 close_fds=False, 子进程
        #   会**继承父进程所有打开的 fd, 包括相机**。yolo 自己不碰相机, 但
        #   只要它活着, 那个 fd 的内核引用计数就不归零, uvcvideo 的 release
        #   不会跑, stream 所有权一直挂在这个"幽灵"fd 上 —— 父进程这边想
        #   release+重开换分辨率, 新 open 的 S_FMT 全 EBUSY("Pixel format
        #   unsupported" / "Unable to stop the stream: Device or resource
        #   busy"), 直到 yolo 退出。实车 2026-08-18 两趟就栽在这。
        self.p = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=(stderr if stderr is not None else sys.stderr), bufsize=0,
            close_fds=True)
        self._id = 1
        # 启动即主动发 HELLO(含 backend / classes / 输入尺寸), 先读掉
        self.hello = self._expect(T_HELLO)

    # ---------------- 底层收发 ----------------
    def _read_exact(self, n, timeout):
        buf = b''
        end = time.time() + timeout
        fd = self.p.stdout
        while len(buf) < n:
            left = end - time.time()
            if left <= 0:
                raise IOError("等 yolo 子进程回包超时 (%.1fs, 收到 %d/%d 字节)"
                              % (timeout, len(buf), n))
            r, _, _ = select.select([fd], [], [], min(0.5, left))
            if not r:
                if self.p.poll() is not None:
                    raise IOError("yolo 子进程已退出 (returncode=%s)"
                                  % self.p.poll())
                continue
            chunk = fd.read(n - len(buf))
            if not chunk:
                raise IOError("yolo 子进程提前退出 (returncode=%s)"
                              % self.p.poll())
            buf += chunk
        return buf

    def _read_frame(self, timeout=None):
        t = self.timeout if timeout is None else timeout
        magic, ftype, flags, fid, length = HDR.unpack(
            self._read_exact(HDR.size, t))
        if magic != MAGIC:
            raise IOError("帧头 magic 错误(%r), 协议已失步" % (magic,))
        payload = self._read_exact(length, t) if length else b''
        return ftype, flags, fid, payload

    def _write_frame(self, ftype, fid, payload=b'', flags=0):
        self.p.stdin.write(HDR.pack(MAGIC, ftype, flags, fid, len(payload)))
        if payload:
            self.p.stdin.write(payload)
        self.p.stdin.flush()

    def _expect(self, want, timeout=None):
        ftype, _, _, payload = self._read_frame(timeout)
        if ftype == T_ERROR:
            raise RuntimeError("yolo 子进程报错: %s"
                               % payload.decode("utf-8", "replace"))
        if ftype != want:
            raise IOError("期望帧类型 0x%02X, 实际 0x%02X" % (want, ftype))
        return _loads(payload) if payload else {}

    # ---------------- 对外接口 ----------------
    @property
    def backend(self):
        return self.hello.get("backend", "?")

    @property
    def classes(self):
        return self.hello.get("classes", [])

    def configure(self, **kw):
        """conf / nms / box_format / timings"""
        fid = self._id
        self._id += 1
        self._write_frame(T_CONFIG, fid, json.dumps(kw).encode("utf-8"))
        return self._expect(T_READY)

    def detect_bytes(self, data, enc=ENC_AUTO, timeout=None):
        fid = self._id
        self._id += 1
        self._write_frame(T_IMAGE, fid, data, flags=enc)
        ftype, _, rid, payload = self._read_frame(timeout)
        body = _loads(payload)
        if ftype == T_ERROR:
            raise RuntimeError(body.get("error", "未知错误"))
        if rid != fid:
            raise IOError("响应 id 对不上: 期望 %d 收到 %d" % (fid, rid))
        return body

    def detect_bgr(self, img, timeout=None):
        """OpenCV 的 BGR ndarray 直送 (ENC_BGR8, 引擎侧省一次 cvtColor)。
        ⚠ 必须是连续内存: cv2.flip / remap 的输出是连续的, 切片不是。"""
        import numpy as np
        if not img.flags["C_CONTIGUOUS"]:
            img = np.ascontiguousarray(img)
        h, w = img.shape[:2]
        return self.detect_bytes(struct.pack("<II", w, h) + img.tobytes(),
                                 ENC_BGR8, timeout)

    def detect_file(self, path):
        f = open(path, "rb")
        try:
            return self.detect_bytes(f.read(), ENC_AUTO)
        finally:
            f.close()

    def close(self, grace=2.0):
        """优雅 BYE -> terminate -> kill, 保证不留僵尸占显存"""
        if self.p is None:
            return
        if self.p.poll() is None:
            try:
                self._write_frame(T_BYE, 0)
                self.p.stdin.close()
            except Exception:          # py2 没有 BrokenPipeError
                pass
            end = time.time() + grace
            while self.p.poll() is None and time.time() < end:
                time.sleep(0.05)
        if self.p.poll() is None:
            try:
                self.p.terminate()
            except Exception:
                pass
            end = time.time() + 1.0
            while self.p.poll() is None and time.time() < end:
                time.sleep(0.05)
        if self.p.poll() is None:
            try:
                self.p.kill()
            except Exception:
                pass
        for f in (self.p.stdin, self.p.stdout):
            try:
                f.close()
            except Exception:
                pass
        self.p = None

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()


if __name__ == "__main__":
    # 自检: yolo_client.py <图片> [...]   (不依赖 ROS)
    here = os.path.dirname(os.path.abspath(__file__))
    d = os.path.join(os.path.dirname(here), "yolo", "yolo_tiny_cuda")
    exe, w = find_exe(d), find_weights(d)
    if not exe or not w:
        sys.exit("在 %s 里没找到 可执行文件/权重 (exe=%s weights=%s)"
                 % (d, exe, w))
    y = YoloProc(exe, w)
    print("backend=%s classes=%s" % (y.backend, y.classes))
    try:
        for p in sys.argv[1:]:
            print(p, json.dumps(y.detect_file(p), ensure_ascii=False))
    finally:
        y.close()
