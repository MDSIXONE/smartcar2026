#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把 yolo_tiny_cuda 拉起来当子进程，通过管道送图、收 JSON。

    ./detect.py a.jpg b.jpg
    {}
    {"left": [[0.1, 0.15, 0.5, 0.55, 0.99]]}

每行对应一张输入图，顺序和命令行一致。框是 [cx, cy, w, h, conf]，
cx/cy/w/h 都是相对**原图**的归一化坐标（和 YOLO 标签格式一致）。

只依赖标准库。协议细节见 README.md。
"""
import argparse
import json
import os
import struct
import subprocess
import sys

MAGIC = b'YV'
T_HELLO, T_CONFIG, T_READY = 0x01, 0x02, 0x03
T_IMAGE, T_RESULT, T_ERROR, T_BYE = 0x10, 0x11, 0x1F, 0x7F
ENC_AUTO, ENC_RGB8, ENC_BGR8 = 0, 1, 2

HDR = struct.Struct('<2sBBII')       # magic, type, flags, id, length


class Detector:
    """yolo_tiny_cuda 子进程的封装。用 with 语句管理生命周期。"""

    def __init__(self, exe, weights, backend='auto', device=0, stderr=None):
        cmd = [exe, '--weights', weights, '--backend', backend, '--device', str(device)]
        self.p = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=(stderr if stderr is not None else sys.stderr), bufsize=0)
        self._next_id = 1
        self.hello = self._expect(T_HELLO)

    # ---------------- 底层收发 ----------------
    def _read_exact(self, n):
        buf = b''
        while len(buf) < n:
            chunk = self.p.stdout.read(n - len(buf))
            if not chunk:
                code = self.p.poll()
                raise IOError('子进程提前退出 (returncode=%s)，收到 %d/%d 字节' % (code, len(buf), n))
            buf += chunk
        return buf

    def _read_frame(self):
        magic, ftype, flags, fid, length = HDR.unpack(self._read_exact(HDR.size))
        if magic != MAGIC:
            raise IOError('帧头 magic 错误: %r，协议已失步' % (magic,))
        payload = self._read_exact(length) if length else b''
        return ftype, flags, fid, payload

    def _write_frame(self, ftype, fid, payload=b'', flags=0):
        self.p.stdin.write(HDR.pack(MAGIC, ftype, flags, fid, len(payload)))
        if payload:
            self.p.stdin.write(payload)
        self.p.stdin.flush()

    def _expect(self, want):
        ftype, _, _, payload = self._read_frame()
        if ftype == T_ERROR:
            raise RuntimeError('子进程报错: %s' % payload.decode('utf-8', 'replace'))
        if ftype != want:
            raise IOError('期望帧类型 0x%02X，实际收到 0x%02X' % (want, ftype))
        return json.loads(payload.decode('utf-8')) if payload else {}

    # ---------------- 对外接口 ----------------
    def configure(self, **kw):
        """可设 conf / nms / box_format / timings。"""
        fid = self._next_id; self._next_id += 1
        self._write_frame(T_CONFIG, fid, json.dumps(kw).encode('utf-8'))
        return self._expect(T_READY)

    def detect_bytes(self, data, enc=ENC_AUTO):
        """送一张图（编码后的文件字节，或 raw）。返回结果 dict。"""
        fid = self._next_id; self._next_id += 1
        self._write_frame(T_IMAGE, fid, data, flags=enc)
        ftype, _, rid, payload = self._read_frame()
        body = json.loads(payload.decode('utf-8'))
        if ftype == T_ERROR:
            raise RuntimeError(body.get('error', '未知错误'))
        if rid != fid:
            raise IOError('响应 id 对不上: 期望 %d 收到 %d' % (fid, rid))
        return body

    def detect_file(self, path):
        with open(path, 'rb') as f:
            return self.detect_bytes(f.read(), ENC_AUTO)

    def detect_raw(self, w, h, pixels, bgr=False):
        """pixels 为 w*h*3 的交错 8 位数据。"""
        return self.detect_bytes(struct.pack('<II', w, h) + bytes(pixels),
                                 ENC_BGR8 if bgr else ENC_RGB8)

    def close(self):
        if self.p.poll() is None:
            try:
                self._write_frame(T_BYE, 0)
                self.p.stdin.close()
            except (BrokenPipeError, OSError):
                pass
            try:
                self.p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.p.kill()

    def __enter__(self):
        return self

    def __exit__(self, *a):
        self.close()


def compact(result, flat=False):
    """把完整结果压成 {类名: [[cx,cy,w,h,conf], ...]}。"""
    out = {}
    for d in result['det']:
        out.setdefault(d['name'], []).append([round(v, 6) for v in d['box']] + [round(d['conf'], 4)])
    if flat:   # 每类只有 1 个框时去掉一层括号
        out = {k: (v[0] if len(v) == 1 else v) for k, v in out.items()}
    return out


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description='yolov4-tiny 管道推理 demo')
    ap.add_argument('images', nargs='+', help='输入图片路径')
    ap.add_argument('--exe', default=os.path.join(here, 'yolo_tiny_cuda'))
    ap.add_argument('--weights', default=os.path.join(here, 'yolov4-tiny-traffic_final.weights'))
    ap.add_argument('--backend', default='auto', choices=['auto', 'cuda', 'cpu'])
    ap.add_argument('--device', type=int, default=0)
    ap.add_argument('--conf', type=float, default=0.25)
    ap.add_argument('--nms', type=float, default=0.45)
    ap.add_argument('--flat', action='store_true', help='单框时输出一维数组')
    ap.add_argument('--full', action='store_true', help='输出完整 JSON（含耗时）')
    ap.add_argument('--quiet', action='store_true', help='丢掉子进程 stderr')
    args = ap.parse_args()

    if not os.path.exists(args.exe):
        sys.exit('找不到可执行文件: %s（先 make）' % args.exe)
    if not os.path.exists(args.weights):
        sys.exit('找不到权重: %s（用 --weights 指定）' % args.weights)

    devnull = open(os.devnull, 'w') if args.quiet else None
    try:
        with Detector(args.exe, args.weights, args.backend, args.device, stderr=devnull) as det:
            det.configure(conf=args.conf, nms=args.nms, timings=args.full)
            for path in args.images:
                if not os.path.exists(path):
                    print(json.dumps({'error': '文件不存在: %s' % path}, ensure_ascii=False))
                    continue
                try:
                    r = det.detect_file(path)
                except RuntimeError as e:
                    print(json.dumps({'error': str(e)}, ensure_ascii=False))
                    continue
                print(json.dumps(r if args.full else compact(r, args.flat),
                                 ensure_ascii=False, separators=(',', ':')))
    finally:
        if devnull:
            devnull.close()


if __name__ == '__main__':
    main()
