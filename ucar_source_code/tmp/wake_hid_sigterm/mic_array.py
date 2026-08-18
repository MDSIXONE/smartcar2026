#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
讯飞环形六麦克风阵列 —— 独立最小可运行包（无绝对路径，无需 ROS/讯飞工程）

目录结构（全部相对本文件）：
    mic_array.py
    res/libhid_lib.so     驱动库
    res/config.txt        固件资源描述
    res/system.tar        固件资源包(8MB)
    res/xf_mic.rules      udev 权限规则

原理要点：
  * 麦阵列走私有 USB HID 协议，不是 ALSA 声卡，arecord 读不到
  * 每次麦克风【上电/拔插】后处于"未开机"态，必须下发 system.tar 才能工作（约10秒）
    —— 系统 reboot 若不断电则状态保留
  * 音频在回调 businessMsg.data 里就是裸 PCM（已验证与库写文件的字节完全一致）
  * 主麦方向"可唤醒或手动设置"，手动 set_major_mic_id 即可跳过唤醒词

两种音频：
    降噪 16kHz/16bit/单声道       msgId 0x02，每帧 1024B
    原始 16kHz/32bit/8通道交织     msgId 0x06，每帧 16384B（ch0-5麦克风, ch6-7参考）

用法：
    python3 mic_array.py                    # 降噪，录5秒 → out_deno.wav
    python3 mic_array.py -m ori             # 原始(关降噪) → out_ori_ch0.wav
    python3 mic_array.py -m ori --all-ch    # 6个麦各存一个
    python3 mic_array.py -m both            # 两种都录，方便对比
    python3 mic_array.py --level            # 实时音量，定VAD阈值
    python3 mic_array.py --mic 3            # 换波束方向(主麦0~5)
    python3 mic_array.py --boot-only        # 只做开机(下发固件)，不录音

作为模块：
    from mic_array import MicArray
    mic = MicArray(); mic.boot()
    pcm = mic.record(5)                     # 返回 16bit 单声道 PCM bytes
    mic.close()

权限：首次使用装 udev 规则免 sudo
    sudo cp res/xf_mic.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger
"""

import os
import sys
import time
import wave
import math
import ctypes
import struct
import argparse
from ctypes import CFUNCTYPE, c_char_p, c_int, c_uint8, POINTER, Structure, c_uint

# ---------------- 全部路径相对本文件 ----------------
HERE    = os.path.dirname(os.path.abspath(__file__))
RES     = os.path.join(HERE, "res")
LIB_SO  = os.path.join(RES, "libhid_lib.so")
CFG_TXT = os.path.join(RES, "config.txt").encode()
SYS_TAR = os.path.join(RES, "system.tar").encode()

RATE         = 16000
DENO_FRAME   = 1024      # 512样本 × 2字节 = 32ms
ORI_FRAME    = 16384     # 512样本 × 8通道 × 4字节 = 32ms
ORI_CH       = 8


class business_msg_t(Structure):
    _fields_ = [("handle",  c_uint), ("version", c_uint8), ("opcode", c_uint8),
                ("modId",   c_uint8), ("msgId",  c_uint8),
                ("data",    POINTER(c_uint8)), ("length", c_int)]


def rms_i16(raw):
    n = len(raw) // 2
    if n == 0:
        return 0.0
    v = struct.unpack("<%dh" % n, raw[:n * 2])
    return math.sqrt(sum(x * x for x in v) / n)


def deinterleave(raw, ch, nch=ORI_CH, shift=16):
    """8通道×32bit 交织 → 取一路，转 int16 bytes"""
    ns = len(raw) // (nch * 4)
    if ns == 0:
        return b""
    v = struct.unpack("<%di" % (ns * nch), raw[:ns * nch * 4])
    out = [max(-32768, min(32767, v[i * nch + ch] >> shift)) for i in range(ns)]
    return struct.pack("<%dh" % len(out), *out)


def write_wav(path, pcm, rate=RATE):
    with wave.open(path, "wb") as w:
        w.setnchannels(1); w.setsampwidth(2); w.setframerate(rate)
        w.writeframes(pcm)


class MicArray(object):
    def __init__(self, lib_path=None, shift=16, verbose=True):
        self.verbose = verbose
        self.shift = shift
        self.mode = "deno"
        self.channel = 0
        self.rms = 0.0
        self.awake = False
        self.awake_mic = -1
        self.awake_angle = -1
        self.wake_set_result = None
        self.deno = bytearray()
        self.ori = bytearray()
        self._dump = 0
        self._opened = False
        self._recording = False

        p = lib_path or LIB_SO
        if not os.path.exists(p):
            sys.exit("找不到驱动库: %s\n（res/ 目录要和本脚本放一起）" % p)
        for f in (CFG_TXT.decode(), SYS_TAR.decode()):
            if not os.path.exists(f):
                sys.exit("找不到固件资源: %s" % f)
        self.lib = ctypes.CDLL(p)
        self._proto()
        self._keep = []          # 防回调被GC

    def _log(self, *a):
        if self.verbose:
            print(*a)

    def _proto(self):
        L = self.lib
        L.hid_open.restype = ctypes.c_void_p
        L.get_software_version.restype = c_char_p
        L.send_resource_info.argtypes = [c_char_p, c_int]
        L.send_resource.argtypes = [POINTER(ctypes.c_ubyte), c_char_p, c_int]
        L.get_denoised_sound.argtypes = [c_char_p, POINTER(ctypes.c_ubyte)]
        L.get_original_sound.argtypes = [c_char_p, POINTER(ctypes.c_ubyte)]
        L.whether_set_succeed.argtypes = [POINTER(ctypes.c_ubyte), POINTER(ctypes.c_ubyte)]
        L.whether_set_succeed.restype = c_int
        L.get_protocol_version.argtypes = [POINTER(ctypes.c_ubyte), POINTER(ctypes.c_char)]
        L.set_major_mic_id.argtypes = [c_int]
        L.get_awake_mic_id.argtypes = [POINTER(ctypes.c_ubyte), POINTER(ctypes.c_ubyte)]
        L.get_awake_mic_id.restype = c_int
        L.get_awake_mic_angle.argtypes = [POINTER(ctypes.c_ubyte), POINTER(ctypes.c_ubyte)]
        L.get_awake_mic_angle.restype = c_int
        L.set_awake_word.argtypes = [c_char_p]
        L.set_awake_word.restype = c_int
        L.whether_set_awake_word.argtypes = [POINTER(ctypes.c_ubyte), POINTER(ctypes.c_ubyte)]
        L.whether_set_awake_word.restype = c_int

    # ---------------- 回调 ----------------
    def _callback(self, m):
        L, mod, msg = self.lib, m.modId, m.msgId
        try:
            if mod == 0x01:
                if msg == 0x02 and self.mode in ("deno", "both"):
                    raw = ctypes.string_at(m.data, DENO_FRAME)
                    self.deno.extend(raw)
                    if self.mode == "deno":
                        self.rms = rms_i16(raw)
                    if self._dump > 0:
                        self._dump -= 1
                        print("\n[deno] " + " ".join("%02x" % b for b in raw[:16]))
                elif msg == 0x06 and self.mode in ("ori", "both"):
                    raw = ctypes.string_at(m.data, ORI_FRAME)
                    self.ori.extend(raw)
                    if self.mode == "ori":
                        self.rms = rms_i16(deinterleave(raw, self.channel, shift=self.shift))
                    if self._dump > 0:
                        self._dump -= 1
                        print("\n[ori] " + " ".join("%02x" % b for b in raw[:16]))

            elif mod == 0x02:
                if msg == 0x01:                                # 被唤醒
                    k1 = (ctypes.c_ubyte * 5)(*map(ord, "beam"))
                    k2 = (ctypes.c_ubyte * 6)(*map(ord, "angle"))
                    mid = L.get_awake_mic_id(m.data, k1)
                    ang = L.get_awake_mic_angle(m.data, k2)
                    if 0 <= mid <= 5 and 0 <= ang <= 360:
                        self.awake = True
                        self.awake_mic, self.awake_angle = mid, ang
                        L.set_major_mic_id(mid)               # 波束转向声源
                        self._log("\n>>>>>已唤醒 主麦=%d 角度=%d°" % (mid, ang))
                elif msg == 0x08:                              # 唤醒词设置结果
                    key = (ctypes.c_ubyte * 10)(*map(ord, "errstring"))
                    r = L.whether_set_awake_word(m.data, key)
                    self.wake_set_result = r
                    self._log(">>>>>唤醒词设置%s (%d)"
                              % ("成功" if r == 0 else "失败", r))

            elif mod == 0x03 and msg == 0x01:                  # 系统状态
                key = (ctypes.c_ubyte * 7)(*map(ord, "status"))
                st = L.whether_set_succeed(m.data, key)
                ver = (ctypes.c_char * 40)()
                L.get_protocol_version(m.data, ver)
                self._log(">>>>>麦克风%s 软件:%s 协议:%s" % (
                    "正常工作" if st == 0 else "正在启动",
                    L.get_software_version().decode("utf-8", "ignore"),
                    ver.value.decode("utf-8", "ignore")))
                if st == 1:
                    L.send_resource_info(CFG_TXT, 0)
                else:
                    ctypes.c_int.in_dll(L, "is_boot").value = 1

            elif mod == 0x04:                                  # 固件下发
                if msg == 0x05:
                    self._log(">>>>>上传固件 system.tar ...")
                    L.send_resource(m.data, SYS_TAR, 1)
                elif msg == 0x04:
                    L.whether_upgrade_succeed(m.data)
                    self._log(">>>>>固件下发完成")
                elif msg in (0x01, 0x03):
                    L.whether_set_resource_info(m.data)
        except Exception as e:
            print("回调异常:", e)
        return 0

    # ---------------- 开机 ----------------
    def boot(self, major_mic=0, timeout=30):
        L = self.lib
        L.hid_close()
        if not L.hid_open():
            sys.exit("无法打开麦克风设备。\n"
                     "  装 udev 规则免 sudo： sudo cp %s /etc/udev/rules.d/ && "
                     "sudo udevadm control --reload && sudo udevadm trigger\n"
                     "  或直接 sudo 运行" % os.path.join(RES, "xf_mic.rules"))
        self._opened = True
        self._log("hid_open ok")

        ps = CFUNCTYPE(c_int, POINTER(c_uint8), c_int)
        pr = CFUNCTYPE(c_int, POINTER(c_uint8), c_int)
        pb = CFUNCTYPE(c_int, business_msg_t)
        pe = CFUNCTYPE(None, c_char_p)
        cbs = [ps(L.send_to_usb_device), pr(L.recv_from_usb_device),
               pb(self._callback), pe(lambda e: print("Err:", e.decode("utf-8", "ignore")))]
        self._keep = cbs
        L.protocol_proc_init.argtypes = [ps, pr, pb, pe]
        L.protocol_proc_init.restype = c_int
        if L.protocol_proc_init(*cbs) != 0:
            sys.exit("protocol_proc_init 失败")

        is_boot = ctypes.c_int.in_dll(L, "is_boot")
        L.get_system_status()
        self._log("等待麦克风启动（拔插后需重新下发固件，约10~30秒）...")
        t0 = time.time()
        while not is_boot.value and time.time() - t0 < timeout:
            time.sleep(0.1)
        if not is_boot.value:
            sys.exit("启动超时：检查 USB 连接与 res/ 下的固件文件")
        self._log("麦克风已启动")

        ret = L.set_major_mic_id(major_mic)     # 手动设波束方向，可跳过唤醒词
        self._log("主麦 id=%d (返回%d, 0=成功)" % (major_mic, ret))
        time.sleep(0.3)
        return True

    def set_wake_word(self, word, threshold=None, timeout=5):
        """
        设置自定义唤醒词（离线，麦克风固件内识别，不需要任何授权）。
        word:      4~6个汉字，或不超过2个单词的英文
        threshold: 唤醒门限 nCM，越大越严格越不易误唤醒（手册示例用600）
        返回 True/False
        """
        n = len(word)
        if not (4 <= n <= 6):
            self._log("警告: 唤醒词建议4~6个汉字，当前%d字，唤醒率可能受影响" % n)
        arg = word if threshold is None else "%s；nCM:%d" % (word, threshold)
        self.wake_set_result = None
        ret = self.lib.set_awake_word(arg.encode("utf-8"))
        if ret != 0:
            self._log("set_awake_word 调用失败 ret=%d (-3=麦克风未启动)" % ret)
            return False
        t0 = time.time()                       # 结果异步回调在 0x02/0x08
        while self.wake_set_result is None and time.time() - t0 < timeout:
            time.sleep(0.1)
        if self.wake_set_result is None:
            self._log("唤醒词设置结果未返回（可能仍已生效）")
            return True
        return self.wake_set_result == 0

    def wait_wake(self, timeout=30, word="快速出发"):
        self.awake = False                     # 每次等待前清标志
        self._log("请说唤醒词【%s】%s..."
                  % (word, "" if timeout is None else "(%ds内)" % timeout))
        t0 = time.time()
        while not self.awake:
            if timeout is not None and time.time() - t0 > timeout:
                return False
            time.sleep(0.05)
        return True

    # ---------------- 录音 ----------------
    def start(self, mode="deno", channel=0, dump=0):
        self.mode, self.channel, self._dump = mode, channel, dump
        self.deno, self.ori = bytearray(), bytearray()
        if mode in ("deno", "both"):
            self.lib.start_to_record_denoised_sound()
        if mode in ("ori", "both"):
            self.lib.start_to_record_original_sound()
        self._recording = True

    def stop(self):
        if not self._recording:
            return
        if self.mode in ("deno", "both"):
            self.lib.finish_to_record_denoised_sound()
        if self.mode in ("ori", "both"):
            self.lib.finish_to_record_original_sound()
        self._recording = False
        time.sleep(0.4)

    def record(self, seconds, mode="deno", channel=0):
        """录指定秒数，返回 16bit 单声道 PCM bytes"""
        self.start(mode, channel)
        time.sleep(seconds)
        self.stop()
        if mode == "ori":
            return deinterleave(self.ori, channel, shift=self.shift)
        return bytes(self.deno)

    def close(self):
        if self._opened:
            self.lib.hid_close()
            self._opened = False


# ---------------- CLI ----------------
def main():
    ap = argparse.ArgumentParser(description="讯飞六麦阵列录音(独立包)")
    ap.add_argument("-m", "--mode", choices=["deno", "ori", "both"], default="deno",
                    help="deno=降噪(默认) ori=原始/关降噪 both=都录")
    ap.add_argument("-c", "--channel", type=int, default=0, help="原始音频通道 0-5麦 6-7参考")
    ap.add_argument("--all-ch", action="store_true", help="原始音频6个麦都存")
    ap.add_argument("--shift", type=int, default=16, help="32→16bit右移位数(声音小就调到12/8)")
    ap.add_argument("-d", "--duration", type=int, default=5, help="录音秒数")
    ap.add_argument("-o", "--output", default="out", help="输出前缀")
    ap.add_argument("--mic", type=int, default=0, help="主麦/波束方向 0~5")
    ap.add_argument("--wait-wake", action="store_true", help="等唤醒词再录")
    ap.add_argument("--set-wake", metavar="词",
                    help="设置自定义唤醒词(4~6个汉字)，如 --set-wake 小飞小飞")
    ap.add_argument("--wake-ncm", type=int, default=None,
                    help="唤醒门限nCM，越大越严格(手册示例600)")
    ap.add_argument("--wake-word", default="快速出发", help="等待时提示用的唤醒词")
    ap.add_argument("--wake-timeout", type=int, default=30)
    ap.add_argument("--dump", action="store_true", help="打印前3帧字节")
    ap.add_argument("--level", action="store_true", help="只看实时音量(定VAD阈值)")
    ap.add_argument("--boot-only", action="store_true", help="只开机不录音")
    ap.add_argument("--lib", default=None, help="指定 libhid_lib.so 路径")
    a = ap.parse_args()

    mic = MicArray(a.lib, shift=a.shift)
    mic.boot(major_mic=a.mic)
    if a.set_wake:
        ok = mic.set_wake_word(a.set_wake, a.wake_ncm)
        print("设置唤醒词【%s】: %s" % (a.set_wake, "成功" if ok else "失败"))
        a.wake_word = a.set_wake

    if a.boot_only:
        mic.close()
        print("开机完成。")
        return 0

    if a.wait_wake:
        if mic.wait_wake(a.wake_timeout, a.wake_word):
            print("唤醒成功，主麦已转向声源(角度%d°)" % mic.awake_angle)
        else:
            print("未唤醒，继续用手动主麦")

    desc = {"deno": "降噪 16k/16bit/单声道",
            "ori":  "原始 16k/32bit/8通道 取ch%d —— 已关降噪" % a.channel,
            "both": "降噪+原始 同时录"}[a.mode]
    print("\n模式: %s" % desc)
    mic.start(a.mode, a.channel, 3 if a.dump else 0)

    dur = 10 ** 9 if a.level else a.duration
    print("实时音量（Ctrl-C 结束）" if a.level else "录音 %d 秒，请说话..." % dur)
    t0, peak, floor = time.time(), 0.0, 1e9
    try:
        while time.time() - t0 < dur:
            v = mic.rms
            peak = max(peak, v)
            if 0 < v < floor:
                floor = v
            print("\r音量 %6.0f |%-50s|" % (v, "#" * min(50, int(v / 20))),
                  end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print()
    mic.stop()
    mic.close()

    print("\n--- 结果 ---")
    if peak > 0:
        print("底噪 %.0f  峰值 %.0f  → VAD阈值可取 %.0f"
              % (0 if floor > 1e8 else floor, peak, (floor + peak) / 3))
    if mic.deno:
        p = a.output + "_deno.wav"
        write_wav(p, bytes(mic.deno))
        print("降噪: %s (%.2fs)" % (p, len(mic.deno) / float(RATE * 2)))
    if mic.ori:
        ns = len(mic.ori) // (ORI_CH * 4)
        for ch in (range(6) if a.all_ch else [a.channel]):
            mono = deinterleave(mic.ori, ch, shift=a.shift)
            p = "%s_ori_ch%d.wav" % (a.output, ch)
            write_wav(p, mono)
            print("原始ch%d: %s (%.2fs, RMS %.0f)" % (ch, p, ns / float(RATE), rms_i16(mono)))
        print("声音太小就减小 --shift（默认16，可试12或8）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
