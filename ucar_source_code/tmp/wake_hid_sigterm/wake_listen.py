#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
唤醒 → VAD录音 → 讯飞在线识别 → 双槽位纠错   单文件版

完整链路：
    开机(下发固件) → 设唤醒词"小飞小飞" → 等唤醒 → 说完自动停
    → 存 wav → 讯飞识别 → 拼音模糊纠错出两个槽位

目标句式：
    小飞小飞，前往物品领取区，取得{类别1}，放置在对应仓库，
    并领取仿真环境中需要的{类别2}放置在对应仓库
    两个槽位候选相同，靠「锚点词 + 出现顺序」区分先后。

用法：
    python3 wake_listen.py                      # 单次：唤醒一次录一次并识别
    python3 wake_listen.py --loop               # 循环：一直等唤醒（比赛用）
    python3 wake_listen.py -w 小飞小飞 --ncm 600 # 换唤醒词/调门限
    python3 wake_listen.py --no-asr             # 只录音存 wav，不联网
    python3 wake_listen.py -f test.wav          # 不录音，识别现成 wav
    python3 wake_listen.py --text "…"           # 不联网，纯测纠错逻辑
    python3 wake_listen.py -t 300               # 手动指定 VAD 阈值，跳过校准

    注：识别现在是默认行为，老版的 --asr 开关保留但不再有作用。
    python3 wake_listen.py --calib              # 先测底噪，自动定VAD阈值

★ 时间戳：
    讯飞鉴权对时间敏感（本地时钟与服务器差超过 5 分钟直接 401
    "HMAC signature cannot be verified"）。本脚本不信任板子的 RTC，
    每次建立 ws 连接前现 curl 一次讯飞的响应头，拿 date 原样拼进签名。
    见 xf_date()。

密钥：同目录 xf_key.conf，或环境变量 XF_APPID / XF_APIKEY / XF_APISECRET

依赖：
    pip3 install websocket-client pypinyin
    同目录需要 mic_array.py（麦阵列驱动）
"""

import os
import signal
import ssl
import sys
import time
import json
import wave
import hmac
import base64
import hashlib
import argparse
import threading
import difflib
import subprocess
from urllib.parse import urlencode, urlparse
from wsgiref.handlers import format_date_time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from mic_array import MicArray, write_wav, RATE

# ==========================================================================
#  ★ 句式与候选词配置（按比赛改这里）
# ==========================================================================
CANDIDATES = ["食品", "日用品", "电子产品"]

# 两个槽位。anchors_before/after 是句子里槽位前后的定位词，
# 允许写多个（含常见误识别写法），命中任一即可。
SLOT_DEFS = [
    {
        "name": "取件类别",
        "candidates": CANDIDATES,
        "anchors_before": ["取得", "取的", "去得", "领取区"],
        "anchors_after": ["放置", "放在", "对应仓库"],
    },
    {
        "name": "仿真类别",
        "candidates": CANDIDATES,
        "anchors_before": ["需要的", "需要", "仿真环境", "仿真"],
        "anchors_after": ["放置", "放在", "对应仓库"],
    },
]

SENTENCE_TEMPLATE = ("小飞小飞，前往物品领取区，取得{取件类别}，放置在对应仓库，"
                     "并领取仿真环境中需要的{仿真类别}放置在对应仓库")

# 相似度阈值：0~1。太高会漏识别，太低会乱认。
MATCH_THRESHOLD = 0.62

# 注意：唤醒由麦阵列硬件完成，录音是从唤醒之后才开始的，
# 句子里通常不含"小飞小飞"，所以这里不再做唤醒词文本校验。

# ==========================================================================
#  讯飞接口常量
# ==========================================================================
IAT_URL    = "wss://iat-api.xfyun.cn/v2/iat"
FRAME_SIZE = 1280             # 官方建议 PCM 每帧 1280 字节
INTERVAL   = 0.04             # 每帧间隔 40ms
ST_FIRST, ST_CONT, ST_LAST = 0, 1, 2

# 取时间戳用的地址：用讯飞自己的域名，保证和校验签名的是同一套时钟
TIME_URL = "https://www.xfyun.cn"

try:
    import websocket
except ImportError:
    sys.exit("缺少依赖： pip3 install websocket-client")

try:
    from pypinyin import lazy_pinyin
    HAS_PINYIN = True
except ImportError:
    HAS_PINYIN = False


# ==========================================================================
#  ★ 时间戳：每次发请求前现取讯飞服务器时间
# ==========================================================================
def xf_date(timeout=3, verbose=False):
    """
    curl -sI https://www.xfyun.cn | grep -i '^date:'
    拿到的就是 RFC1123 GMT 格式（'Wed, 06 Aug 2026 03:04:05 GMT'），
    正好是签名要的格式，不用解析不用转换，原样返回。

    两个坑：
      1. HTTP/2 的响应头是小写 'date:'，HTTP/1.1 是 'Date:'，要不分大小写；
         有跳转时会有多个响应头块，取最后一个（最终响应）的。
      2. 板子时钟错得离谱（比如还停在 1970）时，TLS 会因为"证书尚未生效"
         握手失败 —— 这正是最需要校时的场景。所以失败后用 -k 再试一次，
         只读一个 date 头，最坏情况是被喂假时间导致签名失败，没有别的风险。
    """
    for insecure in (False, True):
        cmd = ["curl", "-sI", "--max-time", str(timeout)]
        if insecure:
            cmd.append("-k")
        cmd.append(TIME_URL)
        try:
            out = subprocess.check_output(cmd, stderr=subprocess.DEVNULL)
        except Exception:
            continue
        for line in reversed(out.decode("utf-8", "ignore").splitlines()):
            if line.lower().startswith("date:"):
                # 只切第一个冒号，后面 03:04:05 里的冒号不能动
                d = line.split(":", 1)[1].strip()
                if d:
                    if verbose:
                        print("服务器时间 %s%s" % (d, "  (跳过证书校验)" if insecure else ""))
                    return d
    # 实在拿不到就退回本地时钟，让它自己去撞 401
    if verbose:
        print("[警告] 取不到讯飞服务器时间，退回本地时钟（若本地表不准会鉴权失败）")
    return format_date_time(time.time())


# ==========================================================================
#  密钥读取：同目录 xf_key.conf > 环境变量
# ==========================================================================
def load_keys():
    conf = os.path.join(HERE, "xf_key.conf")
    keys = {}
    if os.path.exists(conf):
        with open(conf, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                keys[k.strip().upper()] = v.strip().strip('"').strip("'")
    appid  = keys.get("APPID")     or os.environ.get("XF_APPID", "")
    apikey = keys.get("APIKEY")    or os.environ.get("XF_APIKEY", "")
    secret = keys.get("APISECRET") or os.environ.get("XF_APISECRET", "")

    if not (appid and apikey and secret) or "在这里填" in (appid + apikey + secret):
        sys.exit(
            "缺少密钥。请在本脚本同目录创建 xf_key.conf：\n"
            "    cp xf_key.conf.template xf_key.conf\n"
            "    vi xf_key.conf      # 填入 APPID / APIKEY / APISECRET\n"
            "或设置环境变量 XF_APPID / XF_APIKEY / XF_APISECRET\n"
            "获取地址： https://console.xfyun.cn/app/myapp")
    return appid, apikey, secret


# ==========================================================================
#  拼音模糊匹配（核心纠错）
# ==========================================================================
def to_py(s):
    """转拼音（无声调）。没装 pypinyin 就退化成原文比较。"""
    return " ".join(lazy_pinyin(s)) if HAS_PINYIN else s


def sim(a, b):
    return difflib.SequenceMatcher(None, a, b).ratio()


def find_matches(text, candidates, threshold):
    """
    在 text 中找出所有候选词的出现位置（拼音模糊）。
    返回按位置排序、互不重叠的 [(pos, cand, score), ...]
    """
    hits = []
    for cand in candidates:
        cpy, n = to_py(cand), len(cand)
        for i in range(len(text)):
            for w in (n - 1, n, n + 1):          # 容忍多字/少字
                if w <= 0 or i + w > len(text):
                    continue
                seg = text[i:i + w]
                score = 1.0 if seg == cand else sim(to_py(seg), cpy)
                if score >= threshold:
                    hits.append((i, i + w, cand, score))

    # 贪心去重叠：优先保留分数高的
    hits.sort(key=lambda h: (-h[3], h[0]))
    chosen, used = [], []
    for s, e, cand, sc in hits:
        if any(not (e <= us or s >= ue) for us, ue in used):
            continue
        used.append((s, e))
        chosen.append((s, cand, sc))
    chosen.sort(key=lambda c: c[0])              # 按出现位置排序
    return chosen


def anchor_pos(text, anchors):
    """返回锚点词在文中的位置（模糊），找不到返回 -1。"""
    best_pos, best_score = -1, 0.0
    for a in anchors:
        n = len(a)
        for i in range(len(text) - n + 1):
            seg = text[i:i + n]
            score = 1.0 if seg == a else sim(to_py(seg), to_py(a))
            if score > best_score and score >= 0.75:
                best_pos, best_score = i, score
    return best_pos


def parse_slots(text):
    """
    双槽位解析。两个槽位候选相同，必须靠锚点+顺序区分：

      策略1（主）—— 用锚点把句子切成两段，各段内独立匹配
                     段1 = "取得" 之后 ~ 第二个锚点之前
                     段2 = "需要的/仿真" 之后
      策略2（备）—— 锚点找不到时，全句找候选，按出现顺序分配

    注意：句中"物品领取区"的"物品"容易被误认成"电子产品"，
    所以段1必须从"取得"之后开始，避开它。
    """
    results = []

    a1 = anchor_pos(text, SLOT_DEFS[0]["anchors_before"])   # "取得"
    a2 = anchor_pos(text, SLOT_DEFS[1]["anchors_before"])   # "需要的"/"仿真"

    seg1 = seg2 = None
    if a1 != -1:
        end = a2 if (a2 != -1 and a2 > a1) else len(text)
        seg1 = (a1, text[a1:end])
    if a2 != -1:
        seg2 = (a2, text[a2:])

    def pick(seg):
        """在片段里取分数最高的候选。"""
        if not seg:
            return None, 0.0
        off, sub = seg
        hits = find_matches(sub, CANDIDATES, MATCH_THRESHOLD)
        if not hits:
            return None, 0.0
        best = max(hits, key=lambda h: h[2])
        return best[1], best[2]

    if seg1 or seg2:
        v1, s1 = pick(seg1)
        v2, s2 = pick(seg2)
        # 锚点缺失导致某段为空时，退回全句顺序分配补齐
        if v1 is None or v2 is None:
            hits = find_matches(text, CANDIDATES, MATCH_THRESHOLD)
            # 去掉落在"领取区"之前的误匹配
            start = a1 if a1 != -1 else 0
            hits = [h for h in hits if h[0] >= start]
            if v1 is None and v2 is None and len(hits) >= 2:
                v1, s1 = hits[0][1], hits[0][2]
                v2, s2 = hits[1][1], hits[1][2]
            elif v1 is None and len(hits) >= 1 and v2 is not None:
                v1, s1 = hits[0][1], hits[0][2]
            elif v2 is None and len(hits) >= 1:
                v2, s2 = hits[-1][1], hits[-1][2]
        results.append((SLOT_DEFS[0]["name"], v1, s1))
        results.append((SLOT_DEFS[1]["name"], v2, s2))
    else:
        # 完全没锚点：全句按顺序取前两个
        hits = find_matches(text, CANDIDATES, MATCH_THRESHOLD)
        for i, slot in enumerate(SLOT_DEFS):
            if i < len(hits):
                results.append((slot["name"], hits[i][1], hits[i][2]))
            else:
                results.append((slot["name"], None, 0.0))

    return results


# ==========================================================================
#  讯飞识别
# ==========================================================================
class XfAsr(object):
    def __init__(self, appid, apikey, apisecret,
                 language="zh_cn", accent="mandarin", eos=3000, timeout=25):
        self.appid, self.apikey, self.apisecret = appid, apikey, apisecret
        self.language, self.accent = language, accent
        self.eos, self.timeout = eos, timeout
        self._res, self._err, self._done = {}, None, threading.Event()

    def _url(self, verbose=False):
        u = urlparse(IAT_URL)
        # ★ 关键：不用本地时钟，现去讯飞取一次服务器时间，原样拼进签名。
        #   签名里的 date 和 query 里的 date 必须是同一个值。
        date = xf_date(verbose=verbose)
        origin = "host: {}\ndate: {}\nGET {} HTTP/1.1".format(u.netloc, date, u.path)
        sha = hmac.new(self.apisecret.encode(), origin.encode(), hashlib.sha256).digest()
        sig = base64.b64encode(sha).decode()
        auth_origin = ('api_key="{}", algorithm="hmac-sha256", '
                       'headers="host date request-line", signature="{}"').format(self.apikey, sig)
        auth = base64.b64encode(auth_origin.encode()).decode()
        return IAT_URL + "?" + urlencode({"authorization": auth, "date": date, "host": u.netloc})

    @staticmethod
    def load_pcm(path):
        """从 wav/裸pcm 读出 PCM bytes"""
        if path.lower().endswith(".wav"):
            with wave.open(path, "rb") as w:
                ch, wd, rate = w.getnchannels(), w.getsampwidth(), w.getframerate()
                data = w.readframes(w.getnframes())
            if ch != 1 or wd != 2 or rate not in (8000, 16000):
                sys.exit("音频格式不符（当前 {}声道/{}bit/{}Hz），需要 单声道/16bit/16k或8k。\n"
                         "转换： ffmpeg -i in.wav -ac 1 -ar 16000 -sample_fmt s16 out.wav"
                         .format(ch, wd * 8, rate))
            return data, rate
        with open(path, "rb") as f:
            return f.read(), 16000

    # --- 回调：用可变参数，兼容 websocket-client 各版本 ---
    def _on_msg(self, *args):
        ws, message = (args[0], args[1]) if len(args) >= 2 else (None, args[0])
        try:
            m = json.loads(message)
        except Exception as e:
            self._err = "返回解析失败: %s" % e
            self._done.set()
            return
        if m.get("code") != 0:
            self._code = m.get("code")
            self._err = "讯飞错误 code=%s msg=%s sid=%s" % (
                m.get("code"), m.get("message"), m.get("sid"))
            if ws:
                ws.close()
            self._done.set()
            return
        d = m.get("data", {})
        r = d.get("result", {})
        sn = r.get("sn", 0)
        piece = "".join(w["cw"][0]["w"] for w in r.get("ws", []) if w.get("cw"))
        if r.get("pgs") == "rpl":                 # 动态修正：替换旧结果
            rg = r.get("rg", [])
            if len(rg) == 2:
                for i in range(rg[0], rg[1] + 1):
                    self._res.pop(i, None)
        self._res[sn] = piece
        if d.get("status") == 2:                  # 最后一块
            if ws:
                ws.close()
            self._done.set()

    def _on_err(self, *args):
        self._err = str(args[-1])
        self._done.set()

    def _on_close(self, *args):
        self._done.set()

    def _on_open(self, ws, pcm, rate):
        def run():
            try:
                status, off = ST_FIRST, 0
                while True:
                    chunk = pcm[off:off + FRAME_SIZE]
                    off += FRAME_SIZE
                    if not chunk:
                        status = ST_LAST
                    frame = {"data": {"status": status,
                                      "format": "audio/L16;rate=%d" % rate,
                                      "encoding": "raw",
                                      "audio": base64.b64encode(chunk).decode()}}
                    if status == ST_FIRST:
                        frame["common"] = {"app_id": self.appid}
                        frame["business"] = {"language": self.language, "domain": "iat",
                                             "accent": self.accent, "eos": self.eos,
                                             "ptt": 0}      # 不加标点，便于匹配
                        status = ST_CONT
                    ws.send(json.dumps(frame))
                    if frame["data"]["status"] == ST_LAST:
                        break
                    time.sleep(INTERVAL)
            except Exception as e:
                self._err = "发送失败: %s" % e
                try:
                    ws.close()
                except Exception:
                    pass
                self._done.set()
        threading.Thread(target=run, daemon=True).start()

    def _once(self, pcm, rate, verbose):
        self._res, self._err, self._code = {}, None, None
        self._done.clear()
        ws = websocket.WebSocketApp(self._url(verbose=verbose),
                                    on_message=self._on_msg,
                                    on_error=self._on_err,
                                    on_close=self._on_close)
        ws.on_open = lambda w: self._on_open(w, pcm, rate)
        threading.Thread(target=ws.run_forever,
                         kwargs={"sslopt": {"cert_reqs": ssl.CERT_NONE}},
                         daemon=True).start()
        if not self._done.wait(self.timeout):
            return None, "识别超时(%ds)，检查网络" % self.timeout
        if self._err:
            return None, self._err
        return "".join(self._res[k] for k in sorted(self._res)), None

    def recognize(self, pcm, rate=16000, verbose=False):
        """pcm 是 16bit 单声道 bytes。返回 (文本, 错误)，错误为 None 表示成功。"""
        if not pcm:
            return None, "音频为空"
        text, err = self._once(pcm, rate, verbose)
        # 鉴权失败多半还是时间问题（比如刚好卡在秒级边界），重取一次时间再试
        if err and ("signature" in err.lower() or "401" in err or
                    str(self._code) in ("10105", "10005")):
            if verbose:
                print("鉴权被拒，重新取一次服务器时间后重试...")
            text, err = self._once(pcm, rate, verbose)
        return text, err


# ==========================================================================
#  麦阵列：校准 / VAD 录音
# ==========================================================================
def calibrate(mic, seconds=3):
    """测底噪，返回建议阈值"""
    print("校准中：请保持安静 %d 秒..." % seconds)
    vals = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        vals.append(mic.rms)
        time.sleep(0.05)
    vals = [v for v in vals if v > 0]
    if not vals:
        print("没采到数据，用默认阈值 300")
        return 300
    floor = sum(vals) / len(vals)
    peak = max(vals)
    thr = max(80, int(floor * 4))
    print("底噪均值 %.0f 峰值 %.0f → 阈值取 %d" % (floor, peak, thr))
    return thr


def vad_record(mic, threshold, start_pos, silence_sec=2.0, max_sec=15.0,
               verbose=True):
    """
    基于【持续录音】的VAD：录音从头就没停过，start_pos 是唤醒时刻回溯 preroll 后的位置。
    超阈值判定说话，静音 silence_sec 秒结束。
    返回 PCM bytes（16bit 单声道）；没说话返回 None。
    """
    tick = 0.05
    started = False
    silence = 0.0
    t0 = time.time()

    while True:
        v = mic.rms
        if verbose:
            print("\r音量 %5.0f %s" % (v, "#" * min(40, int(v / 20))),
                  end="", flush=True)

        if not started:
            if v > threshold:
                started = True
                if verbose:
                    print("\n检测到说话...")
            elif time.time() - t0 > max_sec:
                if verbose:
                    print("\n等待超时，没人说话")
                return None
        else:
            if v > threshold:
                silence = 0.0
            else:
                silence += tick
                if silence >= silence_sec:
                    break
            if time.time() - t0 > max_sec:
                if verbose:
                    print("\n达到最长录音")
                break
        time.sleep(tick)

    if verbose:
        print()
    pcm = bytes(mic.deno[start_pos:])
    # 去掉尾部多余静音，留 0.3 秒
    trim = int((silence - 0.3) * RATE) * 2
    if trim > 0 and len(pcm) > trim:
        pcm = pcm[:-trim]
    return pcm if pcm else None


# ==========================================================================
#  结果输出
# ==========================================================================
def report(raw, as_json=False, err=None):
    """打印识别+槽位结果，返回 (ok, slots_dict)"""
    slots = parse_slots(raw or "")
    ok = all(v for _, v, _ in slots)
    sd = {n: v for n, v, _ in slots}

    if as_json:
        out = {"raw": raw or "", "ok": ok, "slots": sd}
        if err:
            out["error"] = err
        print(json.dumps(out, ensure_ascii=False))
        return ok, sd

    print("\n" + "=" * 56)
    print("原始识别 : %s" % (raw if raw else "(空)"))
    print("-" * 56)
    for name, val, score in slots:
        if val:
            print("  %-8s → %-8s (相似度 %.2f)" % (name, val, score))
        else:
            print("  %-8s → 未识别   (低于阈值 %.2f)" % (name, MATCH_THRESHOLD))
    print("-" * 56)
    if ok:
        print("规范化 : " + SENTENCE_TEMPLATE.format(**sd))
        print("结构化 : " + json.dumps(sd, ensure_ascii=False))
    else:
        print("槽位未全部命中 —— 建议重说，或调低 MATCH_THRESHOLD")
    print("（唤醒由麦阵列硬件确认，录音里不含唤醒词，故不做文本校验）")
    print("=" * 56)
    return ok, sd


# ==========================================================================
#  主流程
# ==========================================================================
def main():
    ap = argparse.ArgumentParser(description="唤醒→VAD录音→讯飞识别→双槽位纠错")
    # --- 唤醒 / 录音 ---
    ap.add_argument("-w", "--wake-word", "--set-wake", default="小飞小飞",
                    dest="wake_word",
                    help="唤醒词(4~6汉字)，启动时自动设置进麦克风。默认 小飞小飞")
    ap.add_argument("--ncm", type=int, default=None, help="唤醒门限,越大越严格(如600)")
    ap.add_argument("--no-set-wake", action="store_true", help="不重设唤醒词,用麦克风现有的")
    ap.add_argument("-t", "--threshold", type=int, default=None,
                    help="VAD阈值,不填则自动校准")
    ap.add_argument("-s", "--silence", type=float, default=2.0, help="静音几秒结束")
    ap.add_argument("--max", type=float, default=15.0, help="单次最长录音秒数")
    ap.add_argument("--preroll", type=float, default=0.5,
                    help="唤醒时刻往前多录几秒,防止吃掉开头字(默认0.5)")
    ap.add_argument("--mic", type=int, default=0, help="默认主麦(唤醒后会自动转向声源)")
    ap.add_argument("-o", "--output", default="heard.wav")
    ap.add_argument("--loop", action="store_true", help="循环等待唤醒(比赛用)")
    ap.add_argument("--calib", action="store_true", help="强制先校准阈值")
    # --- 识别 ---
    ap.add_argument("--no-asr", action="store_true", help="只录音存wav,不联网识别")
    ap.add_argument("--asr", action="store_true",
                    help=argparse.SUPPRESS)   # 旧版开关,识别现在是默认行为,留着兼容老脚本
    ap.add_argument("-f", "--file", help="不录音,直接识别已有 wav/pcm")
    ap.add_argument("--text", help="不联网,直接用这段文字测纠错逻辑")
    ap.add_argument("--json", action="store_true", help="每次只输出一行JSON,便于上层程序调用")
    ap.add_argument("--eos", type=int, default=3000, help="讯飞端点检测静音时长ms")
    a = ap.parse_args()

    if not HAS_PINYIN and not a.json:
        print("[提示] 未装 pypinyin，同音字纠错不可用： pip3 install pypinyin\n")

    # ---- 离线支路：纯文本测纠错 ----
    if a.text:
        report(a.text, a.json)
        return 0

    asr = None
    if not a.no_asr:
        appid, apikey, apisecret = load_keys()
        asr = XfAsr(appid, apikey, apisecret, eos=a.eos)

    # ---- 支路：识别现成 wav，不碰麦阵列 ----
    if a.file:
        if asr is None:
            sys.exit("--no-asr 和 -f 一起用没有意义")
        pcm, rate = XfAsr.load_pcm(a.file)
        raw, err = asr.recognize(pcm, rate, verbose=not a.json)
        if err and not a.json:
            print("识别失败:", err)
        report(raw, a.json, err)
        return 0 if not err else 1

    # ---- 主流程：唤醒 → 录音 → 识别 ----
    mic = MicArray()

    def release_microphone():
        """Stop recording and release the HID handle exactly once."""
        try:
            mic.stop()
        finally:
            mic.close()

    def handle_sigterm(_signum, _frame):
        print("收到 SIGTERM，释放阵列麦 HID 设备...")
        release_microphone()
        raise SystemExit(0)

    previous_sigterm = signal.signal(signal.SIGTERM, handle_sigterm)
    mic.boot(major_mic=a.mic)

    if not a.no_set_wake:
        ok = mic.set_wake_word(a.wake_word, a.ncm)
        print("唤醒词【%s】设置%s" % (a.wake_word, "成功" if ok else "失败(仍可试试)"))

    # 【关键】从一开始就持续录音，绝不中途 stop。
    # 否则"唤醒→开始录音"之间说的话会整段丢失（会吃掉"前往"这类开头词）。
    mic.start("deno")
    time.sleep(0.3)

    thr = a.threshold
    if thr is None or a.calib:
        thr = calibrate(mic)

    preroll_bytes = int(a.preroll * RATE) * 2
    keep_bytes = int(3.0 * RATE) * 2          # 待机时缓冲只留最近3秒，防止无限增长

    n = 0
    try:
        while True:
            n += 1
            print("\n" + "=" * 46)
            print("请说唤醒词【%s】..." % a.wake_word)
            mic.awake = False
            while not mic.awake:                       # 等待期间持续裁剪缓冲
                if len(mic.deno) > keep_bytes:
                    del mic.deno[:-keep_bytes]
                time.sleep(0.05)

            # 唤醒瞬间的缓冲位置，往前回溯 preroll，保住紧跟唤醒词后面的字
            start_pos = max(0, len(mic.deno) - preroll_bytes)
            print("唤醒！角度 %d°，请说指令..." % mic.awake_angle)

            pcm = vad_record(mic, thr, start_pos, a.silence, a.max)
            if not pcm:
                print("没录到有效语音")
                if not a.loop:
                    break
                continue

            path = a.output if not a.loop else "%s_%d%s" % (
                os.path.splitext(a.output)[0], n, os.path.splitext(a.output)[1])
            write_wav(path, pcm)
            print("已保存 %s (%.2f 秒)" % (path, len(pcm) / float(RATE * 2)))

            if asr is not None:
                print("上传识别中...")
                # PCM 直接进程内送走，不再落盘往返、不再起子进程
                raw, err = asr.recognize(pcm, RATE, verbose=not a.json)
                if err:
                    print("识别失败:", err)
                report(raw, a.json, err)

            if not a.loop:
                break
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        release_microphone()
        signal.signal(signal.SIGTERM, previous_sigterm)
    return 0


if __name__ == "__main__":
    sys.exit(main())
