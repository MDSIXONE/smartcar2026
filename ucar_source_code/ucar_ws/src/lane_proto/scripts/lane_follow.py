#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
lane_follow.py — 直道巡线测试原型 (python2/3 兼容, Melodic 下用 python2 跑)
====================================================================
管线: V4L2 直连相机(独占 /dev/video0, 不走 ROS 相机驱动)
      -> cv2.remap 鱼眼去畸变(预生成映射表, 不依赖 cv2.fisheye)
      -> trackseg 固化分割网(ctypes) 得白线掩码(类2)
      -> 与触发区模板比对(模板由 tools/make_template.py 从居中直道帧生成,
         关于主点严格对称, 楔形按车道半宽比例收敛于消失点):
           白线侵入【左】触发区 -> 车应【右】转 (angular.z < 0)
           白线侵入【右】触发区 -> 车应【左】转 (angular.z > 0)
         侵入度 I∈[0,1] 逐行归一化, 浅侵入轻修正、侵满打满舵
      -> 发 /cmd_vel

前进速度就是 linear_speed:
  linear_speed:=0     原地只转车体对准 —— **先跑这个**, 车不会跑出场地,
                      用来验证符号/镜像/增益对不对; 进死区会打印"已对准"
  linear_speed:=0.1   边走边修, 正常巡线

安全:
  - dry_run:=true(默认) 只打印决策不发速度; 确认无误再 dry_run:=false
  - 节点退出/异常一律发零速; 另有三重兜底: /lane_proto/estop 急停话题、
    max_runtime 运行时长上限、APPROACH 测距超时
  - ⚠ 别按 Ctrl-Z! 那只会挂起 roslaunch, 节点在独立进程组里照跑不误,
    结果是"遥控器冻住、车还在开"。按 Ctrl-C; 已经按了 Ctrl-Z 就 fg
    回来再 Ctrl-C, 或者跑 scripts/stop_lane.sh
  - 取帧线程独立跑, 主循环永远拿最新帧(不吃 V4L2 缓冲里的陈帧)
  - 底盘角速度死区: |az| 小于 min_turn_speed 会被 MCU 吃掉,
    所以映射的下端就从该值起(死区内则严格给 0, 不会来回抖)

岔路 / 分支 (is_fork 三态, 命令行不用加引号):
  false  横线 = 终点, 老行为
  true   第一次检出横线 -> 原地转 fork_turn_deg 度对准一条臂
  yolo   起跑序列(车摆在黄线后面, 大致对着赛道就行):
           1) ALIGN  用**黄线**把车前后顶到固定位置(视觉伺服, 只前后动)
           2) 前进 align_offset(默认 100mm) 到"黄线前 10cm"那个规定位置
              —— 这段必须盲走, 到那儿黄线已经掉出视野下边缘了
           3) 在那儿等红灯变绿, 认出箭头就**记住**, 立刻杀掉 yolo 进程
           4) 前进 start_offset(默认 300mm) 开到三岔口
           5) 按记住的方向动:
           left   -> 原地左转 yolo_turn_deg(60)° -> 巡线跑左臂 -> 横线=终点
           right  -> 原地右转 yolo_turn_deg(60)° -> 巡线跑右臂 -> 横线=终点
           straight -> 不转, 巡线往中间跑 -> 检出横线 = 到了 Y 岔口,
                       转 fork_turn_deg(-45)° 进一条支路 -> 再检出横线=终点
         三条路线一条命令跑完, 不用再按任务改参数。**红灯(stop) 和
         什么都没认出来是同一种处理: 都继续等**, 等到方向灯为止
         (兜底 yolo_wait_max 秒)
  ⚠ 喂给 yolo 的是**已翻正**的去畸变帧(self._und)。相机原始输出是镜像的,
    而这套权重是拿镜像图训的, 差一次镜像 —— 所以 yolo_swap_lr **默认 true**
    (实测左右确实反)。dump/yolo_*.jpg 就是实际喂进去的图, 拿它和日志里的
    判定对: 箭头朝左而最终判成右转, 就把 yolo_swap_lr 改回 false。

转向只有一个旋钮:
    gain = 每单位侵入深度给多少 rad/s, 大=猛。默认 1.2。
    az = min_turn_speed + gain*(深度 - min_intrusion), 上限 turn_max
调参不用重启(每秒重读一次), 跑着的时候直接:
    rosparam set /lane_follow/gain 2.0              # 更猛
    rosparam set /lane_follow/linear_speed 0.15     # 边跑边改速度

耗时诊断(每 stats_every 秒一行, stats_every:=0 关掉):
    [耗时] 取帧0.4 去畸变13.0 分割34.0 决策0.5 横线5.0 dump1.5 其他0.6 = 55.0ms
           | 忙 p50 55 p90 55 max 58  周期 111ms (9.0 fps, rate 上限 10.0, 相机 29.5 fps)
           瓶颈 分割(62%); 空转 56ms/帧 —— 是 rate 在限速, rate:=18 就能提到 18.2 fps
  三个数一起看才能定位"fps 上不去":
    忙 << 周期        -> rate 在限速, 提 rate 即可(免费的)
    周期 ≈ 1/相机fps  -> 相机就那么快, 提 rate 没用, 先看启动日志里的
                         协商结果, YUYV@640x480 常见只有 10fps, 试 fourcc:=MJPG
    忙  ≈ 周期        -> 真算不动, 优化最贵那段(通常是 分割)
  分割那段量的是 GPU 真算完的时间: .so 末尾有 cudaMemcpy D2H 会隐式同步,
  不存在"只量到提交任务"的坑。
"""
from __future__ import print_function
import os
import sys
import threading
import time
import subprocess

import math

import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trackseg import TrackSeg, IN_W, IN_H     # noqa: E402
from lane_common import (load_template, Decider, map_az,  # noqa: E402
                         curve_text, goal_block, yellow_line, yellow_mask)

try:
    text_type = unicode                    # Python 2
except NameError:
    text_type = str                        # Python 3


def format_ros_log(message, args):
    """先把 unicode/UTF-8 混排的消息格式化好, 再交给 rospy。

    rospy/logging 内部会对 ``message % args`` 再做一次格式化。Python 2 下
    只要格式串或参数里混有 unicode 与含中文的 UTF-8 字节串, 那次格式化
    就会按 ASCII 解码抛 UnicodeDecodeError。这里先统一成 unicode 格式化,
    留给 rospy 的只剩一个纯 ASCII 的 ``%s`` 替换, 不会再炸。
    """
    if not isinstance(message, text_type):
        message = message.decode("utf-8", "replace")
    if not args:
        return message.encode("utf-8")
    normalized = tuple(
        value.decode("utf-8", "replace") if isinstance(value, bytes)
        else value for value in args)
    return (message % normalized).encode("utf-8")


_rospy_loginfo = rospy.loginfo
_rospy_logwarn = rospy.logwarn
_rospy_logerr = rospy.logerr
_rospy_logerr_throttle = rospy.logerr_throttle


def _safe_ros_log(emitter, message, *args):
    emitter("%s", format_ros_log(message, args))


def lane_loginfo(message, *args):
    _safe_ros_log(_rospy_loginfo, message, *args)


def lane_logwarn(message, *args):
    _safe_ros_log(_rospy_logwarn, message, *args)


def lane_logerr(message, *args):
    _safe_ros_log(_rospy_logerr, message, *args)


def lane_logerr_throttle(period, message, *args):
    _rospy_logerr_throttle(period, "%s", format_ros_log(message, args))


# 本节点所有日志都走上述包装: Melodic Python 2 下不得把
# unicode/UTF-8 混排参数直接交给 rospy。调用方写法不变。
rospy.loginfo = lane_loginfo
rospy.logwarn = lane_logwarn
rospy.logerr = lane_logerr
rospy.logerr_throttle = lane_logerr_throttle


class FrameGrabber(threading.Thread):
    """独立取帧线程, 只保留最新一帧。
    没有它的话: 相机 30fps 而主循环 10Hz, V4L2 内部队列越积越多,
    cap.read() 拿到的是几百 ms 前的画面 —— 控制环吃陈帧必然振荡。"""

    def __init__(self, cap):
        threading.Thread.__init__(self)
        self.daemon = True
        self.cap = cap
        self.lock = threading.Lock()
        self.frame = None
        self.seq = 0
        self.fail = 0
        self.stopped = False
        self.t_fps = time.time()      # 相机自身帧率(和主循环快慢无关)
        self.n_fps = 0
        self.cam_fps = 0.0

    def run(self):
        while not self.stopped:
            ok, f = self.cap.read()
            if ok and f is not None:
                with self.lock:
                    self.frame, self.seq, self.fail = f, self.seq + 1, 0
                # 相机真实出帧率: 主循环再快也快不过它。很多 USB 相机
                # 默认 YUYV 在 640x480 只有 ~10fps(带宽不够), 换 MJPG
                # 才能到 30 —— 这个数一低, 别的优化都是白费
                self.n_fps += 1
                dt = time.time() - self.t_fps
                if dt >= 1.0:
                    self.cam_fps = self.n_fps / dt
                    self.n_fps, self.t_fps = 0, time.time()
            else:
                self.fail += 1
                time.sleep(0.02)

    def latest(self):
        with self.lock:
            if self.frame is None:
                return None, self.seq
            return self.frame.copy(), self.seq

    def stop(self):
        self.stopped = True


def _tri(val, auto):
    """三态参数: true / false / auto。auto 时返回调用方给的推导值。
    roslaunch 会把裸的 true/false 自动转成 bool, 所以两种类型都得吃
    (和 is_fork 那个坑同一个来源: 传字符串必须 <param type="str">)。"""
    if isinstance(val, bool):
        return val
    t = str(val).strip().lower()
    if t in ("auto", "", "none"):
        return auto
    return t in ("1", "true", "yes", "on")


class RosFrameGrabber(object):
    """订阅共享相机话题, 只留最新一帧 —— 接口和 FrameGrabber 一样。

    故意**不用 cv_bridge**: 这个节点跑在 venv 里(系统 python 库很脆弱),
    cv_bridge 在 venv 里 import 不进来是常事, 而 bgr8/rgb8/mono8 的解码
    本来就是一次 reshape, 没必要为它引一个会炸的依赖。真碰上别的编码
    才回退到 cv_bridge。
    """

    def __init__(self, topic):
        from sensor_msgs.msg import CompressedImage, Image
        self.lock = threading.Lock()
        self.frame = None
        self.seq = 0
        self.stopped = False
        self.t_fps = time.time()
        self.n_fps = 0
        self.cam_fps = 0.0
        self._bridge = None
        self._warned = False
        self.compressed = topic.endswith("compressed")
        typ = CompressedImage if self.compressed else Image
        self.sub = rospy.Subscriber(topic, typ, self._cb, queue_size=1,
                                    buff_size=2 ** 24)

    def _decode(self, msg):
        if self.compressed:
            buf = np.frombuffer(msg.data, np.uint8)
            return cv2.imdecode(buf, cv2.IMREAD_COLOR)
        enc = msg.encoding.lower()
        h, w = msg.height, msg.width
        if enc in ("bgr8", "rgb8"):
            a = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
            return a if enc == "bgr8" else a[:, :, ::-1].copy()
        if enc == "mono8":
            a = np.frombuffer(msg.data, np.uint8).reshape(h, w)
            return cv2.cvtColor(a, cv2.COLOR_GRAY2BGR)
        if self._bridge is None:                 # 少见编码才拖 cv_bridge 进来
            from cv_bridge import CvBridge
            self._bridge = CvBridge()
        return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def _cb(self, msg):
        if self.stopped:
            return
        try:
            f = self._decode(msg)
        except Exception as e:
            if not self._warned:
                self._warned = True
                rospy.logerr("相机话题解码失败(只报一次): %s", e)
            return
        if f is None:
            return
        with self.lock:
            self.frame = f
            self.seq += 1
            self.n_fps += 1
            el = time.time() - self.t_fps
            if el >= 1.0:
                self.cam_fps = self.n_fps / el
                self.n_fps = 0
                self.t_fps = time.time()

    def latest(self):
        with self.lock:
            if self.frame is None:
                return None, self.seq
            return self.frame.copy(), self.seq

    def stop(self):
        self.stopped = True
        try:
            self.sub.unregister()
        except Exception:
            pass


class DumpWorker(threading.Thread):
    """把诊断图的绘制+JPEG 编码扔到后台线程。

    为什么: 实测(Nano, dump_every=5) 平摊下来 dump 占一帧 **42%~47%**,
    比分割(24ms)还贵一倍 —— 单张诊断图要画线画字再编码两次, 差不多
    190ms。而它对控制**毫无贡献**, 纯粹是给人看的。放主循环里等于
    每 5 帧就把控制环卡死近 0.2 秒, p90 直接从 88ms 冲到 130ms。

    丢帧策略: 只留一个槽位, 后台还没画完就来了新的 -> **直接覆盖**,
    宁可少存几张图也不能让队列堆积、更不能阻塞控制。丢了多少会统计,
    退出时打一行。
    """

    def __init__(self, fn):
        threading.Thread.__init__(self)
        self.daemon = True
        self.fn = fn
        self.lock = threading.Lock()
        self.ev = threading.Event()
        self.slot = None
        self.dropped = 0
        self.done = 0
        self.stopped = False

    def submit(self, payload):
        with self.lock:
            if self.slot is not None:
                self.dropped += 1        # 上一张还没画完, 丢掉它
            self.slot = payload
        self.ev.set()

    def run(self):
        while not self.stopped:
            self.ev.wait(0.2)
            self.ev.clear()
            while True:
                with self.lock:
                    p = self.slot
                    self.slot = None
                if p is None:
                    break
                try:
                    self.fn(p)
                    self.done += 1
                except Exception:
                    pass

    def stop(self):
        self.stopped = True
        self.ev.set()


class Prof(object):
    """分阶段耗时统计 —— 回答"到底卡在哪、还有多少余量"。

    为什么需要: 日志里那个 fps 是**主循环**的, 被 rospy.Rate 压着,
    只看它分不清"算不动"还是"在睡觉"。这里把一帧拆成 取帧/去畸变/分割/
    决策/横线/dump/其他 逐段计时, 再和循环周期一比:
      忙时 << 周期  -> 是 rate 在限速, 提 rate 就能涨 fps
      忙时 ≈  周期  -> 真算不动了, 得优化最贵的那一段
    ⚠ 计时准确性: 分割那一步 .so 里最后有 cudaMemcpy D2H(隐式同步),
    所以 time.time() 量到的就是 GPU 真正算完的时间, 不是"提交任务"的
    时间 —— 这点在异步的 CUDA 上很容易量错, 这里不用额外 synchronize。
    """

    ORDER = ["取帧", "去畸变", "分割", "决策", "横线", "认灯", "dump", "其他"]

    def __init__(self):
        self.acc = {}
        self.cur = {}
        self.n = 0
        self.loops = []
        self.periods = []

    def add(self, k, ms):
        self.cur[k] = self.cur.get(k, 0.0) + ms

    def frame(self, busy_ms, period_ms):
        """一帧结束: 把本帧各段并进总账, 剩下没记名的时间归到 其他"""
        self.cur["其他"] = max(0.0, busy_ms - sum(self.cur.values()))
        for k, v in self.cur.items():
            self.acc[k] = self.acc.get(k, 0.0) + v
        self.cur = {}
        self.n += 1
        self.loops.append(busy_ms)
        if period_ms is not None:
            self.periods.append(period_ms)

    def ready(self, every_n):
        return self.n >= every_n

    def text(self, rate_hz, cam_fps=0.0):
        """返回 (给日志的一行, 建议) —— 没数据时返回 None"""
        if not self.n:
            return None, None
        parts = []
        tot = 0.0
        for k in self.ORDER:
            v = self.acc.get(k, 0.0) / self.n
            if v >= 0.05:
                parts.append("%s%.1f" % (k, v))
            tot += v
        busy = sorted(self.loops)
        per = sorted(self.periods) if self.periods else [0.0]
        p50 = busy[len(busy) // 2]
        p90 = busy[int(len(busy) * 0.9)]
        period = per[len(per) // 2]
        line = ("[耗时] %s = %.1fms  |  忙 p50 %.1f p90 %.1f max %.1f  "
                "周期 %.1fms (%.1f fps, rate 上限 %.1f, 相机 %.1f fps)"
                % (" ".join(parts), tot, p50, p90, busy[-1], period,
                   1000.0 / max(period, 1e-6), rate_hz, cam_fps))
        # 建议: 谁最贵 + 是不是被 rate 卡着
        worst_k, worst_v = "", 0.0
        for k in self.ORDER:
            v = self.acc.get(k, 0.0) / self.n
            if v > worst_v:
                worst_k, worst_v = k, v
        tip = "瓶颈 %s(%.0f%%)" % (worst_k, 100.0 * worst_v / max(tot, 1e-6))
        idle = period - p90
        # 相机才是硬上限: 它出不来帧, 主循环只能空等
        if cam_fps > 0.1 and 1000.0 / max(period, 1e-6) > cam_fps * 0.85 \
                and idle > 15.0:
            tip += ("; **相机只有 %.1f fps**, 已经贴着它了 —— 先解决相机"
                    "(fourcc:=MJPG 试试), 提 rate 没用" % cam_fps)
        elif idle > 15.0:
            tip += "; 空转 %.0fms/帧 —— 是 rate 在限速, rate:=%.0f 就能提到" \
                   " %.1f fps" % (idle, min(30.0, 1000.0 / max(p90, 1e-6)),
                                  min(30.0, 1000.0 / max(p90, 1e-6)))
        else:
            tip += "; 已经算不动了(忙≈周期), 想快只能优化上面最贵那段"
        return line, tip

    def reset(self):
        self.acc = {}
        self.cur = {}
        self.n = 0
        self.loops = []
        self.periods = []


class LaneFollow(object):
    def __init__(self):
        gp = rospy.get_param
        here = os.path.dirname(os.path.abspath(__file__))
        pkg = os.path.dirname(here)
        self.dry_run = bool(gp("~dry_run", True))
        self.device = gp("~video_device", "/dev/video0")
        # ---- 相机/控制权归属 ----------------------------------------
        # take_cam_on_start: **单独测试**用的总闸。true = 自己独占 USB 相机、
        #   起来就跑; false(默认) = 听主流程交接 —— 用共享的 ROS 相机话题,
        #   先待在 STANDBY, 等 /lane_proto/set_active 被调用才动。
        # 下面两个是细调, 默认 auto = 跟着总闸走; 单独指定才覆盖。
        self.take_cam = bool(_tri(gp("~take_cam_on_start", False), False))
        self.use_ros_camera = _tri(gp("~use_ros_camera", "auto"),
                                   not self.take_cam)
        self.start_enabled = _tri(gp("~start_enabled", "auto"), self.take_cam)
        self.image_topic = gp("~image_topic", "/usb_cam/image_raw")
        self.cmd_vel_topic = gp("~cmd_vel_topic", "/cmd_vel")
        self.odom_topic = gp("~odom_topic", "/odom")
        self.exit_on_stop = bool(gp("~exit_on_stop", False))
        self.allow_size_mismatch = bool(gp("~allow_size_mismatch", False))
        self._size_checked = False
        self.enabled = self.start_enabled
        # 该相机输出是水平镜像的(挡板上的字是反的), 必须翻回来, 否则
        # 左右判反 -> 车往错误方向修 -> 直接冲出赛道。
        # 自检: 看 dump/latest.jpg, 挡板上的字正着念得通就对了。
        self.mirror = bool(gp("~mirror", True))
        self.v = float(gp("~linear_speed", 0.10))
        # 转向只有一个旋钮 gain: az = az_min + gain*(深度-dead), 上限 az_max
        # 大 = 猛。其余三个是硬件/噪声决定的, 一般不用碰。
        self.gain = float(gp("~gain", 1.2))
        self.dead = float(gp("~min_intrusion", 0.02))
        self.az_max = float(gp("~turn_max", 0.5))
        # 底盘 MCU 有旋转死区: 实测 0.073 rad/s 指令下 2.5s 才转 0.004 rad,
        # 等于没动(见原工程 ocr_alignment_min_speed 的注释)。非零指令抬到这里。
        self.az_min = float(gp("~min_turn_speed", 0.12))
        self.rate_hz = float(gp("~rate", 10.0))
        # 分阶段耗时统计: 每这么多秒打一行 [耗时]; 0=关掉
        self.stats_every = float(gp("~stats_every", 2.0))
        self.prof = Prof()
        # ---- 终点框 ----
        # use_lidar=true : 一检出就停(之后由激光雷达导航接管, 这里不实现)
        # use_lidar=false: 继续跑巡线, 按里程计再走 goal_stop_distance 米才停
        self.use_lidar = bool(gp("~use_lidar", False))
        self.goal_enable = bool(gp("~goal_enable", True))
        # 只看画面最底下 (1-goal_y_lo) 的行 = 只认"很近的"横线。
        # 0.78 -> 底部 22%(实测在你要的停车位姿刚好触发); 调小=更早触发=停得更远
        self.goal_y_lo = float(gp("~goal_y_lo", 0.78))
        # 扫描带的半宽(列). 60 -> 中央 120/320 列。宽一点对"大角度插入"更稳:
        # 判据是**列覆盖率**而不是"某行有连续横线", 所以线斜着穿也照样满分。
        self.goal_half = int(gp("~goal_half", 60))
        # 覆盖率超过这个才算"前方被挡住"。实测: 目标位姿/大角度插入都是 1.00,
        # 255 张任意位姿的正常帧里只有 0.8% 超过 0.8。
        self.goal_cover = float(gp("~goal_cover", 0.80))
        # 直线搜索(补列覆盖率的漏: 覆盖率不要求那些列共线):
        #   goal_line_cover  单条候选线被线像素覆盖多少才算"这条成立"
        #   goal_line_count  这样的候选线要有几条(真有实线时附近一堆候选
        #                    都会成立, 实测终点框帧有 12 条; 偶然对齐只有零星)
        #   goal_max_deg     候选线允许的最大倾角(度)
        self.goal_line_cover = float(gp("~goal_line_cover", 0.75))
        self.goal_line_count = int(gp("~goal_line_count", 8))
        self.goal_max_deg = float(gp("~goal_max_deg", 35.0))
        self.goal_confirm = int(gp("~goal_confirm", 1))   # 连续N帧才算数(车快时1帧)
        self.goal_dist = float(gp("~goal_stop_distance", 0.55))
        # 量这段距离用什么: auto=有 /odom 就用轮式里程计, 没有退速度x时间
        #   odom  轮式编码器积分, 来自 base_driver, **和激光雷达无关**,
        #         这个最小 launch 里就已经在发了, 精度厘米级 —— 首选
        #   time  速度x时间, 简单但受电量/地面摩擦影响, 需现场标一下
        #   imu   加速度二次积分: 便宜 MEMS 零偏 ~0.05m/s², 走 2.7s 就能
        #         漂 18cm(误差 45%), 量短距离是三者里最差的, 不建议
        self.dist_src = str(gp("~distance_source", "auto")).lower()
        # 兜底: 跑够这么久无条件停车退出(0=不限)。原型阶段防"车停不下来
        # 又 Ctrl-C 不掉"。另外随时可以远程急停:
        #   rostopic pub -1 /lane_proto/estop std_msgs/Bool "data: true"
        self.max_runtime = float(gp("~max_runtime", 300.0))
        # 检出后先刹停 goal_pause 秒再打点: 车停稳了里程计才不含刹车滑行,
        # 打的点干净; 顺便肉眼能看出"它确实认出来了"。0=不停直接打点。
        self.goal_pause = float(gp("~goal_pause", 1.0))
        # ---- 岔路(∈ 的中间那条, 末端是个 Y) ----
        # is_fork 三态(字符串/布尔都吃):
        #   false (默认) 检出横线 = 到终点, 走原来的停车流程
        #   true         **第一次**检出横线 = 到了 Y 的分叉口, 原地转
        #                fork_turn_deg 度(负=右转)对准一条臂, 然后继续巡线;
        #                之后再检出横线才按终点处理  —— 老语义, 一字未改
        #   yolo         **一启动就认灯**(车停在起点 3, 直接看 4 的红绿灯),
        #                按需拉起 yolo_tiny_cuda 子进程, 认完立刻杀掉:
        #                  left/right -> 原地转 ±yolo_turn_deg(60)° 进两臂,
        #                                之后横线 = 终点
        #                  straight   -> 不转, 巡线到 Y 岔口的横线, 那时才
        #                                转 fork_turn_deg(-45)°, 再之后
        #                                横线 = 终点
        #                = 三条路线一条命令跑完。
        self.fork_mode = self._fork_mode(gp("~is_fork", False))
        self.is_fork = (self.fork_mode != "off")
        self.fork_turn_deg = float(gp("~fork_turn_deg", -45.0))  # 负=右转
        # yolo 分支: 左/右各转这么多度(左=正, 右=负, 见 branch_deg)
        self.yolo_turn_deg = float(gp("~yolo_turn_deg", 60.0))
        ydir = os.path.join(pkg, "yolo", "yolo_tiny_cuda")
        self.yolo_dir = gp("~yolo_dir", ydir)
        self.yolo_exe = gp("~yolo_exe", "")
        self.yolo_weights = gp("~yolo_weights", "")
        self.yolo_backend = str(gp("~yolo_backend", "auto"))
        self.yolo_device = int(gp("~yolo_device", 0))
        self.yolo_conf = float(gp("~yolo_conf", 0.20))
        self.yolo_nms = float(gp("~yolo_nms", 0.45))
        # 认灯的唯一出口是**认出方向**: left/right/straight 里某个攒够
        # min_votes 票就定案。stop(红灯) 和 什么都没认出来 是同一种处理 ——
        # 都不投票、都继续等下一帧(车停在起点不动), 等到方向灯亮为止。
        self.yolo_min_votes = int(gp("~yolo_min_votes", 2))
        # 兜底上限: 等这么久还没方向就按 fallback 走(0=一直等)。真等到这里
        # 说明灯坏了/没对准, 至少别让车傻站着。
        self.yolo_wait_max = float(gp("~yolo_wait_max", 60.0))
        self.yolo_frames = int(gp("~yolo_frames", 0))     # 0=不限帧数
        # 超时兜底按哪个类处理(straight 最安全: 不转, 继续巡线)
        self.yolo_fallback = str(gp("~yolo_fallback", "straight")).lower()
        # 左右互换。**默认 True**: 实测这套权重给的 left/right 与翻正后画面里
        # 箭头的实际朝向相反 —— 训练集是用这台镜像相机拍的, 而我们喂 yolo 的
        # 是已翻正的帧, 差一次镜像。判断方法: 看 dump/yolo_*.jpg(那就是实际
        # 喂进去的图), 箭头朝向和标签对不上就说明要换。换了非镜像数据重训后
        # 记得改回 False。
        self.yolo_swap_lr = bool(gp("~yolo_swap_lr", True))
        # 认灯喂哪张图: false(默认)=去畸变后的; true=只翻正不去畸变的原始帧。
        # 去畸变会把画面往中间挤、边上留黑边, 灯要是被挤小了/糊了就试试 true
        # (yolo 的训练集本来也不是去畸变图)
        self.yolo_use_raw = bool(gp("~yolo_use_raw", False))
        # 起点看的是**远处**那个灯, 在 640x480 里可能只有十几个像素, 而
        # yolov4-tiny 进网还要缩到 640x352 —— 小目标本来就吃亏。zoom>1 就
        # 先把画面中间抠出来放大再送(纯数字变焦, 等效提高灯的分辨率)。
        # 2.0 = 抠中间 1/2 宽高。只用类别不用框, 所以不必把坐标映射回去。
        self.yolo_zoom = float(gp("~yolo_zoom", 1.0))
        self.yolo_zoom_cy = float(gp("~yolo_zoom_cy", 0.45))  # 抠图中心行(灯偏上)
        self.fork_speed = float(gp("~fork_turn_speed", 0.45))    # rad/s
        self.fork_tol_deg = float(gp("~fork_tol_deg", 2.0))
        self.fork_timeout = float(gp("~fork_timeout", 12.0))
        # 转完之后忽略横线检测的时间: 分叉口那条线还在视野里, 不冷却会
        # 立刻二次触发, 被当成终点
        self.fork_cooldown = float(gp("~fork_cooldown", 3.0))
        # 检出横线后、转弯前的前后微调(米)。0=原地就转; 正=先往前走这么远
        # 再转; 负=先后退。用轮式里程计量, 走完才转。
        self.fork_offset = float(gp("~fork_offset", 0.0))
        self.fork_move_speed = float(gp("~fork_move_speed", 0.12))
        # 起点补偿(米): 车特意往后摆这么多(让远处那个灯落在视野里、也别压
        # 起跑线), 认完灯**先补这一段**回到真正的起跑点, 再执行记住的方向。
        # 正=认完往前走, 负=往后退。认灯全程车不动, 挪的时候灯已经不看了。
        # ---- 起跑流程(仅 is_fork:=yolo, 见 run() 里的相位顺序) ----
        # 1) ALIGN      用黄线把车顶到固定位置(视觉伺服, 只前后动)
        # 2) 前进 align_offset  = 车轮压进"黄线前 10cm"那个规定位置
        #    (到了这儿黄线已经在视野下边缘外了, 所以必须先对齐再盲走)
        # 3) YOLO       等红灯变绿, 记住箭头方向
        # 4) 前进 start_offset  ≈30cm 开到三岔口
        # 5) apply_branch 按记住的方向拐, 之后就是原来的巡线/终点流程
        self.align_yellow = bool(gp("~align_yellow", True))
        # 对齐目标: 黄线该落在画面第几行(占图高的比例)。**必须现场标一次**:
        # 把车摆到你想要的对齐位姿, dry_run 跑一下, 日志里 [对齐] 那行报的
        # 就是当前行号/比例, 填进来即可。
        self.yellow_target = float(gp("~yellow_target", 0.90))
        self.yellow_tol = float(gp("~yellow_tol", 0.012))   # ±0.012*480≈6px
        self.yellow_b = float(gp("~yellow_b_min", 145))     # Lab b 阈值
        self.align_speed = float(gp("~align_speed", 0.08))
        self.align_timeout = float(gp("~align_timeout", 25.0))
        # 一开始就看不到黄线 = 车可能停过头了(线掉到视野下边外), 往后倒着找,
        # 最多倒这么远还找不到就放弃对齐直接往下走(免得一路倒进挡板)
        self.align_back_max = float(gp("~align_back_max", 0.30))
        self.align_offset = float(gp("~align_offset", 0.10))    # 100mm
        self.start_offset = float(gp("~start_offset", 0.30))    # 300mm
        self.start_move_speed = float(gp("~start_move_speed", 0.12))
        # 挪动收尾的最低线速度: 快到位时按剩余距离减速, 但不低于这个值
        # (底盘太低速可能推不动)。车挪到最后停住不动就把它调大。
        self.move_min = float(gp("~move_min_speed", 0.06))
        # 诊断图目录: 默认 <包>/dump/, 每 dump_every 帧覆盖写一张 latest.jpg
        # 另存 f%06d.jpg 便于回看(dump_keep=0 不限额, 启动时清空)
        self.dump_dir = gp("~dump_dir", os.path.join(pkg, "dump"))
        self.dump_every = int(gp("~dump_every", 5))
        # 诊断图放后台线程画(默认开)。实测它平摊占一帧 42%, 比分割还贵,
        # 而且对控制毫无贡献 —— 放后台后主循环基本感觉不到它。
        # 后台忙不过来就丢帧(只留最新一张), 绝不阻塞控制。
        self.dump_async = bool(gp("~dump_async", True))
        self.dumper = None
        # 0 = 不限额(每次启动都会清空目录, 不会越积越多)
        self.dump_keep = int(gp("~dump_keep", 0))
        self._dumped = []                 # 已写文件, 超出 dump_keep 删最旧的
        maps_path = gp("~maps_npz", os.path.join(pkg, "config/maps_640.npz"))
        tpl_path = gp("~template", "") or \
            os.path.join(pkg, "config/red_template.png")
        lib = gp("~trackseg_lib", os.path.join(pkg, "lib/libtrackseg.so"))
        assert os.path.exists(lib), (
            "找不到分割库 %s\n"
            "    模型权重已固化在这个 .so 里, 没有单独的权重文件。\n"
            "    做法: 在 trackseg_cuda/ 工程里 make (Nano 上, 需要\n"
            "          export PATH=/usr/local/cuda/bin:$PATH), 然后\n"
            "          cp libtrackseg.so %s/lib/" % (lib, pkg))

        z = np.load(maps_path)
        self.m1, self.m2 = z["m1"], z["m2"]
        self.W, self.H = int(z["W"]), int(z["H"])
        self.dec = Decider(*load_template(tpl_path))
        # 分割网**懒加载**: 这里只记路径, 不碰 CUDA。
        # 为什么: 常驻主流程(2026.launch)会让本节点先以 STANDBY 起着不干活,
        # 这段时间显存要留给别的节点; 真正要巡线之前再 ensure_seg() 加载。
        # 独立跑(start_enabled 默认 true)时 run() 开头就会加载, 行为不变。
        self.trackseg_lib = lib
        self.seg = None
        rospy.loginfo("转向映射 %s", curve_text(self.gain, self.dead,
                                                self.az_min, self.az_max))
        pre = (("先%s%.2fm再" % ("前进" if self.fork_offset > 0 else "后退",
                                 abs(self.fork_offset)))
               if abs(self.fork_offset) > 0.005 else "原地")
        if self.fork_mode == "fixed":
            rospy.loginfo("岔路模式(固定): 第一次检出横线 -> %s转 %.0f°(%s) "
                          "后继续巡线; 之后再检出才算终点", pre,
                          abs(self.fork_turn_deg),
                          "右" if self.fork_turn_deg < 0 else "左")
        elif self.fork_mode == "yolo":
            rospy.loginfo("分支模式(yolo): **一启动就认灯** -> left 原地左转 "
                          "%.0f°走左臂 / right 原地右转 %.0f°走右臂 (之后横线"
                          "=终点) / straight 不转往中间跑, 到横线再%s转 %.0f°"
                          "进 Y 支路(%s), 再之后的横线才是终点",
                          self.yolo_turn_deg, self.yolo_turn_deg,
                          "右" if self.fork_turn_deg < 0 else "左",
                          abs(self.fork_turn_deg), pre)
            rospy.loginfo("  yolo 目录 %s (按需拉起, 认完立刻杀; conf=%.2f, "
                          "%d 票定案; 红灯/认不出=继续等, 最多 %.0fs 然后按 "
                          "%s 走%s%s)",
                          self.yolo_dir, self.yolo_conf, self.yolo_min_votes,
                          self.yolo_wait_max, self.yolo_fallback,
                          ", 左右已互换" if self.yolo_swap_lr else "",
                          (", 数字变焦 %.1fx" % self.yolo_zoom)
                          if self.yolo_zoom > 1.001 else "")
        rospy.loginfo("终点: %s", "检出即停(等激光雷达接管)" if self.use_lidar
                      else "检出->刹停%.1fs打点->再走%.2fm(测距源 %s)停"
                      % (self.goal_pause, self.goal_dist, self.dist_src))
        if not self.mirror:
            rospy.logwarn("mirror=false: 若相机输出是镜像的(挡板字反着), "
                          "左右判决会整个反过来!")
        if self.v == 0.0:
            rospy.loginfo("linear_speed=0 -> 原地只转车体对准, 不前进")

        if self.use_ros_camera:
            self.cap = None
            self.grab = RosFrameGrabber(self.image_topic)
            rospy.loginfo("相机: 用共享话题 %s (不独占 %s)",
                          self.image_topic, self.device)
        else:
            self._open_v4l2(gp)
        if self.dump_dir:
            if not os.path.isdir(self.dump_dir):
                os.makedirs(self.dump_dir)
            n_old = 0
            for fn in os.listdir(self.dump_dir):
                if fn.lower().endswith((".jpg", ".jpeg", ".png")):
                    try:
                        os.remove(os.path.join(self.dump_dir, fn))
                        n_old += 1
                    except OSError:
                        pass
            rospy.loginfo("诊断图 -> %s/latest.jpg (每 %d 帧一张, 已清掉 %d "
                          "张旧图)", self.dump_dir, self.dump_every, n_old)
        if self.dump_dir and self.dump_every > 0 and self.dump_async:
            self.dumper = DumpWorker(self._dump_now)
            self.dumper.start()

        self.pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self._init_rest(gp)

    def _open_v4l2(self, gp):
        # ⚠ 必须显式指定 CAP_V4L2 后端。
        # 传字符串路径时 OpenCV 自己挑后端, 实测在 Nano 上 cap.set(宽/高)
        # **被静默忽略** —— 相机停在 1920x1080@25, 而鱼眼标定是在 640x480
        # 上做的。to_4x3() 会把 16:9 中心裁再缩, 水平视场比原生 4:3 窄一截,
        # 去畸变映射整个对不上(线的角度/位置、模板触发区全偏), 而且白白多花
        # ~10ms 缩放。加上 CAP_V4L2 后 set() 才生效, 实测拿到 640x480@30。
        self.cap = None
        if hasattr(cv2, "CAP_V4L2"):
            try:
                self.cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
                if not self.cap.isOpened():
                    self.cap.release()
                    self.cap = None
                    rospy.logwarn("CAP_V4L2 打不开 %s, 退回默认后端",
                                  self.device)
            except Exception as e:
                self.cap = None
                rospy.logwarn("CAP_V4L2 不可用(%s), 退回默认后端", e)
        if self.cap is None:
            self.cap = cv2.VideoCapture(self.device)
        assert self.cap.isOpened(), "打不开相机 %s (确认没有别的进程占用)" \
            % self.device
        # ⚠ fourcc 必须在设分辨率**之前**设, 否则很多驱动会忽略它。
        # 默认 MJPG: 大量 USB 相机在 YUYV 下 640x480 只能出 ~10fps(USB2
        # 带宽不够, 未压缩), 换 MJPG 才有 30fps —— 这是"fps 上不去"最常见
        # 的原因, 而且和 CUDA 快慢一点关系都没有。
        # 想退回"不设"就传 fourcc:=none。
        fcc = str(gp("~fourcc", "MJPG")).upper()
        if fcc and fcc != "NONE" and len(fcc) == 4:
            try:
                self.cap.set(cv2.CAP_PROP_FOURCC,
                             cv2.VideoWriter_fourcc(*fcc))
            except Exception as e:
                rospy.logwarn("设 fourcc=%s 失败: %s", fcc, e)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)
        want_fps = float(gp("~cam_fps", 0.0))     # 0 = 不请求, 用驱动默认
        if want_fps > 0:
            self.cap.set(cv2.CAP_PROP_FPS, want_fps)
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 不支持也没关系, 有取帧线程
        except Exception:
            pass
        try:
            got = int(self.cap.get(cv2.CAP_PROP_FOURCC))
            fcc_got = "".join(chr((got >> (8 * k)) & 0xFF) for k in range(4))
            gw = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            gh = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            rospy.loginfo("相机协商结果: %dx%d @%.0f fps, 编码 %s "
                          "(请求 %s; 这里帧率低就别怪 CUDA)",
                          gw, gh, self.cap.get(cv2.CAP_PROP_FPS), fcc_got,
                          fcc or "未指定")
            # 协商结果和标定尺寸对不上 = 去畸变映射失配, 必须**吵**。
            # 之前就是被 to_4x3() 默默兜过去的: 画面看着正常, 实际视场是
            # 裁出来的, 线的角度和模板触发区全偏, 查了很久才找到。
            if (gw, gh) != (self.W, self.H):
                msg = ("相机给的是 %dx%d, 而鱼眼标定/映射表是 %dx%d —— "
                       "去畸变会失配(to_4x3 只是兜底裁缩, 视场对不上)。"
                       % (gw, gh, self.W, self.H))
                if self.allow_size_mismatch:
                    rospy.logerr("%s 已用 allow_size_mismatch 放行, 结果不可信",
                                 msg)
                else:
                    raise RuntimeError(
                        msg + " 相机支持哪些模式: v4l2-ctl -d %s "
                        "--list-formats-ext; 确认要带病跑就传 "
                        "allow_size_mismatch:=true" % self.device)
        except RuntimeError:
            raise
        except Exception:
            pass
        self.grab = FrameGrabber(self.cap)
        self.grab.start()

    def _init_rest(self, gp):
        self.state_pub = rospy.Publisher("/lane_proto/state", String,
                                         queue_size=1, latch=True)
        # GOAL=真到终点 / ABORT=兜底停车 / ESTOP=急停 / CONFIG=配置不对。
        # 主流程只看 state=="STOPPED" 的话, 这四种全是"成功"。
        self.result_pub = rospy.Publisher("/lane_proto/result", String,
                                          queue_size=1, latch=True)
        # 里程计量 40cm: 用轮式里程计而不是 IMU —— IMU 要二次积分加速度,
        # 几秒就漂出几十厘米, 量这种短距离完全不能用; /odom 直接给位置。
        self.odom_xy = None
        self.odom_seen = False
        self.odom_move_t = 0.0        # 上次"位置真的变了"的时刻
        self.odom_frozen_warned = False
        # odom 冻结兜底: 默认**关**。
        # 车不动的时候底盘根本不发新的 odom, "位置没变"是正常状态而不是
        # 故障, 之前按 1.5/5.5s 判会误判(队友把阈值改成 100000 就是在关它,
        # 只是那样连真的 odom 掉线也救不了了)。这里给成显式开关: 确认
        # odom 会掉线再开, 平时宁可不兜底也不要假测距。
        self.odom_fallback = bool(gp("~odom_fallback", False))
        self.odom_frozen_s = float(gp("~odom_frozen_s", 5.5))
        # 交接模式下主流程可能还没起底盘, odom 迟到很正常; 这个只在
        # require_fresh_odom:=true 时才当停车条件, 默认只记录不干预。
        self.odom_recv_t = 0.0
        self.odom_finite = False
        self.require_fresh_odom = bool(gp("~require_fresh_odom", False))
        self.odom_fresh_timeout = float(gp("~odom_fresh_timeout", 0.5))
        self._odom_stale_warned = False
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber("/lane_proto/estop", Bool, self.estop_cb, queue_size=1)
        self.enable_srv = rospy.Service("/lane_proto/set_active", SetBool,
                                        self.set_active)
        self.imu_v = 0.0            # imu 测距模式用: 速度一次积分
        self.imu_s = 0.0            # 位移二次积分
        self.imu_t = None
        # 多圈展开的 yaw(弧度)。**不假设原始读数的范围**(-180~180 还是
        # 0~360 都行): 每次和上一帧的展开值比, 差超过 ±180° 就反复
        # ±360° 直到落进 ±180°, 再覆盖为新的展开值。这样跨 0/360 或
        # ±180 的跳变都不会产生假的大角度。
        self.yaw_unw = None
        rospy.Subscriber("/imu", Imu, self.imu_cb, queue_size=20)
        self.goal_hits = 0
        # FOLLOW -> (检出) PAUSE 刹停打点 -> APPROACH 再走N米 -> STOPPED
        # use_lidar=true 时: FOLLOW -> (检出) STOPPED
        # 交接模式下先待在 STANDBY: 不发速度、不加载 CUDA, 等主流程调
        # /lane_proto/set_active 再进 FOLLOW。
        self.phase = "FOLLOW" if self.enabled else "STANDBY"
        self._done_announced = False
        self.pause_until = 0.0
        self._segs, self._bseg, self._pl, self._pr = [], None, [], []
        self._blk = 0.0                 # 扫描带里被红绿灯箱体挡掉的列比例
        self._pause_next = "approach"   # PAUSE 结束后干什么: approach / fork
        self.fork_done = False          # 岔路已经拐过了
        self.fork_yaw0 = None           # 起转时的多圈 yaw
        self.fork_t0 = 0.0
        self._fork_warned = False
        self.state_pub.publish(String(data=self.phase))   # latch 出起始相位
        # 本次岔路实际要转的角度: fixed 模式=fork_turn_deg, yolo 模式由灯决定
        self.turn_deg = self.fork_turn_deg
        self.yolo = None                # 认灯子进程, 只在 YOLO 相位活着
        self.yolo_started = False       # 起点那次认灯是否已经发起过
        self.branch_cls = ""            # 记住的箭头方向(挪完起点补偿才用)
        self._move_next = "branch"      # 直线挪动结束后干什么
        self.align_t0 = 0.0
        self.align_back = 0.0           # 已经为了找黄线倒了多远
        self.align_n = 0
        self.yellow_row = None
        self.yellow_px = 0
        self.move_dist = 0.0            # 当前这段直线挪动的目标/指令
        self.move_cmd = 0.0
        self.yolo_votes = {}
        self.yolo_n = 0
        self.yolo_err = 0
        self.yolo_t0 = 0.0
        self.yolo_last = ""             # 最近一帧的检出, 只为打日志/画图
        self._und = None                # step() 存下的去畸变(已翻正)帧
        self._raw = None                # 同一帧, 只翻正没去畸变
        self.cool_until = 0.0           # 冷却到这个时刻为止不认横线
        self.mark_xy = None
        self.mark_t = None
        rospy.on_shutdown(self.stop)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.odom_recv_t = time.time()
        # NaN/Inf 的 odom 比没有 odom 更坏: 它会让"位置变了吗"永远成立。
        self.odom_finite = (p.x == p.x and p.y == p.y and
                            abs(p.x) != float("inf") and
                            abs(p.y) != float("inf"))
        if not self.odom_finite:
            return
        if self.odom_xy is None or \
                abs(p.x - self.odom_xy[0]) > 1e-4 or \
                abs(p.y - self.odom_xy[1]) > 1e-4:
            self.odom_move_t = time.time()      # 位置真的在变
        self.odom_xy = (p.x, p.y)
        self.odom_seen = True

    def _check_frame_size(self, frame):
        """共享话题给的尺寸也得和标定表对上 —— 和 V4L2 那条路同一条理由,
        只是话题是异步的, 只能等第一帧到了再查。"""
        gh, gw = frame.shape[:2]
        if (gw, gh) == (self.W, self.H):
            rospy.loginfo("相机话题首帧 %dx%d, 与标定一致", gw, gh)
            return
        msg = ("相机话题给的是 %dx%d, 而鱼眼标定/映射表是 %dx%d —— "
               "去畸变会失配" % (gw, gh, self.W, self.H))
        if self.allow_size_mismatch:
            rospy.logerr("%s; 已用 allow_size_mismatch 放行, 结果不可信", msg)
        else:
            rospy.logerr("%s; 停车。确认要带病跑就传 allow_size_mismatch:=true",
                         msg)
            self.set_phase("STOPPED", "相机尺寸与标定不符")

    def odom_is_fresh(self):
        return (self.odom_finite and self.odom_recv_t > 0.0 and
                time.time() - self.odom_recv_t <= self.odom_fresh_timeout)

    def set_active(self, req):
        """主流程交接控制权: /lane_proto/set_active。
        true  -> 先把 CUDA 加载好**再**回成功, 保证主流程切 cmd_vel owner
                 的那一刻巡线已经能出速度了(否则头几百毫秒是空档)。
        false -> 立刻发一次零速再回 STANDBY。"""
        if req.data:
            if self.phase == "STOPPED":
                return SetBoolResponse(False, "lane follower already stopped")
            try:
                self.ensure_seg()
            except Exception as e:
                rospy.logerr("交接失败: 分割网加载不了: %s", e)
                return SetBoolResponse(False, "trackseg load failed")
            self.enabled = True
            self.set_phase("FOLLOW", "主流程已交接控制权")
            return SetBoolResponse(True, "lane follower active")
        self.enabled = False
        try:
            self.pub.publish(Twist())
        except Exception:
            pass
        self.set_phase("STANDBY", "交回控制权, 待命")
        return SetBoolResponse(True, "lane follower standby")

    def imu_cb(self, msg):
        q = msg.orientation
        if (q.w or q.x or q.y or q.z):       # 有姿态解算才更新 yaw
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.push_yaw(math.atan2(siny, cosy))
        self.imu_accum(msg)

    def push_yaw(self, raw):
        """把任意范围的 yaw 读数并进多圈连续角"""
        if self.yaw_unw is None:
            self.yaw_unw = raw
            return
        n = 0
        while raw - self.yaw_unw > math.pi and n < 100:
            raw -= 2.0 * math.pi
            n += 1
        while raw - self.yaw_unw < -math.pi and n < 100:
            raw += 2.0 * math.pi
            n += 1
        self.yaw_unw = raw

    def imu_accum(self, msg):
        """imu 兜底积分(不推荐, 会漂): a -> v -> s, 只在 APPROACH 期间累"""
        t = msg.header.stamp.to_sec() or time.time()
        if self.imu_t is None:
            self.imu_t = t
            return
        dt = t - self.imu_t
        self.imu_t = t
        if dt <= 0 or dt > 0.5 or self.phase != "APPROACH":
            return
        a = msg.linear_acceleration.x
        self.imu_v += a * dt
        self.imu_s += abs(self.imu_v) * dt

    def speed_now(self):
        """本相位实际发出去的线速度(m/s)。
        ⚠ 不能一律用 self.v: START_MOVE / FORK_MOVE 发的是 move_cmd
        (0.12), 而 self.v 是巡线速度(可能 0.3, 也可能台架上是 0)。
        速度x时间那条兜底路径和 odom 冻结判据都要按**当前真在发的**速度
        算, 否则 50mm 会量成 125mm(早停), 或者 v=0 时永远量到 0(不停)。"""
        if self.phase in ("START_MOVE", "FORK_MOVE"):
            return abs(self.move_cmd)
        if self.phase == "ALIGN":
            return abs(self.align_speed)
        # 这些相位本来就是停着的(等灯/刹停/原地转/停车)。以前一律返回
        # self.v, 等于"我在动但 odom 不变" —— 而底盘停着时 odom 本来就
        # 不更新, 于是必然误判成 odom 冻结。
        if self.phase in ("YOLO", "PAUSE", "STOPPED", "FORK_TURN", "STANDBY"):
            return 0.0
        return abs(self.v)

    def _dist_src_used(self):
        """当前这一刻实际在用哪个测距源(和 moved_since_mark 同一判断)"""
        frozen = (self.odom_fallback and self.odom_xy is not None and
                  time.time() - self.odom_move_t > self.odom_frozen_s and
                  self.speed_now() > 0.01)
        if self.dist_src in ("auto", "odom") and not frozen \
                and self.mark_xy is not None and self.odom_xy is not None:
            return "odom"
        if self.dist_src == "imu":
            return "imu积分"
        return "速度x时间" + ("(odom冻结)" if frozen else "")

    def moved_since_mark(self):
        """自打点以来走了多远(m)。
        odom **冻结**时(收得到消息但位置不动 —— 编码器没回传就会这样)
        自动退回 速度x时间, 否则 APPROACH 永远走不满距离, 车就停不下来。"""
        src = self.dist_src
        frozen = (self.odom_fallback and self.odom_xy is not None and
                  time.time() - self.odom_move_t > self.odom_frozen_s and
                  self.speed_now() > 0.01)
        if frozen and not self.odom_frozen_warned:
            self.odom_frozen_warned = True
            rospy.logwarn("/odom 位置 1.5s 没变化(编码器没回传?), 测距退回 "
                          "速度x时间")
        if src in ("auto", "odom") and not frozen and self.mark_xy is not None \
                and self.odom_xy is not None:
            dx = self.odom_xy[0] - self.mark_xy[0]
            dy = self.odom_xy[1] - self.mark_xy[1]
            return math.sqrt(dx * dx + dy * dy)
        if src == "imu":
            return self.imu_s
        if self.mark_t is not None:            # 兜底: 速度 x 时间
            return self.speed_now() * (time.time() - self.mark_t)
        return 0.0

    @staticmethod
    def _fork_mode(v):
        """is_fork 三态解析。roslaunch 里给了 type="str" 就永远是字符串,
        没给的话 "true"/"false" 会被自动转成 bool —— 两种都接住,
        所以命令行 is_fork:=false / true / yolo 都不用加引号。"""
        if isinstance(v, bool):
            return "fixed" if v else "off"
        s = str(v).strip().lower()
        if s in ("yolo", "auto", "light", "tl"):
            return "yolo"
        if s in ("1", "true", "yes", "y", "on", "fixed"):
            return "fixed"
        if s in ("0", "false", "no", "n", "off", ""):
            return "off"
        rospy.logwarn("is_fork=%r 看不懂, 按 false 处理 "
                      "(可选: false / true / yolo)", v)
        return "off"

    # ---------------- 起跑: 黄线对齐 ----------------
    def start_align(self):
        """用黄线把车顶到固定位置。只前后动, 不转 —— 横向/角度靠人摆。
        必须先对齐再盲走 align_offset, 因为规定位置(黄线前 10cm)上黄线
        已经落到视野下边缘外了, 到那儿再想量就没得量。"""
        self.align_t0 = time.time()
        self.align_back = 0.0
        self.align_n = 0
        self.set_phase("ALIGN", "起跑: 用黄线对齐 (目标行 %.0f/%d)"
                       % (self.yellow_target * self.H, self.H))

    def step_align(self):
        """返回本帧 linear.x; 对齐好/放弃返回 None"""
        im = self._und
        self.align_n += 1
        row, npx = yellow_line(im, b_min=self.yellow_b)
        self.yellow_row, self.yellow_px = row, npx
        h = im.shape[0]
        tgt = self.yellow_target * h
        el = time.time() - self.align_t0
        if self.dump_dir and self.align_n % 3 == 1:
            self._dump_align(im, row, tgt)
        if row is None:
            # 看不到黄线: 多半是车停过头了(线掉到画面下边外), 慢慢倒着找
            if self.align_back >= self.align_back_max or el > self.align_timeout:
                rospy.logwarn("对齐失败: 倒了 %.2fm/等了 %.1fs 还是找不到黄线"
                              "(黄像素 %d) —— 跳过对齐, 直接往下走",
                              self.align_back, el, npx)
                return None
            self.align_back += abs(self.align_speed) / max(1.0, self.rate_hz)
            if self.align_n % 10 == 1:
                rospy.loginfo("[对齐] 没看到黄线(黄像素 %d), 后退找 "
                              "(已退 %.2f/%.2fm)", npx, self.align_back,
                              self.align_back_max)
            return -abs(self.align_speed)
        err = (tgt - row) / float(h)        # >0: 线还在目标上方 = 车太靠后
        if self.align_n % 5 == 1 or abs(err) <= self.yellow_tol:
            rospy.loginfo("[对齐] 黄线在第 %.0f 行 (%.3f), 目标 %.0f (%.3f), "
                          "差 %+.0f px, 黄像素 %d", row, row / float(h),
                          tgt, self.yellow_target, tgt - row, npx)
        if abs(err) <= self.yellow_tol:
            rospy.loginfo("对齐完成: 黄线第 %.0f 行, 用了 %.1fs", row, el)
            return None
        if el > self.align_timeout:
            rospy.logwarn("对齐超时 %.1fs, 还差 %+.0f px, 就这样往下走",
                          el, tgt - row)
            return None
        # 车往前开, 黄线在画面里往下走(行号变大)。所以 线在目标上方 -> 前进
        mag = min(abs(self.align_speed),
                  max(self.move_min, 1.2 * abs(err) * h * 0.004))
        return math.copysign(mag, err)

    def _dump_align(self, im, row, tgt):
        """对齐诊断图: 绿=检出的黄线, 品红=目标行, 顺便把黄掩码涂出来。
        标定 yellow_target 就靠这张图 + 日志里的行号。"""
        try:
            vis = im.copy()
            h, w = vis.shape[:2]
            x0, x1 = int(w * 0.33), int(w * 0.67)
            m = yellow_mask(vis[:, x0:x1], self.yellow_b)
            sub = vis[:, x0:x1]
            sub[m] = (0, 0, 255)                       # 掩码涂红
            cv2.rectangle(vis, (x0, 0), (x1 - 1, h - 1), (255, 255, 0), 1)
            cv2.line(vis, (0, int(tgt)), (w, int(tgt)), (255, 0, 255), 2)
            if row is not None:
                cv2.line(vis, (0, int(row)), (w, int(row)), (0, 255, 0), 2)
            cv2.putText(vis, "row=%s tgt=%.0f px=%d" %
                        ("--" if row is None else "%.0f" % row, tgt,
                         self.yellow_px), (6, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imwrite(os.path.join(self.dump_dir,
                                     "align_%03d.jpg" % self.align_n), vis)
        except Exception:
            pass

    def after_align(self):
        """对齐完(或跳过对齐): 盲走 align_offset 进到规定的起跑位置,
        然后在那儿等绿灯。这一段必须盲走 —— 黄线到这时已经在视野外了。"""
        self._move_next = "yolo"
        if abs(self.align_offset) > 0.005:
            self.start_move(self.align_offset, self.start_move_speed,
                            "START_MOVE", "起跑: %s %.0fmm 进到黄线前的规定"
                            "位置, 然后等绿灯"
                            % ("前进" if self.align_offset > 0 else "后退",
                               abs(self.align_offset) * 1000.0))
        else:
            self.start_yolo()

    # ---------------- 红绿灯分支 ----------------
    def branch_deg(self, cls):
        """红绿灯类 -> 该转多少度。左=正(逆时针), 右=负; 直行=0 不转。"""
        if self.yolo_swap_lr:
            cls = {"left": "right", "right": "left"}.get(cls, cls)
        if cls == "left":
            return abs(self.yolo_turn_deg)
        if cls == "right":
            return -abs(self.yolo_turn_deg)
        return 0.0                      # straight / stop / 认不出来 -> 直行

    def start_yolo(self):
        """到岔路口了: 按需拉起认灯子进程(认完立刻杀)"""
        self.yolo_votes, self.yolo_n, self.yolo_err = {}, 0, 0
        self.yolo_last = ""
        self.yolo_t0 = time.time()
        self.set_phase("YOLO", "起点: 先认红绿灯再决定走哪条路线")
        try:
            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            from yolo_client import (YoloProc, find_exe, find_weights,
                                     best_det)
            self._best_det = best_det
            exe = self.yolo_exe or find_exe(self.yolo_dir)
            wts = self.yolo_weights or find_weights(self.yolo_dir)
            assert exe and os.path.exists(exe), (
                "找不到 yolo 可执行文件 (yolo_dir=%s)。先在那个目录里\n"
                "    make GPU=1 ARCH=sm_53      # 原版 Jetson Nano 是 Maxwell"
                % self.yolo_dir)
            assert wts and os.path.exists(wts), \
                "找不到 .weights (yolo_dir=%s)" % self.yolo_dir
            t0 = time.time()
            self.yolo = YoloProc(exe, wts, self.yolo_backend,
                                 self.yolo_device, timeout=25.0)
            self.yolo.configure(conf=self.yolo_conf, nms=self.yolo_nms,
                                timings=False)
            rospy.loginfo("yolo 起来了: backend=%s classes=%s (%.1fs)",
                          self.yolo.backend, ",".join(self.yolo.classes),
                          time.time() - t0)
            if self.yolo.backend != "cuda":
                rospy.logwarn("yolo 跑在 %s 上! CPU 单帧要好几秒, 认灯会很慢 "
                              "—— 检查 make 时有没有 GPU=1 ARCH=sm_53",
                              self.yolo.backend)
        except Exception as e:
            self.yolo = None
            rospy.logerr("拉起 yolo 失败: %s", e)

    def step_yolo(self):
        """认灯一帧。还在认返回 None; 定了返回类名(str)"""
        if self.yolo is None:                     # 起不来就直接兜底
            return self.yolo_fallback
        el = time.time() - self.yolo_t0
        try:
            im = self.yolo_frame()
            t_y = time.time()
            r = self.yolo.detect_bgr(im, timeout=20.0)
            self._lap("认灯", t_y)
            self.yolo_n += 1
            best = self._best_det(r)
        except Exception as e:
            self.yolo_err += 1
            rospy.logwarn("yolo 第 %d 次出错: %s", self.yolo_err, e)
            if self.yolo_err >= 3:
                return self.yolo_fallback
            return None
        # 只有方向类(left/right/straight)投票。**stop(红灯) 和 什么都没
        # 认出来 走同一条路: 不投票、不定案、继续等下一帧** —— 红灯本来
        # 就是让你停着等, 认不出来也没资格瞎猜。
        if best:
            name, conf, box = best
            self.yolo_last = "%s %.2f" % (name, conf)
            if name in ("left", "right", "straight"):
                self.yolo_votes[name] = self.yolo_votes.get(name, 0) + 1
            if self.dump_dir:
                self._dump_yolo(name, conf, box)
        else:
            self.yolo_last = "-"
        arrows = [(n, c) for n, c in self.yolo_votes.items()]
        arrows.sort(key=lambda kv: -kv[1])
        if arrows and arrows[0][1] >= self.yolo_min_votes:
            rospy.loginfo("[认灯] 第%d帧 %s -> 够票了", self.yolo_n,
                          self.yolo_last)
            return arrows[0][0]
        # 还没方向: 每秒打一行, 别把日志刷爆
        if self.yolo_n <= 3 or self.yolo_n % 10 == 0:
            rospy.loginfo("[认灯] 第%d帧 %s  票数 %s  等待方向灯 (%.0f%s)",
                          self.yolo_n, self.yolo_last,
                          ",".join("%s:%d" % kv for kv in
                                   sorted(self.yolo_votes.items())) or "无",
                          el, ("/%.0fs" % self.yolo_wait_max)
                          if self.yolo_wait_max > 0 else "s, 不限时")
        over = (self.yolo_wait_max > 0 and el > self.yolo_wait_max) or \
               (self.yolo_frames > 0 and self.yolo_n >= self.yolo_frames)
        if over:
            if arrows:                    # 票不够但有, 也认了
                rospy.logwarn("认灯等到点(%d帧 %.1fs), 票不足按最高票 %s 走",
                              self.yolo_n, el, arrows[0][0])
                return arrows[0][0]
            rospy.logwarn("认灯等到点(%d帧 %.1fs)始终没有方向灯, 按兜底 %s 走",
                          self.yolo_n, el, self.yolo_fallback)
            return self.yolo_fallback
        return None

    def kill_yolo(self):
        """认完/退出就杀 —— Nano 4GB 显存留不得"""
        if self.yolo is not None:
            try:
                self.yolo.close()
                rospy.loginfo("yolo 子进程已退出")
            except Exception as e:
                rospy.logwarn("关 yolo 出错: %s", e)
            self.yolo = None

    def finish_yolo(self, cls):
        """起点定案: 杀进程 -> **记住**箭头方向 -> 先按 start_offset 挪到
        真正的起跑点 -> 再按记住的方向做动作。
        车是特意往后摆 start_offset 的(让灯落在视野里/别压起跑线), 所以
        认完必须补这一段; 认灯期间车一直没动, 挪的时候灯已经不看了。"""
        self.kill_yolo()
        self.branch_cls = cls                 # 记忆: 后面挪完了才用
        self.turn_deg = self.branch_deg(cls)
        rospy.loginfo("红绿灯判定: %s -> 记住了(%s), 先前进 %.0fmm 到三岔口"
                      " (约 %.1fs)",
                      cls, "不转直行" if self.turn_deg == 0 else
                      "%s转 %.0f°" % ("左" if self.turn_deg > 0 else "右",
                                      abs(self.turn_deg)),
                      self.start_offset * 1000.0,
                      abs(self.start_offset) /
                      max(0.01, abs(self.start_move_speed)))
        self._move_next = "branch"
        if abs(self.start_offset) > 0.005:
            self.start_move(self.start_offset, self.start_move_speed,
                            "START_MOVE", "%s %.0fmm 开到三岔口"
                            % ("前进" if self.start_offset > 0 else "后退",
                               abs(self.start_offset) * 1000.0))
        else:
            self.apply_branch()

    def apply_branch(self):
        """挪到起跑点了, 执行记忆里的那个方向"""
        cls, deg = self.branch_cls, self.turn_deg
        if deg == 0.0:
            # 直行 -> 中间那条, 末端的 Y 交给原来的岔路逻辑(fork_done=False
            # 意味着"第一次横线 = 岔口, 不是终点")
            self.fork_done = False
            self._pause_next = "approach"
            rospy.loginfo("红绿灯判定: %s -> 不转, 走中间那条; 跑到横线再"
                          "%s转 %.0f° 进 Y 支路, 再之后的横线才是终点", cls,
                          "右" if self.fork_turn_deg < 0 else "左",
                          abs(self.fork_turn_deg))
            self.set_phase("FOLLOW", "起点直行")
        else:
            # 两臂 -> 转完就是普通巡线, 下一次横线就是终点
            self.fork_done = True
            self._pause_next = "approach"
            rospy.loginfo("红绿灯判定: %s -> 原地%s转 %.0f° 进%s臂, 之后检出"
                          "横线 = 终点", cls, "左" if deg > 0 else "右",
                          abs(deg), "左" if deg > 0 else "右")
            self.start_fork_turn()

    def _dump_yolo(self, name, conf, box):
        """把喂给 yolo 的那帧连框一起存下来 —— 左右反没反, 看这张图就知道"""
        try:
            im = self.yolo_frame().copy()
            h, w = im.shape[:2]
            cx, cy, bw, bh = box
            p1 = (int((cx - bw / 2) * w), int((cy - bh / 2) * h))
            p2 = (int((cx + bw / 2) * w), int((cy + bh / 2) * h))
            cv2.rectangle(im, p1, p2, (0, 255, 0), 2)
            cv2.putText(im, "%s %.2f" % (name, conf),
                        (p1[0], max(14, p1[1] - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imwrite(os.path.join(self.dump_dir,
                                     "yolo_%02d.jpg" % self.yolo_n), im)
        except Exception:
            pass

    def begin_fork(self):
        """到 Y 岔口了(检出横线): 转 fork_turn_deg 度进一条支路。
        灯是在**起点**就认完的, 这里不再认 —— 走到这儿说明当时判的直行。"""
        self.turn_deg = self.fork_turn_deg
        if abs(self.fork_offset) > 0.005:
            self.start_fork_move()
        else:
            self.start_fork_turn()

    def start_fork_move(self):
        """转弯前先按 fork_offset 前后挪一段(正=前进 负=后退)"""
        self._move_next = "fork_turn"
        self.start_move(self.fork_offset, self.fork_move_speed, "FORK_MOVE",
                        "岔路: 先%s %.2fm 再转"
                        % ("前进" if self.fork_offset > 0 else "后退",
                           abs(self.fork_offset)))

    # 直线挪一小段(起点补偿 / 岔口微调 共用一套)。用轮式里程计量, odom
    # 冻结时 moved_since_mark 会自己退回 速度x时间, 见那边的注释。
    def start_move(self, dist, speed, phase, why):
        self.move_dist = abs(dist)
        self.move_cmd = math.copysign(abs(speed), dist)
        self.mark_xy = self.odom_xy
        self.mark_t = time.time()
        self.set_phase(phase, why)

    def step_move(self):
        """返回本帧 linear.x; 走够了返回 None
        主循环 10Hz, 0.12m/s 一拍就是 12mm —— 50mm 这种短距离全速冲会
        过冲一整拍。所以快到位时按剩余距离减速(下限 move_min_speed,
        再低底盘可能推不动), 末拍步长降到 6mm 上下。"""
        d = self.moved_since_mark()
        remain = self.move_dist - d
        if remain <= 0:
            rospy.loginfo("挪动完成: 实走 %.3fm (目标 %.3fm, 源=%s)", d,
                          self.move_dist, self._dist_src_used())
            return None
        # 超时按 距离/速度 的 3 倍 + 3s 给, 测距源出问题也不会一直挪
        budget = self.move_dist / max(0.05, abs(self.move_cmd)) * 3.0 + 3.0
        if time.time() - self.mark_t > budget:
            rospy.logwarn("挪动超时(%.1fs 只测到 %.3f/%.3fm), 直接进下一步",
                          budget, d, self.move_dist)
            return None
        mag = min(abs(self.move_cmd), max(self.move_min, 1.5 * remain))
        return math.copysign(mag, self.move_cmd)

    step_fork_move = step_move          # 老名字留着, 语义没变

    def start_fork_turn(self):
        """原地转 turn_deg 度对准 Y 的一条臂
        (fixed 模式 turn_deg=fork_turn_deg; yolo 模式由红绿灯定)"""
        self.fork_yaw0 = self.yaw_unw
        self.fork_t0 = time.time()
        self._fork_warned = False
        if self.fork_yaw0 is None:
            rospy.logwarn("没收到 /imu 姿态, 岔路转向退化成'按时间转', "
                          "角度会不准")
        self.set_phase("FORK_TURN", "岔路口: 原地转 %.0f° (%s)"
                       % (abs(self.turn_deg),
                          "右" if self.turn_deg < 0 else "左"))

    def step_fork_turn(self):
        """返回本帧该发的 angular.z; 转到位/超时返回 None(转完)"""
        want = math.radians(abs(self.turn_deg))
        sgn = -1.0 if self.turn_deg < 0 else 1.0
        el = time.time() - self.fork_t0
        if self.fork_yaw0 is not None and self.yaw_unw is not None:
            # 沿目标方向转过了多少(用多圈角相减, 不受 ±180/0~360 跳变影响)
            done = (self.yaw_unw - self.fork_yaw0) * sgn
            remain = want - done
            if el > 1.5 and done < math.radians(2.0) and not self._fork_warned:
                self._fork_warned = True
                rospy.logwarn("转了 %.1fs 但 imu 只测到 %.1f° —— 可能 "
                              "转向符号反了(负=右转), 或 imu 无数据",
                              el, math.degrees(done))
        else:
            remain = want - min(want, self.fork_speed * el)   # 无 imu: 按时间
        if remain <= math.radians(self.fork_tol_deg):
            rospy.loginfo("岔路转向完成: 实测 %.1f°",
                          math.degrees(want - max(0.0, remain)))
            return None
        if el > self.fork_timeout:
            rospy.logwarn("岔路转向超时 %.1fs, 还差 %.1f°, 强制继续",
                          el, math.degrees(remain))
            return None
        # 快到位时减速, 但不低于底盘旋转死区
        mag = min(self.fork_speed, max(self.az_min, 1.2 * remain))
        return sgn * mag

    def start_approach(self, gy):
        """车已停稳, 在这里打点(里程计不含刹车滑行), 然后继续巡线"""
        self.mark_xy = self.odom_xy
        self.mark_t = time.time()
        self.imu_v = self.imu_s = 0.0
        self.set_phase("APPROACH", "打点完成, 继续巡线 %.2fm 后停"
                       % self.goal_dist)
        if self.dist_src == "imu":
            rospy.logwarn("distance_source=imu: 二次积分会漂(2.7s 就能差"
                          "18cm), 轮子有编码器, 建议用 odom")
        elif not self.odom_seen:
            rospy.logwarn("没收到 /odom(base_driver 读编码器就在发这个), "
                          "退化成 速度x时间 估距, 误差较大")

    def estop_cb(self, msg):
        if msg.data:
            rospy.logwarn("收到 /lane_proto/estop -> 立即停车")
            self.kill_yolo()
            self.set_phase("STOPPED", "远程急停")

    def set_phase(self, ph, why=""):
        self.phase = ph
        self.state_pub.publish(String(data=ph))
        if ph == "STOPPED":
            if "急停" in why:
                res = "ESTOP"
            elif "超时" in why or "兜底" in why or "断流" in why:
                res = "ABORT"
            elif "尺寸" in why:
                res = "CONFIG"
            else:
                res = "GOAL"
            try:
                self.result_pub.publish(String(data=res))
                rospy.loginfo("/lane_proto/result = %s (%s)", res, why or "-")
            except Exception:
                pass
        rospy.loginfo("== %s ==%s", ph, (" " + why) if why else "")
        if (ph == "STOPPED" and not self._done_announced
                and "急停" not in why and "超时" not in why
                and "兜底" not in why):
            self._done_announced = True
            rospy.loginfo("== 终点 STOPPED, 播报任务完成 ==")
            try:
                subprocess.Popen(
                    [b"/usr/bin/python3",
                     b"/home/ucar/wake/tts_say.py",
                     u"任务完成".encode("utf-8")],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            except Exception as exc:
                rospy.logwarn("任务完成播报失败: %s", exc)

    def stop(self):
        """停车: 连发零速。
        底盘本身有看门狗(base_driver 的 cmd_timeout=0.2s: 超过就把三个
        速度分量强制归零), 所以零速包全丢也不会失控 —— 这里是加一道
        "立刻停"而不是等 0.2s。发满 0.6s(> 看门狗窗口)且逐个 try, 因为
        在 on_shutdown 里发布本身有竞态: 连接可能已经在拆, publish 会抛
        "publish() to a closed topic"。"""
        try:
            self.grab.stop()
        except Exception:
            pass
        try:
            if self.cap is not None:       # 交接模式下没开过 V4L2
                self.cap.release()
        except Exception:
            pass
        if self.dumper is not None:
            rospy.loginfo("诊断图后台线程: 写了 %d 张, 忙不过来丢了 %d 张"
                          "(丢的只是图, 控制没受影响)",
                          self.dumper.done, self.dumper.dropped)
            self.dumper.stop()
        self.kill_yolo()                 # 认灯进程别留着占显存
        zero = Twist()
        n_ok = 0
        for _ in range(12):                  # 12 x 50ms = 0.6s
            try:
                self.pub.publish(zero)
                n_ok += 1
            except Exception:
                pass
            time.sleep(0.05)
        if n_ok == 0:
            rospy.logwarn("零速一个都没发出去(话题已关), 靠底盘 %.1fs "
                          "看门狗兜底", 0.2)

    # 这几个参数每秒重读一次: 调参时直接 rosparam set, 不用重启
    # (重启要重新初始化相机+CUDA, 好几秒; 而且一停一起很难连续观察)
    def reload_params(self):
        old = (self.gain, self.dead, self.az_min, self.az_max, self.v,
               self.goal_y_lo, self.goal_dist)
        self.gain = float(rospy.get_param("~gain", self.gain))
        self.goal_y_lo = float(rospy.get_param("~goal_y_lo", self.goal_y_lo))
        self.goal_cover = float(rospy.get_param("~goal_cover",
                                                self.goal_cover))
        self.goal_line_cover = float(rospy.get_param("~goal_line_cover",
                                                     self.goal_line_cover))
        self.goal_line_count = int(rospy.get_param("~goal_line_count",
                                                   self.goal_line_count))
        self.goal_dist = float(rospy.get_param("~goal_stop_distance",
                                               self.goal_dist))
        self.dead = float(rospy.get_param("~min_intrusion", self.dead))
        self.az_min = float(rospy.get_param("~min_turn_speed", self.az_min))
        self.az_max = float(rospy.get_param("~turn_max", self.az_max))
        self.v = float(rospy.get_param("~linear_speed", self.v))
        new = (self.gain, self.dead, self.az_min, self.az_max, self.v,
               self.goal_y_lo, self.goal_dist)
        if new != old:
            rospy.loginfo("参数已更新: v=%.2f 终点扫描y_lo=%.2f 停前%.2fm  %s",
                          self.v, self.goal_y_lo, self.goal_dist,
                          curve_text(self.gain, self.dead, self.az_min,
                                     self.az_max))

    def _dump(self, und, mask, IL, IR, az, i, err, g=None):
        """主线程只做**打包**(拼备注串, 几十微秒), 真正的绘制+编码交给
        后台线程。备注里那些 self.* 是会变的, 所以必须在这里就取好值,
        不能让后台线程回头再读 —— 那样画出来的是几帧之后的状态。
        self._segs / _bseg / _pl / _pr 每帧都被 goal_block 换成新对象,
        这里存的是引用, 后台读到的仍是这一帧的那份, 安全。"""
        latest = os.path.join(self.dump_dir, "latest.jpg")
        cov, lbest, lcnt, gy, hit = g or (0.0, 0.0, 0, -1, False)
        _ = lbest
        note = ("%s  goal cov=%.2f/%.2f line=%.2f/%.2f n=%d/%d "
                "blk=%.2f %s%s") % (
            self.phase, cov, self.goal_cover, lbest, self.goal_line_cover,
            lcnt, self.goal_line_count, self._blk, "HIT" if hit else "-",
            ("  y=%d" % gy) if gy >= 0 else "")
        if self.is_fork:
            note += "  fork:%s" % ("done" if self.fork_done else "pending")
        if self.phase == "YOLO":
            note += "  tl=%s %s" % (self.yolo_last or "?",
                                    ",".join("%s:%d" % kv for kv in
                                             sorted(self.yolo_votes.items())))
        if self.phase in ("FORK_MOVE", "START_MOVE"):
            note += "  moved %.3f/%.3fm" % (self.moved_since_mark(),
                                            self.move_dist)
        if self.phase == "ALIGN":
            note += "  yellow row=%s tgt=%.0f" % (
                "--" if self.yellow_row is None else "%.0f" % self.yellow_row,
                self.yellow_target * self.H)
        if self.phase == "FORK_TURN" and self.fork_yaw0 is not None \
                and self.yaw_unw is not None:
            sgn = -1.0 if self.turn_deg < 0 else 1.0
            note += "  turned %.1f/%.0f deg" % (
                math.degrees((self.yaw_unw - self.fork_yaw0) * sgn),
                abs(self.turn_deg))
        if self.phase == "APPROACH":
            note += "  %.2f/%.2fm" % (self.moved_since_mark(), self.goal_dist)
        payload = dict(und=und, mask=mask, IL=IL, IR=IR, az=az, i=i,
                       err=err, note=note, latest=latest, lbest=lbest,
                       segs=self._segs, bseg=self._bseg,
                       pts=(self._pl + self._pr))
        if self.dump_async and self.dumper is not None:
            self.dumper.submit(payload)          # 后台画, 主循环立刻返回
        else:
            self._dump_now(payload)              # 同步(调试用)

    def _dump_now(self, p):
        """真正干活的那半: 只在后台线程里跑(dump_async=false 时才在主线程)"""
        self.dec.dump(p["und"], p["mask"], p["IL"], p["IR"], p["az"],
                      p["latest"], mirror=self.mirror,
                      err=p["err"], note=p["note"],
                      goal_y0=(int(IN_H * self.goal_y_lo)
                               if self.goal_enable else None),
                      goal_half=self.goal_half, goal_segs=p["segs"],
                      # 最佳线只在过阈时画, 否则那只是"零里挑一", 画出来
                      # 像检出了一样, 误导
                      goal_best=(p["bseg"]
                                 if p["lbest"] >= self.goal_line_cover
                                 else None),
                      goal_pts=p["pts"])
        # 用帧号命名: 按文件名排序 == 按时间排序, 编号最大的就是最新的。
        # dump_keep=0(默认) 不限额, 一直写 —— 每次启动会先清空目录。
        try:
            import shutil
            path = os.path.join(self.dump_dir, "f%06d.jpg" % p["i"])
            shutil.copyfile(p["latest"], path)
            if self.dump_keep > 0:
                self._dumped.append(path)
                while len(self._dumped) > self.dump_keep:
                    old = self._dumped.pop(0)
                    if os.path.exists(old):
                        os.remove(old)
        except Exception:
            pass

    @staticmethod
    def to_4x3(frame, W, H):
        """喂进来的帧不是 4:3 时**先中心裁再缩**, 不能直接压扁。
        标定是在相机的 640x480(=传感器中心裁出的 4:3)上做的; 拿 16:9 的
        帧硬压成 4:3 会把画面横向压缩, 去畸变映射就对不上了(线的角度、
        位置全变)。实机上相机就是 640x480 输出, 这里只是兜底 —— 万一
        驱动忽略了分辨率请求, 或者拿 1920x1080 的照片离线回放。"""
        h, w = frame.shape[:2]
        if w * H != h * W:                       # 宽高比不是 W:H
            if w * H > h * W:                    # 太宽 -> 裁两边
                cw = int(round(h * float(W) / H))
                x0 = (w - cw) // 2
                frame = frame[:, x0:x0 + cw]
            else:                                # 太高 -> 裁上下
                ch = int(round(w * float(H) / W))
                y0 = (h - ch) // 2
                frame = frame[y0:y0 + ch, :]
        if frame.shape[1] != W or frame.shape[0] != H:
            frame = cv2.resize(frame, (W, H))
        return frame

    def yolo_frame(self):
        """喂给认灯的那一帧(见 yolo_use_raw / yolo_zoom)。都是**已翻正**的。"""
        if self.yolo_use_raw and self._raw is not None:
            im = cv2.flip(self._raw, 1) if self.mirror else self._raw
        else:
            im = self._und
        if im is None or self.yolo_zoom <= 1.001:
            return im
        return self.zoom_center(im, self.yolo_zoom, self.yolo_zoom_cy)

    @staticmethod
    def zoom_center(im, z, cy=0.45):
        """数字变焦: 抠中间 1/z 的宽高再放大回原尺寸。远处的小灯靠这个救。"""
        h, w = im.shape[:2]
        cw, ch = int(w / z), int(h / z)
        x0 = max(0, min(w - cw, (w - cw) // 2))
        y0 = max(0, min(h - ch, int(cy * h - ch / 2)))
        return cv2.resize(im[y0:y0 + ch, x0:x0 + cw], (w, h),
                          interpolation=cv2.INTER_LINEAR)

    def step(self, frame, dump_i=None):
        t = time.time()
        frame = self.to_4x3(frame, self.W, self.H)
        self._raw = frame
        # 先去畸变再翻转: 鱼眼标定是在相机原始输出(镜像的)上做的,
        # 所以 remap 必须吃原始帧; 翻转是之后的纯几何操作。
        und = cv2.remap(frame, self.m1, self.m2, cv2.INTER_LINEAR)
        if self.mirror:
            und = cv2.flip(und, 1)
        # 认灯喂的就是这张(去畸变 + 已翻正)。⚠ 一定要喂翻正后的:
        # 相机原始输出是镜像的, 拿镜像图去认"左转/右转"箭头必然反。
        self._und = und
        t = self._lap("去畸变", t)
        # 分割: .so 里最后有 cudaMemcpy D2H(隐式同步), 所以这里量到的
        # 就是 GPU 真算完的时间, 不是"提交任务"的时间
        mask = self.seg.infer(und)                   # (192,320)
        t = self._lap("分割", t)
        IL, IR, err, nL, nR = self.dec.decide(mask)
        az = map_az(err, self.gain, self.dead, self.az_min, self.az_max)
        t = self._lap("决策", t)
        cov, lbest, lcnt, gy = 0.0, 0.0, 0, -1
        if self.goal_enable:
            (cov, lbest, lcnt, gy, self._segs, self._bseg,
             self._pl, self._pr, self._blk) = goal_block(
                mask, y_lo=self.goal_y_lo, half=self.goal_half,
                max_deg=self.goal_max_deg, line_cover=self.goal_line_cover)
        hit = (cov >= self.goal_cover and lbest >= self.goal_line_cover
               and lcnt >= self.goal_line_count)
        t = self._lap("横线", t)
        if dump_i is not None:
            self._dump(und, mask, IL, IR, az, dump_i, err,
                       (cov, lbest, lcnt, gy, hit))
            self._lap("dump", t)          # jpg 编码是同步的, 别小看它
        return IL, IR, az, nL, nR, (cov, lbest, lcnt, gy, hit)

    def _lap(self, name, t0):
        """记一段耗时并返回新的起点"""
        now = time.time()
        self.prof.add(name, (now - t0) * 1000.0)
        return now

    def ensure_seg(self):
        """真正要用分割网之前才把 .so 拉起来(懒加载)。
        幂等: TrackSeg 内部 ts_init() 有 g_ready 守卫, 重复调也只加载一次。"""
        if self.seg is not None:
            return
        rospy.loginfo("lane_follow: 加载 TrackSeg ...")
        self.seg = TrackSeg(self.trackseg_lib)
        rospy.loginfo("lane_follow: backend=%s dry_run=%s v=%.2f mirror=%s",
                      self.seg.backend, self.dry_run, self.v,
                      "ON(翻转)" if self.mirror else "OFF")

    def run(self):
        if self.enabled:
            self.ensure_seg()    # 独立跑: 进循环前就加载, 和以前行为一致
        else:
            rospy.loginfo("STANDBY: 等主流程调 /lane_proto/set_active 交接 "
                          "(单独测试请传 take_cam_on_start:=true)")
        rate = rospy.Rate(self.rate_hz)
        last_seq = -1
        stale = 0
        aligned = False
        hold = 0
        i = 0
        t0 = time.time()
        t_start = None if not self.enabled else time.time()
        t_prev = None            # 上一帧开始的时刻(算循环周期用)
        t_stats = time.time()
        while not rospy.is_shutdown():
            if not self.enabled:          # STANDBY: 不取帧不算不发速度
                t_prev = None
                rate.sleep()
                continue
            if t_start is None:           # 刚被交接: 所有计时从这一刻重新起算
                t_start = t0 = t_stats = time.time()
                last_seq, stale, i = -1, 0, 0
                self.prof.reset()
            # odom 断流保护(默认关): 主流程那边底盘要是掉了, 巡线还在发速度
            # 就是瞎开。只在真的在动的时候判, 且只触发一次。
            if (self.require_fresh_odom and self.phase != "STOPPED"
                    and self.odom_recv_t > 0.0 and not self.odom_is_fresh()
                    and self.speed_now() > 0.01):
                if not self._odom_stale_warned:
                    self._odom_stale_warned = True
                    self.set_phase("STOPPED", "里程计断流超过 %.2fs, 兜底停车"
                                   % self.odom_fresh_timeout)
            if self.max_runtime > 0 and time.time() - t_start > self.max_runtime \
                    and self.phase != "STOPPED":
                self.set_phase("STOPPED", "运行超过 %.0fs 上限, 兜底停车"
                               % self.max_runtime)
            t_grab = time.time()
            frame, seq = self.grab.latest()
            if frame is not None and not self._size_checked:
                self._size_checked = True
                self._check_frame_size(frame)
            if frame is None or seq == last_seq:      # 还没有新帧
                stale += 1
                if stale > self.rate_hz * 3:
                    rospy.logerr("3 秒没有新帧, 相机掉了? 停车退出")
                    break
                rate.sleep()
                continue
            stale = 0
            last_seq = seq
            self.prof.add("取帧", (time.time() - t_grab) * 1000.0)
            period = None if t_prev is None else (t_grab - t_prev) * 1000.0
            t_prev = t_grab

            if i % int(max(1, self.rate_hz)) == 0:
                self.reload_params()          # 每秒重读一次可调参数
            fork_cmd = move_cmd = None
            dump_i = i if (self.dump_dir and self.dump_every > 0
                           and i % self.dump_every == 0) else None
            IL, IR, az, nL, nR, g = self.step(frame, dump_i)
            cov, lbest, lcnt, gy, hit = g

            # ---- 起跑序列: 黄线对齐 -> 进 10cm -> 等绿灯 -> 进 30cm -> 拐
            # 放在这里(而不是 __init__)是因为要有第一帧图像才能干活。
            if self.fork_mode == "yolo" and not self.yolo_started \
                    and self.phase == "FOLLOW":
                self.yolo_started = True      # 这条起跑序列只走一次
                if self.align_yellow:
                    self.start_align()
                else:
                    self.after_align()

            # ---- 终点框相位机 ----
            self.goal_hits = (self.goal_hits + 1) if hit else 0
            confirmed = self.goal_hits >= self.goal_confirm
            if self.phase == "FOLLOW" and confirmed \
                    and time.time() >= self.cool_until:
                if self.is_fork and not self.fork_done:
                    self._pause_next = "fork"
                    if self.goal_pause > 0:
                        self.pause_until = time.time() + self.goal_pause
                        self.set_phase("PAUSE", "岔路口横线(覆盖%.2f), 先刹停"
                                       " %.1fs" % (cov, self.goal_pause))
                    else:
                        self.begin_fork()
                elif self.use_lidar:
                    self.set_phase("STOPPED", "检出终点框(覆盖%.2f 最佳线%.2f "
                                   "%d条), 立即停车, 等激光雷达导航接管"
                                   % (cov, lbest, lcnt))
                elif self.goal_pause > 0:
                    self.pause_until = time.time() + self.goal_pause
                    self.set_phase("PAUSE", "检出终点框(覆盖%.2f 最佳线%.2f "
                                   "%d条, 线在第%d行), 先刹停 %.1fs 打点"
                                   % (cov, lbest, lcnt, gy, self.goal_pause))
                else:
                    self.start_approach(gy)
            elif self.phase == "PAUSE":
                if time.time() >= self.pause_until:
                    if self._pause_next == "fork":
                        self.begin_fork()
                    else:
                        self.start_approach(gy)
            elif self.phase == "YOLO":
                cls = self.step_yolo()
                if cls is not None:
                    self.finish_yolo(cls)
            elif self.phase == "ALIGN":          # 用黄线顶到固定位置
                move_cmd = self.step_align()
                if move_cmd is None:
                    self.after_align()
            elif self.phase == "START_MOVE":     # 直线挪一段, 完了看下一步
                move_cmd = self.step_move()
                if move_cmd is None:
                    if self._move_next == "yolo":
                        self.start_yolo()
                    elif self._move_next == "fork_turn":
                        self.start_fork_turn()
                    else:
                        self.apply_branch()
            elif self.phase == "FORK_MOVE":
                move_cmd = self.step_move()
                if move_cmd is None:
                    self.start_fork_turn()

            elif self.phase == "FORK_TURN":
                fork_cmd = self.step_fork_turn()
                if fork_cmd is None:
                    self.fork_done = True
                    self._pause_next = "approach"
                    self.cool_until = time.time() + self.fork_cooldown
                    self.set_phase("FOLLOW", "继续巡线(%.1fs 内不认横线, "
                                   "免得分叉口那条线被当成终点)"
                                   % self.fork_cooldown)
            elif self.phase == "APPROACH":
                d = self.moved_since_mark()
                # 每帧都打(不是每秒): 终点进给一共就一两秒, 按秒打看不到过程
                rospy.loginfo(
                    "[终点进给] 已走 %.3fm 还剩 %.3fm  (odom %s  打点 %s  "
                    "源=%s)", d, max(0.0, self.goal_dist - d),
                    "(%.3f,%.3f)" % self.odom_xy if self.odom_xy else "无",
                    "(%.3f,%.3f)" % self.mark_xy if self.mark_xy else "无",
                    self._dist_src_used())
                # 硬超时: 按 距离/速度 的 3 倍 + 3s 兜底。测距源出任何问题
                # (odom 冻结、参数写错)都不能让车一直往前跑。
                budget = self.goal_dist / max(0.05, abs(self.v)) * 3.0 + 3.0
                if d >= self.goal_dist:
                    self.set_phase("STOPPED", "已前进 %.2fm, 到位停车 "
                                   "(算法认为剩 %.3fm)"
                                   % (d, self.goal_dist - d))
                elif time.time() - self.mark_t > budget:
                    self.set_phase("STOPPED", "!! 测距超时(%.1fs 只测到 %.2fm/"
                                   "%.2fm), 强制停车" % (budget, d,
                                                       self.goal_dist))

            tw = Twist()
            if self.phase in ("STOPPED", "PAUSE", "YOLO"):   # YOLO 期间停着认
                tw.linear.x, tw.angular.z = 0.0, 0.0
            elif self.phase in ("FORK_MOVE", "START_MOVE", "ALIGN"):  # 直着挪
                tw.linear.x, tw.angular.z = (move_cmd or 0.0), 0.0
            elif self.phase == "FORK_TURN":       # 原地转, 不前进
                tw.linear.x, tw.angular.z = 0.0, (fork_cmd or 0.0)
            else:                         # FOLLOW / APPROACH 都照常巡线
                tw.angular.z = az
                tw.linear.x = self.v      # 0 就是原地转, 不用改代码
            # 连续 5 帧(0.5s)落在死区内才算对准, 防抖
            hold = hold + 1 if (az == 0.0 and self.phase == "FOLLOW") else 0
            now_aligned = hold >= 5
            if now_aligned and not aligned:
                rospy.loginfo("✓ 已对准跑道 (IL=%.2f IR=%.2f)%s", IL, IR,
                              ", 停住" if self.v == 0.0 else "")
            elif aligned and not now_aligned:
                rospy.loginfo("偏了 (IL=%.2f IR=%.2f), 开始修正", IL, IR)
            aligned = now_aligned
            if not self.dry_run:
                self.pub.publish(tw)

            # 交接模式下跑完一趟就退出进程, 让主流程接着走下一步;
            # 独立测试时默认 false, 停在 STOPPED 里等你 Ctrl-C 看日志。
            if self.phase == "STOPPED" and self.exit_on_stop:
                rospy.loginfo("exit_on_stop: 已到 STOPPED, 退出节点")
                break

            if i % int(max(1, self.rate_hz)) == 0:    # 每秒一行
                fps = (i + 1) / max(1e-6, time.time() - t0)
                rospy.loginfo("深度 IL=%.3f IR=%.3f (px %d/%d) vx=%.2f "
                              "az=%+.3f %s%s (%.1f fps)", IL, IR, nL, nR,
                              tw.linear.x, az, self.dec.action_text(az),
                              " [dry_run未发速度]" if self.dry_run else "",
                              fps)
            # 一帧的活干完了(**不含 rate.sleep**): 忙时 vs 周期 一比就知道
            # 是"算不动"还是"在睡觉"
            self.prof.frame((time.time() - t_grab) * 1000.0, period)
            if self.stats_every > 0 and \
                    time.time() - t_stats >= self.stats_every:
                line, tip = self.prof.text(self.rate_hz, self.grab.cam_fps)
                if line:
                    if self.dumper is not None:
                        line += "  后台图 %d写/%d丢" % (self.dumper.done,
                                                       self.dumper.dropped)
                    rospy.loginfo("%s", line)
                    rospy.loginfo("       %s", tip)
                self.prof.reset()
                t_stats = time.time()

            i += 1
            rate.sleep()
        self.stop()


if __name__ == "__main__":
    rospy.init_node("lane_follow")
    node = LaneFollow()
    try:
        node.run()
    except rospy.ROSInterruptException:
        # Ctrl-C 时 rate.sleep() 会抛这个, 属正常退出路径, 别打 traceback
        pass
    finally:
        node.stop()          # on_shutdown 里也注册了, 双保险: 一定发零速
