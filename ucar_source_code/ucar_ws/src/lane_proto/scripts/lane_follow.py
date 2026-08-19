#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
lane_follow.py — 直道巡线测试原型 (python2/3 兼容, Melodic 下用 python2 跑)
====================================================================
管线: V4L2 直连相机(独占 /dev/ucar_camera, 不走 ROS 相机驱动)
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
import json
import os
import sys
import threading
import time
import subprocess

import math

import numpy as np
import cv2
import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, SetBoolResponse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trackseg import TrackSeg, IN_W, IN_H, _s  # noqa: E402
from lane_common import (load_template, Decider, map_az,  # noqa: E402
                         curve_text, goal_block, yellow_line, yellow_mask,
                         board_detect, scan_xy, find_seg_near,
                         corner_wall_fit)


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
            try:
                ok, f = self.cap.read()
            except Exception:
                # cap 被别的线程 release 掉时 read 会直接抛(实车 2026-08-18:
                # "Unknown array type in cvarrToMat"), 别让整个线程带着
                # traceback 死掉 —— 该退就安静退, 不该退就当一次失败
                if self.stopped:
                    break
                self.fail += 1
                time.sleep(0.02)
                continue
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

    def stop(self, join=0.0):
        """停线程。join>0 时等它真的退出, 返回是否退干净了。

        ⚠ 要 release cap 之前**必须**等它退出: 线程还卡在 cap.read() 里就
          release, OpenCV 的 V4L2 后端会一边 STREAMOFF 失败(EBUSY)一边把
          缓冲区抽掉, 线程炸、流停不下来、设备被占死, 之后什么分辨率都
          开不回来, 最后 GC 那个半死的 cap 时 SIGSEGV(实车 2026-08-18)。
        """
        self.stopped = True
        if join > 0 and self.is_alive() and \
                threading.current_thread() is not self:
            self.join(join)
        return not self.is_alive()


try:
    text_type = unicode          # noqa: F821  (py2)
except NameError:
    text_type = str


def format_ros_log(message, args):
    """py2 下先把日志格式化好, 再交给 rospy(它还会再格式化一次)。

    py2 的 % 只要混进一个 unicode, 整条格式串和所有 str 参数都会被按
    ASCII 解码 —— 模板里的中文当场 UnicodeDecodeError。这里先统一成
    unicode 格式化完, 只留给 rospy 一次纯 ASCII 的 %s 替换。

    ⚠ 相对队友原版补了一处: 原版直接 message.decode(), 第一个参数不是
    字符串(整数/对象/None)就 AttributeError。而这几个函数是**进程全局**
    覆盖到 rospy 上的, 同进程里任何库(tf/actionlib/...)调 rospy.loginfo
    都会走进来, 传个非字符串就炸, 而且报错位置在 lane_follow 里, 跟真正
    的调用方八竿子打不着。
    """
    if isinstance(message, bytes):
        message = message.decode("utf-8", "replace")
    elif not isinstance(message, text_type):
        message = text_type(message)
    if not args:
        return message.encode("utf-8")
    normalized = tuple(
        value.decode("utf-8", "replace") if isinstance(value, bytes)
        else value for value in args)
    try:
        return (message % normalized).encode("utf-8")
    except Exception:
        return (message + u" " + u" ".join(text_type(v) for v in normalized)
                ).encode("utf-8")


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


# 进程全局覆盖: 这是最后一道防线。具体调用点已经用 _s() 包过一遍了,
# 但"漏一处就炸一处"的教训在实车上吃过, 所以边界这里再兜一次。
rospy.loginfo = lane_loginfo
rospy.logwarn = lane_logwarn
rospy.logerr = lane_logerr
rospy.logerr_throttle = lane_logerr_throttle


def _parse_windows(spec):
    """"0.6:1.6,2.0:2.6" -> [(0.6,1.6),(2.0,2.6)]; 空/看不懂 -> []。
    看不懂只当没给, 不抛异常 —— 一个参数写错不该让整个节点起不来。"""
    out = []
    for part in str(spec).replace(u"，", ",").split(","):
        part = part.strip()
        if not part:
            continue
        try:
            a, b = part.split(":")
            lo, hi = float(a), float(b)
        except ValueError:
            continue
        if hi > lo:
            out.append((lo, hi))
    return out


# 这些相位不看图: 里程/雷达/IMU 驱动。断帧时可以继续发上一条指令,
# 不然底盘 cmd_timeout(0.2s) 一到就把车刹住了。
# ⚠ FOLLOW / APPROACH **不在**里面 —— 那两个是靠视觉闭环的, 断帧还硬走
#   等于闭着眼睛开。
BLIND_PHASES = ("START_MOVE", "FORK_MOVE", "FORK_TURN", "AVOID_REV",
                "AVOID_OUT", "AVOID_FWD", "AVOID_BACK", "AVOID_ALIGN",
                "AVOID_TURN0", "AVOID_POSE", "CORNER_ADJUST")


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


class BoardKF(object):
    """把拦路板当成一个**静止的线段路标**来跟踪, 融合 雷达 + odom/IMU。

    为什么要跟踪, 而不是每帧重新"检测":
      绕障过程中板子会从正前方转到侧面再转到正后方。侧面/后方的板子
      不在"挡路"的判据里(它本来就不挡路了), 重新检测必然丢。而三步
      走到哪一步为止, 恰恰要靠"板子现在相对车在什么方位"来判。

    状态: 板心在 **odom 系** 的 (x,y) —— 板子不动, 所以状态是常量,
    预测步只放大协方差(放大量代表 odom/IMU 的漂移)。观测来自雷达。

    观测噪声是**各向异性**的, 这是关键: 斜着看一块板子只看得见一段,
    拟合出来的中点会沿着板面方向明显偏移, 而垂直板面方向(距离)测得很准。
    所以 R 在板面方向给大方差、法线方向给小方差, 让滤波器自动少信
    那个会偏的分量。各向同性的 R 会被这种系统性偏差拖着走。
    """

    def __init__(self, sig_n=0.03, sig_u=0.25, q=0.06, gate=3.0):
        self.sig_n, self.sig_u, self.q = sig_n, sig_u, q
        self.gate2 = gate * gate
        self.x = None          # 板心 (odom)
        self.P = None
        self.u = None          # 板面单位方向 (odom), 初始化后不再变
        self.half = 0.21       # 板半长
        self.miss = 0          # 连续多少帧没关联上
        self.n_upd = 0

    def ready(self):
        return self.x is not None

    def start(self, mid_odom, u_odom, half):
        self.x = np.asarray(mid_odom, float)
        self.u = np.asarray(u_odom, float)
        self.u = self.u / max(1e-9, np.hypot(self.u[0], self.u[1]))
        self.half = float(min(0.35, max(0.12, half)))
        self.P = np.eye(2) * (0.06 ** 2)
        self.miss = 0
        self.n_upd = 1

    def predict(self, dt):
        if self.x is None:
            return
        self.P = self.P + np.eye(2) * (self.q * self.q * max(0.0, dt))

    def _R(self):
        n = np.array([-self.u[1], self.u[0]])
        U = np.column_stack([n, self.u])
        D = np.diag([self.sig_n ** 2, self.sig_u ** 2])
        return np.dot(np.dot(U, D), U.T)

    def update(self, z):
        if self.x is None:
            return False
        S = self.P + self._R()
        Si = np.linalg.inv(S)
        y = np.asarray(z, float) - self.x
        if float(np.dot(y, np.dot(Si, y))) > self.gate2:   # 马氏距离门限
            self.miss += 1
            return False
        K = np.dot(self.P, Si)
        self.x = self.x + np.dot(K, y)
        self.P = np.dot(np.eye(2) - K, self.P)
        self.miss = 0
        self.n_upd += 1
        return True

    def sigma_body(self, yaw):
        """位置不确定度投影到**车体系**, 返回 (纵向 sigma, 横向 sigma)。

        必须分方向, 不能取平均: 板子垂直于路线, 所以"沿板面方向"几乎
        就是车的横向 —— 而沿板面方向恰恰是最测不准的(一条没有特征的
        线段, 沿着它平移看起来一模一样, 所以 R 那个方向给了大方差)。
        平均一下会把"纵向测得很准"和"横向本来就不准"搅在一起,
        两个判据都拿到一个不痛不痒的中间值。
        实际上: 第②段(前进)只关心纵向, 第①段(横让)只关心横向。"""
        if self.P is None:
            return 9.9, 9.9
        c, s_ = math.cos(-yaw), math.sin(-yaw)
        R = np.array([[c, -s_], [s_, c]])
        Pb = np.dot(np.dot(R, self.P), R.T)
        return (float(math.sqrt(max(1e-9, Pb[0, 0]))),
                float(math.sqrt(max(1e-9, Pb[1, 1]))))

    def sigma(self):
        """给日志/信任门限用的标量: 取两个方向里更大的那个(保守)。"""
        if self.P is None:
            return 9.9
        w = np.linalg.eigvalsh(self.P)
        return float(math.sqrt(max(1e-9, w.max())))

    def body(self, car_xy, yaw):
        """板心在车体系的 (x前, y左); 再给出朝让向那一端的端点。"""
        d = self.x - np.asarray(car_xy, float)
        c, s = math.cos(-yaw), math.sin(-yaw)
        bx = c * d[0] - s * d[1]
        by = s * d[0] + c * d[1]
        ux = c * self.u[0] - s * self.u[1]
        uy = s * self.u[0] + c * self.u[1]
        return bx, by, ux, uy


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
                rospy.logerr("相机话题解码失败(只报一次): %s", _s(e))
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
        self.device = gp("~video_device", "/dev/ucar_camera")
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
        # 分阶段耗时统计: 每这么多秒打一行 [耗时]。**默认 0 = 关**——
        # 每 2 秒两行, 把真正要看的 [拦路板]/[对齐]/[认灯] 全刷没了。
        # 要看性能再传 stats_every:=2。
        self.stats_every = float(gp("~stats_every", 0.0))
        self.prof = Prof()
        # ---- 终点框 ----
        # use_lidar=true : 视觉命中后直接进入雷达角落闭环，不前进、不调用导航
        # use_lidar=false: 继续跑巡线, 按里程计再走 goal_stop_distance 米才停
        # 白线命中之后怎么进终点:
        #   self / true / lidar  巡线自己用雷达拟合两面墙, 闭环开进终点框
        #                        (CORNER_ADJUST, 默认)
        #   false                老路: PAUSE 1s 打点 -> 盲推 goal_stop_distance
        # ⚠ 三态字符串, 得自己解析: roslaunch 会把裸的 true/false 转成 bool,
        #   传 "self" 就是字符串, bool("false") 又是 True。
        _ul = gp("~use_lidar", "self")
        if isinstance(_ul, bool):
            self.use_lidar = _ul
        else:
            self.use_lidar = str(_ul).strip().lower() in (
                "self", "true", "lidar", "corner", "1", "yes", "on")
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
        self.max_runtime = float(gp("~max_runtime", 0.0))
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
        # 2026-08-16: 默认 2 -> 1。现场连续两帧太严, 灯切向/抖动时容易
        # 漏认卡住, 改成一帧认出方向就定案(误检风险由 yolo_conf 兜底)。
        self.yolo_min_votes = int(gp("~yolo_min_votes", 1))
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
        # 黄灯放不放行。默认**不放行**(当红灯处理, 继续等绿灯) —— 规则
        # 里黄灯是"减速停车", 抢黄灯被判罚的风险不值当。要放行传
        # yellow_go:=true, 那时 "yellow left" 就等同 "left"。
        # 黄灯放不放行: false / true / adaptive(默认)
        #   false    黄灯当红灯等
        #   true     黄灯等同方向灯, 直接走
        #   adaptive 前 yellow_go_after 秒当红灯等(赌它马上变绿, 抢一个
        #            干净的绿灯发车); 等过了还是黄, 就认了它走 —— 总比
        #            一路空等到 yolo_wait_max(60s) 再按 fallback 瞎走强。
        self.yellow_mode = str(gp("~yellow_go", "adaptive")).strip().lower()
        if self.yellow_mode in ("1", "yes", "on"):
            self.yellow_mode = "true"
        elif self.yellow_mode in ("0", "no", "off", ""):
            self.yellow_mode = "false"
        self.yellow_go_after = float(gp("~yellow_go_after", 10.0))
        # 认灯喂哪张图: false(默认)=去畸变后的; true=只翻正不去畸变的原始帧。
        # 去畸变会把画面往中间挤、边上留黑边, 灯要是被挤小了/糊了就试试 true
        # (yolo 的训练集本来也不是去畸变图)
        self.yolo_use_raw = bool(gp("~yolo_use_raw", False))
        # 起点看的是**远处**那个灯, 在 640x480 里可能只有十几个像素, 而
        # yolov4-tiny 进网还要缩到 640x352 —— 小目标本来就吃亏。zoom>1 就
        # 先把画面中间抠出来放大再送(纯数字变焦, 等效提高灯的分辨率)。
        # 2.0 = 抠中间 1/2 宽高。只用类别不用框, 所以不必把坐标映射回去。
        self.yolo_zoom = float(gp("~yolo_zoom", 1.0))
        # ---- 认灯用"高分辨率原生裁剪" ----
        # 相机开到更高分辨率, 从整帧里**按原始像素**抠一块 net 大小的窗口
        # 直接喂 yolo。和 yolo_zoom 的区别: zoom 是先抠再 resize 放大回去,
        # 插值出来的细节是假的; 这里是 1:1 原生像素, 灯有多少像素就是多少。
        # 640x352 的窗口在 1920x1080 上只占 1/3 宽, 等于 3 倍光学变焦。
        # 窗口中心按**归一化**给, 所以换什么分辨率都不用重算。默认值是从
        # 用户 2026-08-18 那张实拍量的: 灯箱 x=288~316 y=106~130 (640x360),
        # 中心 (301,118) -> (0.470, 0.327)。不在物理中心, 偏左偏上。
        self.yolo_crop = bool(gp("~yolo_crop", False))
        self.yolo_crop_cx = float(gp("~yolo_crop_cx", 0.470))
        self.yolo_crop_cy = float(gp("~yolo_crop_cy", 0.327))
        self.yolo_crop_w = int(gp("~yolo_crop_w", 640))
        self.yolo_crop_h = int(gp("~yolo_crop_h", 352))
        # 裁窗口认不出来的兜底: 认了这么久还没定案就改喂整帧(缩到 net 大小,
        # 也就是原来的行为)。灯要是没落在窗口里, 裁剪反而把它裁没了。
        self.yolo_crop_timeout = float(gp("~yolo_crop_timeout", 20.0))
        self._crop_gave_up = False
        self._crop_log = None            # 只在第一帧打一次窗口信息
        self._cam_switched = False       # 认灯期间是否临时切过分辨率
        self._cam_lock = threading.Lock()  # 换 cap 只能一个线程干
        self._preswitch_result = None    # 挪 140mm 时预切的结果(None=没预切)
        # 认灯期间临时切到的分辨率。相机支持 1920x1080/1280x720/800x600/
        # 640x480/640x360/320x240(实测 v4l2), 取最大那个。
        self.yolo_cam_w = int(gp("~yolo_cam_w", 1920))
        self.yolo_cam_h = int(gp("~yolo_cam_h", 1080))
        self._full = None                # 未经 to_4x3 的整帧(认灯用)
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
        # 认到灯之后冲到三岔口那一段的速度倍率。那段是**盲走固定里程**
        # (start_move 打 odom 点, step_move 用 moved_since_mark 计距),
        # 路上没有判断也没有障碍, 慢慢挪纯属浪费 —— 0.30m @0.12m/s 要
        # 2.5s, 加倍就是 1.25s。
        # ⚠ 只在**真认到灯**时加倍; 等超时走 fallback 那次不加 —— 那时候
        #   压根不知道前面是什么, 该稳着来。
        # ⚠ step_move 快到位会按剩余距离减速(下限 move_min_speed), 所以
        #   加倍主要吃在巡航段; 真过冲就把这个调回 1.0。
        self.start_dash_gain = float(gp("~start_dash_gain", 2.0))
        self._verdict_from_light = False   # 这次判定是真认到灯还是超时兜底
        self._yolo_cleanup_pending = False  # 发完车再杀 yolo/切回分辨率
        self._last_tw = None                # 最后发出去的那条指令(断帧兜底)
        self.cam_gap_hold = float(gp("~cam_gap_hold", 1.0))
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
        self.tpl_path = tpl_path
        # Y 支路专用触发区模板。Y 那一段和两臂的车道形状不一样(Y 是直的、
        # 更窄), 用同一张模板要么太松要么太紧, 所以允许单独换一张。
        # 空 = 不换, 全程用上面那张。
        # **开机就加载**, 不等转到 Y 才读 —— 路径写错的话现在就炸, 而不是
        # 车跑到岔口中间才炸(那时候车还在动)。
        self.tpl_y_path = gp("~template_y", "")
        self.dec_y = None
        if self.tpl_y_path:
            if os.path.abspath(self.tpl_y_path) == os.path.abspath(tpl_path):
                self.tpl_y_path = ""      # 一样就别多此一举
            else:
                self.dec_y = Decider(*load_template(self.tpl_y_path))
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
        rospy.loginfo("终点: %s", "检出后雷达两墙闭环(>1m退回原方案)"
                      if self.use_lidar
                      else "检出->刹停%.1fs打点->再走%.2fm(测距源 %s)停"
                      % (self.goal_pause, self.goal_dist, self.dist_src))
        if not self.mirror:
            rospy.logwarn("mirror=false: 若相机输出是镜像的(挡板字反着), "
                          "左右判决会整个反过来!")
        if self.v == 0.0:
            rospy.loginfo("linear_speed=0 -> 原地只转车体对准, 不前进")

        if self.use_ros_camera:
            self.cap = None
            self.cam_now = (self.W, self.H)
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
                rospy.logwarn("CAP_V4L2 不可用(%s), 退回默认后端", _s(e))
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
        self._fourcc = fcc                # 重开相机时要用同一套
        self._cam_fps_req = float(gp("~cam_fps", 0.0))
        if fcc and fcc != "NONE" and len(fcc) == 4:
            try:
                self.cap.set(cv2.CAP_PROP_FOURCC,
                             cv2.VideoWriter_fourcc(*fcc))
            except Exception as e:
                rospy.logwarn("设 fourcc=%s 失败: %s", _s(fcc), _s(e))
        # 请求的采集分辨率。默认就是标定尺寸(self.W x self.H), 给了
        # cam_w/cam_h 就按那个开 —— 认灯要更高分辨率时用(见 yolo_crop)。
        # ⚠ 开高之后巡线那条路不变(to_4x3 会裁缩回 640x480), 但**去畸变
        #   映射是按 640x480 标的**: 相机的 640x480 模式如果不是高分模式
        #   的等比缩放(很多 USB 相机是换了读出窗口/binning, 视场不一样),
        #   缩回来的帧喂进 maps_640 就是错的。开之前务必用 dump/latest.jpg
        #   和原来对一眼, 车道线位置/角度必须一致。
        req_w = int(gp("~cam_w", 0)) or self.W
        req_h = int(gp("~cam_h", 0)) or self.H
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, req_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, req_h)
        self.cam_req = (req_w, req_h)
        self.cam_now = (req_w, req_h)     # 当前实际在用的采集尺寸
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
            self.cam_now = (gw, gh)
            rospy.loginfo("相机协商结果: %dx%d @%.0f fps, 编码 %s "
                          "(请求 %s; 这里帧率低就别怪 CUDA)",
                          gw, gh, self.cap.get(cv2.CAP_PROP_FPS), fcc_got,
                          fcc or "未指定")
            # 协商结果和标定尺寸对不上 = 去畸变映射失配, 必须**吵**。
            # 之前就是被 to_4x3() 默默兜过去的: 画面看着正常, 实际视场是
            # 裁出来的, 线的角度和模板触发区全偏, 查了很久才找到。
            if (gw, gh) == self.cam_req and self.cam_req != (self.W, self.H):
                # 是**我们自己**要的高分辨率, 不是驱动乱给的 —— 提醒但放行
                rospy.logwarn("相机按请求开在 %dx%d(标定是 %dx%d)。巡线会由 "
                              "to_4x3 裁缩回标定尺寸; 认灯用整帧原生裁剪。"
                              "⚠ 务必对一眼 dump/latest.jpg, 车道线位置和"
                              "原来一致才说明这个模式的视场和标定一致。",
                              gw, gh, self.W, self.H)
            elif (gw, gh) != self.cam_req:
                msg = ("相机给的是 %dx%d, 而请求的是 %dx%d(标定 %dx%d) —— "
                       "去畸变会失配(to_4x3 只是兜底裁缩, 视场对不上)。"
                       % (gw, gh, self.cam_req[0], self.cam_req[1],
                          self.W, self.H))
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
        self.result_pub.publish(String(data="PENDING"))
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
        self.odom_yaw = None
        self.odom_finite = False
        # 硬要求: 没有 /odom 就**不许动**。
        # 以前默认是"没有就退回速度x时间接着跑", 结果 2026-08-16 实车整趟
        # 没有 odom, 里程全靠估, 板子检测在第一帧就被误判成"已走 3.56m ->
        # 本趟无板"而整个关掉。宁可不跑也不要瞎跑。
        self.require_odom = bool(gp("~require_odom", True))
        # 等 odom 的宽限期(s), 超了就判定底盘没起来, 停在 STOPPED。
        self.odom_wait_max = float(gp("~odom_wait_max", 10.0))
        self._odom_wait_t0 = None
        self._odom_wait_logged = 0.0
        self.require_fresh_odom = bool(gp("~require_fresh_odom", True))
        # ⚠ 别调回 0.5: 这台底盘的 odom/imu 有约 1.0~1.04s 的固有发布间隙
        # (ucar_2026 那边 EKF sensor_timeout 和任务的 odom/tf_timeout 都给
        # 3.0, navigation_scan_relay 的 transform_max_age 给 2.4, 同一个
        # 原因)。0.5 会在正常跑的时候不停误触发断流停车。
        self.odom_fresh_timeout = float(gp("~odom_fresh_timeout", 3.0))
        self._odom_stale_warned = False
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber("/lane_proto/estop", Bool, self.estop_cb, queue_size=1)
        self.enable_srv = rospy.Service("/lane_proto/set_active", SetBool,
                                        self.set_active)
        # ---- 拦路板(2D 雷达) ----
        self.board_on = bool(gp("~board_in_lane", False))
        self.board_stop = float(gp("~board_stop_dist", 0.90))
        self.board_lane_half = float(gp("~board_lane_half", 0.25))
        self.board_min_w = float(gp("~board_min_w", 0.24))
        self.board_max_w = float(gp("~board_max_w", 0.80))
        self.board_gap = float(gp("~board_gap", 0.12))
        self.board_min_pts = int(gp("~board_min_pts", 5))
        self.board_confirm = int(gp("~board_confirm", 3))
        self.board_min_travel = float(gp("~board_min_travel", 0.50))
        self.board_fov = float(gp("~board_fov_deg", 15.0))
        # 常规检测的前视距离(m)。>0 = 写死; 0 = 自动(见 board_range)。
        self.board_range_cfg = float(gp("~board_detect_range", 0.0))
        # Y 支路自动值的下限: 板子离岔口只有 1.1m, 前视太浅会来不及。
        self.board_range_y = float(gp("~board_detect_range_y", 1.60))
        # 航向闸: 车相对**进场航向**转过这么多度, 才开始认板子。
        # 这一条是冲着 Y 岔口那个红绿灯箱体去的 —— 车正对岔口(偏航≈0)时
        # 箱体就杵在正前方, 而那时候本来也不该有板子; 等车拐上两臂(±60°)
        # 或者拐进 Y 支路(45°)之后才开闸, 箱体自然就不在正前方了。
        self.board_arm_deg = float(gp("~board_arm_deg", 30.0))
        # 里程窗口: "0.6:1.6" 或 "0.6:1.2,2.0:2.6"; 空 = 不按里程卡。
        # 板子位置是固定的, 知道大概在哪一段就能把其余路段全部关掉。
        self.board_win = _parse_windows(gp("~board_dist_windows", ""))
        # ---- 绕障(go_around) ----
        # 麦轮底盘可以纯横移(base_driver 里 vw = x ∓ y ∓ w), 所以绕障就是
        # 三段开环: 横move 40cm -> 前进一段 -> 反向横move 40cm 回赛道。
        # 全程不看视觉(此时车压根不在车道上, 巡线的判决没有意义)。
        self.go_around = bool(gp("~go_around", False))
        # 横让多远。车实测 25.6cm 宽(footprint ±0.128, 与 urdf 的 box
        # 0.342x0.256 一致, 24 份 costmap 配置全一样), 板子边缘在 ±0.21:
        #   让 0.40 -> 车内侧边缘 0.40-0.128=0.272, 离板子边只剩 6.2cm
        #   让 0.45 -> 11.2cm
        # 而车道 42cm、车宽 25.6cm, 巡线本身就允许 ±4cm 的横向误差, 所以
        # 0.40 的最坏情况只剩 2cm —— 默认给 0.45。
        # 0 = 自动 = 板半长 + 车半宽 + 禁区。实测 footprint 半宽 0.128,
        # 板半长 0.21, 禁区 0.20 -> 0.538m。这只是闭环失效时的开环兜底,
        # 也是闭环的上限夹取值。
        self.ga_side = float(gp("~go_around_side", 0.0))
        # 前进多远: 0 = 自动 = **雷达量到的板子距离** + go_around_pass。
        # 拍一个固定值是不对的 —— 停车距离(board_stop_dist)一调, 需要往前
        # 走的距离跟着变; 而板子实际是在 d 米外被判到的, d 才是基准。
        self.ga_fwd = float(gp("~go_around_fwd", 0.0))
        # 越过板子的余量: 要让**车尾**(base_link 后 0.171m)也越过板子,
        # 所以下限是 0.171 + 板厚; 给 0.40 留足。
        # 0 = 自动 = 车半长 + 禁区(让车尾也越过禁区)
        self.ga_pass = float(gp("~go_around_pass", 0.0))
        self.ga_speed = float(gp("~go_around_speed", 0.12))
        self.ga_dir_cfg = str(gp("~go_around_dir", "auto")).strip().lower()
        self.ga_max = int(gp("~go_around_max", 1))
        self.ga_cool = float(gp("~go_around_cooldown", 3.0))
        # 绕完之后**终点横线**要压制多久。和 go_around_cooldown 拆开是因为
        # 那一个参数原来同时管两件事, 而两件事的道理完全不同:
        #   不认板子(ga_cool=3.0)  —— 合理: 刚绕完别立刻又把同一块板认出来
        #   不认横线(这一个, 默认0)—— 没道理: 绕完板子在**车屁股后面**,
        #     相机朝前根本看不见, 不存在"把板面当终点线"; 而防这个的本来
        #     就是里程闸(两臂 2.40m), 那时早就放行了。
        # 实车 2026-08-18 就栽在这: 终点线以 cov=1.00/line=1.00/16条 满分
        # 检出, 却整段落在 3s 冷却里被丢弃, 线滑出扫描带比闸门打开只早了
        # **0.045 秒**; 车又往前跑了 0.46m 才认到下一条横线, 在那儿打点再
        # 推 0.5m, 撞墙。
        # ⚠ 只关"绕障后"这一次。**岔口转弯后那次冷却照旧 3.0s** —— 那是防
        #   分叉口自己那条横线被当成终点, 那个风险是真的。
        self.ga_cool_goal = float(gp("~go_around_cooldown_goal", 0.0))
        # 闭环: 用雷达跟踪板子(+odom/IMU 卡尔曼)判断每一段走没走完;
        # fixed = 退回按固定距离开环走。
        self.ga_mode = str(gp("~go_around_mode", "track")).strip().lower()
        # 板子周围的禁区。**长方形**, 不是把线段膨胀成胶囊 —— 车全程横平
        # 竖直地走(航向角不变), 用不着圆角, 长方形算起来也简单: 横向和
        # 纵向两条约束互相独立, 各自加一次就完了。
        #   横向到位: 车侧边 到 板端  >= keepout  ->  |板端横向| >= 车半宽+keepout
        #   纵向到位: 车前/后边 到 板面 >= keepout  ->  板心在车心后方 车半长+keepout
        #   横回到位: 板心横向回到 0(板心就在路中心线上)
        # ⚠ 长方形版多一条**前提**: 第①段是纯横移, 横移期间车必须已经在
        #   纵向禁区带外面, 否则是贴着板面横着蹭过去。见 start_go_around
        #   里的 d >= 车半长 + keepout 检查。
        self.ga_keep = float(gp("~go_around_keepout", 0.20))
        self.ga_clear = float(gp("~go_around_clear", self.ga_keep))
        self.ga_tail = float(gp("~go_around_tail", self.ga_keep))
        self.ga_back_tol = float(gp("~go_around_back_tol", 0.02))
        # 横回段的"离中垂线多远"用哪个量:
        #   odom(默认) 起绕那一刻(正面看, 整块板都在视野里, 估计最准)冻结板面
        #        方向 u0 和当时的横向偏差 lat0, 之后 lat = lat0 + (odom 位移)·u0。
        #        odom 十几秒内漂不了几毫米, 而卡尔曼从侧面/背面看板子只看得见
        #        一小段弦(背面时车身挡掉大半, 实车 2026-08-19 只见 0.19m/0.42m),
        #        拿弦中点当板心, 估计一路往看得见的那一侧偏 —— 回中"到位"了
        #        实际还差 10cm。
        #   kf   老逻辑, 用卡尔曼估的板心。
        self.ga_back_src = str(gp("~go_around_back_src", "odom")).strip().lower()
        # AVOID_POSE 到位(法向距 / 航向 / 离中垂线三项都进容差)后再稳这么久
        # (秒)才横移: 边转边挪时估计还在动, 让它有时间真正对上板子中垂线;
        # 稳定期里哪项又出了容差就接着调, 计时重来。0 = 到位立刻横移。
        self.ga_pose_settle = float(gp("~go_around_pose_settle", 1.0))
        self._ga_pose_ok_t = None
        self._ga_u0 = None            # 起绕时冻结的板面方向(odom 系)
        self._ga_p0 = None            # 起绕时车心(odom)
        self._ga_lat0 = 0.0           # 起绕时离中垂线多远(正面看的估计)
        self._ga_lon0 = 0.0           # 起绕时车心到板面的法向距
        self._ga_rear_n = 0           # 横回段背面重捕获成功次数
        self._ga_rear_last = None     # 最近一次量到的法向距
        # 闭环只允许在开环几何目标的 [min_frac, 1.25] 倍之间修正
        self.ga_min_frac = float(gp("~go_around_min_frac", 0.85))
        # 板子张角小于这么多个角度分辨率就认为"几何上看不见", 纯预测滑过去
        self.ga_min_span = float(gp("~go_around_min_span", 5.0))
        # 终点横线要不要用雷达复核(默认跟着 board_in_lane 走)
        self.goal_need_clear = bool(_tri(gp("~goal_need_clear", "auto"),
                                         self.board_on))
        self.goal_clear_rng = float(gp("~goal_clear_range", 1.5))
        # 板子三态: 未知 -> (已避 | 确认无板)。
        # 从图上量的(图是准的): 板子在路程 52%~63% 处 —— 两臂 2.25m、
        # Y 支路 2.02m(从黄线算); 终点在 3.58m / 3.87m。换成代码实际测的
        # "进 FOLLOW 后走了多远"(起跑序列约占 0.5m):
        #     最晚的板子 1.76m, 最早的终点 3.08m  -> 中间 1.32m 空档
        # 阈值取 2.40m 落在空档正中, 两头各留 0.64m 余量。
        # 走过它还没见到板子 = 这趟没有板子, 果断切回初赛逻辑:
        # 关掉整套板子检测, 放开终点白线检测。
        self.board_state = "未知"
        self.board_clear_dist = float(gp("~board_clear_dist", 2.40))
        # Y 支路从**岔口**起算, 不从起跑算: 岔口是"检出横线"这个真事件,
        # 不是从黄线航位推算出来的, 所以这一段彻底不受"起跑序列占多少米"
        # 那个估计值的影响。图上量: 从岔口起 板子 1.10m / 终点 2.94m,
        # 阈值取中 2.00m, 两头各留 0.9m。
        self.board_clear_dist_y = float(gp("~board_clear_dist_y", 2.00))
        self.board_anchor = "起跑"
        self._board_trav = 0.0        # 当前锚点起算走了多远(给日志用)
        # Y 支路的**早判**: 转完 45° 之后板子就在正前方 1.1m(图上量的),
        # 张角 21.6° 约 54 个点, 完全看得见。所以不用等着开过去 ——
        # 岔口这一下就能定死"这条支路上有没有板子"。
        # 红绿灯在岔口尖端(两支路中间的岛上), 转完 45° 后落到侧后方,
        # 本来就不在前向走廊里, 不会被当成板子。
        # 两臂做不了早判: 板子在 1.76m 外而且臂是弯的, 那时不在正前方。
        self.board_look = float(gp("~board_look_ahead", 1.60))
        self.board_early_dist = float(gp("~board_early_dist", 0.35))
        self._early_done = False
        self._early_hits = 0
        self._veto_warn = False
        self._veto_log_t = 0.0
        # 终点判据: visual = 只看白线(默认) / both = 白线 **或** 地图定位
        # (离 goal_map_xy 不到 goal_map_dist 米)都触发。白线那一路始终是视觉,
        # 地图那一路靠完整版驱动里 robot_pose_publisher 发的 map->base_link
        # 位姿(/robot_pose, geometry_msgs/Pose 或 PoseStamped 都收); 精简版
        # 没这个 topic, both 就等于 visual, 不报错不告警。
        # 旧值 vision/wall 兼容: vision->visual, wall->both(整场矩形拟合那套
        # 已经删了, 别再用)。
        gm = str(gp("~goal_mode", "visual")).strip().lower()
        self.goal_mode = {"vision": "visual", "wall": "both"}.get(gm, gm)
        if self.goal_mode not in ("visual", "both"):
            rospy.logwarn("goal_mode=%r 不认识, 按 visual", gm)
            self.goal_mode = "visual"
        self.pose_topic = str(gp("~pose_topic", "/robot_pose"))
        self.goal_map_dist = float(gp("~goal_map_dist", 0.50))
        self.pose_fresh = float(gp("~pose_fresh", 1.0))
        self.goal_map_xy = None
        gxy = str(gp("~goal_map_xy", "")).strip()
        if gxy:
            try:
                parts = [float(v) for v in gxy.replace(";", ",").split(",")]
                if len(parts) != 2:
                    raise ValueError("要 2 个数")
                self.goal_map_xy = (parts[0], parts[1])
            except Exception as e:
                rospy.logwarn("goal_map_xy=%r 解析失败(%s), 地图终点不用", gxy, e)
        if self.goal_mode == "both" and self.goal_map_xy is None:
            rospy.logwarn("goal_mode=both 但没给 goal_map_xy(地图系终点坐标), "
                          "地图那一路不会触发 -> 等于 visual")
        self._map_pose = None            # (x, y, yaw) map 系
        self._map_pose_t = 0.0
        self._map_pose_type = ""
        self._map_log_t = 0.0
        self._map_why = ""
        if self.goal_mode == "both" and self.goal_map_xy is not None:
            # AnyMsg: Pose / PoseStamped 都收, 按连接头里的类型再反序列化;
            # 订错类型 rospy 会拒连并刷 ERROR, 用 AnyMsg 就没这个问题
            rospy.Subscriber(self.pose_topic, rospy.AnyMsg, self.pose_cb,
                             queue_size=1)
            rospy.loginfo("终点: 白线(视觉) 或 地图定位 %s 离 (%.2f, %.2f) "
                          "< %.2fm; 没这个 topic 就只剩白线",
                          self.pose_topic, self.goal_map_xy[0],
                          self.goal_map_xy[1], self.goal_map_dist)
        # ---- 终点角落雷达闭环 --------------------------------------
        # 目标距离是雷达坐标已换算到 base_link 车心后的距离，不是雷达
        # 光心到墙的原始 range。默认 0.25m，对应用户要求的 0.24~0.26m。
        self.corner_target_dist = float(gp("~corner_target_dist", 0.25))
        # 侧墙(车道那一侧的围挡)目标距离: 车道 42cm, 车在道中间就是 0.21
        self.corner_target_side_dist = float(
            gp("~corner_target_side_dist", 0.21))
        self.corner_target_tol = float(gp("~corner_target_tol", 0.01))
        # 必须能看到 1m 以上的墙，才能决定是否退回原来的前进停车。
        self.corner_max_fit_dist = float(gp("~corner_max_fit_dist", 2.00))
        self.corner_fallback_dist = float(
            gp("~corner_fallback_dist", 1.00))
        # 身后 back_excl 度锥内的线不当墙(那是刚绕过的拦路板); 只拟到一条
        # 时它得在前方 ±front_half 度内才按"只有前墙"闭环
        self.corner_back_excl_deg = float(gp("~corner_back_excl_deg", 40.0))
        self.corner_front_half_deg = float(gp("~corner_front_half_deg", 60.0))
        # 航向对齐: 离最近车体轴 <= 这个角才去转(转到和墙平行); 更斜(比如
        # 顶着角尖 45° 进场)就不转了, 直接沿墙法向斜着滑进去 —— 终点只看
        # 位置不看朝向, 转 45° 白花好几秒
        self.corner_yaw_align_max_deg = float(
            gp("~corner_yaw_align_max_deg", 35.0))
        # true = 老逻辑: 先把航向转到位再平移; false(默认) = 边转边平移
        # (平移沿**当帧量到的墙法向**走, 转不转都不耦合)
        self.corner_turn_first = bool(gp("~corner_turn_first", False))
        self.corner_min_pts = int(gp("~corner_min_pts", 8))
        self.corner_min_span = float(gp("~corner_min_span", 0.12))
        self.corner_max_residual = float(
            gp("~corner_max_residual", 0.025))
        self.corner_wall_angle_tol_deg = float(
            gp("~corner_wall_angle_tol_deg", 10.0))
        self.corner_cluster_gap = float(gp("~corner_cluster_gap", 0.05))
        self.corner_kp = float(gp("~corner_kp", 0.80))
        self.corner_max_speed = float(gp("~corner_max_speed", 0.08))
        self.corner_yaw_kp = float(gp("~corner_yaw_kp", 0.80))
        self.corner_yaw_hold_deg = float(
            gp("~corner_yaw_hold_deg", 5.0))
        self.corner_max_yaw_speed = float(
            gp("~corner_max_yaw_speed", 0.16))
        self.corner_stable_frames = int(
            gp("~corner_stable_frames", 5))
        self.corner_timeout = float(gp("~corner_timeout", 30.0))
        # 进 CORNER_ADJUST 后这么久还没拟合到前墙就退回盲推(秒)。以前是原地
        # 干等到 corner_timeout(30s) 才 STOPPED —— 车停在半路不动。
        self.corner_no_wall_grace = float(gp("~corner_no_wall_grace", 1.5))
        # 停滞判定: 这么多秒没进展就升级(放宽容差 -> 直接下一步)
        self.corner_stall_s = float(gp("~corner_stall_s", 3.0))
        self._corner_level = 0
        self._corner_yaw_forced = False
        self._corner_best_prog = float("inf")
        self._corner_prog_t = 0.0
        # 终点雷达闭环逐帧落盘(dump/corner_trace_NN.jsonl), 默认开
        self.corner_trace_on = bool(gp("~corner_trace", True))
        self._corner_fh = None
        self._corner_n = 0
        self._corner_trace_warned = False
        self._corner_first_ok_t = None
        self.corner_t0 = None
        self.corner_stable = 0
        self._corner_log_t = 0.0
        self._corner_last_fit = None
        # 板子位置标准差超过这个就不再信闭环(退回开环)。纯预测下
        # q=0.06 m/√s, 6cm 起步, 约 1.8s 才涨到 8cm。
        self.ga_max_sigma = float(gp("~go_around_max_sigma", 0.08))
        self.ga_k_sigma = float(gp("~go_around_k_sigma", 1.0))
        self.ga_blind = 0
        self.ga_miss_max = int(gp("~go_around_miss_max", 8))
        self.car_half_w = float(gp("~car_half_width", 0.128))  # 实测 footprint
        self.car_half_l = float(gp("~car_half_length", 0.171))
        # 参数自洽性检查: board_stop_dist 必须大于 车半长+禁区, 否则每次
        # 触发都会先倒车。这两个数**说的是同一段纵向距离** —— 车头到板子
        # 的间隙就是横移时车头角需要的纵向余量, 所以"车头间隙"必须 > 禁区。
        _need = self.car_half_l + self.ga_keep
        if self.go_around and self.board_stop < _need:
            rospy.logwarn("board_stop_dist=%.3f 比横移站位 %.3f(车半长 %.3f + "
                          "禁区 %.3f)还近, 每次绕障都要先**倒车**才能横移。"
                          "把它调大就是纯前进: board_stop_dist:=%.2f 起",
                          self.board_stop, _need, self.car_half_l,
                          self.ga_keep, round(_need + 0.05, 2))
        self.kf = BoardKF()
        self.ga_lost = 0
        self.ga_n = 0                 # 已经绕过几次
        self.ga_fwd_now = 0.0         # 本次绕障实际要前进多远
        self.ga_out_dist = 0.0        # 第一段实际横让了多少(第三段照它横回)
        self.ga_rev = 0.0             # 检出太近时先倒多少
        self.ga_sign = 0.0            # +1 = 往左让, -1 = 往右让
        self.board_cool_until = 0.0   # 绕障刚结束的一小段不再认板子
        self.board_yl = self.board_yr = 0.0   # 最近一次检出的板子横向范围
        self.board_d = 0.0            # 最近一次检出时板子有多远
        self.az_hist = []             # 最近约2秒的转向指令, 给方向决策交叉验证
        # 雷达在 base_link 后 11cm(urdf 里 -0.11 0 0.165, 和 ucar_bringup
        # 的静态 TF 一致), 所以量到的 d 是**从车体中心算**的, 车头还在前面
        # 0.171m 处。
        # 自车剔除余量。车上两根小 WiFi 天线立在 footprint 里面, 雷达扫得
        # 到, 会打出 0.1~0.3m 的近距离回波。scan_xy 按**位置**把落在车身
        # 矩形(±0.171 x ±0.128)+ 这个余量之内的点全丢掉 —— 不能改用"最小
        # 距离"一刀切: 板子实测可以停在 0.285m, 一刀切会把板子也切没。
        # 天线要是支出车外一点, 把这个调大; 0 = 只剔 footprint 内部。
        # 绕障扫描图的画面半宽(米)。2.0 = 画 4x4m, 5x2.5m 的场地够看全。
        self.ga_dump_span = float(gp("~go_around_dump_span", 2.0))
        # 终点横线的里程闸(m): 从 FOLLOW/岔口起算, 没走够就不认终点线。
        # -1 = 自动: 开了板子检测就用 board_clear_now()(两臂 2.40 / Y 2.00),
        # 没开就是 0(不设闸, 和初赛行为一致)。0 = 明确关掉。
        self.goal_min_travel = float(gp("~goal_min_travel", -1.0))
        # goal_min_travel=-1(自动)时用的两个值。**故意和 board_clear_dist
        # 解耦**: 那个是"走这么远还没检出就判无板", 两臂已放宽到 3.00m;
        # 而两臂最早的终点在 3.08m —— 终点闸要是跟着变成 3.00 就只剩 8cm
        # 余量, odom 尺度差 6% 就永远认不到终点。这两个数仍取"板子最晚
        # 位置(两臂 1.76 / Y 1.10)与最早终点(3.08 / 2.94)之间"的空档。
        self.goal_gate_arm = float(gp("~goal_gate_arm", 2.40))
        self.goal_gate_y = float(gp("~goal_gate_y", 2.00))
        self._goal_gate_t = 0.0
        self._goal_why = None        # 这一帧 HIT 却没停的原因(给 dump 图用)
        self._goal_why_last = ""     # 上一条打过的原因(原因一变就重新打)
        self.board_self_margin = float(gp("~board_self_margin", 0.03))
        self.board_lidar_x = float(gp("~board_lidar_x", -0.11))
        self.board_yaw_off = math.radians(float(gp("~board_lidar_yaw_deg", 0.0)))
        self.scan = None
        self.scan_t = 0.0
        self.scan_stale = float(gp("~scan_stale", 0.4))
        self.board_hits = 0
        self.t_boot = time.time()
        self.yaw_entry = None       # 进场航向(起跑序列开始那一刻的 yaw)
        self._yaw_warned = False
        self.board_seen_t = 0.0
        self._board_log_t = 0.0
        self._scan_warned = False
        if self.board_on or self.use_lidar:
            rospy.Subscriber(str(gp("~scan_topic", "/scan")), LaserScan,
                             self.scan_cb, queue_size=1)
        if self.use_lidar:
            rospy.loginfo("终点停车: 视觉命中后进入雷达角落闭环，不调用导航")
        if self.board_on:
            rospy.loginfo("拦路板检测: 开 (话题 %s, 停车距离 %.2fm, 宽度 "
                          "%.2f~%.2fm, 连续 %d 帧, 走够 %.2fm 才开始查)",
                          str(gp("~scan_topic", "/scan")), self.board_stop,
                          self.board_min_w, self.board_max_w,
                          self.board_confirm, self.board_min_travel)
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
        # use_lidar=true 时: FOLLOW -> CORNER_ADJUST -> STOPPED
        # 交接模式下先待在 STANDBY: 不发速度、不加载 CUDA, 等主流程调
        # /lane_proto/set_active 再进 FOLLOW。
        self.phase = "FOLLOW" if self.enabled else "STANDBY"
        self._done_announced = False
        self.pause_until = 0.0
        self._corner_cmd = (0.0, 0.0, 0.0)
        self._segs, self._bseg, self._pl, self._pr = [], None, [], []
        self._blk = 0.0                 # 扫描带里被红绿灯箱体挡掉的列比例
        self._pause_next = "approach"   # PAUSE 结束后干什么: approach / fork
        self.fork_done = False          # 岔路已经拐过了
        self.fork_yaw0 = None           # 起转时的多圈 yaw
        self.fork_t0 = 0.0
        # 绕障逐帧轨迹(原始雷达 + 判定量), 给离线对账用
        self.ga_trace_on = bool(gp("~go_around_trace", True))
        # 绕障收尾时用板子法向把航向也归位(度)。板子横跨赛道, 法向 = 车道
        # 方向。0 = 不对齐(保持"航向角全程不变"的老行为)。
        self.ga_align_deg = float(gp("~go_around_align_deg", 3.0))
        # 两臂的车道中心线半径(米), 0 = 关掉曲率修正。
        #
        # 为什么需要它: 绕板三段全在**板子系**里走直线, 而板法线只是
        # **板子那一点**的车道切线。第二段前进那 d 米走的是直线, 车道却是
        # 弧 —— 等车走到板后 d 米, 真正的车道中心线已经转过 φ=asin(d/R)、
        # 并往弯道内侧偏了 R(1-cosφ)。于是收尾"回中垂线 + 对齐板法向"把车
        # 摆在了一个过期的基准上, 航向和横向双双差一截, 后半程越走越偏。
        # 用户实测(2026-08-18): 偏航 15°、横向 5cm。这两个数是自洽的 ——
        #   由 15° 反解 R = 0.371/sin15°    = 1.433m
        #   由 5cm 反解 R = 0.05/(1-cos15°) = 1.467m
        # 差 2%, 是同一个圆, 所以默认取 1.43。
        # ⚠ 修正量跟着**实际**前进距离走(用板子系的 lon), 不是写死 15°:
        #   前进 0.371m -> 15.0°/4.9cm;  0.45m -> 18.3°/7.2cm;
        #   0.58m -> 23.9°/12.3cm(加了 sigma 余量时会走这么远)
        # ⚠ 只在**两臂**上修: Y 支路是直线, 没有曲率。
        self.arc_r = float(gp("~board_arc_r", 1.43))
        self.arc_max_deg = float(gp("~board_arc_max_deg", 30.0))
        # 横向修正的倍率。航向和横向本来是同一个 R 算出来的(phi=asin(d/R),
        # off=R(1-cos phi)), 绑死在一起 —— 调 R 会同时动两个。这个倍率把
        # **横向**单独拎出来, 航向不受影响:
        #   1.0 = 按几何算(默认)   0 = 只修航向不修横向
        #   0.5 = 只修一半
        # 为什么留这个口子: 航向 15° 是"车头朝向对不对", 错了后半程会一路
        # 越走越偏, 修它稳赚; 横向 5cm 是"站位偏不偏", 而收尾之后马上就回到
        # 视觉巡线, 巡线本身就在纠横向偏差 —— 万一 R 估得不准, 横向修过头
        # 反而是给巡线添乱。所以这两件事该能分开调。
        self.arc_lat_scale = float(gp("~board_arc_lat_scale", 1.0))
        self._ga_az = 0.0
        self._ga_lat_min = 9.9      # AVOID_BACK 过冲保护: 见过的最小 |lat|
        self._ga_preturn = False    # 这一次绕障要不要先转正
        # 横回段的过冲保护要走够这么远才上膛(m)。太小会被"车还没起步、
        # 板子估计在漂"的那几帧误触发。
        self.ga_back_arm = float(gp("~go_around_back_arm", 0.08))
        self._ga_fh = None
        self._ga_trace_warned = False
        # 指令速度积分(m): 主循环每帧 += speed_now()*dt。没有 /odom 时
        # board_travel 拿它当测距源 —— 见那边的注释。
        self.cmd_s = 0.0
        self.t_follow0 = [None, None, None]
        self._fork_warned = False
        self.state_pub.publish(String(data=self.phase))   # latch 出起始相位
        # 本次岔路实际要转的角度: fixed 模式=fork_turn_deg, yolo 模式由灯决定
        self.turn_deg = self.fork_turn_deg
        self.start_turn_deg = 0.0   # 起点那次转角(±60 走臂 / 0 走中间直行)
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
        # 姿态直接取 odom 自带的四元数: 和 odom_xy 同一个来源, 拿它做
        # 车体<->odom 的旋转是自洽的。混用 IMU 的 yaw 会引入一个常值
        # 偏置(两者原点不同), 在 1~2m 的绕障里就是几厘米的横向误差。
        try:
            q = msg.pose.pose.orientation
            self.odom_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        except Exception:
            self.odom_yaw = None

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

    def board_yaw_dev(self):
        """车相对进场航向转过了多少度(0~180)。IMU 没数据就返回 None。"""
        if self.yaw_unw is None or self.yaw_entry is None:
            return None
        d = math.degrees(self.yaw_unw - self.yaw_entry)
        while d > 180.0:
            d -= 360.0
        while d < -180.0:
            d += 360.0
        return abs(d)

    def board_expect_dev(self):
        """IMU 不可用时的退路: 按**指令**推算应该转过多少度。
        两臂是 ±yolo_turn_deg(60), 直行那条要等拐进 Y 支路(fork_turn_deg)。
        比 IMU 粗, 但它不会因为 IMU 掉线就把整个功能哑掉。"""
        if abs(self.turn_deg) > 1.0:
            return abs(self.turn_deg)
        return abs(self.fork_turn_deg) if self.fork_done else 0.0

    def board_armed(self):
        """航向闸开了没有。返回 (开没开, 用来打日志的说明)"""
        if self.board_arm_deg <= 0.0:
            return True, "闸关闭"
        dev = self.board_yaw_dev()
        if dev is None:
            dev = self.board_expect_dev()
            src = "指令推算"
            if not self._yaw_warned:
                self._yaw_warned = True
                rospy.logwarn("拦路板航向闸: 拿不到 IMU 偏航(yaw_unw/进场航向 "
                              "缺一), 退回按指令角度推算")
        else:
            src = "IMU"
        return dev >= self.board_arm_deg, "%s偏航 %.0f°/%.0f°" % (
            src, dev, self.board_arm_deg)

    def board_early_verdict(self, trav):
        """Y 支路在岔口后那一小段里直接看死"有没有板子"。
        返回 True 表示这一帧已经把结论定下来了(不管有板还是没板)。"""
        # ⚠ 只对**走中间那条 -> Y 支路**生效。两臂的 ±60° 起点转向也走
        # FORK_TURN 相位, 转完同样会把 board_anchor 置成"岔口", 于是早判
        # 在两臂上也跑了 —— 而它的前提("板子应在 ~1.1m 处, 1.60m 内看得
        # 很清楚")只对 Y 支路成立: 两臂的板子在 2.21m 外, 早判必然报
        # "没有板子", 把整趟板子逻辑关掉。2026-08-16 实车右臂就是这么
        # 把板子当成终点线撞上去的。
        if abs(self.start_turn_deg) > 1.0:
            return False
        if self._early_done or self.board_anchor != "岔口":
            return False
        if not self.scan_fresh():
            return False
        m = self.scan
        xs, ys = scan_xy(m.ranges, m.angle_min, m.angle_increment,
                         max(0.05, m.range_min), min(16.0, m.range_max),
                         lidar_x=self.board_lidar_x, yaw_off=self.board_yaw_off,
                         self_margin=self.board_self_margin)
        hit, info = board_detect(
            xs, ys, lane_half=self.board_lane_half, x_max=self.board_look,
            min_w=self.board_min_w, max_w=self.board_max_w,
            min_pts=self.board_min_pts, gap=self.board_gap,
            fov_deg=self.board_fov)
        if hit:
            self._early_hits += 1
            if self._early_hits >= 2:
                self._early_done = True
                rospy.loginfo("岔口早判: 支路上有板子, 距离 %.2fm 角度 %s "
                              "—— 保持板子逻辑", info["d"],
                              self.bearing_txt(info["d"],
                                               (info["yl"] + info["yr"]) / 2.0))
            return self._early_done
        if trav >= self.board_early_dist:
            self._early_done = True
            self.board_state = "确认无板"
            rospy.loginfo("=" * 56)
            rospy.loginfo("岔口早判: 前方 %.2fm 内没有板子(板子应在 ~1.1m 处, "
                          "看得很清楚) —— 判定本支路无板, 切回初赛逻辑",
                          self.board_look)
            rospy.loginfo("=" * 56)
            return True
        return False

    def board_range(self):
        """常规检测的前视距离(m): 板子要落在这个纵深内才会被认出来。

        以前写死 board_stop + 0.6。Y 支路的问题是板子离岔口只有 1.1m, 而
        转完 45° 到触发之间可用的距离本来就短 —— 前视太浅的话, 车要开到
        很近才第一次看见板子, 留给"连续 confirm 帧 -> 站位 -> 转正 -> 横移"
        的余量不够。所以给它一个独立参数, 并且 Y 支路默认放得更远。
        board_detect_range > 0 就直接用它; = 0 走下面的自动值。
        """
        if self.board_range_cfg > 0.0:
            return self.board_range_cfg
        base = self.board_stop + 0.6
        if abs(self.start_turn_deg) <= 1.0:      # 走中间那条 -> Y 支路
            return max(base, self.board_range_y)
        return base

    def board_clear_now(self):
        """当前这条路线的"再走多远就判定无板"。两臂从进 FOLLOW 起算,
        Y 支路从岔口起算(见 board_clear_dist_y)。"""
        # ⚠ 按**路线**选, 不能按 board_anchor 选: 两臂的起点 ±60° 转向
        # 同样走 FORK_TURN 相位, 转完一样把 board_anchor 置成"岔口",
        # 于是两臂也会用上 Y 支路的 2.00m —— 实车 2026-08-16 就是这样,
        # 明明设了 board_clear_dist=3.00, 日志里却是"已走 2.00m 判定无板"。
        return (self.board_clear_dist if abs(self.start_turn_deg) > 1.0
                else self.board_clear_dist_y)

    def board_travel(self, anchor):
        """从进 FOLLOW 那一刻起走了多远(m)。anchor 是
        [时刻, odom_xy, cmd_s 快照]。有 odom 就用 odom 量, 没有就退回
        **指令速度积分**(cmd_s)。

        ⚠ 兜底这条以前写的是 (now - t0) * self.v, 有两个错:
          1. self.v 是巡线速度常量, 车停着(等灯/刹停/原地转)照样在累。
             实车认灯站了 8.2s, 白算 2.05m。
          2. 连带地, 从节点启动到真正进 FOLLOW 的十几秒全被算成在跑,
             结果第一帧 FOLLOW 就报"已走 3.56m, 判定本趟无板", 板子
             检测整趟没跑过。
        cmd_s 由主循环按 speed_now() 积分, 那个函数在停着的相位返回 0。
        """
        if anchor[0] is None:
            anchor[0] = time.time()
            anchor[1] = self.odom_xy
            anchor[2] = self.cmd_s
        if anchor[1] is not None and self.odom_xy is not None:
            dx = self.odom_xy[0] - anchor[1][0]
            dy = self.odom_xy[1] - anchor[1][1]
            return math.hypot(dx, dy)
        return max(0.0, self.cmd_s - (anchor[2] or 0.0))

    def side_clear(self):
        """左右两侧各有多空(m)。取车侧前方那块矩形里最近的点。
        返回 (左, 右); 没有点就给一个大数(=很空)。"""
        msg = self.scan
        if msg is None:
            return 9.9, 9.9
        xs, ys = scan_xy(msg.ranges, msg.angle_min, msg.angle_increment,
                         max(0.05, msg.range_min), min(16.0, msg.range_max),
                         lidar_x=self.board_lidar_x, yaw_off=self.board_yaw_off,
                         self_margin=self.board_self_margin)
        # ⚠ 必须把**板子自己**排掉。板子横在正前方 ±21cm, 如果只按"这一侧
        # 最近的点"算, 量到的就是板子的边缘(0.10~0.21), 两边都显示"堵死",
        # 方向决策直接退化成瞎猜 —— 第一版就是这么错的。
        ign = max(0.20, abs(self.board_yl), abs(self.board_yr)) + 0.06
        band = (xs > -0.15) & (xs < self.ga_fwd_eff() + 0.20)
        out = []
        for sgn in (1.0, -1.0):
            sel = band & (ys * sgn > ign)
            out.append(float(np.min(np.abs(ys[sel]))) if sel.any() else 9.9)
        return out[0], out[1]

    def ga_turn_side(self):
        """"小半径侧" —— 也就是**车拐弯的那一侧**。返回 (+1 左 / -1 右, 说明)。

        ⚠ 两条分支的符号是**相反**的, 别合并(见下面的推导和实车结论)。
          两臂: 往小半径方向 = 往圆心那侧。起点转 ±60° 切进去, 但臂本身的
                弧朝反方向弯, 所以圆心在转角的反侧。
          Y 支路: 往转角的**同**侧让(用户 2026-08-16 确认: 顺时针转
                45° 走 4 号线 -> 往右让)。

        另外把最近一段的实际转向 az 平均一下做**交叉验证**: 弯道上巡线
        一直在往圆心那侧打舵, 两者应该同号, 不同号就打日志提醒。"""
        # 两臂: 起点先转 ±60° 切进去, 但**臂本身的弧是朝反方向弯的**。
        #   量图得到: 车从顶部中央朝下(南)进场, 左转 +60° 走的是画面**右**臂,
        #   而右臂的拟合圆心(R=107cm, 正好对上图上标的 108cm)落在车的右手边。
        #   所以小半径侧 = 起点转角的**反**侧。
        # Y 支路: 是直线, 没有曲率; 判据是"远离红绿灯" -> 转角**同**侧。
        #   ⚠ 这条我来回改错过两次, 最后由用户直接定死(2026-08-16):
        #     Y 顺时针 -45° -> 往右让;  顺时针 60° 的臂(-60°) -> 往左让。
        #   也就是: 两臂是转角**反**侧, Y 支路是转角**同**侧, 两者符号相反,
        #   不要再想当然地把它们合并成一条规则。
        # ⚠ 用 start_turn_deg 不是 turn_deg: 后者在岔口原地转时被改写成
        # fork_turn_deg(-45), 于是走中间那条的时候也会命中这个分支, Y 支路
        # 那条规则等于从来没生效过。实车 2026-08-16 就是这么绕错的。
        if abs(self.start_turn_deg) > 1.0:
            ref = self.start_turn_deg
            sgn = -1.0 if ref > 0 else 1.0
            why = "小半径侧=起点转角反侧(转 %+.0f°)" % ref
        else:
            ref = self.fork_turn_deg
            sgn = 1.0 if ref > 0 else -1.0
            why = "远离红绿灯=Y口转角同侧(转 %+.0f°)" % ref
        if self.az_hist:
            az = sum(self.az_hist) / float(len(self.az_hist))
            if abs(az) > 0.05:
                if (az > 0) != (sgn > 0):
                    why += " ⚠实际转向 az=%+.2f 与之相反" % az
                else:
                    why += " (实际 az=%+.2f 同向)" % az
        return sgn, why

    def ga_choose_dir(self):
        """往哪边让。返回 (+1 左 / -1 右, 说明)。

        默认 auto = **按小半径侧/远离红绿灯这条几何规则定方向**, 雷达只做
        一票否决: 选中那侧要是被挡住而另一侧是空的, 就改走另一侧并大声
        打日志(宁可绕错方向, 也不能横着撞上去)。
        想完全写死就 go_around_dir:=left / right; 想只按规则不让雷达
        插嘴就 go_around_dir:=turn。"""
        sgn, why = self.ga_turn_side()
        if self.ga_dir_cfg == "left":
            return 1.0, "参数指定左"
        if self.ga_dir_cfg == "right":
            return -1.0, "参数指定右"
        if self.ga_dir_cfg == "turn":
            return sgn, why + " (不看雷达)"
        cl, cr = self.side_clear()
        # ⚠ 用 ga_side_eff() 不是 ga_side: 后者是**参数**, 默认 0.0(=自动),
        # 于是 need 变成 0.15m —— 只有窄到 15cm 才算堵, 实车左侧只剩 0.33m
        # 照样放行, 直接横着撞过去。真正要让的距离是 ga_side_eff()。
        need = self.ga_side_eff() + 0.15
        mine = cl if sgn > 0 else cr
        other = cr if sgn > 0 else cl
        if mine < need <= other:
            rospy.logwarn("绕障: 规则要往%s让, 但那边只有 %.2fm(需要 %.2f), "
                          "另一侧有 %.2fm —— 改走另一侧",
                          "左" if sgn > 0 else "右", mine, need, other)
            return -sgn, "雷达否决: %s侧只剩 %.2fm" % (
                "左" if sgn > 0 else "右", mine)
        return sgn, why + " [雷达 左%.2f 右%.2f]" % (cl, cr)

    def ga_side_eff(self):
        if self.ga_side > 0.0:
            return self.ga_side
        half = self.kf.half if self.kf.ready() else 0.21
        return half + self.car_half_w + self.ga_keep

    def ga_fwd_eff(self):
        """这一次绕障要往前走多远。ga_fwd>0 就用它, 否则按雷达量到的
        板子距离 + 余量(车头到板子 d, 还要越过板子本身和车身)。"""
        if self.ga_fwd > 0.0:
            return self.ga_fwd
        d = self.board_d if self.board_d > 0.05 else self.board_stop
        if self.ga_pass > 0.0:
            return d + self.ga_pass
        return d + self.car_half_l + self.ga_keep

    def car_pose(self):
        """(x, y, yaw) in odom; 拿不到就 None。yaw 优先用 odom 自带的,
        没有再退回 IMU 的多圈展开 yaw。"""
        if self.odom_xy is None:
            return None
        yaw = self.odom_yaw
        if yaw is None:
            yaw = self.yaw_unw
        if yaw is None:
            return None
        return self.odom_xy[0], self.odom_xy[1], yaw

    def pose_cb(self, msg):
        """/robot_pose(map->base_link)。AnyMsg 进来, 看类型再反序列化。"""
        try:
            typ = msg._connection_header.get("type", "")
            if typ == "geometry_msgs/PoseStamped":
                m = PoseStamped()
                m.deserialize(msg._buff)
                p = m.pose
            elif typ == "geometry_msgs/Pose":
                m = Pose()
                m.deserialize(msg._buff)
                p = m
            else:
                if self._map_pose_type != typ:
                    self._map_pose_type = typ
                    rospy.logwarn("%s 是 %s, 不是 Pose/PoseStamped, 地图终点不用",
                                  self.pose_topic, typ)
                return
            q = p.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self._map_pose = (float(p.position.x), float(p.position.y), yaw)
            self._map_pose_t = time.time()
            if self._map_pose_type != typ:
                self._map_pose_type = typ
                rospy.loginfo("收到 %s (%s), 地图终点判据启用", self.pose_topic, typ)
        except Exception as e:
            if self._map_pose_type != "err":
                self._map_pose_type = "err"
                rospy.logwarn("%s 解析失败(%r), 地图终点不用", self.pose_topic, e)

    def map_goal(self):
        """地图定位判终点: 返回 (到了没, 离终点多远(m, 没定位=-1), 说明)。
        goal_mode=both 且拿到了新鲜的 /robot_pose 才可能 True; 没 topic /
        位姿过期(> pose_fresh 秒) / 没配 goal_map_xy 都静默返回 False。"""
        if self.goal_mode != "both" or self.goal_map_xy is None:
            return False, -1.0, ""
        if self._map_pose is None:
            return False, -1.0, "无 %s" % self.pose_topic
        age = time.time() - self._map_pose_t
        if age > self.pose_fresh:
            return False, -1.0, "位姿过期 %.1fs" % age
        dx = self.goal_map_xy[0] - self._map_pose[0]
        dy = self.goal_map_xy[1] - self._map_pose[1]
        d = math.hypot(dx, dy)
        return d <= self.goal_map_dist, d, "离终点 %.2fm" % d

    def goal_board_veto(self):
        """视觉说"前方有终点横线", 但那其实是拦路板吗?

        为什么用雷达而不是 odom/IMU 来"确定": 这三个里只有雷达是**物理上
        可分**的判据 —— 真终点线是垫子上的白胶带, 雷达什么也看不到;
        板子是立起来的实物, 雷达一定看得见。分割网只看颜色, 板子的白色
        板面和白胶带在它眼里就是一回事, 所以视觉自己分不开。
        odom/IMU 的路子是"按里程判断还没到终点区", 那要靠几米的航位推算
        + 事先知道每条路线的长度, 一旦累计误差或者选错分支就全错;
        而雷达这一条是**每帧独立**判的, 不累积、不依赖路线先验。

        ⚠ 不能简单用"雷达前方有东西就否决": 终点框外面就是场地围挡,
          那玩意一直在视野里。所以必须用**有限宽**这条判据(board_detect),
          围挡宽度超上限会被自动放行。

        返回 (要不要否决, 板子距离)。"""
        if not (self.board_on and self.goal_need_clear):
            return False, -1.0
        if self.board_state != "未知":      # 已避 / 确认无板 -> 不再复核
            return False, -1.0
        if not self.scan_fresh():
            # 雷达没数据: 这里**放行**而不是拦住 —— 拦住的话车永远到不了
            # 终点, 那是更坏的失败。只是要吵一声。
            if not self._veto_warn:
                self._veto_warn = True
                rospy.logwarn("终点横线的雷达复核: 拿不到雷达数据, 只能"
                              "按视觉判 —— 板子有可能被当成终点线")
            return False, -1.0
        m = self.scan
        xs, ys = scan_xy(m.ranges, m.angle_min, m.angle_increment,
                         max(0.05, m.range_min), min(16.0, m.range_max),
                         lidar_x=self.board_lidar_x, yaw_off=self.board_yaw_off,
                         self_margin=self.board_self_margin)
        hit, info = board_detect(
            xs, ys, lane_half=self.board_lane_half, x_max=self.goal_clear_rng,
            min_w=self.board_min_w, max_w=self.board_max_w,
            min_pts=self.board_min_pts, gap=self.board_gap,
            fov_deg=self.board_fov)
        if hit and info["d"] <= self.goal_clear_rng:
            return True, info["d"]
        return False, info["d"]

    def board_span(self):
        """板子从**雷达**看过去张多大的角(弧度)。

        板子是一块平板。雷达一旦落到板面所在的那条直线上, 射线就是擦着
        板面走的, 打不到面 —— 张角趋近 0, 回波掉到零。绕障第二段前进时
        雷达**一定**会经过这个位置(实测: 纵向差 ±1.5cm 内点数就 <3,
        连续 5~6 帧一个点都没有)。
        这不是跟丢, 是几何上就看不见, 所以要提前算出来、跳过关联、让
        卡尔曼纯预测滑过去 —— 板子不动、车姿来自 odom, 滑 0.25s 完全够准。
        """
        if not self.kf.ready():
            return None
        pose = self.car_pose()
        if pose is None:
            return None
        cx, cy, yaw = pose
        bx, by, ux, uy = self.kf.body((cx, cy), yaw)
        h = self.kf.half
        lx = self.board_lidar_x            # 雷达在车体系的位置
        a1 = math.atan2(by + h * uy, bx + h * ux - lx)
        a2 = math.atan2(by - h * uy, bx - h * ux - lx)
        d = a1 - a2
        while d > math.pi:
            d -= 2 * math.pi
        while d < -math.pi:
            d += 2 * math.pi
        return abs(d)

    def kf_observe(self):
        """从当前这一帧雷达里找板子, 喂给卡尔曼。返回是否关联上。"""
        msg, pose = self.scan, self.car_pose()
        if not self.scan_fresh() or pose is None or not self.kf.ready():
            self.kf.miss += 1        # 陈旧/缺数据一律记跟丢, 别拿老帧硬更新
            return False
        span = self.board_span()
        if span is not None and span < self.ga_min_span * msg.angle_increment:
            self.ga_blind += 1       # 几何上就看不见, 不算跟丢
            return False
        cx, cy, yaw = pose
        xs, ys = scan_xy(msg.ranges, msg.angle_min, msg.angle_increment,
                         max(0.05, msg.range_min), min(16.0, msg.range_max),
                         lidar_x=self.board_lidar_x, yaw_off=self.board_yaw_off,
                         self_margin=self.board_self_margin)
        bx, by, _, _ = self.kf.body((cx, cy), yaw)      # 预测的板心(车体系)
        got = find_seg_near(xs, ys, bx, by, gate=0.40, gap=self.board_gap,
                            min_pts=max(3, self.board_min_pts - 2))
        if got is None:
            self.kf.miss += 1
            return False
        mid, u, half, npts = got
        c, sn = math.cos(yaw), math.sin(yaw)
        z = (cx + c * mid[0] - sn * mid[1], cy + sn * mid[0] + c * mid[1])
        return self.kf.update(z)

    def ga_geom(self):
        """三段各自的**几何**判据(全部由车身尺寸和板子实测算出, 不拍数):
          横让到位 : 板子朝让向那一端, 横向要让开 车半宽 + 余量
          前进到位 : 板心落到车尾之后 车半长 + 余量
          横回到位 : 板心横向回到 0 (板心在路中心线上, 所以 by=0 就是回中)
        返回 (bx, by, 端点横向 by_end, 各段阈值)"""
        pose = self.car_pose()
        if pose is None or not self.kf.ready():
            return None
        cx, cy, yaw = pose
        bx, by, ux, uy = self.kf.body((cx, cy), yaw)
        # 板子朝"让向"那一端: 让向 = +ga_sign 的车体 y 方向
        end = self.kf.half * (uy if uy * self.ga_sign > 0 else -uy)
        by_end = by + end
        # 把不确定度当成安全余量加进去, 而且**各用各方向的**:
        # 横让只受横向 sigma 影响, 前进只受纵向 sigma 影响。
        # 估计越不准, 让得越开、走得越远 —— 这两个方向"多一点"总是更安全;
        # 第三段回中是"对准", 加余量没有意义, 所以不加。
        sx, sy = self.kf.sigma_body(yaw)
        need_lat = self.car_half_w + self.ga_clear + sy * self.ga_k_sigma
        need_back = -(self.car_half_l + self.ga_tail + sx * self.ga_k_sigma)
        # ---- 板子系(u = 板面方向, n = 法向 = 车道方向) ----
        # 板子横跨赛道, 所以它的**中垂线就是车道中心线**, 法向就是航向。
        # 前两段在车体系里量没问题(要的是"离板子多远"), 但第三段"回中"和
        # 收尾的航向必须在板子系里量 —— 车进来时歪了多少, 在车体系里回中
        # 就原样保留多少, 三段走下来只会更歪。
        ox, oy = self.kf.x            # 板心(odom)
        uxo, uyo = self.kf.u          # 板面方向(odom, 单位向量)
        dx, dy = cx - ox, cy - oy     # 板心 -> 车心
        lat = dx * uxo + dy * uyo     # 沿板面: 离中垂线多远(带符号)
        lon = -dx * uyo + dy * uxo    # 沿法向: 在板子哪一侧
        # 航向目标: 法向里和当前车头同向的那一支
        n1 = math.atan2(uxo, -uyo)
        yaw_err = math.atan2(math.sin(n1 - yaw), math.cos(n1 - yaw))
        if abs(yaw_err) > math.pi / 2.0:      # 取反向那一支
            n1 = math.atan2(-uxo, uyo)
            yaw_err = math.atan2(math.sin(n1 - yaw), math.cos(n1 - yaw))
        # ---- 曲率修正(只在两臂) ----
        # 板法线是**板子那一点**的切线; 车已经沿着它直走了 |lon| 米, 而真正
        # 的车道在这段里转过 phi=asin(|lon|/R)、并往弯道内侧偏了 R(1-cos phi)。
        # 所以把"目标中垂线"和"目标航向"都往圆心那侧搬这么多。
        # ⚠ 只在**板后收尾**那两段修, 不是整个绕障过程都修:
        #     AVOID_BACK  横回 -> 目标中垂线要搬
        #     AVOID_ALIGN 对航向 -> 目标航向要搬
        #   板**前**的 AVOID_TURN0(预转正)绝对不能修 —— 那一段的目标本来
        #   就该是板法向, 车得摆正了才能干净地横移过去。在那儿按 |lon| 算
        #   出来的 phi(板前 0.3m 就是 12°)会把车一开局就拧歪, 横移直接
        #   蹭板面。AVOID_OUT/FWD/REV 用的是 by_end/bx, 不碰 lat/yaw_err,
        #   本来就不受影响, 列在这里只是为了说清楚范围。
        arc_phi = arc_lat = 0.0
        if (self.arc_r > 0.0 and abs(self.start_turn_deg) > 1.0
                and self.phase in ("AVOID_BACK", "AVOID_ALIGN")):
            # ⚠ 方向取 ga_turn_side() 的几何规则(小半径侧 = 圆心那侧),
            #   **不能**用 ga_sign: 那一侧被挡住时 ga_choose_dir 会翻到另一
            #   边去让, 而赛道往哪弯跟车从哪边绕过去毫无关系。
            arc_sgn = self.ga_turn_side()[0]          # +1 左 / -1 右
            phi = math.asin(min(0.95, abs(lon) / self.arc_r))
            phi = min(phi, math.radians(self.arc_max_deg))
            arc_phi = arc_sgn * phi
            # 横向单独乘一个倍率, 航向不受影响(见 arc_lat_scale 的说明)
            off = (self.arc_r * (1.0 - math.cos(phi))) * self.arc_lat_scale
            # lat 是沿 +u 量的; +u 在车体系里的 y 分量是 uy, 车体 +y = 左。
            # 所以 +u 指向圆心那侧 <=> uy * arc_sgn > 0。
            u_side = 1.0 if (uy * arc_sgn) > 0 else -1.0
            arc_lat = u_side * off
            lat -= arc_lat                            # 改成"离修正后中线多远"
            n1 = n1 + arc_phi
            yaw_err = math.atan2(math.sin(n1 - yaw), math.cos(n1 - yaw))
        # odom 推算的横向偏差(见 go_around_back_src): 起绕时冻结的 u0 / lat0
        out = dict(bx=bx, by=by, by_end=by_end, need_lat=need_lat,
                   need_back=need_back, yaw=yaw, sx=sx, sy=sy,
                   lat=lat, lon=lon, yaw_tgt=n1, yaw_err=yaw_err,
                   arc_phi=arc_phi, arc_lat=arc_lat,
                   ux=float(ux), uy=float(uy))      # 板面方向(车体系)
        if self._ga_u0 is not None and self._ga_p0 is not None:
            ddx, ddy = cx - self._ga_p0[0], cy - self._ga_p0[1]
            out["lat_odom"] = (self._ga_lat0 + ddx * self._ga_u0[0] +
                               ddy * self._ga_u0[1] - arc_lat)
            # 法向距同样按 odom 推(和上面 lon 同一个符号约定):
            # 车越过板子后雷达看不见背面, 卡尔曼一丢前进段就只剩开环 "moved",
            # 而 moved 是路程(含横移里蹭出来的纵向漂移), 起算点又是名义站位
            # 而不是实测 —— 实车 2026-08-19: 站位实测 0.319, 开环按 0.271 算,
            # 前进少走 5cm, 车尾离板面只剩 2cm(要 10cm)。
            out["lon_odom"] = (self._ga_lon0 - ddx * self._ga_u0[1] +
                               ddy * self._ga_u0[0])
        return out

    def dump_avoid_scan(self, sgn, why):
        """触发绕障的那一帧, 把整圈雷达点画成散点图存进 dump/。

        为什么要这张图: 实车把 Y 岔口的红绿灯箱体当成板子绕了, 光看
        "宽=0.38 簇=4" 这行数字没法判断视野里到底有什么。画出来一眼就知道
        那一簇是板子、是灯箱还是围挡。

        画法: **上方是车头前方(+x)**, 左边是车体左侧(+y), 原点是 base_link。
          白点   = 这一帧所有回波(已剔掉车身内的天线)
          红点+红线 = 被判成板子的那一段(线画在拟合出的板面上)
          青框   = 车体轮廓, 青十字 = 雷达实际位置
          黄箭头 = 决定要让的方向
          绿虚框 = 禁区(板子周围 go_around_keepout)
        """
        if not self.dump_dir or self.scan is None:
            return
        cl, cr = self.side_clear()
        try:
            m = self.scan
            xs, ys = scan_xy(m.ranges, m.angle_min, m.angle_increment,
                             max(0.05, m.range_min), min(16.0, m.range_max),
                             lidar_x=self.board_lidar_x,
                             yaw_off=self.board_yaw_off,
                             self_margin=self.board_self_margin)
            R = 600                      # 画布边长(像素)
            SPAN = float(self.ga_dump_span)   # 画面半宽(米)
            s = (R / 2.0) / SPAN         # 像素/米
            img = np.zeros((R, R, 3), np.uint8)

            def P(x, y):
                """车体系(x前 y左) -> 像素。上=前, 左=左"""
                return (int(R / 2 - y * s), int(R / 2 - x * s))

            # 网格: 每 0.5m 一圈, 标注半径
            for k in range(1, int(SPAN / 0.5) + 1):
                cv2.circle(img, P(0, 0), int(k * 0.5 * s), (40, 40, 40), 1)
                cv2.putText(img, "%.1f" % (k * 0.5),
                            (R // 2 + 4, int(R / 2 - k * 0.5 * s) - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.34, (90, 90, 90), 1)
            cv2.line(img, (R // 2, 0), (R // 2, R), (40, 40, 40), 1)
            cv2.line(img, (0, R // 2), (R, R // 2), (40, 40, 40), 1)

            # 被判成板子的那一簇: 用检出时记下的 (d, yl..yr) 圈出来
            d, yl, yr = self.board_d, self.board_yl, self.board_yr
            has_b = d > 0.05 and abs(yr - yl) > 1e-3
            for x, y in zip(xs, ys):
                near_b = has_b and abs(x - d) <= 0.18 \
                    and min(yl, yr) - 0.05 <= y <= max(yl, yr) + 0.05
                cv2.circle(img, P(x, y), 2,
                           (60, 60, 255) if near_b else (235, 235, 235), -1)

            # 车体轮廓 + 雷达位置
            hl, hw = self.car_half_l, self.car_half_w
            cv2.rectangle(img, P(hl, hw), P(-hl, -hw), (200, 200, 0), 1)
            lp = P(self.board_lidar_x, 0.0)
            cv2.drawMarker(img, lp, (200, 200, 0), cv2.MARKER_CROSS, 10, 1)

            if has_b:
                # 板面: 在 x=d 处从 yl 画到 yr
                cv2.line(img, P(d, yl), P(d, yr), (60, 60, 255), 2)
                # 禁区(长方形膨胀, 不是胶囊 —— 车只做横平竖直的平移)
                k = self.ga_keep
                cv2.rectangle(img, P(d + k, max(yl, yr) + k),
                              P(d - k, min(yl, yr) - k), (0, 180, 0), 1)
            # 让向箭头
            ay = 0.45 * (1.0 if sgn > 0 else -1.0)
            cv2.arrowedLine(img, P(0.0, 0.0), P(0.0, ay), (0, 230, 230), 2,
                            tipLength=0.25)

            # ⚠ 只能用 ASCII: cv2 的 Hershey 字体没有中文字形, 写中文
            # 出来全是 ????(第一版就是这样)。
            txt = ["FRONT = UP   avoid #%d" % self.ga_n,
                   "board d=%.2fm y=[%+.2f,%+.2f] w=%.2f" % (
                       d, yl, yr, abs(yr - yl)),
                   "dodge %s   side clear L=%.2f R=%.2f" % (
                       "LEFT" if sgn > 0 else "RIGHT", cl, cr),
                   "keepout %.2fm   shift %.2fm" % (self.ga_keep,
                                                    self.ga_side_eff()),
                   "%d pts" % len(xs)]
            for i2, t in enumerate(txt):
                cv2.putText(img, t, (8, 18 + 16 * i2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            fn = os.path.join(self.dump_dir, "avoid_%02d.png" % self.ga_n)
            cv2.imwrite(fn, img)
            rospy.loginfo("绕障扫描图 -> %s", fn)
        except Exception as e:
            rospy.logwarn("绕障扫描图没画成(不影响绕障): %r", e)

    def ga_trace(self, note=""):
        """绕障全程逐帧落盘: 原始雷达 + 当帧的所有判定量, 一行一个 JSON。

        为的是能离线把每一帧重放出来对账 —— 实车反映"站位停在 20cm 而不是
        10cm, 回程车尾擦板", 这种系统性偏置只有把 raw ranges 和代码算出的
        board_d 摆在一起才能定位到底是哪一步引入的。
        ranges 存成毫米整数(省一半体积, 精度远超雷达本身)。
        """
        if not self.ga_trace_on or not self.dump_dir:
            return
        try:
            m = self.scan
            pose = self.car_pose()
            g = self.ga_geom()
            rec = {
                "t": round(time.time() - self.t_boot, 3),
                "phase": self.phase,
                "note": note,
                "odom": None if pose is None else [round(v, 4) for v in pose],
                "board_d": round(self.board_d, 4),
                "board_yl": round(self.board_yl, 4),
                "board_yr": round(self.board_yr, 4),
                "ga_sign": self.ga_sign,
                "ga_rev": round(self.ga_rev, 4),
                "ga_side": round(self.ga_side_eff(), 4),
                "rear_lon": self._ga_rear_last,
                "rear_n": self._ga_rear_n,
                "ga_fwd": round(self.ga_fwd_now, 4),
                "keepout": self.ga_keep,
                "half_l": self.car_half_l, "half_w": self.car_half_w,
                "lidar_x": self.board_lidar_x,
                "yaw_off": round(self.board_yaw_off, 5),
                "moved": round(self.moved_since_mark(), 4),
                # ⚠ 板心是 self.x 不是 self.c(我第一版写错了, 实车直接
                # AttributeError 把整个 trace 废掉)。u 是板面方向单位向量。
                "kf": (None if not self.kf.ready() else
                       {"x": [round(float(v), 4) for v in self.kf.x],
                        "u": [round(float(v), 4) for v in self.kf.u],
                        "half": round(self.kf.half, 4),
                        "miss": self.kf.miss, "n_upd": self.kf.n_upd,
                        # ⚠ sigma() 返回**标量**(两个方向取大的那个),
                        # sigma_body(yaw) 才返回 (纵, 横) 两个数。
                        # 第一版写成 for v in self.kf.sigma() ->
                        # TypeError: 'float' object is not iterable。
                        "sig": round(float(self.kf.sigma()), 4),
                        "sig_body": [round(float(v), 4) for v in
                                     self.kf.sigma_body(pose[2])]}),
                "geom": (None if g is None else
                         dict((k, round(v, 4)) for k, v in g.items()
                              if isinstance(v, float))),
            }
            if m is not None:
                rec["scan"] = {
                    "amin": round(m.angle_min, 6),
                    "ainc": round(m.angle_increment, 8),
                    "rmin": m.range_min, "rmax": m.range_max,
                    "mm": [0 if (r != r or r in (float("inf"),
                                                 float("-inf")))
                           else int(r * 1000.0) for r in m.ranges],
                }
            self._ga_fh.write(json.dumps(rec) + "\n")
            self._ga_fh.flush()
        except Exception as e:
            if not self._ga_trace_warned:
                self._ga_trace_warned = True
                rospy.logwarn("绕障轨迹没写成(不影响绕障): %r", e)

    def ga_refit_board(self):
        """越过板子之后, 拿**背面**这一帧重新拟合板子, 覆盖卡尔曼的估计。

        为什么值得重来一次: BoardKF.start() 在触发那一帧定下板面方向 u,
        之后**只更新板心位置, u 再也不变**。而触发时车在板子正前方 0.29m
        且本身还偏着十几度, 那一帧的 u 一旦有偏, 中垂线和法向就整体偏,
        后面 BACK/ALIGN 收得再干净也是往一条歪线上收。
        越过之后条件反而更好: 车在侧后方, 看的是板子背面, 视角宽、点数多。
        用这一帧重新拟合, 比一路沿用最初那个估计可靠。

        拟合不成(点太少/关联不上)就保持原估计, 返回 False。
        """
        if not self.scan_fresh() or not self.kf.ready():
            return False
        pose = self.car_pose()
        if pose is None:
            return False
        cx, cy, yaw = pose
        m = self.scan
        xs, ys = scan_xy(m.ranges, m.angle_min, m.angle_increment,
                         max(0.05, m.range_min), min(16.0, m.range_max),
                         lidar_x=self.board_lidar_x,
                         yaw_off=self.board_yaw_off,
                         self_margin=self.board_self_margin)
        bx, by, _, _ = self.kf.body((cx, cy), yaw)     # 预测的板心(车体系)
        got = find_seg_near(xs, ys, bx, by, gate=0.35, gap=self.board_gap,
                            min_pts=self.board_min_pts)
        if got is None:
            rospy.logwarn("绕障: 背面重拟合没关联上板子, 沿用原估计")
            return False
        mid, u, half, npts = got
        if npts < 12:
            rospy.logwarn("绕障: 背面只看到 %d 个点, 太少, 沿用原估计", npts)
            return False
        # ⚠ 还要看**看到了多长**。find_seg_near 给的中点是"可见那一段"的
        # 中点, 不是板子真正的中点 —— 只露出一小截时它会明显偏, 而中垂线
        # 就是按这个中点画的。实车 2026-08-16 这里用 13 个点拟出半长 0.11
        # (真值 0.19), 等于只看到 60%, 中点最多能偏 5cm。
        if half < 0.75 * self.kf.half:
            rospy.logwarn("绕障: 背面只看到半长 %.2f(原 %.2f), 露得太少, "
                          "中点不可信, 沿用原估计", half, self.kf.half)
            return False
        c, sn = math.cos(yaw), math.sin(yaw)
        u_old = self.kf.u.copy()
        u_new = (c * u[0] - sn * u[1], sn * u[0] + c * u[1])
        # 方向有 180° 的二义性(线段没有正负), 取和原来同向的那一支, 免得
        # lat 的符号突然翻过来
        if u_new[0] * u_old[0] + u_new[1] * u_old[1] < 0:
            u_new = (-u_new[0], -u_new[1])
        d_deg = math.degrees(math.acos(max(-1.0, min(1.0,
                u_new[0] * u_old[0] + u_new[1] * u_old[1]))))
        self.kf.start((cx + c * mid[0] - sn * mid[1],
                       cy + sn * mid[0] + c * mid[1]), u_new, half)
        rospy.loginfo("绕障: 用板子背面重新定位 (%d 点, 半长 %.2f, "
                      "板面方向改了 %.1f°) -> 中垂线/法向按这个来",
                      npts, half, d_deg)
        self.ga_trace("refit %.1fdeg %dpts" % (d_deg, npts))
        return True

    def start_go_around(self):
        """开始绕障。三段: OUT(横让) -> FWD(前进) -> BACK(横回)。"""
        self.ga_n += 1
        self.ga_fwd_now = self.ga_fwd_eff()
        self.ga_lost = 0
        self.ga_blind = 0
        self.ga_out_dist = 0.0
        self.ga_side_now = 0.0        # 本次横让开环距离(按板端实际位置补偿)
        self.kf = BoardKF()
        pose = self.car_pose()
        if self.ga_mode == "track" and pose is not None and self.scan is not None:
            cx, cy, yaw = pose
            m = self.scan
            xs, ys = scan_xy(m.ranges, m.angle_min, m.angle_increment,
                             max(0.05, m.range_min), min(16.0, m.range_max),
                             lidar_x=self.board_lidar_x,
                             yaw_off=self.board_yaw_off,
                             self_margin=self.board_self_margin)
            got = find_seg_near(xs, ys, self.board_d,
                                (self.board_yl + self.board_yr) / 2.0,
                                gate=0.40, gap=self.board_gap,
                                min_pts=self.board_min_pts)
            if got is not None:
                mid, u, half, npts = got
                c, sn = math.cos(yaw), math.sin(yaw)
                self.kf.start((cx + c * mid[0] - sn * mid[1],
                               cy + sn * mid[0] + c * mid[1]),
                              (c * u[0] - sn * u[1], sn * u[0] + c * u[1]),
                              half)
                rospy.loginfo("绕障: 锁定板子 中点(车体)(%.2f,%.2f) 半长 %.2f "
                              "点数 %d -> 闭环跟踪", mid[0], mid[1], half, npts)
        if not self.kf.ready():
            rospy.logwarn("绕障: 没能锁定板子(没有雷达/odom?), 退回按固定"
                          "距离开环走")
        self.ga_sign, why = self.ga_choose_dir()
        # 冻结正面看到的板面方向和当前横向偏差, 横回段按 odom 位移回中
        # (站位段结束时会再冻结一次, 那时车已经对到中垂线上, lat0≈0)
        self._ga_freeze_ref("起绕")
        self.dump_avoid_scan(self.ga_sign, why)
        if self.ga_trace_on and self.dump_dir:
            try:
                if self._ga_fh is not None:
                    self._ga_fh.close()
                fn = os.path.join(self.dump_dir,
                                  "avoid_trace_%02d.jsonl" % self.ga_n)
                self._ga_fh = open(fn, "w")
                rospy.loginfo("绕障逐帧轨迹 -> %s", fn)
            except Exception as e:
                rospy.logwarn("绕障轨迹开不了(不影响绕障): %r", e)
                self._ga_fh = None
        self.ga_trace("start " + _s(why))
        self.mark_xy, self.mark_t = self.odom_xy, time.time()
        # 前提: 横移期间车必须整个在纵向禁区带外面。检出太近(比如出弯
        # 才看到)就先倒一段再横移, 否则是贴着板面蹭过去。
        # 加 2cm 容差: 触发发生在"这一帧 d 刚好 <= board_stop", d 的量化
        # 步长约 1.2cm(20Hz x 0.25m/s), 所以 board_stop 正好等于 need 时
        # 几乎每次都会差几毫米而白倒一次车。
        # AVOID_REV 现在是"站位调整"段, **双向**: 先把车开到"车头离板子
        # 正好一个禁区"的位置, 再横移。以前只处理"太近了往后倒", 于是
        # board_stop_dist 只要大于 need, 车就在检出的那个距离原地横移 ——
        # 而用户要的是"0.35m 检出, 前进到板前 0.1m, 再横让"。
        # ga_rev > 0 = 往后倒; < 0 = 往前挪。
        # 要不要先把车头摆正: 三段全程不转头, 车进场歪 theta 的话, "横移"
        # 在板子系里其实是 cos(theta) 的横向 + sin(theta) 的**纵向**分量。
        # 实车 Y 支路进场歪 24.9°, 横移 0.414m 里有 0.174m 是不请自来的
        # 纵向位移 —— 三段互相污染, 净空全不作数。所以先转正再绕。
        gg = self.ga_geom()
        preturn = (self.ga_align_deg > 0 and gg is not None and
                   abs(gg["yaw_err"]) > math.radians(self.ga_align_deg))
        self._ga_preturn = False           # 老的 REV->TURN0->REV 三段不再走
        if preturn:
            # 站位+转正合成**一段**(AVOID_POSE): 边转边沿板法向挪到"车头离板
            # 一个禁区"。以前是 倒车(留原地转的扫掠半径) -> 原地转正 -> 再前
            # 进到站位, 三段串行白花好几秒。合并的依据: 车身沿板法向的伸出量
            # 是 半长|cos ψ|+半宽|sin ψ|, ψ<36.8°(atan 半宽/半长)时它随 ψ 单调
            # 减小 —— 一边转正一边前进, 净空只会越来越富余; 每帧按**当前 ψ**
            # 算目标站位 keep+伸出量(ψ), 转到 0 时它就是 半长+keep, 和老站位
            # 一样。ψ 更大(罕见)时先倒到 hypot 半径外再转, 见 step_go_around。
            self.ga_rev = -(self.board_d - (self.car_half_l + self.ga_keep))
            self._ga_pose_ok_t = None
            self.set_phase("AVOID_POSE", "绕障: 边转正(还差 %.1f°)边%s到板前 "
                           "%.2fm 站位" % (math.degrees(gg["yaw_err"]),
                                          "前进" if self.ga_rev < 0 else "后退",
                                          self.ga_keep))
            return
        need = self.car_half_l + self.ga_keep
        gap = self.board_d - need          # 正=还差这么多才到位, 负=太近了
        if self.board_d > 0.05 and abs(gap) > 0.02:
            self.ga_rev = -gap             # gap>0 -> 负 -> 前进
            if gap > 0:
                rospy.loginfo("绕障: 先前进 %.2fm 到板前 %.2fm(车头间隙"
                              "=禁区), 再横移", gap, need)
            else:
                rospy.logwarn("绕障: 板子只有 %.2fm(需要 %.2fm 才够横移), "
                              "先倒 %.2fm", self.board_d, need, -gap)
            self.set_phase("AVOID_REV", "绕障: %s %.2fm 到横移站位"
                           % ("前进" if gap > 0 else "后退", abs(gap)))
            return
        self.set_phase("AVOID_OUT", "绕障第%d次: 往%s横移 %.2fm, 之后前进 "
                       "%.2fm (板子在 %.2fm 处) [%s]"
                       % (self.ga_n, "左" if self.ga_sign > 0 else "右",
                          self.ga_side_eff(), self.ga_fwd_now,
                          self.board_d, why))

    def ga_rear_plane(self, g):
        """横回段: 从这一帧雷达里找车**后方**板子的背面(哪怕只露一小截弦),
        拟直线, 返回车心到板面的法向距(负数 = 板在车后), 找不到 None。

        为什么单独来一遍而不用卡尔曼: 卡尔曼更新的是**板心**, 背面只露一
        小截时弦中点当板心会偏(横向); 但**板面在哪**(法向距)一小截弦也量得
        准 —— 直线的垂距不看弦长。横回段真正危险的是车尾贴着板面蹭过板端,
        要的就是这个法向距。实车 2026-08-19 雷达后方 -118°~-94° 整块无回波,
        前进段末尾根本看不见板子, 只能等横回时板子转进可见扇区再量。
        """
        m = self.scan
        if m is None or g is None or "lon_odom" not in g:
            return None
        xs, ys = scan_xy(m.ranges, m.angle_min, m.angle_increment,
                         max(0.05, m.range_min), min(16.0, m.range_max),
                         lidar_x=self.board_lidar_x, yaw_off=self.board_yaw_off,
                         self_margin=self.board_self_margin)
        if len(xs) == 0:
            return None
        lon = g["lon_odom"]                        # odom 推的板面法向距(<0)
        ux, uy = g["ux"], g["uy"]                  # 板面方向(车体系)
        nx, ny = -uy, ux                           # 板面法向(车体系)
        if nx > 0:                                 # 取指向车后的那一支
            nx, ny = -nx, -ny
        # 沿法向离 odom 预测的板面 12cm 以内、沿板面方向落在板子范围(+余量)
        # 内的点。lat_odom 是车心离中垂线的距离, 板心在车体系里沿 u 就是 -lat
        d_n = xs * nx + ys * ny                    # 沿法向(车后为正)
        d_u = xs * ux + ys * uy                    # 沿板面
        lat = g.get("lat_odom", g["lat"])
        keep = (np.abs(d_n - (-lon)) <= 0.12) & \
            (np.abs(d_u + lat) <= self.kf.half + 0.15)
        if int(keep.sum()) < 5:
            return None
        P = np.column_stack([xs[keep], ys[keep]])
        c = P.mean(axis=0)
        A = P - c
        vals, vecs = np.linalg.eigh(np.dot(A.T, A))
        tg = vecs[:, int(np.argmax(vals))]
        # 方向得和板面方向差不多(<20°), 否则是别的东西(围挡/车尾杂点)
        if abs(float(tg[0] * ux + tg[1] * uy)) < math.cos(math.radians(20.0)):
            return None
        span = float(np.dot(A, tg).max() - np.dot(A, tg).min())
        if span < 0.06:
            return None
        nm = np.array([-tg[1], tg[0]])
        perp = np.dot(A, nm)
        if float(np.sqrt(np.mean(perp ** 2))) > 0.03:
            return None
        dist = float(np.dot(c, nm))                # 车心到板面的有向距离
        if nm[0] * nx + nm[1] * ny < 0:            # 法向统一成指向车后
            dist = -dist
        return -dist                               # 板在车后 -> 负

    def _ga_freeze_ref(self, tag):
        """冻结板面方向 u0(odom 系)、车心 p0 和当前离中垂线 lat0, 给横回段按
        odom 回中用。要在**正面看得见整块板**的时候冻(起绕 / 站位结束)。"""
        pose = self.car_pose()
        if not (self.kf.ready() and pose is not None):
            return
        g0 = self.ga_geom()
        if g0 is None:
            return
        self._ga_u0 = (float(self.kf.u[0]), float(self.kf.u[1]))
        self._ga_p0 = (pose[0], pose[1])
        # 不带曲率修正的原始 lat(ga_geom 在板前不修, 但保险起见加回来)
        self._ga_lat0 = float(g0["lat"] + g0.get("arc_lat", 0.0))
        self._ga_lon0 = float(g0["lon"])
        rospy.loginfo("绕障(%s): 冻结板面方向 u0=(%.2f,%.2f) 横向偏差 %.3fm "
                      "法向距 %.3fm, 前进/横回按 %s 判", tag, self._ga_u0[0],
                      self._ga_u0[1], self._ga_lat0, self._ga_lon0,
                      self.ga_back_src)

    def _ga_pose_target(self, psi):
        """站位目标(车心到板面的法向距离): 禁区 + 车身沿板法向的伸出量。
        车头歪 ψ 时伸出量 = 半长|cos ψ| + 半宽|sin ψ|; ψ=0 就是 半长+keep。"""
        return (self.ga_keep + self.car_half_l * abs(math.cos(psi)) +
                self.car_half_w * abs(math.sin(psi)))

    def step_go_around(self):
        """返回 (vx, vy); 相位推进在内部完成。

        每一段的"走完没有"优先用**闭环几何判据**(见 ga_geom), 卡尔曼跟丢了
        就退回开环距离。两者同时生效: 开环距离是**上限夹取**, 跟踪出任何
        问题都不会让车一直挪下去。"""
        d = self.moved_since_mark()
        lim = {"AVOID_REV": abs(self.ga_rev),
               "AVOID_POSE": abs(self.ga_rev) + 0.10,
               "AVOID_OUT": self.ga_side_now or self.ga_side_eff(),
               "AVOID_FWD": self.ga_fwd_now,
               "AVOID_BACK": self.ga_out_dist or self.ga_side_now or
               self.ga_side_eff(),
               "AVOID_ALIGN": 9.9, "AVOID_TURN0": 9.9}[self.phase]
        name = {"AVOID_REV": "站位", "AVOID_POSE": "站位+转正",
                "AVOID_OUT": "横移", "AVOID_FWD": "前进",
                "AVOID_BACK": "横回", "AVOID_ALIGN": "对航向",
                "AVOID_TURN0": "转正"}[self.phase]
        lim_hard = lim * 1.25 + 0.10        # 闭环可以比开环多走一点点

        g = None
        if self.kf.ready():
            self.kf.predict(1.0 / max(1.0, self.rate_hz))
            self.kf_observe()
            # 信不信得过, 看卡尔曼自己给的标准差, 不看"丢了几帧"。
            # 短暂看不见(擦面盲区)时 sigma 只涨一点点, 照常闭环;
            # 真丢了 sigma 会一路涨, 到阈值自动退回开环。
            if self.kf.sigma() <= self.ga_max_sigma:
                g = self.ga_geom()
            elif self.ga_lost == 0:
                self.ga_lost = 1
                rospy.logwarn("绕障: 板子位置不确定度 %.3fm 超过 %.3fm"
                              "(连续跟丢 %d 帧), 本段改用开环距离",
                              self.kf.sigma(), self.ga_max_sigma,
                              self.kf.miss)
            # 横回段按 odom 回中不需要信卡尔曼(u0/lat0 是起绕时冻结的):
            # 卡尔曼跟丢也照常闭环。实车 2026-08-19: 车刚过板子雷达看不到
            # 背面, sigma 涨过阈值, 横回前 60 帧全是开环。
            if g is None and self.phase in ("AVOID_FWD", "AVOID_BACK") and \
                    self.ga_back_src == "odom" and self._ga_u0 is not None:
                g = self.ga_geom()
        if g is None:
            self.ga_lost += 1
            done = d >= lim
            src = "开环 %.2f/%.2f" % (d, lim)
        else:
            if self.phase == "AVOID_POSE":
                psi = g["yaw_err"]
                tgt = self._ga_pose_target(psi)
                e = abs(g["lon"]) - tgt
                # 三项: 法向距(站位) / 航向(转正) / 离中垂线(对中)。对中用的
                # 是正面看的卡尔曼 lat(整块板在视野里, 这时最准)。
                # 两项: 法向距(站位) / 航向(转正)。**不对中垂线**: 离中垂线的
                # 偏差 lat 在站位到位时冻结成 lat0, 横让距离按板端实际位置算、
                # 横回按 odom 回到 lat=0, 偏差就这么补掉了, 不用在板前多挪。
                in_tol = (abs(e) <= 0.02 and
                          abs(psi) <= math.radians(self.ga_align_deg))
                if in_tol:
                    if self._ga_pose_ok_t is None:
                        self._ga_pose_ok_t = time.time()
                    held = time.time() - self._ga_pose_ok_t
                    done = held >= self.ga_pose_settle
                else:
                    self._ga_pose_ok_t = None
                    held = 0.0
                    done = False
                src = ("板法向距 %.3f/%.3f 航向偏 %.1f°/%.1f° (离中垂线 %.3f, 不在"
                       "这儿对)%s" % (abs(g["lon"]), tgt, math.degrees(psi),
                                    self.ga_align_deg, g["lat"],
                                    (" 已到位, 稳定 %.1f/%.1fs" %
                                     (held, self.ga_pose_settle)) if in_tol else ""))
            elif self.phase == "AVOID_REV":
                # 双向: 往后倒是要把板心推远到 need 以上, 往前挪是要把它
                # 拉近到 need 以下, 判据跟着方向反过来。
                _n = self.car_half_l + self.ga_keep
                done = (g["bx"] >= _n) if self.ga_rev > 0 else (g["bx"] <= _n)
                src = "板心纵向 %.3f/%.3f" % (g["bx"], _n)
            elif self.phase == "AVOID_OUT":
                done = abs(g["by_end"]) >= g["need_lat"]
                src = "板端横向 %.3f/%.3f" % (abs(g["by_end"]), g["need_lat"])
            elif self.phase == "AVOID_FWD":
                use_odom = self.ga_back_src == "odom" and "lon_odom" in g
                if use_odom:
                    # 车尾过板面 tail: 板心法向距 <= -(半长 + tail), 不加 σ
                    # (odom 量没有"估计越不准"这回事)。卡尔曼还信得过(σ 没超)
                    # 的话它也得点头 —— 两个量谁说"还没过"就听谁, 宁多走几厘米。
                    # (odom 推的 lon 会被冻结方向 u0 的角误差串扰: 横让 0.5m、
                    #  u0 偏 3° 就是 2.6cm; 实车 2026-08-19 odom 说 -0.266,
                    #  背面实测 -0.23)
                    tgt = -(self.car_half_l + self.ga_tail)
                    kf_ok = self.kf.sigma() <= self.ga_max_sigma
                    kf_clear = (g["bx"] <= -(self.car_half_l + self.ga_tail)) \
                        if kf_ok else True
                    done = g["lon_odom"] <= tgt and kf_clear
                    src = "板面法向距 %+.3f/%.3f [odom; kf bx=%+.3f%s]" % (
                        g["lon_odom"], tgt, g["bx"],
                        "" if kf_ok else " 不可信")
                else:
                    done = g["bx"] <= g["need_back"]
                    src = "板心纵向 %+.3f/%.3f" % (g["bx"], g["need_back"])
            elif self.phase == "AVOID_BACK":
                # ⚠ 在**板子系**里回中: lat 是车心离板子中垂线的距离。
                # 以前用车体系的 by(板心在车体 y 上的投影), 那等于"板心在
                # 车正前方", 只有车头恰好垂直板面时才等价于中垂线 —— 车
                # 进来时歪多少, 回完中就原样歪多少, 后面越走越偏。
                lat_kf = abs(g["lat"])
                use_odom = self.ga_back_src == "odom" and "lat_odom" in g
                lat = abs(g["lat_odom"]) if use_odom else lat_kf
                # 过冲保护: |lat| 一旦从最小值明显回升, 说明已越过中垂线
                # 正在越走越远, 立刻收 —— 这条判据是**单峰**的, 错过最低点
                # 就再也不满足了。实车 2026-08-16: 0.512 一路降到 0.014
                # (早够 0.040)却被下限夹取否掉, 然后冲到 0.325 还在走。
                if lat < self._ga_lat_min:
                    self._ga_lat_min = lat
                done = lat <= self.ga_back_tol
                extra = ""
                # ⚠ 过冲保护必须等车**真的横移过一段**才能上膛。实车
                # 2026-08-16: 刚切进 AVOID_BACK 的 0.25s 里车只挪了 2.2mm
                # (底盘还在收前一段的速度), 而板子估计自己在漂, |lat| 就
                # 涨了 20mm —— 保护第一帧误触发, 三段绕障只跑了两段。
                armed = d >= self.ga_back_arm
                if armed and not done and lat > self._ga_lat_min + 0.02:
                    done = True
                    extra = " [已过中垂线, 最近到过 %.3f, 收]" % \
                        self._ga_lat_min
                elif not armed:
                    extra = " [横移 %.3f<%.2f, 过冲保护未上膛]" % (
                        d, self.ga_back_arm)
                src = "离中垂线 %.3f/%.3f [%s; kf=%.3f] (最近 %.3f, 车体系 by=%.3f)%s%s" % (
                    lat, self.ga_back_tol, "odom" if use_odom else "kf",
                    lat_kf, self._ga_lat_min, g["by"], extra,
                    ("  [曲率修正 %+.1f° %+.0fmm]"
                     % (math.degrees(g["arc_phi"]), 1000 * g["arc_lat"]))
                    if g.get("arc_phi") else "")
            else:                       # AVOID_ALIGN / AVOID_TURN0
                done = abs(g["yaw_err"]) <= math.radians(self.ga_align_deg)
                src = "航向偏 %.1f°/%.1f°%s" % (
                    math.degrees(g["yaw_err"]), self.ga_align_deg,
                    ("  [曲率修正 %+.1f°]" % math.degrees(g["arc_phi"]))
                    if g.get("arc_phi") else "")
            src += " sig纵%.3f横%.3f 跟丢%d 更新%d" % (
                g["sx"], g["sy"], self.kf.miss, self.kf.n_upd)
            if self.ga_blind:
                src += " 擦面盲区%d帧" % self.ga_blind
            # 下限夹取: lim 本身就是"板半长+车半宽+禁区"这个几何目标,
            # 闭环只是在它附近做修正(补 odom 比例误差、板子没摆正)。
            # 要是闭环喊着在半路就到位了, 那一定是跟踪出了问题, 不听它。
            # ⚠ 下限夹取只对"走直线的那几段"有意义(OUT/FWD/REV): 它们的
            # 目标和开环距离有固定比例关系, 闭环只在附近修正。
            #   AVOID_BACK  判据是"离中垂线多远", 单峰量, 到位点由板子实际
            #               位置决定, 车歪一点就可能落在开环距离的 60% 处。
            #   AVOID_ALIGN 是**原地转**, 位移恒等于 0, 而 lim 取的 9.9,
            #               夹取永远成立 —— 实车航向已经收到 0.4°(阈值 3°)
            #               还在原地转了半分钟, 就是这么卡住的。
            # 两段都豁免。
            if done and self.phase not in ("AVOID_BACK", "AVOID_ALIGN",
                                           "AVOID_TURN0", "AVOID_POSE") \
                    and d < self.ga_min_frac * lim:
                done = False
                src += " [未到%.0f%%下限, 不采信]" % (self.ga_min_frac * 100)
            if d >= lim_hard and not done:       # 上限夹取
                rospy.logwarn("绕障%s段: 闭环判据没满足但已走 %.2f/%.2fm, "
                              "按上限收尾 (%s)", name, d, lim_hard, src)
                done = True

        if time.time() - self._board_log_t > 0.5:
            self._board_log_t = time.time()
            ang = self.bearing_txt(g["bx"], g["by"]) if g else "-"
            rospy.loginfo("[绕障] %s 已走 %.3fm  %s  板心方位 %s",
                          name, d, src, ang)

        # 超时预算: 走直线的段按"距离/速度"给, 原地转那段按角度给
        # (它位移恒为 0, 用距离算出来是几百秒, 等于没有超时保护)。
        if self.phase in ("AVOID_ALIGN", "AVOID_TURN0"):
            budget = 8.0
        elif self.phase == "AVOID_POSE":
            budget = (lim_hard / max(0.05, self.ga_speed) * 2.0 + 8.0 +
                      self.ga_pose_settle)
        else:
            budget = lim_hard / max(0.05, self.ga_speed) * 2.0 + 4.0
        if time.time() - self.mark_t > budget:
            rospy.logwarn("绕障%s段超时 %.1fs, 强制收尾", name, budget)
            done = True
        if done:
            self.mark_xy, self.mark_t = self.odom_xy, time.time()
            if self.phase == "AVOID_REV" and self._ga_preturn:
                # 退够了, 先原地转正对板子, 转完再重新站位。
                self._ga_preturn = False
                # ⚠ 别写成 "...%.1f" % f(x) if cond else "..." —— % 比
                # if-else 结合得紧, f(x) 会在判断 cond 之前就求值, None
                # 直接崩在这儿。老老实实分两句。
                gt = self.ga_geom()
                why0 = ("绕障: 先转正对准板子法向(还差 %.1f°)"
                        % math.degrees(gt["yaw_err"])) if gt else \
                       "绕障: 先转正对准板子法向"
                self.set_phase("AVOID_TURN0", why0)
            elif self.phase == "AVOID_TURN0":
                # 转正了, 现在"横移"才真的平行于板面。重新量一次板子离车心
                # 多远(用板子系的法向分量 |lon|, 车已经对正了所以它就是
                # 纵向距离), 再决定站位要前进还是后退 —— 不用再留原地转的
                # 扫掠半径, 回到 车半长 + 禁区。
                g2 = self.ga_geom()
                if g2 is not None:
                    self.board_d = max(0.06, abs(g2["lon"]))
                self.ga_rev = -(self.board_d
                                - (self.car_half_l + self.ga_keep))
                self.ga_fwd_now = self.ga_fwd_eff()
                self.set_phase("AVOID_REV", "绕障: 已转正, %s %.2fm 到横移站位"
                               % ("前进" if self.ga_rev < 0 else "后退",
                                  abs(self.ga_rev)))
            elif self.phase in ("AVOID_REV", "AVOID_POSE"):
                # 站位段走完后板子离车心就是 need 了, 前进段要按这个新
                # 距离重算 —— 否则开环兜底会比实际多走一个站位段的长度。
                self.board_d = self.car_half_l + self.ga_keep
                self._ga_pose_ok_t = None
                self._ga_freeze_ref("站位到位")   # 车已在中垂线上, lat0≈0
                # 开环兜底的前进距离也按**实测**站位算(有卡尔曼的话)
                if self._ga_u0 is not None and self._ga_lon0 > 0.05:
                    self.board_d = self._ga_lon0
                self.ga_fwd_now = self.ga_fwd_eff()
                # 横让开环距离按板端**实际**横向位置补偿离中垂线的偏差:
                # 要走 = 让向那端此刻的横向坐标(ga_sign 方向为正) + 需要的净空
                gg = self.ga_geom()
                self.ga_side_now = 0.0
                if gg is not None and self.ga_side <= 0.0:
                    self.ga_side_now = max(0.05, self.ga_sign * gg["by_end"] +
                                           gg["need_lat"])
                self.set_phase("AVOID_OUT", "绕障: 站位到板前 %.2fm, 开始往"
                               "%s横移 %.2fm(之后前进 %.2fm)"
                               % (self.ga_keep,
                                  "左" if self.ga_sign > 0 else "右",
                                  self.ga_side_now or self.ga_side_eff(),
                                  self.ga_fwd_now))
            elif self.phase == "AVOID_OUT":
                self.ga_out_dist = d          # 记下实际横让了多少
                # 横移里车会蹭出几厘米纵向漂移(实车 0.29 -> 0.32), 开环兜底的
                # 前进距离按此刻 odom 推算的法向距重算
                gg = self.ga_geom()
                if gg is not None and "lon_odom" in gg and gg["lon_odom"] > 0.05:
                    self.ga_fwd_now = (gg["lon_odom"] + self.car_half_l +
                                       self.ga_keep) if self.ga_fwd <= 0.0 \
                        else self.ga_fwd_now
                self.set_phase("AVOID_FWD", "绕障: 已横让 %.3fm, 前进 %.3fm 到"
                               "板子落在车尾之后" % (d, self.ga_fwd_now))
            elif self.phase == "AVOID_FWD":
                # 车已经整个越过板子, 现在看的是背面 —— 用这一帧把板子
                # 重新定位一次, 后面回中和对航向都以它为准。
                self.ga_refit_board()
                self._ga_lat_min = 9.9        # 过冲保护每段重置
                self._ga_rear_n = 0
                self._ga_rear_last = None
                self.set_phase("AVOID_BACK", "绕障: 往%s横移回板子中垂线"
                               % ("右" if self.ga_sign > 0 else "左"))
            elif self.phase == "AVOID_BACK" and self.ga_align_deg > 0 \
                    and g is not None:
                # 收尾对航向: 板子横跨赛道, 它的法向就是车道方向。三段横平
                # 竖直走下来航向没变, 而车进来时本来就可能是歪的 —— 在这里
                # 用板子把航向也归位, 免得把偏差原样带到后半程。
                self.set_phase("AVOID_ALIGN", "绕障: 对齐板子法向(还差 %.1f°)"
                               % math.degrees(g["yaw_err"]))
            else:
                self.board_hits = 0
                self.board_cool_until = time.time() + self.ga_cool
                if self.ga_cool_goal > 0.0:
                    self.cool_until = max(self.cool_until,
                                          time.time() + self.ga_cool_goal)
                self.kf = BoardKF()
                self.board_state = "已避"
                rospy.loginfo("板子已处理 -> 切回初赛逻辑(不再查板子, "
                              "终点白线检测放开)")
                self.set_phase("FOLLOW", "绕障完成, 回到巡线 "
                               "(%.1fs 内不认板子; 横线%s)"
                               % (self.ga_cool,
                                  ("%.1fs 内不认" % self.ga_cool_goal)
                                  if self.ga_cool_goal > 0.0 else "立刻放开"))
                self.ga_trace("done")
                if self._ga_fh is not None:
                    try:
                        self._ga_fh.close()
                    except Exception:
                        pass
                    self._ga_fh = None
            return 0.0, 0.0
        self.ga_trace()
        if self.phase in ("AVOID_ALIGN", "AVOID_TURN0"):
            g = self.ga_geom()
            if g is None:
                return 0.0, 0.0
            self._ga_az = max(-0.4, min(0.4, 2.0 * g["yaw_err"]))
            if abs(self._ga_az) < 0.12:      # 底盘旋转死区
                self._ga_az = math.copysign(0.12, self._ga_az)
            return 0.0, 0.0
        if self.phase == "AVOID_POSE":
            g = self.ga_geom()
            if g is None:                     # 跟丢: 只按开环距离挪, 不转
                self._ga_az = 0.0
                return (-self.ga_speed if self.ga_rev > 0
                        else self.ga_speed), 0.0
            psi = g["yaw_err"]
            lon = abs(g["lon"])
            az = max(-0.4, min(0.4, 2.0 * psi))
            if abs(psi) <= math.radians(self.ga_align_deg):
                az = 0.0                      # 转正了就别抖
            elif abs(az) < 0.12:
                az = math.copysign(0.12, az)  # 底盘旋转死区
            # ψ 超过 atan(半宽/半长)=36.8° 时转正过程中伸出量先涨后跌, 最大到
            # hypot; 这时净空不够就先只倒不转
            if abs(psi) > math.atan2(self.car_half_w, self.car_half_l) and \
                    lon < math.hypot(self.car_half_l, self.car_half_w) + \
                    self.ga_keep:
                az = 0.0
            self._ga_az = az
            e = lon - self._ga_pose_target(psi)
            vx = vy = 0.0
            # 沿板法向挪(车体系里法向 = (cos ψ, sin ψ)); e>0 靠近, e<0 退开
            if abs(e) > 0.02:
                sgn = 1.0 if e > 0 else -1.0
                vx += sgn * self.ga_speed * math.cos(psi)
                vy += sgn * self.ga_speed * math.sin(psi)
            return vx, vy
        self._ga_az = 0.0
        if self.phase == "AVOID_REV":
            # ga_rev>0 倒车, <0 前进
            return (-self.ga_speed if self.ga_rev > 0 else self.ga_speed), 0.0
        if self.phase == "AVOID_FWD":
            return self.ga_speed, 0.0
        sgn = self.ga_sign if self.phase == "AVOID_OUT" else -self.ga_sign
        if self.phase == "AVOID_BACK" and self.ga_back_src == "odom":
            # 横回段和板面的距离也闭环: 背面一露出来就量法向距, 修正 odom
            # 推算; 车尾离板面不够 tail 就先前进补够, 再接着横回 —— 别贴着
            # 板面把屁股蹭过板端。
            gb = self.ga_geom()
            if gb is not None and "lon_odom" in gb:
                meas = self.ga_rear_plane(gb)
                if meas is not None:
                    corr = meas - gb["lon_odom"]
                    self._ga_lon0 += 0.5 * corr        # 一半, 免得单帧跳
                    self._ga_rear_n += 1
                    self._ga_rear_last = meas
                    if self._ga_rear_n in (1, 10, 40) or abs(corr) > 0.03:
                        rospy.loginfo("绕障横回: 背面重捕获板面 法向距 %.3f "
                                      "(odom 推算 %.3f, 修 %+.3f, 第 %d 次)",
                                      meas, gb["lon_odom"], corr,
                                      self._ga_rear_n)
                    gb = self.ga_geom()
                need = -(self.car_half_l + self.ga_tail)
                if gb["lon_odom"] > need + 0.005:
                    if time.time() - self._board_log_t > 0.5:
                        rospy.loginfo("绕障横回: 车尾离板面只 %.3f(要 %.3f), "
                                      "先前进补够再横回", -gb["lon_odom"] -
                                      self.car_half_l, self.ga_tail)
                    return self.ga_speed, 0.0
        return 0.0, sgn * self.ga_speed

    def scan_cb(self, msg):
        self.scan = msg
        self.scan_t = time.time()

    def scan_fresh(self):
        return (self.scan is not None
                and time.time() - self.scan_t <= self.scan_stale)

    @staticmethod
    def bearing_txt(bx, by):
        """方位角文字: 正前方 0°, 左右各到 180°。"""
        a = math.degrees(math.atan2(abs(by), bx))
        return "%s%.0f°" % ("正前" if abs(by) < 1e-6 else
                            ("左" if by > 0 else "右"), a)

    def check_board(self):
        """返回 True = 这一帧判到拦路板。只在 FOLLOW 相位、且已经走够一段
        距离之后才调用(见 run 里的门)。"""
        msg = self.scan
        if not self.scan_fresh():
            if not self._scan_warned and time.time() - self.t_boot > 5.0:
                self._scan_warned = True
                rospy.logwarn("拦路板检测开着, 但 5s 没收到雷达数据 —— "
                              "雷达驱动起了吗? 话题名对吗?")
            return False
        xs, ys = scan_xy(msg.ranges, msg.angle_min, msg.angle_increment,
                       max(0.05, msg.range_min), min(16.0, msg.range_max),
                       lidar_x=self.board_lidar_x, yaw_off=self.board_yaw_off,
                         self_margin=self.board_self_margin)
        hit, info = board_detect(
            xs, ys, lane_half=self.board_lane_half,
            x_max=self.board_range(), min_w=self.board_min_w,
            max_w=self.board_max_w, min_pts=self.board_min_pts,
            gap=self.board_gap, fov_deg=self.board_fov)
        if hit:
            self.board_yl, self.board_yr = info["yl"], info["yr"]
            self.board_d = info["d"]
            self.board_seen_t = time.time()   # 给"别在看得见板子时判无板"用
        near = hit and info["d"] <= self.board_stop
        self.board_hits = self.board_hits + 1 if near else 0
        now = time.time()
        if now - self._board_log_t > 1.0:      # 每秒一行, 调参就看它
            self._board_log_t = now
            ang = (self.bearing_txt(info["d"], (info["yl"] + info["yr"]) / 2.0)
                   if hit else "-")
            rospy.loginfo("[拦路板] 距离 %.2fm(走廊内 %.2f) 角度 %s "
                          "长=%.2f(横跨%.2f 斜%.0f°) 点=%d 簇=%d "
                          "-> %s (连续 %d/%d)  |  再走 %.2fm 还没扫到就"
                          "判定无板", info["d"],
                          info.get("d_lane", info["d"]), ang,
                          info.get("L", 0.0), info["w"],
                          info.get("tilt", 0.0), info["n"],
                          info["nclus"],
                          info["why"] + ("" if near or not hit
                                         else " 但还没到 %.2fm" % self.board_stop),
                          self.board_hits, self.board_confirm,
                          max(0.0, self.board_clear_now() - self._board_trav))
        return self.board_hits >= self.board_confirm

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
                self.ensure_segmentation_model()
            except Exception as e:
                rospy.logerr("交接失败: 分割网加载不了: %s", _s(e))
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
        if self.phase in ("AVOID_REV", "AVOID_OUT", "AVOID_FWD", "AVOID_BACK",
                           "AVOID_ALIGN", "AVOID_TURN0", "AVOID_POSE"):
            return abs(self.ga_speed)
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
            # 这段是盲走(odom 计距, 不看图), 正好把切分辨率(几百 ms 到 1s)
            # 藏进去 —— 到位的时候相机已经是 1920x1080 了, 直接认灯。
            if self.yolo_crop and self.yolo_cam_w > 0 and self.cap is not None:
                self._preswitch_result = None
                threading.Thread(target=self._preswitch_cam).start()
            self.start_move(self.align_offset, self.start_move_speed,
                            "START_MOVE", "起跑: %s %.0fmm 进到黄线前的规定"
                            "位置, 然后等绿灯"
                            % ("前进" if self.align_offset > 0 else "后退",
                               abs(self.align_offset) * 1000.0))
        else:
            self.start_yolo()

    # ---------------- 红绿灯分支 ----------------
    def yellow_ok(self):
        """这一刻黄灯算不算"可以走"。adaptive 下是**随时间变的**。"""
        if self.yellow_mode == "true":
            return True
        if self.yellow_mode != "adaptive":
            return False
        if self.yellow_go_after <= 0:
            return True
        return (time.time() - (self.yolo_t0 or time.time())
                >= self.yellow_go_after)

    def yolo_norm(self, name):
        """7 类模型的类名 -> (方向类 or None, 是不是黄灯)。

        left/right/straight/stop 原样; yellow X 按 yellow_go 决定:
          开  -> 归一成 X, 当普通方向灯用
          关  -> 归一成 "stop", 走"继续等"那条路
        认不出来的类名一律返回 None(和"什么都没检出"同等对待)。

        ⚠ 关掉时必须归一成 "stop" 而**不是** None: None 会被当成"这一帧
          什么都没认出来", 于是一路空等到 yolo_wait_max(60s) 超时再按
          fallback 瞎走; "stop" 走的是"红灯继续等"那条正路, 绿灯一亮
          立刻发车。
        """
        go = self.yellow_ok()
        if not name:
            return None, False
        n = str(name).strip().lower()
        if n.startswith("yellow"):
            core = n[len("yellow"):].strip().replace("_", " ").strip()
            if core in ("left", "right", "straight"):
                return (core if go else "stop"), True
            return ("stop" if not go else None), True
        if n in ("left", "right", "straight", "stop"):
            return n, False
        return None, False

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
        self._yellow_warned = False
        self.yolo_t0 = time.time()
        self._crop_gave_up = False
        self._crop_log = None
        # 只在认灯这一小段切到高分辨率, 认完立刻切回去(见 set_cam_res)
        self._cam_switched = False
        if self.yolo_crop and self.yolo_cam_w > 0:
            with self._cam_lock:            # 预切线程还在干活的话等它做完
                pre = self._preswitch_result
            self._preswitch_result = None
            if pre is False:
                # 预切已经试过一次失败了(退回 640 了), 别在这儿再花一秒重试
                rospy.logwarn("认灯: 挪动时预切高分辨率失败, 不再重试, "
                              "用整帧缩放")
            else:
                # pre 为 True 时 set_cam_res 发现已是目标尺寸, 立刻返回
                self._cam_switched = self.set_cam_res(
                    self.yolo_cam_w, self.yolo_cam_h, "认灯")
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
            rospy.logerr("拉起 yolo 失败: %s", _s(e))

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
            rospy.logwarn("yolo 第 %d 次出错: %s", self.yolo_err, _s(e))
            if self.yolo_err >= 3:
                return self.yolo_fallback
            return None
        # 只有方向类(left/right/straight)投票。**stop(红灯) 和 什么都没
        # 认出来 走同一条路: 不投票、不定案、继续等下一帧** —— 红灯本来
        # 就是让你停着等, 认不出来也没资格瞎猜。
        if best:
            name, conf, box = best
            norm, is_yellow = self.yolo_norm(name)
            self.yolo_last = "%s %.2f%s" % (
                name, conf,
                "" if not is_yellow else
                ("->%s" % norm if self.yellow_ok() else "(黄灯, 按红灯等)"))
            if is_yellow and not self._yellow_warned:
                self._yellow_warned = True
                if self.yellow_ok():
                    rospy.logwarn("认到黄灯(%s), 放行(mode=%s) -> 按 %s 走",
                                  _s(name), _s(self.yellow_mode), _s(norm))
                elif self.yellow_mode == "adaptive":
                    rospy.loginfo("认到黄灯(%s), adaptive: 已等 %.1fs, 满 "
                                  "%.0fs 还是黄就走", _s(name),
                                  time.time() - (self.yolo_t0 or time.time()),
                                  self.yellow_go_after)
                else:
                    rospy.loginfo("认到黄灯(%s), yellow_go=false -> 继续等绿灯",
                                  _s(name))
            if norm in ("left", "right", "straight"):
                self.yolo_votes[norm] = self.yolo_votes.get(norm, 0) + 1
            if self.dump_dir:
                self._dump_yolo(name, conf, box)
        else:
            self.yolo_last = "-"
        arrows = [(n, c) for n, c in self.yolo_votes.items()]
        arrows.sort(key=lambda kv: -kv[1])
        if arrows and arrows[0][1] >= self.yolo_min_votes:
            # 这一行是日志里**唯一**能反推认灯频率的地方: 帧数 / 秒数。
            # 以前只打帧数不打秒数, 事后想知道"车上 yolo 多少 Hz"没处查。
            el = time.time() - self.yolo_t0
            rospy.loginfo("[认灯] 第%d帧 %s -> 够票了 (共 %.1fs, %.2f 帧/秒, "
                          "单帧 %.0fms)", self.yolo_n, self.yolo_last, el,
                          (self.yolo_n / el) if el > 1e-3 else 0.0,
                          1000.0 * el / max(1, self.yolo_n))
            self._verdict_from_light = True     # 真认到灯了, 可以冲线
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

    def _yolo_cleanup(self):
        """认灯收尾: 杀子进程 + 相机切回 640x480。**在后台线程里跑**。

        线程安全: 这时候主线程已经不碰 self.yolo 了(定案之后不再认灯);
        self.cap 只有取帧线程在读, 而 set_cam_res 会先 pause 它。所以这
        两个资源在这一小段时间里都是单写者。
        """
        t0 = time.time()
        try:
            self.kill_yolo()
        except Exception as e:
            rospy.logwarn("认灯收尾出错: %s", _s(e))
        dt = time.time() - t0
        if dt > 0.05:
            rospy.loginfo("认灯收尾(杀进程+切回相机)耗时 %.0fms —— 在后台做的, "
                          "没占发车时间", 1000 * dt)

    def kill_yolo(self):
        """认完/退出就杀 —— Nano 4GB 显存留不得。

        顺手把相机切回标定分辨率: 放在这里而不是 finish_yolo, 是因为退出
        认灯的路不止一条(定案 / 等超时走 fallback / 连错三次 / 远程急停 /
        节点关闭), 漏掉任何一条都会让巡线一直吃高分辨率的帧, 而去畸变映射
        是按 640x480 标的。

        ⚠ 顺序: **先杀子进程, 再切相机**。子进程要是继承了相机 fd(见
          yolo_client 里 close_fds 的说明), 它不退出设备就一直被占着, 切
          分辨率必失败。现在 close_fds=True 了按理不会继承, 但顺序仍按
          "先放资源再拿资源"来, 双保险。
        """
        if self.yolo is not None:
            try:
                self.yolo.close()
                rospy.loginfo("yolo 子进程已退出")
            except Exception as e:
                rospy.logwarn("关 yolo 出错: %s", _s(e))
            self.yolo = None
        self.restore_cam_res()

    def finish_yolo(self, cls):
        """起点定案: 杀进程 -> **记住**箭头方向 -> 先按 start_offset 挪到
        真正的起跑点 -> 再按记住的方向做动作。
        车是特意往后摆 start_offset 的(让灯落在视野里/别压起跑线), 所以
        认完必须补这一段; 认灯期间车一直没动, 挪的时候灯已经不看了。"""
        # ⚠ **先发车再清理**。kill_yolo 里两件事都可能阻塞主线程:
        #   yolo.close() 最坏等 grace=2.0s; cap.set 切分辨率也是同步调用。
        # 放在 start_move 之前的话, 这段阻塞全落在"绿灯已经认出来了、车还
        # 没动"这个最不该等的窗口里。改成挂个标志, 等主循环把第一条 cmd_vel
        # 发出去、车已经在冲了, 下一拍再去清理。
        # (restore_cam_res / kill_yolo 都是幂等的, 晚调、重复调都没问题;
        #  急停和节点关闭那两条路仍然当场调, 那时候阻塞无所谓。)
        self._yolo_cleanup_pending = True
        self.branch_cls = cls                 # 记忆: 后面挪完了才用
        self.turn_deg = self.branch_deg(cls)
        # ⚠ 单独存一份: turn_deg 在 start_fork_turn 里会被改写成
        # fork_turn_deg(复用它驱动原地转), 之后就再也认不出"这趟是走臂
        # 还是走中间那条"了。绕障让向要靠这个分辨走哪条路线。
        self.start_turn_deg = self.turn_deg
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
            gain = max(1.0, self.start_dash_gain
                       if self._verdict_from_light else 1.0)
            self.start_move(self.start_offset, self.start_move_speed * gain,
                            "START_MOVE", "%s %.0fmm 开到三岔口%s"
                            % ("前进" if self.start_offset > 0 else "后退",
                               abs(self.start_offset) * 1000.0,
                               (" (冲线 %.1fx = %.2fm/s)"
                                % (gain, self.start_move_speed * gain))
                               if gain > 1.001 else " (超时兜底, 不加速)"))
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
            self.t_follow0 = [None, None, None]   # 里程从这一刻起算
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

    def goal_gate(self, hit):
        """终点横线的闸门体检: 这一帧 HIT 了, 为什么**没**停车?

        叠加图上的 HIT 只是 goal_block 的**纯几何**三者与门(覆盖率/最佳线/
        条数), 它和"会不会停车"之间还隔着四道闸。以前只有里程闸和雷达否决
        会打日志, 投票和冷却被拦时**一声不吭** —— 于是就出现"图上橙线画得
        好好的、抬头写着 HIT, 车却一路开过去"这种查不下去的情况(实车
        2026-08-18)。现在四道全部有据可查。

        返回 (goal_ok, why):
          goal_ok = 里程闸过没过(给 goal_hits 计数用, 语义和原来完全一样)
          why     = None 表示这一帧四道全过、可以停; 否则是拦住它的那道闸

        ⚠ 只做判断, **不改任何状态**, 所以一帧里调几次都安全 ——
          sense 那边要拿它写进 dump 图, 相位机这边要拿它做判决。
        ⚠ 雷达否决(第四道)不在这里: 它要真去打一次雷达, 有副作用而且贵,
          由相位机调用后把结果塞进 self._goal_why 留给下一帧的 dump。
        """
        gmin = self.goal_min_travel
        if gmin < 0.0:
            gmin = ((self.goal_gate_arm
                     if abs(self.start_turn_deg) > 1.0
                     else self.goal_gate_y) if self.board_on else 0.0)
        if self.is_fork and not self.fork_done:
            gmin = 0.0                      # 岔口那条线不设闸
        gtrav = self.board_travel(self.t_follow0) if gmin > 0.0 else 0.0
        goal_ok = (gmin <= 0.0) or (gtrav >= gmin)
        if not hit:
            return goal_ok, None
        if not goal_ok:
            return goal_ok, ("里程闸: 从%s起才走 %.2fm, 要 %.2fm"
                             % (self.board_anchor, gtrav, gmin))
        # 这一帧记上之后票数会变成多少(不真的写回去)
        votes = self.goal_hits + 1
        if votes < self.goal_confirm:
            return goal_ok, ("投票: 才连上 %d 帧, 要 %d 帧(goal_confirm)"
                             % (votes, self.goal_confirm))
        left = self.cool_until - time.time()
        if left > 0:
            return goal_ok, ("冷却: 绕障/岔口后还有 %.2fs 不认横线"
                             "(go_around_cooldown)" % left)
        if self.phase != "FOLLOW":
            return goal_ok, "相位: 当前是 %s, 不是 FOLLOW" % self.phase
        return goal_ok, None

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

    def corner_trace(self, note="", fit=None, cmd=None, extra=None):
        """终点雷达闭环全程逐帧落盘: 原始雷达 + 当帧拟合/判定/指令, 一行一个
        JSON, **写一行 flush 一行**(append 模式) —— Ctrl-C 杀掉进程也不丢已写
        的帧, 出问题直接把 dump/corner_trace_NN.jsonl 发来对账。

        和 avoid_trace 同一种格式(scan.mm 是毫米整数), 离线脚本能复用。
        """
        if not self.corner_trace_on or self._corner_fh is None:
            return
        try:
            m = self.scan
            pose = self.car_pose()
            rec = {
                "t": round(time.time() - self.t_boot, 3),
                "wall": round(time.time(), 3),
                "phase": self.phase,
                "note": note,
                "odom": None if pose is None else [round(v, 4) for v in pose],
                "elapsed": (None if self.corner_t0 is None else
                            round(time.time() - self.corner_t0, 3)),
                "stable": self.corner_stable,
                "target": self.corner_target_dist,
                "target_side": self.corner_target_side_dist,
                "tol": self.corner_target_tol,
                "lidar_x": self.board_lidar_x,
                "yaw_off": round(self.board_yaw_off, 5),
            }
            if fit is not None:
                fr = {"ok": bool(fit.get("ok")), "why": _s(fit.get("why", "")),
                      "partial": bool(fit.get("partial", False))}
                for k in ("x_wall", "y_wall"):
                    w = fit.get(k)
                    fr[k] = None if w is None else {
                        "d": round(float(w["distance"]), 4),
                        "n": [round(float(v), 4) for v in w["normal"]],
                        "res": round(float(w["residual"]), 4),
                        "span": round(float(w["span"]), 3),
                        "pts": int(w["points"])}
                fr["x_sign"] = fit.get("x_sign")
                fr["y_sign"] = fit.get("y_sign")
                rec["fit"] = fr
            if cmd is not None:
                rec["cmd"] = [round(float(v), 4) for v in cmd]
            if extra:
                rec.update(extra)
            if m is not None:
                rec["scan"] = {
                    "amin": round(m.angle_min, 6),
                    "ainc": round(m.angle_increment, 8),
                    "rmin": m.range_min, "rmax": m.range_max,
                    "stamp": (round(m.header.stamp.to_sec(), 3)
                              if hasattr(m, "header") else None),
                    "mm": [0 if (r != r or r in (float("inf"),
                                                 float("-inf")))
                           else int(r * 1000.0) for r in m.ranges],
                }
            self._corner_fh.write(json.dumps(rec) + "\n")
            self._corner_fh.flush()          # 一行一刷, 别攒
        except Exception as e:
            if not self._corner_trace_warned:
                self._corner_trace_warned = True
                rospy.logwarn("终点雷达 trace 写失败(不影响控制): %r", e)

    def _corner_trace_open(self):
        if not (self.corner_trace_on and self.dump_dir):
            return
        try:
            self._corner_trace_close()
            self._corner_n += 1
            fn = os.path.join(self.dump_dir,
                              "corner_trace_%02d.jsonl" % self._corner_n)
            # ⚠ append 模式: 万一同名文件已存在(重启没清 dump)也只是接着写,
            #   而且每行都 flush, 被 Ctrl-C 杀掉最多丢半行
            self._corner_fh = open(fn, "a")
            rospy.loginfo("终点雷达逐帧轨迹 -> %s (append, 每帧 flush)", fn)
        except Exception as e:
            rospy.logwarn("终点雷达 trace 开不了(不影响控制): %r", e)
            self._corner_fh = None

    def _corner_trace_close(self, note=""):
        if self._corner_fh is not None:
            try:
                if note:
                    self.corner_trace(note)
                self._corner_fh.close()
            except Exception:
                pass
            self._corner_fh = None

    def start_corner_adjust(self, gy):
        """视觉命中终点后，直接切入两墙雷达闭环。"""
        self._corner_goal_y = gy
        self.corner_t0 = time.time()
        self.corner_stable = 0
        self._corner_last_fit = None
        self._corner_first_ok_t = None
        self._corner_level = 0            # 停滞升级到第几级
        self._corner_yaw_forced = False   # 第 2 级: 航向不等了
        self._corner_best_prog = float("inf")
        self._corner_prog_t = time.time()
        # 看到白线就开始落盘: 入口这一帧的扫描先写一行(白线命中那一刻雷达
        # 看到什么), 之后 step_corner_adjust 每帧一行
        self._corner_trace_open()
        self.corner_trace("enter gy=%d" % gy)
        self.set_phase("CORNER_ADJUST", "终点视觉命中, 直接用雷达找两面墙停车")

    @staticmethod
    def _corner_clip(value, limit):
        return max(-limit, min(limit, value))

    def _corner_fallback_to_approach(self, reason):
        """雷达确认两面墙仍在 1m 外时，使用原来的前进停车流程。"""
        self._pause_next = "corner_fallback"
        if self.goal_pause > 0.0:
            self.pause_until = time.time() + self.goal_pause
            self.set_phase("PAUSE", "雷达墙距超过 %.2fm(%s), 退回原方案"
                           % (self.corner_fallback_dist, reason))
        else:
            self.start_approach(self._corner_goal_y)

    def step_corner_adjust(self):
        """返回 (vx, vy, wz)，稳定后返回零并切换 STOPPED。

        两面墙均以车心到墙的垂直距离计算, 平移沿**每面墙自己的法向**逼近
        (车斜着进也对)。航向只做小角度软锁(离最近车体轴 <= align_max 才转,
        更斜就不转直接滑进去); 默认边转边平移, corner_turn_first 时先转后移。
        """
        if self.corner_t0 is None:
            self.set_phase("STOPPED", "雷达角落控制未初始化, 超时停车")
            return 0.0, 0.0, 0.0
        elapsed = time.time() - self.corner_t0
        if elapsed > self.corner_timeout:
            self.set_phase("STOPPED", "雷达角落调整超时 %.1fs, 强制停车" % elapsed)
            self._corner_trace_close("timeout")
            return 0.0, 0.0, 0.0
        if not self.scan_fresh():
            if time.time() - self._corner_log_t > 1.0:
                self._corner_log_t = time.time()
                rospy.logwarn("[终点雷达] /scan %s, 暂停调整 %.1fs/%.1fs%s",
                              "从没收到过" if self.scan is None else "陈旧",
                              elapsed, self.corner_timeout,
                              "(雷达起了吗? lane_proto.launch 的 start_lidar "
                              "跟 board_in_lane/use_lidar 走)"
                              if self.scan is None else "")
            self.corner_stable = 0
            self.corner_trace("scan stale")
            # 一直没雷达就别干等 30s: 和"拟合不到墙"一样, 过了宽限期退回盲推
            # (实车 2026-08-19: board_in_lane:=false 没起雷达, 这里卡了 16s)
            if self._corner_first_ok_t is None and \
                    elapsed > self.corner_no_wall_grace:
                self._corner_fallback_to_approach(
                    "%.1fs 内没有雷达数据(/scan %s)" % (
                        elapsed, "从没收到" if self.scan is None else "陈旧"))
                self._corner_trace_close("fallback: no scan")
            return 0.0, 0.0, 0.0
        msg = self.scan
        xs, ys = scan_xy(
            msg.ranges, msg.angle_min, msg.angle_increment,
            max(0.05, msg.range_min), min(16.0, msg.range_max),
            lidar_x=self.board_lidar_x, yaw_off=self.board_yaw_off,
            self_margin=self.board_self_margin)
        fit = corner_wall_fit(
            xs, ys, max_dist=self.corner_max_fit_dist,
            min_pts=self.corner_min_pts, min_span=self.corner_min_span,
            max_residual=self.corner_max_residual,
            angle_tol_deg=self.corner_wall_angle_tol_deg,
            cluster_gap=self.corner_cluster_gap,
            back_excl_deg=self.corner_back_excl_deg,
            front_half_deg=self.corner_front_half_deg)
        self._corner_last_fit = fit
        if not fit["ok"]:
            if time.time() - self._corner_log_t > 1.0:
                self._corner_log_t = time.time()
                rospy.logwarn("[终点雷达] 拟合未通过: %s (%.1fs)",
                              fit["why"], elapsed)
            self.corner_stable = 0
            # 连前墙都拟合不到, 闭环无从谈起。以前这里原地干等到 30s 超时;
            # 现在等 corner_no_wall_grace 秒(雷达偶尔一两帧丢墙很正常),
            # 还不行就退回盲推 —— 至少车会动。
            self.corner_trace("fit fail", fit=fit, cmd=(0.0, 0.0, 0.0))
            if self._corner_first_ok_t is None and \
                    elapsed > self.corner_no_wall_grace:
                self._corner_fallback_to_approach(
                    "%.1fs 内没拟合到前墙" % elapsed)
                self._corner_trace_close("fallback: no wall")
            return 0.0, 0.0, 0.0
        if self._corner_first_ok_t is None:
            self._corner_first_ok_t = time.time()

        xwall, ywall = fit["x_wall"], fit["y_wall"]
        partial = ywall is None                 # 只有前墙
        # 只用前墙的距离做"退回盲推"的判据。侧墙远(比如车道不贴墙)不是
        # 退回的理由 —— 前墙近就说明真到角落了, 按前墙闭环开进去就是。
        if xwall["distance"] > self.corner_fallback_dist:
            self.corner_trace("fallback: front wall too far", fit=fit)
            self._corner_fallback_to_approach(
                "前墙 %.2fm > %.2f" % (xwall["distance"],
                                     self.corner_fallback_dist))
            self._corner_trace_close()
            return 0.0, 0.0, 0.0

        ex = xwall["distance"] - self.corner_target_dist
        # 前墙(x) 0.25 / 侧墙(y, 车道那一侧的围挡) 0.21 —— 车道 42cm 车在中间
        ey = 0.0 if partial else ywall["distance"] - self.corner_target_side_dist
        # 航向误差 = 墙法向离最近车体轴还差多少(折到 ±45°, 两面墙平均, 由
        # corner_wall_fit 算好)。正 = 车要左转。**只对齐, 不挑轴**: 车斜着
        # 进场时转到最近的对齐位就行, 不管哪面墙最后算"前"。
        yaw_error = math.radians(float(fit["yaw_err_deg"]))
        yaw_limit = math.radians(self.corner_wall_angle_tol_deg)
        # 太斜(> align_max)就不转了, 直接滑进去; 终点只看位置
        skip_align = abs(yaw_error) > math.radians(self.corner_yaw_align_max_deg)
        # ---- 停滞检测 / 升级 ----
        # 进度量 = 离"全部满足"还差多少(各项超出容差的部分取最大)。3s 没有
        # 明显进展(< 3mm / 0.5°)就升级:
        #   第 1 级: 容差放宽一倍(位置 x2, 航向 x1.5)
        #   第 2 级: 直接进下一步 —— 还在对航向就不等了直接平移; 已经在
        #           平移就就地当作到位, 结束播报。
        # 实车 2026-08-18: 停在 x=0.267(要 <=0.26), cmd 只有 0.014m/s, 底盘
        # 平移死区吃掉了, 于是 1.7cm 永远差着 —— 有了下面的托底本不该再卡,
        # 这个升级是**第二道**保险(比如轮子打滑/雷达偏置 1cm 之类)。
        tol_p = self.corner_target_tol * (2.0 if self._corner_level >= 1 else 1.0)
        yaw_hold = math.radians(self.corner_yaw_hold_deg *
                                (1.5 if self._corner_level >= 1 else 1.0))
        yaw_locked = (abs(yaw_error) <= yaw_hold or self._corner_yaw_forced
                      or skip_align)
        # 平移放不放行: 默认边转边平移; corner_turn_first 时先转到位
        translate = yaw_locked or not self.corner_turn_first
        prog = max(abs(ex) - tol_p,
                   0.0 if partial else abs(ey) - tol_p,
                   0.0 if skip_align else
                   (abs(yaw_error) - yaw_hold) / math.radians(20.0) * 0.1,
                   0.0)                       # 航向按 20°≈10cm 折算
        if prog < self._corner_best_prog - 0.003:
            self._corner_best_prog = prog
            self._corner_prog_t = time.time()
        stalled = time.time() - self._corner_prog_t > self.corner_stall_s
        if stalled and prog > 0.0:
            self._corner_prog_t = time.time()
            self._corner_best_prog = prog
            if self._corner_level == 0:
                self._corner_level = 1
                rospy.logwarn("[终点雷达] %.0fs 没进展(还差 %.3f), 容差放宽一倍: "
                              "位置 ±%.0fmm 航向 ±%.1f°",
                              self.corner_stall_s, prog,
                              1000 * self.corner_target_tol * 2,
                              self.corner_yaw_hold_deg * 1.5)
                tol_p = self.corner_target_tol * 2.0
                yaw_hold = math.radians(self.corner_yaw_hold_deg * 1.5)
                yaw_locked = (abs(yaw_error) <= yaw_hold or
                              self._corner_yaw_forced or skip_align)
                translate = yaw_locked or not self.corner_turn_first
            elif not yaw_locked:
                self._corner_level = 2
                self._corner_yaw_forced = True
                yaw_locked = translate = True
                rospy.logwarn("[终点雷达] 放宽后又 %.0fs 没进展, 航向不等了"
                              "(还偏 %.1f°), 直接平移", self.corner_stall_s,
                              math.degrees(yaw_error))
            else:
                self._corner_level = 3
                self.corner_trace("give up: stalled", fit=fit)
                self._corner_trace_close()
                self.set_phase(
                    "STOPPED", "雷达角落: 放宽后仍不收敛(x=%.3f y=%s yaw=%.1f°), "
                    "就地结束" % (xwall["distance"],
                                 "-" if partial else "%.3f" % ywall["distance"],
                                 math.degrees(yaw_error)))
                return 0.0, 0.0, 0.0

        # ---- 平移: 沿每面墙**自己的法向**逼近, 不假设墙和车体轴平行 ----
        # v = kp*(ex*n_x + ey*n_y), n 是车体系下从车指向墙的单位法向。车斜着
        # 也对: 该往哪面墙靠就沿它的垂线走, 两面互相垂直互不干扰。
        vx = vy = 0.0
        if translate:
            if abs(ex) > tol_p:
                vx += self.corner_kp * ex * xwall["normal"][0]
                vy += self.corner_kp * ex * xwall["normal"][1]
            if not partial and abs(ey) > tol_p:
                vx += self.corner_kp * ey * ywall["normal"][0]
                vy += self.corner_kp * ey * ywall["normal"][1]
            spd = math.hypot(vx, vy)
            if spd > self.corner_max_speed:
                vx, vy = (vx * self.corner_max_speed / spd,
                          vy * self.corner_max_speed / spd)
            # ⚠ 平移死区托底: 底盘 |v| < move_min_speed(0.06) 基本不动。实车
            #   2026-08-18: 差 1.7cm 时 P 只给 0.014m/s, 车纹丝不动, 稳定计数
            #   永远 0/5。还没进容差就至少给 move_min(按合速度算, 方向不变);
            #   进了容差上面就没加, 是 0(别来回蹭)。
            #   12Hz 雷达一拍 0.06*0.083 = 5mm, 容差 ±10mm, 不会过冲振荡。
            elif 0.0 < spd < self.move_min:
                vx, vy = (vx * self.move_min / spd, vy * self.move_min / spd)
        wz = self._corner_clip(self.corner_yaw_kp * yaw_error,
                               self.corner_max_yaw_speed)
        # ⚠ 转向死区托底: 底盘 |wz| < az_min(≈0.12) 电机根本不转。实车
        #   2026-08-18: yaw 从 -21.8° 收到 -5.6° 后卡死 —— 差 0.6° 锁不上,
        #   而 P 给的 0.078 rad/s 电机吃掉了, 于是永远锁不上、永远不平移。
        #   还没锁定就至少给 az_min; 锁定之后小抖动就别管了(给 0)。
        if not yaw_locked and 0.0 < abs(wz) < self.az_min:
            wz = math.copysign(self.az_min, wz)
        elif yaw_locked:
            wz = 0.0
        # 只有前墙时侧向没有约束: 前墙到位就算到位(横向由前面的巡线保证,
        # 车本来就在车道里)。停下来时日志里会写明"侧墙未见"。
        in_target = (abs(ex) <= tol_p and
                     (partial or abs(ey) <= tol_p) and
                     (abs(yaw_error) <= yaw_limit or self._corner_yaw_forced
                      or skip_align))
        self.corner_stable = self.corner_stable + 1 if in_target else 0
        if time.time() - self._corner_log_t > 0.5:
            self._corner_log_t = time.time()
            rospy.loginfo("[终点雷达] %s yaw=%.1f° 距离 x=%.3f y=%s "
                          "误差=(%+.3f,%+.3f) cmd=(%+.3f,%+.3f,%+.3f) "
                          "稳定=%d/%d",
                          fit["why"], math.degrees(yaw_error),
                          xwall["distance"],
                          "-" if partial else "%.3f" % ywall["distance"],
                          ex, ey, vx, vy, wz, self.corner_stable,
                          self.corner_stable_frames)
        self.corner_trace("step", fit=fit, cmd=(vx, vy, wz), extra={
            "ex": round(ex, 4), "ey": round(ey, 4),
            "yaw_err_deg": round(math.degrees(yaw_error), 2),
            "yaw_locked": bool(yaw_locked), "in_target": bool(in_target),
            "skip_align": bool(skip_align),
            "level": self._corner_level, "tol": round(tol_p, 4),
            "prog": round(prog, 4)})
        if self.corner_stable >= self.corner_stable_frames:
            self._corner_trace_close("stopped")
            self.set_phase(
                "STOPPED", "雷达角落停车稳定: x=%.3fm y=%s yaw=%.1f° "
                "连续%d帧%s" % (
                    xwall["distance"],
                    "-(侧墙未见, 仅按前墙)" if partial
                    else "%.3fm" % ywall["distance"],
                    math.degrees(yaw_error), self.corner_stable_frames,
                    "" if partial else "(名义朝向 %.0f°)"
                    % fit["nominal_yaw_deg"]))
            return 0.0, 0.0, 0.0
        return vx, vy, wz

    def estop_cb(self, msg):
        if msg.data:
            rospy.logwarn("收到 /lane_proto/estop -> 立即停车")
            self.kill_yolo()
            self.set_phase("STOPPED", "远程急停")

    def set_phase(self, ph, why=""):
        self.phase = ph
        self.state_pub.publish(String(data=ph))
        if ph == "STOPPED":
            if "拦路板" in why:
                res = "BOARD"
            elif "急停" in why:
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
                rospy.logwarn("任务完成播报失败: %s", _s(exc))

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
        self._corner_trace_close("shutdown")
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
        if hit and self._goal_why:
            # 图上直接写死"为什么没停", 省得再去翻日志对时间戳
            note += "  |没停: %s" % self._goal_why
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
        if self.phase == "CORNER_ADJUST" and self._corner_last_fit:
            fit = self._corner_last_fit
            if fit.get("ok"):
                note += "  corner x=%.2f y=%.2f stable=%d/%d" % (
                    fit["x_wall"]["distance"], fit["y_wall"]["distance"],
                    self.corner_stable, self.corner_stable_frames)
            else:
                note += "  corner %s" % fit.get("why", "invalid")
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
        """喂给认灯的那一帧(见 yolo_crop / yolo_use_raw / yolo_zoom)。
        都是**已翻正**的。"""
        crop = self.crop_frame()
        if crop is not None:
            return crop
        if self.yolo_use_raw and self._raw is not None:
            im = cv2.flip(self._raw, 1) if self.mirror else self._raw
        else:
            im = self._und
        if im is None or self.yolo_zoom <= 1.001:
            return im
        return self.zoom_center(im, self.yolo_zoom, self.yolo_zoom_cy)

    def restore_cam_res(self):
        """认完把相机切回标定分辨率。**必须切回来** —— 巡线的去畸变映射
        是按 640x480 标的, 高分辨率的帧经 to_4x3 裁缩之后视场对不上。"""
        if self._cam_switched:
            self._cam_switched = False
            self.set_cam_res(self.W, self.H, "认灯完")

    def _new_cap(self, w, h, tries=3):
        """按 (w,h) 重新打开相机。返回 cap 或 None。

        ⚠ **必须重开, 不能 cap.set**。V4L2 不允许在 streaming 中换分辨率:
          实车 2026-08-18 试过 cap.set, 驱动只是把参数记下来, 原来那批
          mmap buffer 立刻作废(VIDIOC_DQBUF/QBUF: Invalid argument), 而
          cap.get 还照样回报 1920x1080 **说成功了**。
          所以这里认的不是 cap.get, 而是**真读到一帧、且尺寸对得上**。
        ⚠ 刚 release 完设备节点未必立刻空出来, 所以开失败会隔 150ms 再试,
          一共 tries 次。
        """
        fcc = getattr(self, "_fourcc", "MJPG")
        for k in range(max(1, tries)):
            cap = None
            try:
                # ⚠ 只走 V4L2, **不要**退回不带后端的 VideoCapture(dev):
                #   这台 nvidia 版 OpenCV 会把设备路径当 GStreamer 文件管线
                #   去开, 对 /dev/video* 永远 "unable to start pipeline",
                #   而且那个半死的对象析构时 gst_adapter_push 断言 -> SIGSEGV
                #   (实车 2026-08-18 两趟进程都是这么 -11 死的)。
                try:
                    cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
                    if not cap.isOpened():
                        cap.release()
                        cap = None
                except Exception:
                    cap = None
                if cap is not None and cap.isOpened():
                    if fcc and fcc != "NONE" and len(fcc) == 4:
                        cap.set(cv2.CAP_PROP_FOURCC,
                                cv2.VideoWriter_fourcc(*fcc))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                    if getattr(self, "_cam_fps_req", 0.0) > 0:
                        cap.set(cv2.CAP_PROP_FPS, self._cam_fps_req)
                    try:
                        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    except Exception:
                        pass
                    # 真读几帧: 头一两帧常常是空的/上一档尺寸的
                    for _ in range(8):
                        ok, f = cap.read()
                        if (ok and f is not None and f.shape[1] == w
                                and f.shape[0] == h):
                            return cap
                        time.sleep(0.02)
                    rospy.logwarn("重开相机到 %dx%d(第 %d 次): 读不到对尺寸"
                                  "的帧", w, h, k + 1)
                else:
                    rospy.logwarn("重开相机到 %dx%d(第 %d 次): 打不开 %s",
                                  w, h, k + 1, self.device)
            except Exception as e:
                rospy.logwarn("重开相机到 %dx%d(第 %d 次)出错: %s",
                              w, h, k + 1, _s(e))
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
            time.sleep(0.15)
        return None

    def _set_cam_res_locked(self, w, h, why):
        """set_cam_res 的正体, 调用方必须已经拿着 self._cam_lock。"""
        if self.cap is None or self.grab is None:
            rospy.logwarn("%s: 用的是 ROS 相机话题, 改不了分辨率 —— "
                          "退回整帧缩放(和以前一样)", _s(why))
            return False
        cur = self.cam_now
        if cur == (w, h):
            return True
        t0 = time.time()
        old_grab, old_cap = self.grab, self.cap
        # ---- 第一步, 也是最要紧的一步: 等取帧线程**真的退出**再动 cap ----
        if not old_grab.stop(join=1.0):
            # 1s 都没从 cap.read() 出来 = 相机本身卡了。这时候强行 release
            # 就是实车 2026-08-18 那一套(线程炸/流停不下来/开不回来/SIGSEGV)。
            # 宁可不切: 撤销 stop 让它继续跑, 原样返回。
            old_grab.stopped = False
            if not old_grab.is_alive():          # 刚好在这缝里退出了
                self.grab = FrameGrabber(old_cap)
                self.grab.start()
            rospy.logwarn("%s: 取帧线程 1s 内没退出(cap.read 卡住?), 放弃切"
                          "换, 相机保持 %dx%d", _s(why), cur[0], cur[1])
            return False
        try:
            old_cap.release()
        except Exception as e:
            rospy.logwarn("%s: release 旧相机出错: %s", _s(why), _s(e))
        del old_cap
        cap = self._new_cap(w, h)
        ok = cap is not None
        if not ok:
            cap = self._new_cap(cur[0], cur[1])     # 退回原分辨率
            if cap is None:
                rospy.logerr("%s: 相机切换失败且**开不回原分辨率**了 —— "
                             "这趟没法继续了", _s(why))
                self.cap = None
                return False
        self.cap = cap
        self.cam_now = (w, h) if ok else cur
        self.grab = FrameGrabber(self.cap)
        self.grab.start()
        (rospy.loginfo if ok else rospy.logwarn)(
            "%s: 相机 %dx%d -> %dx%d %s (%.0fms)", _s(why), cur[0], cur[1],
            w, h, "成功" if ok else "失败, 退回 %dx%d" % cur,
            1000.0 * (time.time() - t0))
        return ok

    def set_cam_res(self, w, h, why):
        """临时改采集分辨率(release + 重开)。返回 True = 真的换成了。

        为什么"临时": 巡线全程用 640x480(鱼眼标定就是在这个模式上做的),
        只有认灯那一小段切到 1920x1080 —— 从整帧里原生裁一块 640x352 喂
        yolo, 灯从 28px 变 84px。认完立刻切回去, 去畸变映射一秒都没被
        高分辨率的帧碰过。

        加锁: 预切线程(挪 140mm 时)、start_yolo(主线程)、认灯收尾线程都会
        调它, 两个同时换 cap 就又是一场事故。锁里是幂等的 —— 已经是目标
        尺寸就直接返回 True。
        """
        with self._cam_lock:
            return self._set_cam_res_locked(w, h, why)

    def _preswitch_cam(self):
        """在盲走那 140mm 的时候提前把相机切到高分辨率, 到位就能直接认灯。
        切换本身几百毫秒到一秒, 藏在盲走段里就不占"绿灯->发车"的时间。"""
        with self._cam_lock:
            ok = self._set_cam_res_locked(self.yolo_cam_w, self.yolo_cam_h,
                                          "认灯(预切)")
            self._preswitch_result = ok

    def crop_frame(self):
        """从**整帧**里按原始像素抠一块 net 大小的窗口。不适用就返回 None。

        ⚠ 先翻正再裁: 相机原始输出是镜像的, 而 yolo_crop_cx 是在**翻正后**
          (也就是 dump 图里你看到的那个朝向)量的。反过来做的话中心会镜像
          到另一边去 —— 0.47 变成 0.53, 在 1920 宽上差 115px, 灯就出框了。
        ⚠ 认了 yolo_crop_timeout 秒还没定案就放弃裁剪, 改喂整帧(缩到 net
          大小 = 原来的行为)。灯要是压根没落在窗口里, 裁剪就是在帮倒忙,
          得有条退路。
        """
        if not self.yolo_crop or self._full is None:
            return None
        # 只有**真的切到高分辨率了**才裁。切换失败时整帧还是 640x480,
        # 那时候裁 640x352 等于把画面上下切掉一块(宽度一样, 高度变小),
        # 灯反而更容易被切没 —— 该老老实实退回整帧缩放。
        if not self._cam_switched:
            return None
        if self._crop_gave_up:
            return None
        if (self.yolo_crop_timeout > 0 and self.yolo_t0 and
                time.time() - self.yolo_t0 > self.yolo_crop_timeout):
            self._crop_gave_up = True
            rospy.logwarn("认灯: 裁窗口认了 %.0fs 还没定案 -> 改喂整帧"
                          "(灯可能不在窗口里, 调 yolo_crop_cx/cy)",
                          self.yolo_crop_timeout)
            return None
        im = self._full
        h, w = im.shape[:2]
        cw, ch = min(self.yolo_crop_w, w), min(self.yolo_crop_h, h)
        if cw >= w and ch >= h:
            return None                  # 整帧本来就没比窗口大, 裁了没意义
        if self.mirror:
            im = cv2.flip(im, 1)
        x0 = int(round(self.yolo_crop_cx * w - cw / 2.0))
        y0 = int(round(self.yolo_crop_cy * h - ch / 2.0))
        x0 = max(0, min(w - cw, x0))
        y0 = max(0, min(h - ch, y0))
        out = im[y0:y0 + ch, x0:x0 + cw]
        if self._crop_log is None:
            self._crop_log = (w, h, x0, y0, cw, ch)
            rospy.loginfo("认灯: 整帧 %dx%d -> 原生裁 %dx%d @(%d,%d) "
                          "(中心 %.3f,%.3f); 等效变焦 %.2fx",
                          w, h, cw, ch, x0, y0, self.yolo_crop_cx,
                          self.yolo_crop_cy, float(w) / cw)
            if self.dump_dir:
                try:
                    cv2.imwrite(os.path.join(self.dump_dir, "yolo_crop.jpg"),
                                out)
                    rospy.loginfo("认灯窗口存了一张 -> %s/yolo_crop.jpg "
                                  "(灯不在里面就调 yolo_crop_cx/cy)",
                                  self.dump_dir)
                except Exception as e:
                    rospy.logwarn("存认灯窗口失败: %s", _s(e))
        return out

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
        # 认灯要的是**没被 to_4x3 缩过**的整帧: 相机开到 1920x1080 时,
        # to_4x3 会把它裁成 4:3 再缩到 640x480, 灯的像素在这一步就没了。
        # 巡线那条路完全不变, 还是吃 to_4x3 之后的 640x480。
        self._full = frame
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
        # HIT 了却停不下来的原因, 现在就算好写进 dump 图 —— 相位机在后面,
        # 等它算完再画就晚一帧了。**只管终点那一支**: Y 岔口那条横线本来
        # 就不设闸, 报"没触发"纯属噪声。
        if hit and not (self.is_fork and not self.fork_done) and \
                self.phase not in ("CORNER_ADJUST", "PAUSE", "APPROACH",
                                   "STOPPED"):
            self._goal_why = self.goal_gate(hit)[1]
        elif not hit:
            self._goal_why = None
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

    def ensure_segmentation_model(self):
        """真正要用分割网之前才把 .so 拉起来(懒加载)。
        幂等: TrackSeg 内部 ts_init() 有 g_ready 守卫, 重复调也只加载一次。"""
        if self.seg is not None:
            return
        rospy.loginfo("lane_follow: 加载 TrackSeg ...")
        self.seg = TrackSeg(self.trackseg_lib)
        # ⚠ 打日志**绝不能**把加载搞失败。实车上出过一次: .so 明明已经
        # 加载成功, 却因为这行日志里 backend 是 unicode、模板里"翻转"是
        # utf-8 字节串, py2 的 % 格式化按 ASCII 去解中文 -> UnicodeDecodeError
        # -> 被外层 except 抓成"分割网加载不了" -> 主流程直接中止任务。
        try:
            rospy.loginfo("lane_follow: backend=%s dry_run=%s v=%.2f mirror=%s",
                          _s(self.seg.backend), self.dry_run, self.v,
                          "ON(翻转)" if self.mirror else "OFF")
        except Exception as e:
            rospy.logwarn("backend 日志打印失败(不影响加载): %r", e)

    def run(self):
        if self.enabled:
            self.ensure_segmentation_model()    # 独立跑: 进循环前就加载, 和以前行为一致
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
        # ⚠ 必须是实例属性: 起跑序列真正切进 FOLLOW 的地方(branch_go)
        # 要能把它清掉。self.phase 初值就是 "FOLLOW"(见 __init__), 所以
        # 第一帧就会走到下面的板子分支并把锚点设在**节点刚起来那一刻**;
        # 而真正的 FOLLOW 要等对齐+挪位+认灯结束, 中间隔了十几秒。
        self.t_follow0 = [None, None, None]
        t_prev = None            # 上一帧开始的时刻(算循环周期用)
        t_stats = time.time()
        while not rospy.is_shutdown():
            if not self.enabled:          # STANDBY: 不取帧不算不发速度
                t_prev = None
                rate.sleep()
                continue
            if t_start is None:           # 刚被交接: 所有计时从这一刻重新起算
                t_start = t0 = t_stats = time.time()
                self.t_follow0 = [None, None, None]
                self.yaw_entry = None       # 交接后重新锁进场航向
                last_seq, stale, i = -1, 0, 0
                self.prof.reset()
            # ---- 没有 odom 就不许动 ----
            # 以前只有"断流"这一道闸, 而且带 odom_recv_t > 0.0 的前提 ——
            # odom **从来没来过**的情况反而一路放行, 全程速度x时间瞎估。
            # 这里补上"一次都没收到"那半边: 停着等, 等不到就停机。
            if self.require_odom and not self.odom_seen \
                    and self.phase != "STOPPED":
                try:                      # 停着等的时候必须持续发零速:
                    self.pub.publish(Twist())   # 底盘 cmd_timeout 只有 0.2s,
                except Exception:               # 不发它会一直等新指令
                    pass
                if self._odom_wait_t0 is None:
                    self._odom_wait_t0 = time.time()
                waited = time.time() - self._odom_wait_t0
                if self.odom_wait_max > 0 and waited > self.odom_wait_max:
                    self.set_phase("STOPPED",
                                   "等了 %.0fs 一帧 /odom 都没收到, 拒绝盲跑"
                                   % waited)
                    rospy.logerr("话题 %s 没有数据。base_driver 起了吗? "
                                 "rostopic hz %s 看一下。确实要在没有里程计"
                                 "的情况下跑就传 require_odom:=false",
                                 self.odom_topic, self.odom_topic)
                elif time.time() - self._odom_wait_logged > 1.0:
                    self._odom_wait_logged = time.time()
                    rospy.logwarn("等 %s ... 已等 %.1f/%.0fs (没有里程计不发"
                                  "速度)", self.odom_topic, waited,
                                  self.odom_wait_max)
                t_prev = None
                rate.sleep()
                continue
            # odom 断流保护: 底盘要是中途掉了, 巡线还在发速度就是瞎开。
            # 只在真的在动的时候判, 且只触发一次。
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
            # 进场航向: 起跑序列开始前(还没做任何转向)锁一次。两臂要转
            # ±60°、直行那条要转 45°, 都是从这个基准算偏航。
            if self.yaw_entry is None and self.yaw_unw is not None \
                    and self.phase in ("ALIGN", "START_MOVE", "YOLO"):
                self.yaw_entry = self.yaw_unw
                rospy.loginfo("锁定进场航向 %.1f°",
                              math.degrees(self.yaw_entry))
            # ---- 拦路板 ----
            # 三道闸串起来: 相位(只在 FOLLOW) -> 航向(转过 arm_deg 才开,
            # 专治 Y 岔口那个红绿灯箱体) -> 里程(离开起点 + 可选的窗口)。
            if self.board_on and self.phase == "FOLLOW" \
                    and self.board_state == "未知":
                self.board_early_verdict(self.board_travel(self.t_follow0))
            # ⚠ "正看着板子"的时候绝不能判无板。实车 2026-08-16: 板子在
            # 0.47m 处每一帧都检出("-> 板子"), 只是还没近到 board_stop
            # 不够触发绕障; 里程恰好在这时走到阈值, 于是判"本趟无板"、
            # 关掉板子检测, 然后直接撞上去。里程闸只该用来关掉"一路没见过
            # 板子"的情况。
            seen_recently = (self.board_seen_t > 0.0 and
                             time.time() - self.board_seen_t < 2.0)
            if self.board_on and self.phase == "FOLLOW" \
                    and self.board_state == "未知" and not seen_recently \
                    and self.board_travel(self.t_follow0) >= self.board_clear_now():
                self.board_state = "确认无板"
                rospy.loginfo("=" * 56)
                rospy.loginfo("已走 %.2fm 仍未检出拦路板(板子最晚出现在 "
                              "~1.8m), 判定本趟无板 —— 切回初赛逻辑: "
                              "关闭板子检测, 放开终点白线检测",
                              self.board_travel(self.t_follow0))
                rospy.loginfo("=" * 56)
            if self.board_on and self.phase == "FOLLOW" \
                    and self.board_state == "未知":
                trav = self.board_travel(self.t_follow0)
                self._board_trav = trav          # 给倒计时日志用
                armed, why_arm = self.board_armed()
                far = trav >= self.board_min_travel
                inwin = (not self.board_win) or any(
                    lo <= trav <= hi for lo, hi in self.board_win)
                if armed and far and inwin \
                        and time.time() >= self.board_cool_until:
                    if self.check_board():
                        if self.go_around and self.ga_n < self.ga_max:
                            self.start_go_around()
                        else:
                            why = "雷达检出拦路板, 停车"
                            if self.go_around:
                                why += "(已绕 %d 次, 到上限 %d)" % (
                                    self.ga_n, self.ga_max)
                            self.set_phase("STOPPED", why)
                elif time.time() - self._board_log_t > 2.0:
                    self._board_log_t = time.time()
                    self.board_hits = 0
                    rospy.loginfo("[拦路板] 未开闸: %s%s%s (从%s起已走 %.2fm, "
                                  "再走 %.2fm 没扫到就判定无板)",
                                  "" if armed else "航向未到(%s) " % why_arm,
                                  "" if far else "里程不足 ",
                                  "" if inwin else "不在里程窗口 ",
                                  self.board_anchor, trav,
                                  max(0.0, self.board_clear_now() - trav))
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
                # 盲走那几个相位(odom/雷达驱动, 压根不看图)在断帧时**继续
                # 发上一条指令** —— 底盘 cmd_timeout 只有 0.2s, 断帧就不发
                # 的话车会被刹停。认灯完切回 640x480 时取帧线程要暂停一下,
                # 正好撞在冲线途中(实车会看到车走一半顿一下)。
                # 只兜 cam_gap_hold 秒, 再久就是真掉相机了, 该停。
                if (self._last_tw is not None
                        and self.phase in BLIND_PHASES
                        and stale <= self.rate_hz * self.cam_gap_hold):
                    if not self.dry_run:
                        self.pub.publish(self._last_tw)
                    if stale == 1:
                        rospy.loginfo("断帧但在 %s(盲走), 继续发上一条指令 "
                                      "免得底盘 0.2s 超时刹车", self.phase)
                rate.sleep()
                continue
            stale = 0
            last_seq = seq
            self.prof.add("取帧", (time.time() - t_grab) * 1000.0)
            period = None if t_prev is None else (t_grab - t_prev) * 1000.0
            if t_prev is not None:
                # 指令速度积分: 没有 /odom 时的测距兜底。用 speed_now()
                # 而不是 self.v —— 等灯/刹停/原地转的相位它返回 0, 车站着
                # 就不该累里程(这次实车 8.2s 认灯白算了 2.05m)。
                self.cmd_s += self.speed_now() * min(0.5, t_grab - t_prev)
            t_prev = t_grab

            if i % int(max(1, self.rate_hz)) == 0:
                self.reload_params()          # 每秒重读一次可调参数
            fork_cmd = move_cmd = None
            avoid_cmd = None
            corner_cmd = None
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
            # 里程闸: 路程还没走够就不认终点横线。板子的白面在分割网眼里
            # 和终点白胶带是一回事, 而板子在路程一半(两臂 2.21m / Y 支路
            # 离岔口 1.08m), 终点在 3.5~3.8m —— 用里程就能把两者分开。
            # ⚠ 这道闸**只管终点线, 不管 Y 岔口那条横线**。岔口横线出现在
            # 约 1.4m 处, 远早于任何阈值; 拿终点的里程闸去卡它, goal_hits
            # 永远攒不满 3 帧, 车就直接开过岔口不转弯了(实车 dump 里
            # fork:pending 一路 HIT 却没反应, 就是我这里漏掉的)。
            # 起点到 Y 岔口之间本来也不会有板子, 不需要防。
            # 四道闸集中在 goal_gate() 里, dump 图和这里用的是同一份判断
            goal_ok, why = self.goal_gate(hit)
            # ⚠ 顺序: 先按**这一帧**的结果打日志, 再累加票数。反过来的话
            #   "才连上 N 帧"里的 N 会多算一帧。
            fork_line = self.is_fork and not self.fork_done
            # 已经在处理终点(CORNER_ADJUST/PAUSE/APPROACH/STOPPED)时白线当然
            # 还一直 HIT, "为什么没停"这个问题不成立, 别刷屏
            in_goal_phase = self.phase in ("CORNER_ADJUST", "PAUSE",
                                           "APPROACH", "STOPPED")
            if hit and why and not fork_line and not in_goal_phase:
                # 只有原因变了、或者隔了 0.5s 才打, 免得 19fps 刷屏;
                # 但**原因一变必打**, 不会漏掉状态切换那一下。
                if (why != self._goal_why_last
                        or time.time() - self._goal_gate_t > 0.5):
                    self._goal_gate_t = time.time()
                    self._goal_why_last = why
                    rospy.loginfo("[终点] t=%.3f 画了橙线(覆盖%.2f 最佳线%.2f "
                                  "%d条 y=%d)但没停车 -> %s",
                                  time.time(), cov, lbest, lcnt, gy, why)
            elif not hit:
                self._goal_why_last = ""
            self.goal_hits = (self.goal_hits + 1) if (hit and goal_ok) else 0
            confirmed = self.goal_hits >= self.goal_confirm
            # 地图定位判终点(goal_mode=both): 离 goal_map_xy 不到 goal_map_dist
            # 就和白线一样触发(OR)。只在终点那一支(岔口横线不管), 冷却期内不
            # 触发; 没 /robot_pose 就永远 False, 等于只看白线。
            map_hit = False
            if self.phase == "FOLLOW" and self.goal_mode == "both" \
                    and self.goal_map_xy is not None:
                map_hit, md, mtxt = self.map_goal()
                fork_branch_now = self.is_fork and not self.fork_done
                if fork_branch_now or time.time() < self.cool_until:
                    map_hit = False
                # 有定位每秒一行; 没 topic/过期只是每 30s 提一句(静默忽略)
                if mtxt and time.time() - self._map_log_t > (
                        1.0 if md >= 0 else 30.0):
                    self._map_log_t = time.time()
                    rospy.loginfo("[地图终点] %s -> %s%s", mtxt,
                                  "触发" if map_hit else "未到(阈值 %.2f)"
                                  % self.goal_map_dist,
                                  " (岔口支路, 不算)" if fork_branch_now else "")
                if map_hit and not confirmed:
                    self._goal_why = ""
                    self._map_why = "地图定位 %s <= %.2f" % (mtxt, self.goal_map_dist)
                    rospy.loginfo("[终点] t=%.3f %s, 白线%s -> 按终点处理",
                                  time.time(), self._map_why,
                                  "也检出" if hit else "没检出")
            # 终点横线的雷达复核。⚠ 只复核**终点**那一支: 起点到 Y 岔口
            # 之间不会有板子, 所以 Y 岔口那条横线照原样走, 不受影响。
            fork_branch = self.is_fork and not self.fork_done
            if (self.phase == "FOLLOW" and confirmed and not fork_branch
                    and time.time() >= self.cool_until):
                veto, vd = self.goal_board_veto()
                if veto and map_hit:
                    rospy.logwarn("[终点] 雷达 %.2fm 处有块实体, 但地图定位说已"
                                  "到终点(%s), 按终点算", vd, self._map_why)
                    veto = False
                if veto:
                    self.goal_hits = 0
                    confirmed = False
                    # 第四道闸的结果也塞进去, 下一帧的 dump 图就能写出来
                    self._goal_why = ("雷达否决: %.2fm 处是一块有限宽的实体"
                                      "(拦路板), 不是终点线" % vd)
                    if time.time() - self._veto_log_t > 1.0:
                        self._veto_log_t = time.time()
                        rospy.logwarn("[终点] t=%.3f 视觉判到终点横线(覆盖"
                                      "%.2f), 但雷达在 %.2fm 处看到一块有限宽"
                                      "的实体 —— 那是拦路板不是终点线, 不停车",
                                      time.time(), cov, vd)
            if map_hit and not fork_branch:
                confirmed = True                 # 地图定位 OR 白线
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
                    if map_hit:
                        rospy.loginfo("[终点] 触发源: %s%s", self._map_why,
                                      " + 白线" if self.goal_hits >=
                                      self.goal_confirm else "(白线未检出)")
                    self.start_corner_adjust(gy)
                elif self.goal_pause > 0:
                    self.pause_until = time.time() + self.goal_pause
                    self.set_phase("PAUSE", "检出终点框(%s覆盖%.2f 最佳线%.2f "
                                   "%d条, 线在第%d行), 先刹停 %.1fs 打点"
                                   % (self._map_why + "; " if map_hit else "",
                                      cov, lbest, lcnt, gy, self.goal_pause))
                else:
                    self.start_approach(gy)
            elif self.phase == "PAUSE":
                if time.time() >= self.pause_until:
                    if self._pause_next == "corner_fallback":
                        self.start_approach(self._corner_goal_y)
                    elif self._pause_next == "fork":
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
                    # 只有走中间那条才会经过 FORK_TURN(两臂是在 apply_branch
                    # 里直接置 fork_done, 不进这个相位), 所以在这里换模板
                    # 正好就是"Y 字分叉之后"。
                    if self.dec_y is not None:
                        self.dec = self.dec_y
                        rospy.loginfo("Y 支路: 触发区模板换成 %s",
                                      _s(os.path.basename(self.tpl_y_path)))
                    t_follow0 = [None, None]      # 里程从岔口重新起算
                    self.board_anchor = "岔口"
                    self._early_done = False
                    self._early_hits = 0
                    self._pause_next = "approach"
                    self.cool_until = time.time() + self.fork_cooldown
                    self.set_phase("FOLLOW", "继续巡线(%.1fs 内不认横线, "
                                   "免得分叉口那条线被当成终点)"
                                   % self.fork_cooldown)
            elif self.phase in ("AVOID_REV", "AVOID_OUT", "AVOID_FWD", "AVOID_BACK",
                           "AVOID_ALIGN", "AVOID_TURN0", "AVOID_POSE"):
                avoid_cmd = self.step_go_around()
            elif self.phase == "CORNER_ADJUST":
                corner_cmd = self.step_corner_adjust()
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
            elif self.phase in ("AVOID_REV", "AVOID_OUT", "AVOID_FWD", "AVOID_BACK",
                           "AVOID_ALIGN", "AVOID_TURN0", "AVOID_POSE"):
                # 麦轮横移: 开环, 不转车头(转了之后横回来对不上原航向),
                # 也不看视觉 —— 这几秒车根本不在车道上。
                vx, vy = avoid_cmd or (0.0, 0.0)
                # 前四段横平竖直不转头; 最后 AVOID_ALIGN 段只转不走,
                # 用板子法向把航向归位(step_go_around 里算好放在 _ga_az)。
                tw.linear.x, tw.linear.y = vx, vy
                tw.angular.z = getattr(self, "_ga_az", 0.0)
            elif self.phase == "CORNER_ADJUST":
                vx, vy, wz = corner_cmd or (0.0, 0.0, 0.0)
                tw.linear.x, tw.linear.y, tw.angular.z = vx, vy, wz
            else:                         # FOLLOW / APPROACH 都照常巡线
                tw.angular.z = az
                tw.linear.x = self.v      # 0 就是原地转, 不用改代码
            # 攒最近约 2 秒的转向指令, 给绕障方向决策做交叉验证
            if self.phase == "FOLLOW":
                self.az_hist.append(az)
                if len(self.az_hist) > int(max(4, 2 * self.rate_hz)):
                    self.az_hist.pop(0)

            # 连续 5 帧(0.5s)落在死区内才算对准, 防抖
            hold = hold + 1 if (az == 0.0 and self.phase == "FOLLOW") else 0
            now_aligned = hold >= 5
            if now_aligned and not aligned:
                rospy.loginfo("✓ 已对准跑道 (IL=%.2f IR=%.2f)%s", IL, IR,
                              ", 停住" if self.v == 0.0 else "")
            elif aligned and not now_aligned:
                rospy.loginfo("偏了 (IL=%.2f IR=%.2f), 开始修正", IL, IR)
            aligned = now_aligned
            self._last_tw = tw
            if not self.dry_run:
                self.pub.publish(tw)

            # 认灯的收尾(杀子进程 + 相机切回 640x480)拖到**指令发出之后**:
            # 这两件事都可能阻塞几百 ms 到 2s, 而车这会儿已经在冲线了,
            # 阻塞落在这里只是晚几拍收油, 落在发车之前就是白等。
            if self._yolo_cleanup_pending:
                self._yolo_cleanup_pending = False
                # 丢到后台线程去做: 这两件事最坏能卡 2s(close 的 grace)+
                # 几百 ms(cap.set), 卡在控制环里就是 2 秒不发 cmd_vel,
                # 底盘 cmd_timeout 只有 0.2s, 车当场被刹停在冲线半路。
                threading.Thread(target=self._yolo_cleanup).start()

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
