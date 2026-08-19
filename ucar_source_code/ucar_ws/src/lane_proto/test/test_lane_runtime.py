#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Regression checks for the interpreter used by lane_follow."""

from __future__ import print_function

import os
import sys
import io
import unittest
import xml.etree.ElementTree as ET

# lane_follow.py and the V29 trace fixtures are UTF-8; keep text reads
# independent of the host locale (Windows defaults to GBK here).
def open(path, mode="r", *args, **kwargs):
    if "b" not in mode and "encoding" not in kwargs:
        kwargs["encoding"] = "utf-8"
    return io.open(path, mode, *args, **kwargs)

try:
    import imp
except ImportError:
    imp = None


PACKAGE_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), os.pardir))


class LaneRuntimeTest(unittest.TestCase):

    @staticmethod
    def _load_lane_follow():
        script = os.path.join(PACKAGE_ROOT, "scripts", "lane_follow.py")
        if imp is not None:
            return imp.load_source("lane_follow_runtime_test", script)
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            "lane_follow_runtime_test", script)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_lane_follow_uses_melodic_python2(self):
        script = os.path.join(PACKAGE_ROOT, "scripts", "lane_follow.py")
        with open(script, "rb") as handle:
            self.assertEqual(
                handle.readline().strip(),
                b"#!/usr/bin/env python2")

    def test_launch_uses_the_melodic_python2_runner(self):
        launch = os.path.join(PACKAGE_ROOT, "launch", "lane_proto.launch")
        with open(launch, "rb") as handle:
            content = handle.read()
        self.assertIn(
            b'launch-prefix="$(find ucar_2026)/scripts/run_melodic_python2.sh"',
            content)

    def test_lane_exit_does_not_shutdown_parent_launch(self):
        launch = os.path.join(PACKAGE_ROOT, "launch", "lane_proto.launch")
        root = ET.parse(launch).getroot()
        node = root.find("node[@name='lane_follow']")
        self.assertIsNotNone(node)
        self.assertEqual(node.get("required"), "false")

    def test_resident_handoff_keeps_start_sequence_parameters(self):
        """2026.launch 常驻交接必须传起跑序列参数。

        2026-08-15 现场回归：交接 include 漏传 is_fork:=yolo，FOLLOW 相位
        把交接点画面的黄线当终点横线，交接后 2.8 s 即 APPROACH/STOPPED，
        整段巡线被跳过。此处锁住参数，防止再次遗漏。
        """
        ucar_2026_launch = os.path.join(
            PACKAGE_ROOT, os.pardir, "ucar_2026", "launch", "2026.launch")
        root = ET.parse(ucar_2026_launch).getroot()
        lane_includes = [
            el for el in root.findall(".//include")
            if el.get("file", "").endswith("lane_proto.launch")]
        self.assertEqual(len(lane_includes), 1, "2026.launch 应有且仅有一个 "
                         "lane_proto 常驻 include")
        args = {arg.get("name"): arg.get("value", "")
                for arg in lane_includes[0].findall("arg")}
        self.assertEqual(args.get("is_fork"), "yolo")
        self.assertIn("red_template_band.png", args.get("template", ""))
        self.assertEqual(args.get("start_enabled"), "false")
        self.assertEqual(args.get("use_lidar"), "self")
        self.assertEqual(args.get("goal_mode"), "visual")
        self.assertEqual(args.get("board_in_lane"), "false")
        self.assertEqual(args.get("go_around"), "false")
        self.assertEqual(args.get("goal_y_lo"), "0.75")
        self.assertEqual(args.get("dump_every"), "5")
        for name in ("align_offset", "start_offset", "goal_y_lo",
                     "yellow_target", "linear_speed", "gain", "rate"):
            self.assertIn(name, args, "交接 include 缺少参数 %s" % name)

    def test_national_handoff_uses_latest_board_avoidance_parameters(self):
        national_launch = os.path.join(
            PACKAGE_ROOT, os.pardir, "ucar_2026_national", "launch",
            "2026.launch")
        root = ET.parse(national_launch).getroot()
        lane_includes = [
            el for el in root.findall(".//include")
            if el.get("file", "").endswith("lane_proto.launch")]
        self.assertEqual(len(lane_includes), 1,
                         "国赛 2026.launch 应有且仅有一个 lane_proto 常驻 include")
        args = {arg.get("name"): arg.get("value", "")
                for arg in lane_includes[0].findall("arg")}
        self.assertEqual(args.get("dry_run"), "false")
        self.assertEqual(args.get("start_base_driver"), "false")
        self.assertEqual(args.get("use_ros_camera"), "true")
        self.assertEqual(args.get("start_enabled"), "false")
        self.assertEqual(args.get("is_fork"), "yolo")
        self.assertIn("red_template_band.png", args.get("template", ""))
        self.assertEqual(args.get("yellow_target"), "0.90")
        self.assertEqual(args.get("align_offset"), "0.14")
        self.assertEqual(args.get("start_offset"), "0.23")
        self.assertEqual(args.get("goal_y_lo"), "0.75")
        self.assertEqual(args.get("linear_speed"), "0.2")
        self.assertEqual(args.get("gain"), "1.2")
        self.assertEqual(args.get("rate"), "20")
        self.assertEqual(args.get("dump_every"), "5")
        self.assertEqual(args.get("goal_pause"), "1.0")
        self.assertEqual(args.get("use_lidar"), "true")
        self.assertEqual(args.get("board_in_lane"), "true")
        self.assertEqual(args.get("go_around"), "true")
        self.assertEqual(args.get("board_stop_dist"), "0.321")
        self.assertEqual(args.get("use_lidar"), "true")
        self.assertEqual(args.get("goal_mode"), "visual")
        self.assertEqual(args.get("go_around_keepout"), "0.08")
        self.assertEqual(args.get("board_arc_lat_scale"), "0.3")
        for legacy in ("goal_control_mode", "goal_grid_path",
                       "goal_point_111", "goal_point_120"):
            self.assertNotIn(legacy, args)

    def test_trackseg_is_not_constructed_during_standby_initialization(self):
        script = os.path.join(PACKAGE_ROOT, "scripts", "lane_follow.py")
        with open(script, "rb") as handle:
            source = handle.read().decode("utf-8")
        initializer, model_loader = source.split(
            "    def ensure_segmentation_model(self):", 1)
        self.assertIn("self.seg = None", initializer)
        self.assertNotIn("TrackSeg(", initializer)
        self.assertIn("self.seg = TrackSeg(self.trackseg_lib)", model_loader)

    def test_activation_constructs_trackseg_once(self):
        try:
            import rospy  # noqa: F401
        except ImportError:
            self.skipTest("requires the vehicle ROS Melodic Python runtime")
        lane_follow = self._load_lane_follow()
        node = object.__new__(lane_follow.LaneFollow)
        node.phase = "STANDBY"
        node.enabled = False
        node.seg = None
        node.trackseg_lib = "/test/libtrackseg.so"
        node.dry_run = False
        node.v = 0.10
        node.mirror = True
        phases = []
        node.set_phase = lambda phase, reason: phases.append((phase, reason))
        calls = []

        class FakeSeg(object):
            backend = "cuda"

        class Request(object):
            data = True

        original_trackseg = lane_follow.TrackSeg
        original_response = lane_follow.SetBoolResponse
        original_loginfo = lane_follow.rospy.loginfo
        lane_follow.TrackSeg = lambda path: calls.append(path) or FakeSeg()
        lane_follow.SetBoolResponse = lambda ok, text: (ok, text)
        lane_follow.rospy.loginfo = lambda *args, **kwargs: None
        try:
            first = node.set_active(Request())
            second = node.set_active(Request())
        finally:
            lane_follow.TrackSeg = original_trackseg
            lane_follow.SetBoolResponse = original_response
            lane_follow.rospy.loginfo = original_loginfo
        self.assertEqual(calls, ["/test/libtrackseg.so"])
        self.assertEqual(first[0], True)
        self.assertEqual(second[0], True)
        self.assertEqual(phases, [
            ("FOLLOW", "主流程已交接控制权"),
            ("FOLLOW", "主流程已交接控制权"),
        ])

    def test_ros_log_preformats_unicode_before_rospy(self):
        try:
            import rospy  # noqa: F401
        except ImportError:
            self.skipTest("requires the vehicle ROS Melodic Python runtime")
        lane_follow = self._load_lane_follow()
        line = lane_follow.format_ros_log(
            u"lane_follow: backend=%s dry_run=%s v=%.2f mirror=%s",
            (u"cuda", False, 0.10, "ON(翻转)"))
        self.assertEqual(
            line.decode("utf-8"),
            u"lane_follow: backend=cuda dry_run=False v=0.10 mirror=ON(翻转)")
        self.assertIs(lane_follow.rospy.loginfo, lane_follow.lane_loginfo)
        self.assertIs(lane_follow.rospy.logwarn, lane_follow.lane_logwarn)
        self.assertIs(lane_follow.rospy.logerr, lane_follow.lane_logerr)

    def test_ros_frame_grabber_exposes_camera_fps(self):
        try:
            import rospy  # noqa: F401
        except ImportError:
            self.skipTest("requires the vehicle ROS Melodic Python runtime")
        lane_follow = self._load_lane_follow()
        original_subscriber = lane_follow.rospy.Subscriber
        original_bridge = lane_follow.CvBridge
        lane_follow.rospy.Subscriber = lambda *args, **kwargs: object()
        lane_follow.CvBridge = lambda: object()
        try:
            grabber = lane_follow.RosFrameGrabber("/test/image")
            self.assertEqual(grabber.cam_fps, 0.0)
        finally:
            lane_follow.rospy.Subscriber = original_subscriber
            lane_follow.CvBridge = original_bridge


if __name__ == "__main__":
    unittest.main()


class SelfCallSanityTest(unittest.TestCase):
    """静态检查: lane_follow.py 里每个 self.xxx(...) 调用都得真有定义。

    2026-08-16 我加"没 odom 不许动"那道闸时写了 self.send(0.0, 0.0) ——
    这个方法根本不存在(发速度用的是 self.pub.publish(Twist()))。py_compile
    和单测都发现不了, 车在场地上跑到那一行才 AttributeError 崩掉。
    这个用例就是那次的护栏: 纯 AST, 不需要 ROS。
    """

    def test_every_self_call_is_defined(self):
        import ast
        import os
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            os.pardir, "scripts", "lane_follow.py")
        with open(path, "rb") as handle:
            tree = ast.parse(handle.read().decode("utf-8"))
        for node in ast.walk(tree):
            if not isinstance(node, ast.ClassDef):
                continue
            defined = set()
            # 继承来的方法也算: FrameGrabber(threading.Thread) 会调 self.join()
            # / self.is_alive(), 那是 Thread 的。按基类名查, 查不到就当没有,
            # 免得把护栏拆宽了。
            import threading as _threading
            for base in node.bases:
                bname = base.attr if isinstance(base, ast.Attribute) else \
                    getattr(base, "id", "")
                if bname == "Thread":
                    defined.update(n for n in dir(_threading.Thread)
                                   if not n.startswith("__"))
                elif bname == "object":
                    pass
            for sub in ast.walk(node):
                if isinstance(sub, (ast.FunctionDef,)):
                    defined.add(sub.name)
                # self.foo = ... 也算(有些是运行时绑上去的可调用对象)
                elif isinstance(sub, ast.Attribute) and \
                        isinstance(sub.value, ast.Name) and \
                        sub.value.id == "self" and \
                        isinstance(sub.ctx, ast.Store):
                    defined.add(sub.attr)
            missing = []
            for sub in ast.walk(node):
                if not isinstance(sub, ast.Call):
                    continue
                f = sub.func
                if isinstance(f, ast.Attribute) and \
                        isinstance(f.value, ast.Name) and \
                        f.value.id == "self" and f.attr not in defined:
                    missing.append("%s: self.%s() 第 %d 行"
                                   % (node.name, f.attr, sub.lineno))
            self.assertEqual(missing, [], "调用了不存在的方法:\n  " +
                             "\n  ".join(missing))


class KfAttrSanityTest(unittest.TestCase):
    """ga_trace 里引用的 self.kf.<attr> 必须在 BoardKF 里真的存在。

    2026-08-16: 我写成 self.kf.c(板心其实叫 self.x), 实车绕障时
    AttributeError 把整份轨迹废掉 —— 而那份轨迹正是用来定位另一个 bug 的。
    """

    def test_ga_trace_only_uses_real_kf_attrs(self):
        import ast
        import os
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            os.pardir, "scripts", "lane_follow.py")
        with open(path, "rb") as handle:
            tree = ast.parse(handle.read().decode("utf-8"))
        cls = dict((n.name, n) for n in ast.walk(tree)
                   if isinstance(n, ast.ClassDef))
        self.assertIn("BoardKF", cls)

        def attrs(c):
            out = set()
            for sub in ast.walk(c):
                if isinstance(sub, ast.FunctionDef):
                    out.add(sub.name)
                elif isinstance(sub, ast.Attribute) and \
                        isinstance(sub.value, ast.Name) and \
                        sub.value.id == "self" and \
                        isinstance(sub.ctx, ast.Store):
                    out.add(sub.attr)
            return out

        kf = attrs(cls["BoardKF"])
        bad = []
        for sub in ast.walk(cls["LaneFollow"]):
            if isinstance(sub, ast.Attribute) and \
                    isinstance(sub.value, ast.Attribute) and \
                    isinstance(sub.value.value, ast.Name) and \
                    sub.value.value.id == "self" and \
                    sub.value.attr == "kf" and sub.attr not in kf:
                bad.append("self.kf.%s (第 %d 行)" % (sub.attr, sub.lineno))
        self.assertEqual(bad, [], "BoardKF 没有这些属性:\n  " +
                         "\n  ".join(bad))


class DodgeSideTest(unittest.TestCase):
    """绕障让向的真值表, 由用户 2026-08-16 直接定死。

    这条规则我来回改错过两次(把 Y 支路从"同侧"改成"反侧"又改回来),
    每次都是在场地上撞了才发现。锁成测试, 以后谁再动 ga_turn_side 的
    符号都会当场红。

    ⚠ 两臂和 Y 支路的符号是**相反**的, 不要合并成一条规则。
    """

    CASES = [
        # (起点转角, 岔口转角, 期望让向: +1 左 / -1 右, 说明)
        (60.0, 0.0, -1.0, "左臂(逆时针60°) -> 往右让"),
        (-60.0, 0.0, 1.0, "右臂(顺时针60°) -> 往左让"),
        (0.0, -45.0, -1.0, "Y 顺时针45°(4号线) -> 往右让"),
        (0.0, 45.0, 1.0, "Y 逆时针45° -> 往左让"),
    ]

    @staticmethod
    def _side(start_turn_deg, fork_turn_deg):
        """和 lane_follow.ga_turn_side() 的符号逻辑保持一致"""
        if abs(start_turn_deg) > 1.0:
            return -1.0 if start_turn_deg > 0 else 1.0      # 两臂: 反侧
        return 1.0 if fork_turn_deg > 0 else -1.0           # Y: 同侧

    def test_truth_table(self):
        for st, fk, want, why in self.CASES:
            self.assertEqual(self._side(st, fk), want, why)

    def test_source_matches_truth_table(self):
        """源码里的两个分支必须还是"两臂反侧 / Y 同侧"这个组合"""
        import os
        import re
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            os.pardir, "scripts", "lane_follow.py")
        with open(path, "rb") as handle:
            src = handle.read().decode("utf-8")
        body = src[src.index("def ga_turn_side"):]
        body = body[:body.index("def ga_choose_dir")]
        self.assertIn("abs(self.start_turn_deg) > 1.0", body,
                      "两臂/Y 的分支判据必须用 start_turn_deg —— "
                      "turn_deg 在岔口原地转时会被改写成 fork_turn_deg")
        arm = re.search(r"sgn = -1\.0 if ref > 0 else 1\.0", body)
        fork = re.search(r"sgn = 1\.0 if ref > 0 else -1\.0", body)
        self.assertTrue(arm, "两臂那支应为反侧: -1.0 if ref > 0 else 1.0")
        self.assertTrue(fork, "Y 那支应为同侧: 1.0 if ref > 0 else -1.0")
        self.assertLess(arm.start(), fork.start(),
                        "顺序反了: 先两臂(start_turn_deg)后 Y(fork_turn_deg)")


class YellowGoTest(unittest.TestCase):
    """yellow_go 的真值表。

    7 类红绿灯模型比原来多了 yellow left/right/straight 三类。默认**不放行**
    (当红灯等), 传 yellow_go:=true 才把 "yellow left" 当 "left" 用。

    这里**不重写一份实现**, 而是把 lane_follow.py 里真正那个 yolo_norm 的
    源码抠出来 exec 进一个壳类再测 —— 照抄一份实现去测, 测的是抄的那份,
    改了真代码而忘了改测试就什么都发现不了。

    真值表里最要命的一条: 关掉 yellow_go 时黄灯必须归一成 "stop" 而**不是**
    None。None 会和"这一帧什么都没检出"同等对待, 于是一路空等到
    yolo_wait_max(默认 60s)超时再按 fallback 瞎走; "stop" 走的是"红灯继续
    等"那条正路, 灯一变绿立刻发车。
    """

    @classmethod
    def setUpClass(cls):
        import ast as _ast
        import textwrap
        path = os.path.join(PACKAGE_ROOT, "scripts", "lane_follow.py")
        with open(path) as fh:
            src = fh.read()
        tree = _ast.parse(src)
        fn = None
        for node in _ast.walk(tree):
            if isinstance(node, _ast.FunctionDef) and node.name == "yolo_norm":
                fn = node
                break
        assert fn is not None, "lane_follow.py 里找不到 yolo_norm"
        lines = src.splitlines()[fn.lineno - 1:fn.end_lineno]
        ns = {}
        exec(textwrap.dedent("\n".join(lines)), ns)          # noqa: S102

        class Shell(object):
            yolo_norm = ns["yolo_norm"]

            def __init__(self, go):
                self._go = bool(go)

            def yellow_ok(self):
                return self._go

        cls.Shell = Shell

    CASES = [
        # (类名, yellow_go 关时归一成, 开时归一成)
        ("left", "left", "left"),
        ("right", "right", "right"),
        ("straight", "straight", "straight"),
        ("stop", "stop", "stop"),
        ("yellow left", "stop", "left"),
        ("yellow right", "stop", "right"),
        ("yellow straight", "stop", "straight"),
        ("YELLOW_LEFT", "stop", "left"),      # 大小写 / 下划线都要吃得下
        ("  yellow   right  ", "stop", "right"),
    ]

    def test_truth_table(self):
        for name, off, on in self.CASES:
            self.assertEqual(self.Shell(False).yolo_norm(name)[0], off,
                             "yellow_go=false: %r" % name)
            self.assertEqual(self.Shell(True).yolo_norm(name)[0], on,
                             "yellow_go=true: %r" % name)

    def test_yellow_never_none_when_off(self):
        """关掉时黄灯必须是 stop, 不能是 None —— None 会走到 60s 超时兜底"""
        for name in ("yellow left", "yellow right", "yellow straight",
                     "yellow", "yellow banana"):
            got, is_y = self.Shell(False).yolo_norm(name)
            self.assertIsNotNone(got, "%r 归一成 None 了" % name)
            self.assertTrue(is_y, "%r 应该被认出是黄灯" % name)

    def test_unknown_is_none(self):
        for name in ("banana", "", None):
            self.assertEqual(self.Shell(False).yolo_norm(name)[0], None)
            self.assertEqual(self.Shell(True).yolo_norm(name)[0], None)

    def test_launch_exposes_yellow_go_as_tristate(self):
        """三态: false / true / adaptive, 默认 adaptive。

        ⚠ param 必须 type="str" —— roslaunch 会把 adaptive 之外的裸值
          自动转成 bool, 传字符串不声明类型就会飘(is_fork 踩过同一个坑)。
        """
        path = os.path.join(PACKAGE_ROOT, "launch", "lane_proto.launch")
        root = ET.parse(path).getroot()
        args = [a for a in root.iter("arg") if a.get("name") == "yellow_go"]
        self.assertEqual(len(args), 1, "launch 里应该有且只有一个 yellow_go")
        self.assertEqual(args[0].get("default"), "adaptive",
                         "yellow_go 默认应该是 adaptive")
        params = [p for p in root.iter("param") if p.get("name") == "yellow_go"]
        self.assertEqual(len(params), 1, "yellow_go 没被传给节点")
        self.assertEqual(params[0].get("type"), "str",
                         "三态参数必须 type=str, 否则 roslaunch 转成 bool")
        after = [a.get("default") for a in root.iter("arg")
                 if a.get("name") == "yellow_go_after"]
        self.assertEqual(len(after), 1)
        self.assertAlmostEqual(float(after[0]), 10.0, places=3)


class GoalGateReasonTest(unittest.TestCase):
    """HIT 了却没停车 —— 四道闸每一道都必须报得出原因。

    背景(实车 2026-08-18): 叠加图上橙线画得好好的、抬头写着 HIT, 车却一路
    开过终点。查日志发现里程闸和雷达否决会打提示, 而**投票和冷却被拦时
    一声不吭**, 于是根本查不出是哪一道吃掉的。

    和 YellowGoTest 一样, 这里不照抄实现, 而是把 lane_follow.py 里真正的
    goal_gate 抠出来 exec 后再测。
    """

    @classmethod
    def setUpClass(cls):
        import ast as _ast
        import textwrap
        import time as _time
        path = os.path.join(PACKAGE_ROOT, "scripts", "lane_follow.py")
        with open(path) as fh:
            src = fh.read()
        tree = _ast.parse(src)
        fn = None
        for node in _ast.walk(tree):
            if isinstance(node, _ast.FunctionDef) and node.name == "goal_gate":
                fn = node
                break
        assert fn is not None, "lane_follow.py 里找不到 goal_gate"
        ns = {"time": _time}
        exec(textwrap.dedent(                                    # noqa: S102
            "\n".join(src.splitlines()[fn.lineno - 1:fn.end_lineno])), ns)

        class Shell(object):
            goal_gate = ns["goal_gate"]

            def __init__(self, **kw):
                self.goal_min_travel = -1.0
                self.goal_gate_arm, self.goal_gate_y = 2.40, 2.00
                self.start_turn_deg = 60.0        # 两臂
                self.board_on = True
                self.is_fork, self.fork_done = True, True
                self.board_anchor = "岔口"
                self.goal_hits, self.goal_confirm = 0, 1
                self.cool_until = 0.0
                self.phase = "FOLLOW"
                self.t_follow0 = None
                self._trav = 3.4                  # 已经走够了
                self.__dict__.update(kw)

            def board_travel(self, anchor):
                return self._trav

        cls.Shell = Shell
        cls.time = _time

    def test_all_gates_open(self):
        ok, why = self.Shell().goal_gate(True)
        self.assertTrue(ok)
        self.assertIsNone(why, "四道全过时不该给理由")

    def test_每道闸都报得出原因(self):
        cases = [
            ("里程", dict(_trav=1.0)),
            ("投票", dict(goal_confirm=3, goal_hits=0)),
            ("冷却", dict(cool_until=self.time.time() + 2.0)),
            ("相位", dict(phase="PAUSE")),
        ]
        for key, kw in cases:
            ok, why = self.Shell(**kw).goal_gate(True)
            self.assertIsNotNone(why, "%s 被拦住却没给理由" % key)
            self.assertIn(key, why, "理由里应该点明是哪道闸: %r" % why)

    def test_votes_counts_this_frame(self):
        """票数要算上**当前这一帧**: goal_confirm=3 时第 3 帧就该放行"""
        self.assertIsNotNone(
            self.Shell(goal_confirm=3, goal_hits=1).goal_gate(True)[1])
        self.assertIsNone(
            self.Shell(goal_confirm=3, goal_hits=2).goal_gate(True)[1])

    def test_no_hit_no_reason(self):
        ok, why = self.Shell(_trav=0.1).goal_gate(False)
        self.assertIsNone(why, "没 HIT 就不该有'为什么没停'")

    def test_fork_line_has_no_travel_gate(self):
        """Y 岔口那条横线不设里程闸 —— 拿终点的闸去卡它, 票永远攒不满"""
        ok, why = self.Shell(fork_done=False, _trav=0.3).goal_gate(True)
        self.assertTrue(ok)
        self.assertIsNone(why)

    def test_source_only_logs_for_the_goal_line(self):
        """打日志/写 dump 只能是终点那一支, Y 岔口不报(用户 2026-08-18 要求)"""
        path = os.path.join(PACKAGE_ROOT, "scripts", "lane_follow.py")
        with open(path) as fh:
            src = fh.read()
        self.assertIn("if hit and why and not fork_line and not in_goal_phase:",
                      src, "相位机里的日志必须排除 Y 岔口那一支, 且进了终点"
                           "相位(CORNER_ADJUST/PAUSE/APPROACH)就别再刷")
        self.assertIn("if hit and not (self.is_fork and not self.fork_done) and",
                      src, "写 dump 备注时也必须排除 Y 岔口那一支")
        self.assertIn('note += "  |没停: %s" % self._goal_why', src,
                      "dump 图的备注里要带上原因")


class GoalCooldownSplitTest(unittest.TestCase):
    """绕障后的"横线冷却"必须和"板子冷却"分开。

    实车 2026-08-18: go_around_cooldown 一个参数同时管两件事, 于是绕完之后
    3 秒内的终点横线全被丢弃。那次终点线是 cov=1.00/line=1.00/16条 满分检出,
    线滑出扫描带比冷却到期只早 0.045s, 车又跑了 0.46m 才认到下一条横线,
    在那儿打点再推 0.5m, 撞墙。

    绕完板子在**车屁股后面**, 相机朝前看不见, 压制横线没有道理; 防"板面被
    当成终点线"的本来就是里程闸。所以这一支默认关掉(go_around_cooldown_goal
    = 0), 而**岔口转弯后那次照旧**(fork_cooldown=3.0) —— 那是防分叉口自己
    那条横线, 风险是真的。
    """

    @classmethod
    def setUpClass(cls):
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            cls.src = fh.read()

    def test_avoid_branch_is_guarded_by_the_new_param(self):
        self.assertIn('self.ga_cool_goal = float(gp("~go_around_cooldown_goal"',
                      self.src, "少了 go_around_cooldown_goal 这个参数")
        self.assertIn("if self.ga_cool_goal > 0.0:\n"
                      "                    self.cool_until = max(",
                      self.src,
                      "绕障收尾必须只在 ga_cool_goal>0 时才压制横线")
        self.assertNotIn("self.cool_until = max(self.cool_until,\n"
                         "                                      time.time() "
                         "+ self.ga_cool)", self.src,
                         "绕障那支还在用 ga_cool 压横线 —— 就是撞墙那个 bug")

    def test_fork_cooldown_untouched(self):
        """岔口那次冷却用的是自己的参数, 不能被这次改动带偏"""
        self.assertIn("self.cool_until = time.time() + self.fork_cooldown",
                      self.src, "岔口转弯后的横线冷却被改坏了")

    def test_board_cooldown_still_full(self):
        """板子冷却仍是 ga_cool, 没跟着一起关"""
        self.assertIn("self.board_cool_until = time.time() + self.ga_cool",
                      self.src)

    def test_launch_defaults(self):
        root = ET.parse(os.path.join(PACKAGE_ROOT, "launch",
                                     "lane_proto.launch")).getroot()

        def dflt(name):
            got = [a.get("default") for a in root.iter("arg")
                   if a.get("name") == name]
            self.assertEqual(len(got), 1, "%s 应该有且只有一个 arg" % name)
            return got[0]

        self.assertEqual(float(dflt("go_around_cooldown_goal")), 0.0,
                         "绕障后横线冷却默认必须是 0(立刻放开)")
        self.assertEqual(float(dflt("go_around_cooldown")), 3.0,
                         "板子冷却不该动")
        self.assertEqual(float(dflt("fork_cooldown")), 3.0,
                         "岔口横线冷却不该动")
        names = [p.get("name") for p in root.iter("param")]
        self.assertIn("go_around_cooldown_goal", names, "新参数没传给节点")


class BoardArcCorrectionTest(unittest.TestCase):
    """绕板收尾的曲率修正(只在两臂)。

    板法线只是**板子那一点**的车道切线。第二段前进那 d 米走的是直线, 而两臂
    的车道是弧 —— 等车走到板后 d 米, 真正的中心线已经转过 phi=asin(d/R)、
    并往弯道内侧偏了 R(1-cos phi)。实测(2026-08-18): 偏航 15°、横向 5cm,
    两个数反解出同一个半径(1.433 / 1.467), 所以默认取 R=1.43。

    这里锁三件事: 大小、方向、以及"方向取几何规则而不是实际避让方向"。
    """

    R = 1.433

    def _phi_off(self, fwd):
        import math
        phi = math.asin(fwd / self.R)
        return phi, self.R * (1.0 - math.cos(phi))

    def test_magnitude_matches_the_measurement(self):
        import math
        phi, off = self._phi_off(0.171 + 0.20)      # 车半长 + 禁区 = 0.371
        self.assertAlmostEqual(math.degrees(phi), 15.0, places=1)
        self.assertAlmostEqual(off, 0.049, places=3)

    def test_scales_with_the_forward_leg(self):
        """修正量跟**实际**前进距离走, 不是写死 15°"""
        import math
        for fwd, deg in ((0.371, 15.0), (0.45, 18.3), (0.58, 23.9)):
            phi, _ = self._phi_off(fwd)
            self.assertAlmostEqual(math.degrees(phi), deg, places=1)

    def test_signs_land_on_the_true_centre_line(self):
        """把源码里那套符号推导跑一遍, 落点必须和真实弧上的点重合。

        板子系: 板心原点, 法向 n=+Y(车道方向), 板面 u=+X;
        赛道往 -X 弯(圆心 (-R,0)) => 小半径侧 = 左 => arc_sgn=+1。
        车头朝 +Y, 车体 +y=左=世界 -X, 所以 u 在车体系的 y 分量是 -1。
        """
        import math
        arc_sgn, uy_body = +1.0, -1.0
        fwd = 0.371
        phi, off = self._phi_off(fwd)
        u_side = 1.0 if (uy_body * arc_sgn) > 0 else -1.0
        arc_lat = u_side * off                       # 源码里的 arc_lat
        true_lat = -self.R + self.R * math.cos(phi)  # 真实弧上那一点
        self.assertAlmostEqual(arc_lat, true_lat, places=9,
                               msg="横向修正的符号反了")
        self.assertGreater(arc_sgn * phi, 0.0, "航向修正应该往圆心那侧转")

    def test_only_after_the_board(self):
        """只修**板后**收尾那两段。

        板前的 AVOID_TURN0(预转正)目标本来就该是板法向 —— 车得摆正了才能
        干净地横移过去。在那儿按 |lon| 算 phi(板前 0.3m 就是 12°)会把车一
        开局就拧歪, 横移直接蹭板面。
        """
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            src = fh.read()
        i = src.index("arc_phi = arc_lat = 0.0")
        j = src.index("arc_sgn = self.ga_turn_side()[0]", i)
        cond = src[i:j]
        self.assertIn('self.phase in ("AVOID_BACK", "AVOID_ALIGN")', cond,
                      "曲率修正必须限定在板后收尾那两段")
        self.assertNotIn("AVOID_TURN0", cond.split("#")[0],
                         "板前预转正绝对不能带曲率修正")

    def test_source_uses_geometry_rule_not_dodge_side(self):
        """方向必须取 ga_turn_side()(小半径侧), **不能**用 ga_sign ——
        那一侧被挡住时 ga_choose_dir 会翻到另一边让, 但赛道往哪弯和车从
        哪边绕过去毫无关系。"""
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            src = fh.read()
        i = src.index("arc_phi = arc_lat = 0.0")
        j = src.index("out = dict(bx=bx", i)
        blk = src[i:j]
        self.assertIn("arc_sgn = self.ga_turn_side()[0]", blk)
        self.assertNotIn("self.ga_sign", blk,
                         "曲率修正不能跟着实际避让方向走")
        self.assertIn("abs(self.start_turn_deg) > 1.0", blk,
                      "只能在两臂生效, Y 支路是直线")

    def test_lateral_scale_is_independent_of_heading(self):
        """横向倍率只能碰横向, 不许把航向也乘进去。

        航向 15° 是"车头朝向对不对", 错了后半程一路越走越偏, 修它稳赚;
        横向 5cm 收尾后马上交给视觉巡线, 巡线本身就在纠横向。所以这两件
        事必须能分开调。
        """
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            src = fh.read()
        i = src.index("arc_phi = arc_lat = 0.0")
        j = src.index("out = dict(bx=bx", i)
        blk = src[i:j]
        self.assertIn("* self.arc_lat_scale", blk, "横向倍率没接上")
        # 倍率只能出现在算 off 的那一行, 不能碰 arc_phi
        for line in blk.splitlines():
            code = line.split("#")[0]          # 注释里提到不算
            if "arc_lat_scale" not in code:
                continue
            # off 里用到 cos(phi) 是应该的, 不能一看到 phi 就报错;
            # 真正不许发生的是把倍率乘到 arc_phi(航向)上。
            self.assertIn("off", code,
                          "倍率只能乘在横向 off 上: %r" % code.strip())
            self.assertNotIn("arc_phi", code,
                             "倍率不许碰航向: %r" % code.strip())
        # 再正面确认一次: arc_phi 那一行不含倍率
        for line in blk.splitlines():
            code = line.split("#")[0]
            if "arc_phi =" in code and "arc_lat" not in code:
                self.assertNotIn("arc_lat_scale", code,
                                 "航向被倍率影响了: %r" % code.strip())

    def test_launch_defaults(self):
        root = ET.parse(os.path.join(PACKAGE_ROOT, "launch",
                                     "lane_proto.launch")).getroot()
        scale = [a.get("default") for a in root.iter("arg")
                 if a.get("name") == "board_arc_lat_scale"]
        self.assertEqual(len(scale), 1)
        self.assertAlmostEqual(float(scale[0]), 1.0, places=3)
        got = [a.get("default") for a in root.iter("arg")
               if a.get("name") == "board_arc_r"]
        self.assertEqual(len(got), 1)
        self.assertAlmostEqual(float(got[0]), 1.43, places=2)
        names = [p.get("name") for p in root.iter("param")]
        for n in ("board_arc_r", "board_arc_max_deg",
                  "board_arc_lat_scale"):
            self.assertIn(n, names, "%s 没传给节点" % n)


class TemplateYSwitchTest(unittest.TestCase):
    """Y 支路单独换触发区模板。

    Y 那一段和两臂的车道形状不一样(直、更窄), 同一张模板要么太松要么太紧。
    切换点选在 FORK_TURN 结束 —— 只有走**中间那条**才会经过那个相位(两臂
    是在 apply_branch 里直接置 fork_done, 不进 FORK_TURN), 所以那里正好
    就是"Y 字分叉之后"。
    """

    @classmethod
    def setUpClass(cls):
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            cls.src = fh.read()

    def test_switch_happens_in_fork_turn_only(self):
        i = self.src.index('elif self.phase == "FORK_TURN":')
        j = self.src.index("elif self.phase in (\"AVOID_REV\"", i)
        blk = self.src[i:j]
        self.assertIn("self.dec = self.dec_y", blk,
                      "转完 Y 之后没换模板")
        # 两臂那一支(apply_branch)不许碰模板
        k = self.src.index("# 两臂 -> 转完就是普通巡线")
        self.assertNotIn("dec_y", self.src[k:k + 600],
                         "两臂不该换成 Y 的模板")

    def test_loaded_at_startup_not_lazily(self):
        """开机就加载: 路径写错要立刻炸, 不能等车跑到岔口中间才炸"""
        i = self.src.index('self.tpl_y_path = gp("~template_y"')
        j = self.src.index("# 分割网**懒加载**", i)
        self.assertIn("Decider(*load_template(self.tpl_y_path))",
                      self.src[i:j], "template_y 必须在 __init__ 里就加载")

    def test_same_path_is_a_noop(self):
        i = self.src.index('self.tpl_y_path = gp("~template_y"')
        j = self.src.index("# 分割网**懒加载**", i)
        self.assertIn("os.path.abspath", self.src[i:j],
                      "和 template 指到同一个文件时应该等于不换")

    def test_launch_default_is_band2(self):
        root = ET.parse(os.path.join(PACKAGE_ROOT, "launch",
                                     "lane_proto.launch")).getroot()
        got = [a.get("default") for a in root.iter("arg")
               if a.get("name") == "template_y"]
        self.assertEqual(len(got), 1, "template_y 应该有且只有一个 arg")
        self.assertTrue(got[0].endswith("red_template_band2.png"),
                        "Y 支路默认应该是 red_template_band2.png, 实际 %r"
                        % got[0])
        names = [p.get("name") for p in root.iter("param")]
        self.assertIn("template_y", names, "template_y 没传给节点")

    def test_the_two_templates_actually_differ(self):
        """换了得有意义 —— 两张图的触发区不能是同一个"""
        sys.path.insert(0, os.path.join(PACKAGE_ROOT, "scripts"))
        try:
            from lane_common import load_template
        except ImportError:
            self.skipTest("没有 cv2/numpy, 跳过")
        import numpy as np
        cfg = os.path.join(PACKAGE_ROOT, "config")
        a = load_template(os.path.join(cfg, "red_template_band.png"))
        b = load_template(os.path.join(cfg, "red_template_band2.png"))
        self.assertFalse(np.array_equal(a[0], b[0]) and
                         np.array_equal(a[1], b[1]),
                         "band 和 band2 的触发区完全一样, 换了等于没换")


class YoloCropTest(unittest.TestCase):
    """认灯用整帧原生裁剪。

    相机开到高分辨率, 从整帧里按**原始像素**抠一块 net 大小的窗口喂 yolo。
    和 yolo_zoom 的区别: zoom 先抠再插值放大回去, 细节是假的; 这里 1:1。
    640x352 的窗口在 1920x1080 上只占 1/3 宽 = 3 倍光学变焦。

    窗口中心 (0.470, 0.327) 是从 2026-08-18 那张实拍量的: 灯箱
    x=288~316 y=106~130 (640x360), 中心 (301,118)。偏左偏上, 不是物理中心。
    """

    CX, CY, CW, CH = 0.470, 0.327, 640, 352
    # 灯在那张 640x360 实拍里的包围盒
    LIGHT = (288.0, 316.0, 106.0, 130.0, 640.0, 360.0)

    def _win(self, W, H):
        x0 = int(round(self.CX * W - self.CW / 2.0))
        y0 = int(round(self.CY * H - self.CH / 2.0))
        return (max(0, min(W - self.CW, x0)),
                max(0, min(H - self.CH, y0)))

    def test_light_falls_inside_the_window(self):
        lx0, lx1, ly0, ly1, sw, sh = self.LIGHT
        for W, H in ((1280, 720), (1920, 1080)):
            x0, y0 = self._win(W, H)
            a, b = lx0 / sw * W, lx1 / sw * W
            c, d = ly0 / sh * H, ly1 / sh * H
            self.assertTrue(x0 <= a and b <= x0 + self.CW,
                            "%dx%d: 灯横向出框" % (W, H))
            self.assertTrue(y0 <= c and d <= y0 + self.CH,
                            "%dx%d: 灯纵向出框" % (W, H))

    def test_zoom_gain_is_real(self):
        """1920 宽下等效 3 倍变焦, 灯从 28px 变 84px"""
        lx0, lx1, _, _, sw, _ = self.LIGHT
        px_now = (lx1 - lx0)                       # 640 宽整帧: 28px
        px_crop = (lx1 - lx0) / sw * 1920          # 1920 整帧原生裁: 84px
        self.assertAlmostEqual(1920.0 / self.CW, 3.0, places=6)
        self.assertGreater(px_crop, 2.9 * px_now)

    def test_flip_before_crop(self):
        """必须**先翻正再裁**: cx 是在翻正后的朝向上量的, 反过来会镜像到
        另一边 —— 0.47 变 0.53, 1920 宽上差 115px, 灯直接出框。"""
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            src = fh.read()
        i = src.index("def crop_frame(self)")
        j = src.index("def step(self, frame", i)
        blk = src[i:j]
        self.assertLess(blk.index("cv2.flip(im, 1)"),
                        blk.index("x0 = int(round(self.yolo_crop_cx"),
                        "翻正必须在算窗口位置之前")
        # 镜像搞反的话中心会差这么多
        self.assertGreater(abs((1 - self.CX) - self.CX) * 1920, 100)

    def test_timeout_falls_back_to_full_frame(self):
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            src = fh.read()
        i = src.index("def crop_frame(self)")
        j = src.index("def step(self, frame", i)
        blk = src[i:j]
        self.assertIn("self._crop_gave_up = True", blk, "少了超时兜底")
        self.assertIn("yolo_crop_timeout", blk)

    def test_lane_path_untouched(self):
        """巡线仍吃 to_4x3 之后的帧, 只有认灯用整帧"""
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            src = fh.read()
        i = src.index("def step(self, frame, dump_i=None):")
        blk = src[i:i + 700]
        self.assertLess(blk.index("self._full = frame"),
                        blk.index("frame = self.to_4x3("),
                        "整帧必须在 to_4x3 之前留存")
        self.assertIn("self._raw = frame", blk)

    def test_launch_defaults(self):
        root = ET.parse(os.path.join(PACKAGE_ROOT, "launch",
                                     "lane_proto.launch")).getroot()

        def d(n):
            got = [a.get("default") for a in root.iter("arg")
                   if a.get("name") == n]
            self.assertEqual(len(got), 1, "%s 应该有且只有一个 arg" % n)
            return got[0]

        # 现在默认开: 只在认灯那一小段临时切高分辨率, 认完立刻切回,
        # 巡线的去畸变映射一秒都没被高分辨率的帧碰过, 风险已经消掉了。
        self.assertEqual(d("yolo_crop"), "true")
        self.assertEqual(int(d("yolo_cam_w")), 1920, "取相机支持的最大档")
        self.assertEqual(int(d("yolo_cam_h")), 1080)
        self.assertAlmostEqual(float(d("yolo_crop_cx")), 0.470, places=3)
        self.assertAlmostEqual(float(d("yolo_crop_cy")), 0.327, places=3)
        self.assertEqual(int(d("yolo_crop_w")), 640)
        self.assertEqual(int(d("yolo_crop_h")), 352)
        self.assertAlmostEqual(float(d("yolo_crop_timeout")), 20.0, places=3)
        self.assertEqual(int(d("cam_w")), 0, "cam_w 默认 0 = 不改相机")
        names = [p.get("name") for p in root.iter("param")]
        for n in ("cam_w", "cam_h", "yolo_crop", "yolo_crop_cx",
                  "yolo_crop_cy", "yolo_crop_timeout"):
            self.assertIn(n, names, "%s 没传给节点" % n)


class YellowAdaptiveTest(unittest.TestCase):
    """adaptive 黄灯: 前 N 秒当红灯等, 等过了还是黄就认了它走。

    为什么不是一直等: 一路空等到 yolo_wait_max(60s) 再按 fallback 瞎走,
    比"认了这个黄灯"糟得多 —— fallback 是猜的, 黄灯是看见的。
    """

    @classmethod
    def setUpClass(cls):
        import ast as _ast
        import textwrap
        import time as _time
        path = os.path.join(PACKAGE_ROOT, "scripts", "lane_follow.py")
        with open(path) as fh:
            src = fh.read()
        fn = None
        for node in _ast.walk(_ast.parse(src)):
            if isinstance(node, _ast.FunctionDef) and node.name == "yellow_ok":
                fn = node
                break
        assert fn is not None, "找不到 yellow_ok"
        ns = {"time": _time}
        exec(textwrap.dedent(                                    # noqa: S102
            "\n".join(src.splitlines()[fn.lineno - 1:fn.end_lineno])), ns)

        class Shell(object):
            yellow_ok = ns["yellow_ok"]

            def __init__(self, mode, elapsed, after=10.0):
                self.yellow_mode = mode
                self.yellow_go_after = after
                self.yolo_t0 = _time.time() - elapsed

        cls.Shell = Shell

    def test_true_and_false_are_absolute(self):
        for el in (0.0, 5.0, 30.0):
            self.assertTrue(self.Shell("true", el).yellow_ok())
            self.assertFalse(self.Shell("false", el).yellow_ok())

    def test_adaptive_flips_at_the_threshold(self):
        self.assertFalse(self.Shell("adaptive", 0.0).yellow_ok(), "刚开始该等")
        self.assertFalse(self.Shell("adaptive", 9.5).yellow_ok(), "9.5s 还该等")
        self.assertTrue(self.Shell("adaptive", 10.5).yellow_ok(), "过 10s 该走")
        self.assertTrue(self.Shell("adaptive", 30.0).yellow_ok())

    def test_threshold_is_configurable(self):
        self.assertFalse(self.Shell("adaptive", 3.0, after=5.0).yellow_ok())
        self.assertTrue(self.Shell("adaptive", 6.0, after=5.0).yellow_ok())
        self.assertTrue(self.Shell("adaptive", 0.0, after=0.0).yellow_ok(),
                        "after=0 应该等于立刻放行")

    def test_flips_before_the_wait_timeout(self):
        """阈值必须**远小于** yolo_wait_max, 否则 adaptive 等于没用"""
        root = ET.parse(os.path.join(PACKAGE_ROOT, "launch",
                                     "lane_proto.launch")).getroot()

        def d(n):
            return float([a.get("default") for a in root.iter("arg")
                          if a.get("name") == n][0])

        self.assertLess(d("yellow_go_after"), d("yolo_wait_max") / 2.0)


class StartDashTest(unittest.TestCase):
    """认到灯之后冲到三岔口那一段加速(odom 盲走固定里程, 路上没有判断)"""

    @classmethod
    def setUpClass(cls):
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            cls.src = fh.read()

    def test_only_when_a_light_was_actually_seen(self):
        """超时兜底那次不许加速 —— 那时候不知道前面是什么"""
        i = self.src.index("gain = max(1.0, self.start_dash_gain")
        self.assertIn("if self._verdict_from_light else 1.0",
                      self.src[i:i + 200])
        # 标记只能在"够票定案"那一支置位
        j = self.src.index("self._verdict_from_light = True")
        k = self.src.rindex("够票了", 0, j)
        self.assertLess(j - k, 400, "标记应该紧跟在够票定案那一支")

    def test_distance_is_odom_based(self):
        """用户问的: 这段是不是走 odom —— start_move 打 odom 点"""
        i = self.src.index("def start_move(self, dist, speed, phase, why):")
        self.assertIn("self.mark_xy = self.odom_xy", self.src[i:i + 300])

    def test_launch_default_is_double(self):
        root = ET.parse(os.path.join(PACKAGE_ROOT, "launch",
                                     "lane_proto.launch")).getroot()
        got = [a.get("default") for a in root.iter("arg")
               if a.get("name") == "start_dash_gain"]
        self.assertEqual(len(got), 1)
        self.assertAlmostEqual(float(got[0]), 2.0, places=3, msg="默认加倍")
        names = [p.get("name") for p in root.iter("param")]
        self.assertIn("start_dash_gain", names)


class YoloCleanupOrderTest(unittest.TestCase):
    """认到灯之后: **先发车, 再切 USB**。

    kill_yolo 里两件事都会阻塞: yolo.close() 最坏等 grace=2.0s, cap.set 切
    分辨率也是同步调用。原来它们都在 start_move **之前**, 阻塞正好落在
    "绿灯已经认出来、车还没动"这个最不该等的窗口里。

    现在: finish_yolo 只挂个标志 -> 主循环把第一条 cmd_vel 发出去 -> 丢到
    后台线程收尾。而且切分辨率时取帧线程要暂停, 主循环拿不到新帧会 continue,
    一条指令都不发 —— 底盘 cmd_timeout 只有 0.2s, 车会在冲线半路被刹停,
    所以盲走相位断帧时要继续发上一条指令。
    """

    @classmethod
    def setUpClass(cls):
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            cls.src = fh.read()

    def test_finish_yolo_does_not_block_before_moving(self):
        i = self.src.index("def finish_yolo(self, cls):")
        j = self.src.index("def kill_yolo(self):", i) if \
            self.src.find("def kill_yolo(self):", i) > 0 else len(self.src)
        blk = self.src[i:self.src.index("def start_move(self", i)]
        self.assertNotIn("self.kill_yolo()", blk,
                         "finish_yolo 里不能当场 kill_yolo(会阻塞发车)")
        self.assertIn("self._yolo_cleanup_pending = True", blk)
        self.assertLess(blk.index("self._yolo_cleanup_pending = True"),
                        blk.index("self.start_move("),
                        "挂标志要在 start_move 之前(顺序无所谓, 但别漏)")

    def test_cleanup_runs_after_publish_and_off_thread(self):
        i = self.src.index("self._last_tw = tw")
        blk = self.src[i:i + 900]
        self.assertLess(blk.index("self.pub.publish(tw)"),
                        blk.index("_yolo_cleanup_pending"),
                        "收尾必须在 publish 之后")
        self.assertIn("threading.Thread(target=self._yolo_cleanup)", blk,
                      "收尾要丢后台线程, 别卡控制环")

    def test_blind_phases_keep_publishing_on_frame_gap(self):
        self.assertIn("BLIND_PHASES", self.src)
        i = self.src.index("BLIND_PHASES = (")
        names = self.src[i:self.src.index(")", i)]
        for p in ("START_MOVE", "FORK_TURN", "AVOID_OUT", "CORNER_ADJUST"):
            self.assertIn(p, names, "%s 该算盲走相位" % p)
        for p in ("FOLLOW", "APPROACH"):
            self.assertNotIn('"%s"' % p, names,
                             "%s 靠视觉闭环, 断帧不能硬走" % p)
        j = self.src.index("还没有新帧")
        gap = self.src[j:j + 1200]
        self.assertIn("self.pub.publish(self._last_tw)", gap)
        self.assertIn("self.cam_gap_hold", gap, "兜底要有时间上限")

    def test_frame_gap_still_stops_eventually(self):
        """兜底只兜 cam_gap_hold 秒, 3 秒无帧照旧停车退出"""
        j = self.src.index("还没有新帧")
        gap = self.src[j:j + 1200]
        self.assertIn("3 秒没有新帧", gap)
        self.assertLess(gap.index("3 秒没有新帧"),
                        gap.index("self.pub.publish(self._last_tw)"),
                        "掉相机的判定要在兜底之前")

    def test_launch_default(self):
        root = ET.parse(os.path.join(PACKAGE_ROOT, "launch",
                                     "lane_proto.launch")).getroot()
        got = [a.get("default") for a in root.iter("arg")
               if a.get("name") == "cam_gap_hold"]
        self.assertEqual(len(got), 1)
        self.assertAlmostEqual(float(got[0]), 1.0, places=3)
        self.assertIn("cam_gap_hold",
                      [p.get("name") for p in root.iter("param")])


class CamReopenTest(unittest.TestCase):
    """换分辨率必须 release + 重开, 不能 cap.set。

    实车 2026-08-18: 用 cap.set 之后驱动只是把参数记下来, 原来那批 mmap
    buffer 立刻作废 ——
        VIDIOC_DQBUF: Invalid argument
        VIDIOC_QBUF: Invalid argument
        Corrupt JPEG data: premature end of data segment
    而 cap.get 还照样回报 1920x1080 **说成功了**。结果再也读不到帧, 3 秒后
    节点自判"相机掉了"退出, 且 cap 已废, 想切回 640x480 都失败(实际 0x0)。
    """

    @classmethod
    def setUpClass(cls):
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            cls.src = fh.read()

    def _blk(self, name, end):
        i = self.src.index("def %s(self" % name)
        return self.src[i:self.src.index("def %s(self" % end, i)]

    def test_reopens_instead_of_set(self):
        blk = self._blk("_set_cam_res_locked", "set_cam_res")
        self.assertIn("release()", blk, "必须先 release 老的")
        self.assertIn("self._new_cap(", blk, "必须重开")
        self.assertNotIn("CAP_PROP_FRAME_WIDTH, w", blk,
                         "不能再用 cap.set 改分辨率")

    def test_verifies_by_reading_a_real_frame(self):
        """cap.get 会撒谎, 只认真读到的帧尺寸"""
        blk = self._blk("_new_cap", "_set_cam_res_locked")
        self.assertIn("cap.read()", blk)
        flat = " ".join(blk.split())            # 源码里那句折了行
        self.assertIn("f.shape[1] == w and f.shape[0] == h", flat,
                      "要拿真帧的 shape 判断, 不是 cap.get")
        # 成功与否不能由 cap.get 决定
        i = flat.index("return cap")
        self.assertNotIn("cap.get(cv2.CAP_PROP_FRAME_WIDTH)", flat[:i],
                         "返回成功之前不该问 cap.get")

    def test_falls_back_to_the_old_resolution(self):
        """新的开不起来就把原分辨率开回来 —— 半路丢相机整趟就废了"""
        blk = self._blk("_set_cam_res_locked", "set_cam_res")
        self.assertIn("self._new_cap(cur[0], cur[1])", blk)
        self.assertIn("开不回原分辨率", blk)

    def test_joins_grabber_before_release(self):
        """**先 join 取帧线程, 再 release cap** —— 就是这次事故的根因。

        实车 2026-08-18: stop() 只设标志, 线程还卡在 cap.read() 里就
        release, 于是取帧线程炸(Unknown array type)、STREAMOFF 失败(EBUSY)、
        设备被占死、连 640x480 都开不回来, 最后 GC 半死的 cap 时 SIGSEGV。
        """
        blk = self._blk("_set_cam_res_locked", "set_cam_res")
        self.assertIn(".stop(join=", blk, "stop 必须带 join")
        self.assertLess(blk.index(".stop(join="), blk.index(".release()"),
                        "join 必须在 release 之前")

    def test_gives_up_instead_of_forcing(self):
        """join 不到就放弃切换、原样继续跑 —— 宁可不切也别把相机弄死"""
        blk = self._blk("_set_cam_res_locked", "set_cam_res")
        i = blk.index("if not old_grab.stop(join=")
        j = blk.index("old_cap.release()")
        self.assertLess(i, j)
        self.assertIn("return False", blk[i:j], "join 超时要直接返回")
        self.assertIn("old_grab.stopped = False", blk[i:j],
                      "放弃时要撤销 stop 让它继续跑")

    def test_grabber_stop_is_joinable_and_read_is_guarded(self):
        i = self.src.index("class FrameGrabber(threading.Thread):")
        j = self.src.index("\ntry:\n    text_type", i)
        blk = self.src[i:j]
        self.assertIn("def stop(self, join=0.0):", blk)
        self.assertIn("self.join(join)", blk)
        self.assertIn("return not self.is_alive()", blk)
        # read 要包 try: cap 被 release 时会抛
        k = blk.index("ok, f = self.cap.read()")
        self.assertIn("try:", blk[max(0, k - 40):k])
        self.assertNotIn("def pause(self)", blk,
                         "pause/resume 是 cap.set 方案的遗留, 该删")

    def test_switch_is_serialised_by_a_lock(self):
        i = self.src.index("def set_cam_res(self, w, h, why):")
        blk = self.src[i:self.src.index("def _preswitch_cam", i)]
        self.assertIn("with self._cam_lock:", blk)
        self.assertIn("self._cam_lock = threading.Lock()", self.src)

    def test_preswitch_during_blind_move(self):
        """挪 140mm 那段是盲走, 切换藏在里面, 到位就直接认灯"""
        i = self.src.index('self._move_next = "yolo"')
        blk = self.src[i:i + 900]
        self.assertIn("threading.Thread(target=self._preswitch_cam)", blk)
        self.assertLess(blk.index("_preswitch_cam"),
                        blk.index("self.start_move("),
                        "预切要在 start_move 之前发出去")
        # start_yolo 拿预切结果, 失败过就别再花一秒重试
        k = self.src.index("def start_yolo(self):")
        sy = self.src[k:k + 1600]
        self.assertIn("pre = self._preswitch_result", sy)
        self.assertIn("if pre is False:", sy)

    def test_crop_only_when_switch_succeeded(self):
        """没切成就别裁: 整帧还是 640x480, 裁 640x352 是把上下切掉一块"""
        blk = self._blk("crop_frame", "step")
        self.assertIn("if not self._cam_switched:", blk)


class YoloChildFdTest(unittest.TestCase):
    """yolo 子进程不能继承相机 fd, 而且 kill_yolo 要先杀进程再切相机。

    实车 2026-08-18 两趟, 1920->640 切回都失败, 而 640->1920 都成功。差别是
    切回时 yolo 子进程还活着: py2 Popen 默认 close_fds=False, 子进程继承了
    相机 fd; 父进程 release 之后内核引用计数不归零, uvcvideo 的 release 不跑,
    stream 所有权挂在那个幽灵 fd 上, 新 open 的 S_FMT 全 EBUSY, 直到 yolo
    退出 —— 日志里"yolo 子进程已退出"正好出现在三次重开全失败之后。
    """

    def test_popen_closes_fds(self):
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "yolo_client.py")) as fh:
            src = fh.read()
        i = src.index("self.p = subprocess.Popen(")
        call = src[i:src.index("self._id = 1", i)]     # 到下一条语句为止
        # 参数可能跨行, 压平再看
        self.assertIn("close_fds=True", " ".join(call.split()),
                      "yolo 子进程必须 close_fds=True, 否则继承相机 fd")

    def test_kill_yolo_closes_child_before_switching_camera(self):
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            src = fh.read()
        i = src.index("def kill_yolo(self):")
        blk = src[i:src.index("def finish_yolo(self", i)]
        self.assertLess(blk.index("self.yolo.close()"),
                        blk.index("self.restore_cam_res()"),
                        "必须先杀子进程再切相机: 子进程还持有 fd 就切不了")

    def test_no_gstreamer_fallback_when_reopening(self):
        """GStreamer 兜底对设备节点永远开不起来, 半死对象析构还会 SIGSEGV"""
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            src = fh.read()
        i = src.index("def _new_cap(self")
        blk = src[i:src.index("def _set_cam_res_locked", i)]
        self.assertIn("cv2.VideoCapture(self.device, cv2.CAP_V4L2)", blk)
        # 不带后端参数的那种 open 一处都不能有
        import re
        bare = re.findall(r"cv2\.VideoCapture\(self\.device\)", blk)
        self.assertEqual(bare, [], "重开时不许退回不带后端的 VideoCapture")


class CornerDefaultTest(unittest.TestCase):
    """白线命中后默认走雷达两面墙闭环(CORNER_ADJUST), 不再盲推 50cm。

    白线**始终**是视觉判的; 变的只是"命中之后怎么进终点"。
    """

    @classmethod
    def setUpClass(cls):
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            cls.src = fh.read()
        cls.root = ET.parse(os.path.join(PACKAGE_ROOT, "launch",
                                         "lane_proto.launch")).getroot()

    def _arg(self, n):
        got = [a.get("default") for a in self.root.iter("arg")
               if a.get("name") == n]
        self.assertEqual(len(got), 1, "%s 应该有且只有一个 arg" % n)
        return got[0]

    def test_use_lidar_defaults_to_self_and_is_a_string(self):
        self.assertEqual(self._arg("use_lidar"), "self")
        p = [x for x in self.root.iter("param") if x.get("name") == "use_lidar"]
        self.assertEqual(len(p), 1)
        self.assertEqual(p[0].get("type"), "str",
                         "三态字符串必须 type=str, 否则 roslaunch 转 bool")

    def test_use_lidar_parser_accepts_self(self):
        i = self.src.index('_ul = gp("~use_lidar", "self")')
        blk = self.src[i:i + 500]
        for word in ('"self"', '"true"', '"lidar"'):
            self.assertIn(word, blk)
        self.assertIn("isinstance(_ul, bool)", blk,
                      "roslaunch 传裸 true/false 会变 bool, 得单独接")

    def test_only_front_wall_still_drives_forward(self):
        i = self.src.index("def step_corner_adjust(self):")
        blk = self.src[i:self.src.index("def estop_cb", i)]
        self.assertIn("partial = ywall is None", blk)
        self.assertIn("(partial or abs(ey) <= tol_p)", blk,
                      "只有前墙时侧向不该卡住到位判定")
        # 退回盲推只看前墙, 侧墙远不算理由(有两处 fallback: 先是"没前墙"
        # 的宽限退回, 再是"前墙太远"的距离退回, 这里查后者)
        k = blk.index('xwall["distance"] > self.corner_fallback_dist')
        self.assertIn("_corner_fallback_to_approach(", blk[k:k + 300])
        self.assertNotIn('ywall["distance"] > self.corner_fallback_dist',
                         blk, "侧墙远不是退回盲推的理由")

    def test_no_wall_falls_back_after_grace_not_30s(self):
        i = self.src.index("def step_corner_adjust(self):")
        blk = self.src[i:self.src.index("def estop_cb", i)]
        self.assertIn("self.corner_no_wall_grace", blk)
        self.assertIn('"%.1fs 内没拟合到前墙"', blk)
        self.assertAlmostEqual(float(self._arg("corner_no_wall_grace")), 1.5,
                               places=3)
        self.assertLess(float(self._arg("corner_no_wall_grace")),
                        float(self._arg("corner_timeout")) / 4.0)

    def test_fallback_dist_covers_a_normal_hit(self):
        """白线在画面底部命中时前墙约 0.8~1.1m, 阈值得留余量"""
        self.assertGreaterEqual(float(self._arg("corner_fallback_dist")), 1.5)


class CornerYawDeadbandTest(unittest.TestCase):
    """corner 的角速度出锁定窗时必须托底到 az_min, 否则卡死。

    实车 2026-08-18: yaw 从 -21.8° 收到 -5.6° 后不动了 —— 锁定窗 5°, 差 0.6°
    锁不上; P 给的 0.078 rad/s 低于底盘转向死区(0.12), 电机不转; 于是永远
    锁不上、永远不平移, 车在终点前干等到超时。
    """

    @classmethod
    def setUpClass(cls):
        import ast as _ast
        import textwrap
        import math as _math
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            src = fh.read()
        i = src.index("        wz = self._corner_clip(self.corner_yaw_kp * yaw_error,")
        j = src.index("        in_target = (abs(ex)", i)
        cls.snippet = textwrap.dedent(src[i:j])
        cls.math = _math

    def _wz(self, yaw_deg, hold_deg=6.0):
        m = self.math

        class S(object):
            corner_yaw_kp = 0.8
            corner_max_yaw_speed = 0.16
            az_min = 0.12

            def _corner_clip(self, v, lim):
                return max(-lim, min(lim, v))

        yaw_error = m.radians(yaw_deg)
        ns = dict(self=S(), yaw_error=yaw_error, math=m,
                  yaw_locked=abs(yaw_error) <= m.radians(hold_deg))
        exec(self.snippet, ns)                                  # noqa: S102
        return ns["wz"], ns["yaw_locked"]

    def test_the_stalled_frame_now_turns(self):
        """就是实车卡死那一帧: yaw=-5.6°, 窗 5° -> 以前 wz=-0.078 卡死"""
        wz, locked = self._wz(-5.6, hold_deg=5.0)
        self.assertFalse(locked)
        self.assertGreaterEqual(abs(wz), 0.12, "出锁定窗就得至少 az_min")
        self.assertLess(wz, 0.0, "方向不能变")

    def test_locked_gives_zero(self):
        for d in (-5.6, -3.0, 0.5):
            wz, locked = self._wz(d, hold_deg=6.0)
            self.assertTrue(locked)
            self.assertEqual(wz, 0.0, "锁定后小抖动别去追, 给 0")

    def test_large_error_still_clipped(self):
        wz, locked = self._wz(-21.8)
        self.assertFalse(locked)
        self.assertAlmostEqual(wz, -0.16, places=6)

    def test_launch_hold_default(self):
        root = ET.parse(os.path.join(PACKAGE_ROOT, "launch",
                                     "lane_proto.launch")).getroot()
        got = [a.get("default") for a in root.iter("arg")
               if a.get("name") == "corner_yaw_hold_deg"]
        self.assertEqual(len(got), 1)
        self.assertGreaterEqual(float(got[0]), 6.0)


class CornerTraceTest(unittest.TestCase):
    """终点雷达闭环要逐帧落盘, append + 每行 flush(Ctrl-C 也不丢)。"""

    @classmethod
    def setUpClass(cls):
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            cls.src = fh.read()

    def test_opens_on_white_line_hit(self):
        i = self.src.index("def start_corner_adjust(self, gy):")
        blk = self.src[i:i + 900]
        self.assertIn("self._corner_trace_open()", blk)
        self.assertIn('self.corner_trace("enter', blk, "入口那一帧也要记")

    def test_append_and_flush_every_line(self):
        i = self.src.index("def _corner_trace_open(self):")
        blk = self.src[i:self.src.index("def _corner_trace_close", i)]
        self.assertIn('open(fn, "a")', blk, "必须 append 模式")
        j = self.src.index("def corner_trace(self")
        body = self.src[j:self.src.index("def _corner_trace_open", j)]
        self.assertIn("self._corner_fh.write(json.dumps(rec)", body)
        w = body.index("self._corner_fh.write(")
        self.assertIn("self._corner_fh.flush()", body[w:w + 200],
                      "写一行必须紧跟 flush 一行")

    def test_every_exit_writes_and_closes(self):
        i = self.src.index("def step_corner_adjust(self):")
        blk = self.src[i:self.src.index("def estop_cb", i)]
        for tag in ('"scan stale"', '"fit fail"', '"fallback: front wall too far"',
                    '"step"', '"stopped"', '"timeout"'):
            self.assertIn(tag, blk, "缺 %s 那一行" % tag)
        self.assertIn('self._corner_trace_close("shutdown")', self.src,
                      "节点关闭要把文件关掉")

    def test_scan_encoding_matches_avoid_trace(self):
        j = self.src.index("def corner_trace(self")
        body = self.src[j:self.src.index("def _corner_trace_open", j)]
        self.assertIn('"mm": [0 if (r != r or r in (float("inf"),', body,
                      "scan.mm 要和 avoid_trace 同一种编码, 离线脚本才能复用")

    def test_launch_default_on(self):
        root = ET.parse(os.path.join(PACKAGE_ROOT, "launch",
                                     "lane_proto.launch")).getroot()
        got = [a.get("default") for a in root.iter("arg")
               if a.get("name") == "corner_trace"]
        self.assertEqual(got, ["true"])
        self.assertIn("corner_trace",
                      [p.get("name") for p in root.iter("param")])


class CornerTranslateDeadbandTest(unittest.TestCase):
    """平移也要托底 + 停滞升级。

    实车 2026-08-18: 转正了、也前进了, 停在 x=0.267(要 <=0.26), cmd 只有
    +0.014m/s, 底盘平移死区吃掉, 1.7cm 永远差着, 稳定计数永远 0/5。
    """

    @classmethod
    def setUpClass(cls):
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            cls.src = fh.read()
        i = cls.src.index("def step_corner_adjust(self):")
        cls.blk = cls.src[i:cls.src.index("def estop_cb", i)]

    def test_translation_floored_at_move_min(self):
        # 合速度托底(方向不变), 进了容差那一项根本不加(= 0, 别来回蹭)
        self.assertIn("elif 0.0 < spd < self.move_min:", self.blk)
        self.assertIn("vx * self.move_min / spd", self.blk)
        self.assertIn("if abs(ex) > tol_p:", self.blk)
        # 平移沿墙法向, 不再按 x_sign/y_sign 轴向
        self.assertIn('xwall["normal"][0]', self.blk)
        self.assertNotIn('fit["x_sign"] * self.corner_kp', self.blk)

    def test_floor_step_fits_inside_tolerance(self):
        """一拍走的距离要小于容差, 否则托底会引起来回振荡"""
        root = ET.parse(os.path.join(PACKAGE_ROOT, "launch",
                                     "lane_proto.launch")).getroot()

        def d(n):
            return float([a.get("default") for a in root.iter("arg")
                          if a.get("name") == n][0])
        lidar_hz = 12.0
        step = d("move_min_speed") / lidar_hz
        self.assertLess(step, d("corner_target_tol"),
                        "move_min/12Hz 一拍 %.1fmm 超过容差 %.0fmm 会振荡"
                        % (1000 * step, 1000 * d("corner_target_tol")))

    def test_stall_escalation_levels(self):
        self.assertIn("self.corner_stall_s", self.blk)
        self.assertIn("self._corner_level = 1", self.blk, "第 1 级: 放宽容差")
        self.assertIn("self._corner_level = 2", self.blk, "第 2 级: 航向不等了")
        self.assertIn("self._corner_yaw_forced = True", self.blk)
        self.assertIn("self._corner_level = 3", self.blk, "第 3 级: 就地结束")
        self.assertIn('"就地结束"', self.blk)
        # 升级后容差要真的用到 in_target 里
        self.assertIn("in_target = (abs(ex) <= tol_p and", self.blk)

    def test_state_reset_on_entry(self):
        i = self.src.index("def start_corner_adjust(self, gy):")
        blk = self.src[i:i + 1200]
        for f in ("_corner_level = 0", "_corner_yaw_forced = False",
                  '_corner_best_prog = float("inf")'):
            self.assertIn(f, blk, "进 corner 要重置 %s" % f)

    def test_launch_default(self):
        root = ET.parse(os.path.join(PACKAGE_ROOT, "launch",
                                     "lane_proto.launch")).getroot()
        got = [a.get("default") for a in root.iter("arg")
               if a.get("name") == "corner_stall_s"]
        self.assertEqual(len(got), 1)
        self.assertAlmostEqual(float(got[0]), 3.0, places=3)


class MapGoalTest(unittest.TestCase):
    """goal_mode=both: 白线 **或** 地图定位(/robot_pose 离 goal_map_xy <
    goal_map_dist)都触发终点; 没 topic / 过期 / 没配坐标 -> 静默 False。
    Pose 和 PoseStamped 都得能收(AnyMsg 按连接头类型反序列化)。"""

    @classmethod
    def setUpClass(cls):
        import math as m
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            cls.src = fh.read()

        def grab(name):
            i = cls.src.index("    def %s(self" % name)
            j = cls.src.index("\n    def ", i + 10)
            body = cls.src[i:j]
            return "\n".join(l[4:] for l in body.split("\n"))
        try:
            from geometry_msgs.msg import Pose, PoseStamped
        except ImportError:                     # 离线机器没 ROS: 用假消息
            import json as _json

            class _V(object):
                def __init__(self, **kw):
                    self.__dict__.update(dict(x=0.0, y=0.0, z=0.0, w=0.0))
                    self.__dict__.update(kw)

            class Pose(object):
                def __init__(self):
                    self.position, self.orientation = _V(), _V()

                def serialize(self, b):
                    b.write(_json.dumps([self.position.__dict__,
                                         self.orientation.__dict__]).encode())

                def deserialize(self, buff):
                    a, b = _json.loads(buff.decode())
                    self.position, self.orientation = _V(**a), _V(**b)
                    return self

            class PoseStamped(object):
                def __init__(self):
                    self.pose = Pose()

                def serialize(self, b):
                    self.pose.serialize(b)

                def deserialize(self, buff):
                    self.pose.deserialize(buff)
                    return self
        cls.Pose, cls.PoseStamped = Pose, PoseStamped
        cls.T = {"now": 100.0}

        class Tm(object):
            @staticmethod
            def time():
                return cls.T["now"]

        class R(object):
            def logwarn(self, *a):
                pass
            loginfo = logwarn
        cls.ns = dict(math=m, time=Tm, rospy=R(), Pose=Pose,
                      PoseStamped=PoseStamped)
        exec(grab("pose_cb"), cls.ns)                          # noqa: S102
        exec(grab("map_goal"), cls.ns)                         # noqa: S102

    def _car(self, mode="both", xy=(2.25, -2.75)):
        class C(object):
            goal_mode = mode
            goal_map_xy = xy
            goal_map_dist = 0.5
            pose_fresh = 1.0
            pose_topic = "/robot_pose"
            _map_pose = None
            _map_pose_t = 0.0
            _map_pose_type = ""
        return C()

    def _anymsg(self, typ, msg):
        from io import BytesIO
        b = BytesIO()
        msg.serialize(b)

        class A(object):
            _connection_header = {"type": typ}
            _buff = b.getvalue()
        return A()

    def test_pose_and_pose_stamped_both_parse(self):
        Pose, PoseStamped = self.Pose, self.PoseStamped
        car = self._car()
        p = Pose()
        p.position.x, p.position.y = 2.0, -2.5
        p.orientation.w = 1.0
        self.ns["pose_cb"](car, self._anymsg("geometry_msgs/Pose", p))
        self.assertAlmostEqual(car._map_pose[0], 2.0)
        ps = PoseStamped()
        ps.pose.position.x, ps.pose.position.y = 1.0, -1.0
        ps.pose.orientation.w = 1.0
        self.ns["pose_cb"](car, self._anymsg("geometry_msgs/PoseStamped", ps))
        self.assertAlmostEqual(car._map_pose[0], 1.0)
        # 别的类型: 不炸, 也不更新
        self.ns["pose_cb"](car, self._anymsg("nav_msgs/Odometry", ps))
        self.assertAlmostEqual(car._map_pose[0], 1.0)

    def test_hit_or_miss_and_silence(self):
        Pose = self.Pose
        car = self._car()
        # 没 topic: 静默 False
        ok, d, why = self.ns["map_goal"](car)
        self.assertFalse(ok)
        self.assertEqual(d, -1.0)
        p = Pose()
        p.position.x, p.position.y = 2.0, -2.5      # 离 (2.25,-2.75) 0.354m
        p.orientation.w = 1.0
        self.T["now"] = 100.0
        self.ns["pose_cb"](car, self._anymsg("geometry_msgs/Pose", p))
        ok, d, why = self.ns["map_goal"](car)
        self.assertTrue(ok)
        self.assertAlmostEqual(d, 0.3536, places=3)
        # 过期
        self.T["now"] = 102.0
        ok, d, why = self.ns["map_goal"](car)
        self.assertFalse(ok)
        self.assertIn("过期", why)
        # visual 模式: 永远 False
        car2 = self._car(mode="visual")
        self.ns["pose_cb"](car2, self._anymsg("geometry_msgs/Pose", p))
        self.assertFalse(self.ns["map_goal"](car2)[0])
        # 没配坐标
        car3 = self._car(xy=None)
        self.assertFalse(self.ns["map_goal"](car3)[0])

    def test_wall_mode_is_gone_and_launch_defaults(self):
        self.assertNotIn("def wall_goal", self.src)
        self.assertIn('rospy.Subscriber(self.pose_topic, rospy.AnyMsg', self.src)
        self.assertIn("if map_hit and not fork_branch:", self.src)
        root = ET.parse(os.path.join(PACKAGE_ROOT, "launch",
                                     "lane_proto.launch")).getroot()
        d = {a.get("name"): a.get("default") for a in root.iter("arg")}
        self.assertEqual(d["goal_mode"], "visual")
        self.assertEqual(d["goal_map_xy"], "")
        self.assertAlmostEqual(float(d["goal_map_dist"]), 0.5)
        self.assertNotIn("goal_wall_dist", d)


class LidarStartTest(unittest.TestCase):
    """board_in_lane:=false + use_lidar:=self 时雷达也得起, 否则 CORNER_ADJUST
    进去就 '/scan 陈旧' 干等(实车 2026-08-19 卡 16s); 没雷达也要过宽限期退回盲推"""

    def test_start_lidar_follows_use_lidar_too(self):
        root = ET.parse(os.path.join(PACKAGE_ROOT, "launch",
                                     "lane_proto.launch")).getroot()
        d = {a.get("name"): a.get("default") for a in root.findall("arg")}
        self.assertIn("use_lidar", d["start_lidar"])
        self.assertIn("board_in_lane", d["start_lidar"])
        for inc in root.iter("include"):
            for a in inc.iter("arg"):
                if a.get("name") == "start_lidar":
                    self.assertEqual(a.get("value"), "$(arg start_lidar)")

    def test_no_scan_falls_back_after_grace(self):
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            src = fh.read()
        i = src.index("def step_corner_adjust(self):")
        blk = src[i:src.index("def estop_cb", i)]
        j = blk.index("if not self.scan_fresh():")
        stale = blk[j:blk.index("msg = self.scan", j)]
        self.assertIn("_corner_fallback_to_approach", stale)
        self.assertIn("corner_no_wall_grace", stale)


class AvoidPoseMergeTest(unittest.TestCase):
    """板前 倒车->原地转正->再前进 三段合成一段 AVOID_POSE: 边转正边沿板法向
    挪到 keep+伸出量(ψ) 的站位; 伸出量 = 半长|cosψ|+半宽|sinψ|, ψ<36.8° 时随
    ψ 单调减, 所以边转边靠近净空只会更富余。"""

    @classmethod
    def setUpClass(cls):
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            cls.src = fh.read()

    def test_target_shrinks_as_car_squares_up(self):
        import math as m
        i = self.src.index("    def _ga_pose_target(self, psi):")
        j = self.src.index("\n    def ", i + 10)
        ns = {"math": m}
        exec("\n".join(l[4:] for l in self.src[i:j].split("\n")), ns)  # noqa

        class C(object):
            ga_keep, car_half_l, car_half_w = 0.20, 0.171, 0.128
        f = ns["_ga_pose_target"]
        self.assertAlmostEqual(f(C(), 0.0), 0.371, places=3)
        t25 = f(C(), m.radians(25.0))
        self.assertGreater(t25, f(C(), m.radians(10.0)))
        self.assertGreater(f(C(), m.radians(10.0)), f(C(), 0.0))
        # 最大就是 hypot(半长,半宽)+keep, 在 36.8°
        self.assertLessEqual(t25, m.hypot(0.171, 0.128) + 0.20 + 1e-9)

    def test_preturn_goes_to_pose_not_turn0(self):
        i = self.src.index("    def start_go_around(self):")
        blk = self.src[i:self.src.index("    def _ga_pose_target", i)]
        self.assertIn('self.set_phase("AVOID_POSE"', blk)
        self.assertIn("self._ga_preturn = False", blk)
        self.assertNotIn("math.hypot(self.car_half_l, self.car_half_w)\n                if self._ga_preturn", blk)
        # 相位表里都得有它
        self.assertIn('"AVOID_POSE", "CORNER_ADJUST")', self.src)
        self.assertEqual(self.src.count('"AVOID_TURN0", "AVOID_POSE"):'), 3)

    def test_pose_step_rotates_and_translates_along_normal(self):
        i = self.src.index('        if self.phase == "AVOID_POSE":\n            g = self.ga_geom()')
        blk = self.src[i:self.src.index('        self._ga_az = 0.0\n        if self.phase == "AVOID_REV":', i)]
        self.assertIn("math.cos(psi)", blk)
        self.assertIn("math.sin(psi)", blk)
        self.assertIn("math.copysign(0.12, az)", blk)
        # ψ 大且净空不够: 先只倒不转
        self.assertIn("math.atan2(self.car_half_w, self.car_half_l)", blk)


class BackByOdomTest(unittest.TestCase):
    """横回段按 odom 回中(go_around_back_src=odom): 起绕时冻结 u0/lat0。
    实车 2026-08-19 avoid_trace_01: board_arc_lat_scale=0, 卡尔曼说回中到
    0.037 收了, 按 odom 推算车心其实还离中垂线 0.106m —— 背面只看得见 0.19m
    的弦, 弦中点当板心偏了。"""

    def test_code_paths(self):
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            src = fh.read()
        self.assertIn('out["lat_odom"] = (self._ga_lat0 +', src)
        self.assertIn('use_odom = self.ga_back_src == "odom" and "lat_odom" in g', src)
        # 卡尔曼跟丢也照常闭环
        self.assertIn('if g is None and self.phase in ("AVOID_FWD", "AVOID_BACK") and', src)
        root = ET.parse(os.path.join(PACKAGE_ROOT, "launch",
                                     "lane_proto.launch")).getroot()
        d = {a.get("name"): a.get("default") for a in root.findall("arg")}
        self.assertEqual(d["go_around_back_src"], "odom")
        self.assertAlmostEqual(float(d["go_around_back_tol"]), 0.02)

    def test_replay_real_trace_shows_kf_bias(self):
        import json
        fn = os.path.join(PACKAGE_ROOT, "test", "data", "avoid_trace_01.jsonl")
        if not os.path.exists(fn):
            self.skipTest("没有实车 trace")
        L = [json.loads(l) for l in open(fn)]
        u0 = L[0]["kf"]["u"]
        p0 = L[0]["odom"][:2]
        lat0 = L[0]["geom"]["lat"]
        last = [r for r in L if r["phase"] == "AVOID_ALIGN"][-1]
        lat_odom = lat0 + (last["odom"][0] - p0[0]) * u0[0] + \
            (last["odom"][1] - p0[1]) * u0[1]
        # 卡尔曼以为 <0.07, odom 说差了 ~0.10 —— 这就是"没绕到正后方"
        self.assertLess(abs(last["geom"]["lat"]), 0.07)
        self.assertGreater(abs(lat_odom), 0.09)
        self.assertLess(abs(lat_odom), 0.13)


class AvoidPoseSettleTest(unittest.TestCase):
    """AVOID_POSE 三项(法向距/航向/离中垂线)都进容差后再稳 go_around_pose_settle
    秒才横移; 对中用正面看的卡尔曼 lat, 沿板面方向 (ux,uy) 挪; 到位后重新冻结
    u0/lat0 给横回用。"""

    def test_structure(self):
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            src = fh.read()
        self.assertNotIn('abs(g["lat"]) <= self.ga_back_tol)', src)
        self.assertIn("done = held >= self.ga_pose_settle", src)
        self.assertIn('def ga_rear_plane(self, g):', src)
        self.assertIn('self._ga_freeze_ref("站位到位")', src)
        self.assertIn('ux=float(ux), uy=float(uy))', src)
        root = ET.parse(os.path.join(PACKAGE_ROOT, "launch",
                                     "lane_proto.launch")).getroot()
        d = {a.get("name"): a.get("default") for a in root.findall("arg")}
        self.assertAlmostEqual(float(d["go_around_pose_settle"]), 1.0)


class FwdByOdomTest(unittest.TestCase):
    """前进段也按 odom 判(车过板后雷达看不见背面, 卡尔曼必丢): 车尾过板面
    tail 才停, 不加 σ; 开环兜底按实测站位/横移后法向距重算。
    实车 2026-08-19 avoid_trace: 板前留 10cm, 板后车尾只剩 2cm。"""

    def test_paths(self):
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            src = fh.read()
        self.assertIn('out["lon_odom"] = (self._ga_lon0 - ddx * self._ga_u0[1] +', src)
        self.assertIn('done = g["lon_odom"] <= tgt', src)
        self.assertIn('self.board_d = self._ga_lon0', src)
        self.assertIn('self.ga_fwd_now = (gg["lon_odom"] + self.car_half_l +', src)

    def test_replay_shows_short_fwd(self):
        import json
        fn = os.path.join(PACKAGE_ROOT, "test", "data", "avoid_trace_02.jsonl")
        if not os.path.exists(fn):
            self.skipTest("没有实车 trace")
        L = [json.loads(l) for l in open(fn)]
        fwd = [r for r in L if r["phase"] == "AVOID_FWD"]
        st = fwd[0]["geom"]["lon"]           # 前进段起点实测法向距
        en = fwd[-1]["geom"]["lon"]          # 终点(卡尔曼开环外推)
        # 名义站位 0.271 但实测 0.319; 结束时板心离车心不到 0.20 -> 车尾 2cm
        self.assertGreater(st, 0.30)
        self.assertGreater(en, -0.22)
        # 按 odom 判 tail=0.1 该走到 lon <= -0.271, 比实际多走 >= 5cm
        self.assertGreater(en - (-0.271), 0.05)


class RearPlaneRecaptureTest(unittest.TestCase):
    """横回段背面重捕获板面(只露一小截弦也能量法向距), 车尾不够 tail 先前进。
    实车 2026-08-19 avoid_trace_02: 前进段末尾雷达 -118°~-94° 整块无回波看不见
    板子; 横回到 lat≈0.23 时背面弦露出, 实测法向距 -0.23(odom 推 -0.26)。"""

    def test_structure(self):
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            src = fh.read()
        self.assertIn('meas = self.ga_rear_plane(gb)', src)
        self.assertIn('self._ga_lon0 += 0.5 * corr', src)
        self.assertIn('先前进补够再横回', src)
        self.assertIn('done = g["lon_odom"] <= tgt and kf_clear', src)
        self.assertIn('self.ga_side_now = max(0.05, self.ga_sign * gg["by_end"] +', src)

    def test_replay_rear_plane(self):
        import json
        import math as m
        import numpy as np
        fn = os.path.join(PACKAGE_ROOT, "test", "data", "avoid_trace_02.jsonl")
        if not os.path.exists(fn):
            self.skipTest("没有实车 trace")
        sys.path.insert(0, os.path.join(PACKAGE_ROOT, "scripts"))
        from lane_common import scan_xy
        with open(os.path.join(PACKAGE_ROOT, "scripts",
                               "lane_follow.py")) as fh:
            src = fh.read()
        i = src.index("    def ga_rear_plane(self, g):")
        j = src.index("    def _ga_freeze_ref", i)
        ns = {"np": np, "math": m, "scan_xy": scan_xy}
        exec("\n".join(l[4:] for l in src[i:j].split("\n")), ns)     # noqa
        L = [json.loads(l) for l in open(fn)]

        class KF(object):
            half = 0.21

        class C(object):
            board_lidar_x, board_yaw_off, board_self_margin = -0.11, 0.0, 0.03
            kf = KF()

        class Msg(object):
            pass
        c = C()
        f = L[39]
        u0, p0, lon0, lat0 = f["kf"]["u"], f["odom"][:2], f["geom"]["lon"], f["geom"]["lat"]
        got = []
        for r in L:
            if r["phase"] != "AVOID_BACK":
                continue
            sc = r["scan"]
            mm = np.array(sc["mm"], float)
            rr = mm / 1000.0
            rr[mm == 0] = np.inf
            msg = Msg()
            msg.ranges, msg.angle_min, msg.angle_increment = rr, sc["amin"], sc["ainc"]
            msg.range_min, msg.range_max = sc["rmin"], sc["rmax"]
            c.scan = msg
            cx, cy, yaw = r["odom"]
            ddx, ddy = cx - p0[0], cy - p0[1]
            cs, sn = m.cos(-yaw), m.sin(-yaw)
            g = dict(lon_odom=lon0 - ddx * u0[1] + ddy * u0[0],
                     lat_odom=lat0 + ddx * u0[0] + ddy * u0[1],
                     lat=r["geom"]["lat"],
                     ux=cs * u0[0] - sn * u0[1], uy=sn * u0[0] + cs * u0[1])
            meas = ns["ga_rear_plane"](c, g)
            if meas is not None:
                got.append(meas)
        self.assertGreater(len(got), 5, "背面一次都没量到")
        for v in got:
            self.assertAlmostEqual(v, -0.23, delta=0.02)
