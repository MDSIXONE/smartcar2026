#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Regression checks for the interpreter used by lane_follow."""

from __future__ import print_function

import os
import unittest
import xml.etree.ElementTree as ET

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
        self.assertIn("red_template_band2.png", args.get("template", ""))
        self.assertEqual(args.get("start_enabled"), "false")
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
        self.assertIn("red_template_band2.png", args.get("template", ""))
        self.assertEqual(args.get("yellow_target"), "0.90")
        self.assertEqual(args.get("align_offset"), "0.14")
        self.assertEqual(args.get("start_offset"), "0.23")
        self.assertEqual(args.get("goal_y_lo"), "0.85")
        self.assertEqual(args.get("linear_speed"), "0.2")
        self.assertEqual(args.get("gain"), "1.2")
        self.assertEqual(args.get("rate"), "20")
        self.assertEqual(args.get("dump_every"), "3")
        self.assertEqual(args.get("goal_pause"), "1.0")
        self.assertEqual(args.get("use_lidar"), "true")
        self.assertEqual(args.get("board_in_lane"), "true")
        self.assertEqual(args.get("go_around"), "true")
        self.assertEqual(args.get("board_stop_dist"), "0.321")
        self.assertEqual(args.get("go_around_keepout"), "0.15")

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
