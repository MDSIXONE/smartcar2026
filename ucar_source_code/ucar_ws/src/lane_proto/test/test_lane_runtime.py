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
