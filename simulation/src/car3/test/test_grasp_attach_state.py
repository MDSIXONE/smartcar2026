#!/usr/bin/env python3
"""Regression tests for the official r_joint follower and project baseline."""

import importlib.util
import hashlib
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from geometry_msgs.msg import Pose
from std_msgs.msg import Float64


CAR3_DIR = Path(__file__).resolve().parents[1]
SCRIPT = CAR3_DIR / "scripts" / "grasp_attach.py"
CONTROL = CAR3_DIR / "config" / "car3_control.yaml"
GAZEBO_LAUNCH = CAR3_DIR / "launch" / "gazebo.launch"
PLUGIN_DIR = CAR3_DIR.parent / "roboticsgroup_gazebo_plugins" / "src"
URDF = CAR3_DIR / "urdf" / "car3.urdf"
WORLD = CAR3_DIR / "world" / "math.world"


def _content_hash(path):
    normalized = "\n".join(
        path.read_text(encoding="utf-8").rstrip("\r\n").splitlines()
    )
    return hashlib.sha256(normalized.encode("utf-8")).hexdigest()


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "grasp_attach_under_test", SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class GraspAttachStateTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_module()

    def _attachment(self):
        attachment = self.module.GraspAttach.__new__(
            self.module.GraspAttach
        )
        attachment._state_lock = threading.RLock()
        attachment.attachment_enabled = True
        attachment.state = "IDLE"
        attachment.r_joint_pos = None
        attachment.commanded_position = None
        attachment.close_threshold = 0.8
        attachment.open_threshold = 0.89
        attachment.release_calls = 0
        attachment.grasp_calls = 0
        attachment._do_grasp = lambda: setattr(
            attachment, "grasp_calls", attachment.grasp_calls + 1
        )
        attachment._do_release = lambda: setattr(
            attachment, "release_calls", attachment.release_calls + 1
        )
        return attachment

    def test_official_close_feedback_automatically_attaches(self):
        attachment = self._attachment()
        attachment.r_joint_pos = 0.76

        self.module.GraspAttach._tick_state(attachment)

        self.assertEqual(1, attachment.grasp_calls)

    def test_disabled_attachment_ignores_close_feedback(self):
        attachment = self._attachment()
        attachment.attachment_enabled = False
        attachment.r_joint_pos = 0.76

        self.module.GraspAttach._tick_state(attachment)

        self.assertEqual(0, attachment.grasp_calls)

    def test_joint_rebound_cannot_release_commanded_closed_attachment(self):
        attachment = self._attachment()
        attachment.state = "GRASPING"
        attachment.commanded_position = 0.76
        attachment.r_joint_pos = 1.0

        self.module.GraspAttach._tick_state(attachment)

        self.assertEqual(0, attachment.release_calls)

    def test_explicit_open_command_releases_attachment(self):
        attachment = self._attachment()
        attachment.state = "GRASPING"
        attachment.r_joint_pos = 0.76

        self.module.GraspAttach._command_cb(
            attachment, Float64(data=1.0)
        )

        self.assertEqual(1, attachment.release_calls)

    def test_explicit_task_service_uses_official_attachment(self):
        attachment = self._attachment()
        attachment.commanded_position = 0.76
        attachment._do_grasp = lambda: True

        response = self.module.GraspAttach._attach_cb(attachment, None)

        self.assertTrue(response.success)

    def test_disabled_explicit_service_is_rejected(self):
        attachment = self._attachment()
        attachment.attachment_enabled = False
        attachment.commanded_position = 0.76

        response = self.module.GraspAttach._attach_cb(attachment, None)

        self.assertFalse(response.success)

    def test_physical_check_does_not_change_attachment_state(self):
        attachment = self._attachment()
        attachment.commanded_position = 0.76
        attachment.physical_check_half_x = 0.06
        attachment.physical_check_half_y = 0.06
        attachment.physical_check_half_z = 0.08
        attachment._find_closest_in_box = lambda *_args: (
            "cube_0",
            0.0,
            0.0,
            0.0,
            0.0,
            (0.0, 0.0, 0.0, 1.0),
        )

        response = self.module.GraspAttach._check_physical_cb(
            attachment, None
        )

        self.assertTrue(response.success)
        self.assertEqual("IDLE", attachment.state)

    def test_physical_check_box_is_wider_than_attachment_box(self):
        attachment = self._attachment()
        attachment.object_models = ["cube_0"]
        attachment.obj_half_x = 0.02
        attachment.obj_half_y = 0.02
        attachment.obj_half_z = 0.02
        attachment.physical_check_half_x = 0.06
        attachment.physical_check_half_y = 0.06
        attachment.physical_check_half_z = 0.08
        attachment._get_gripper_pose = lambda: object()
        attachment._get_model_offset = lambda *_args: (
            (0.045, 0.0, 0.0),
            0.045,
            (0.0, 0.0, 0.0, 1.0),
        )

        fallback_match = attachment._find_closest_in_box()
        physical_match = attachment._find_closest_in_box(
            attachment.physical_check_half_x,
            attachment.physical_check_half_y,
            attachment.physical_check_half_z,
        )

        self.assertIsNone(fallback_match)
        self.assertIsNotNone(physical_match)

    def _grasp_candidate(self, actual_position):
        attachment = self._attachment()
        attachment.current_object = None
        attachment.offset_pos = None
        attachment.offset_quat = None
        attachment.grasp_success = False
        attachment.r_joint_pos = actual_position
        attachment._follow_timer = None
        attachment.update_rate = 100.0
        attachment._find_closest_in_box = lambda: (
            "cube_0",
            0.01,
            0.0,
            0.0,
            0.01,
            (0.0, 0.0, 0.0, 1.0),
        )
        gripper_pose = Pose()
        gripper_pose.orientation.w = 1.0
        attachment._get_gripper_pose = lambda: gripper_pose
        attachment.gripper_commands = []
        attachment.gripper_pub = SimpleNamespace(
            publish=lambda message: attachment.gripper_commands.append(
                message.data
            )
        )
        return attachment

    def test_official_grasp_starts_100hz_follow_timer(self):
        attachment = self._grasp_candidate(0.76)
        timer = object()

        with patch.object(
            self.module.rospy, "Timer", return_value=timer
        ) as create_timer:
            result = self.module.GraspAttach._do_grasp(attachment)

        self.assertTrue(result)
        self.assertEqual("GRASPING", attachment.state)
        self.assertEqual("cube_0", attachment.current_object)
        self.assertEqual(timer, attachment._follow_timer)
        self.assertEqual([0.76], attachment.gripper_commands)
        create_timer.assert_called_once()

    def test_explicit_early_attach_does_not_echo_stale_open_feedback(self):
        attachment = self._grasp_candidate(1.0)

        with patch.object(
            self.module.rospy, "Timer", return_value=object()
        ):
            result = self.module.GraspAttach._do_grasp(attachment)

        self.assertTrue(result)
        self.assertEqual([], attachment.gripper_commands)

    def test_follow_callback_uses_official_set_model_state(self):
        attachment = self._attachment()
        attachment.state = "GRASPING"
        attachment.current_object = "cube_0"
        attachment.offset_pos = (0.01, 0.0, 0.0)
        attachment.offset_quat = (0.0, 0.0, 0.0, 1.0)
        attachment.gripper_link = "car3::tcp_link"
        gripper_pose = Pose()
        gripper_pose.position.x = 2.0
        gripper_pose.orientation.w = 1.0
        attachment.get_link_srv = lambda *_args: SimpleNamespace(
            success=True,
            link_state=SimpleNamespace(pose=gripper_pose),
        )
        published = []
        attachment.set_model_srv = lambda state: published.append(state)

        self.module.GraspAttach._follow_cb(attachment, None)

        self.assertEqual(1, len(published))
        self.assertEqual("cube_0", published[0].model_name)
        self.assertAlmostEqual(2.01, published[0].pose.position.x)
        self.assertEqual("world", published[0].reference_frame)
        self.assertEqual(0.0, published[0].twist.linear.x)

    def test_release_stops_official_follow_timer(self):
        attachment = self._attachment()
        attachment.state = "GRASPING"
        attachment.current_object = "cube_0"
        attachment.offset_pos = (0.0, 0.0, 0.0)
        attachment.offset_quat = (0.0, 0.0, 0.0, 1.0)
        attachment.grasp_success = True
        stopped = []
        attachment._follow_timer = SimpleNamespace(
            shutdown=lambda: stopped.append(True)
        )

        self.module.GraspAttach._do_release(attachment)

        self.assertEqual([True], stopped)
        self.assertEqual("IDLE", attachment.state)
        self.assertIsNone(attachment.current_object)

    def test_official_configuration_has_no_grasp_system_plugin(self):
        plugin_paths = list(
            PLUGIN_DIR.glob("grasp_*_system_plugin.cpp")
        )
        launch = GAZEBO_LAUNCH.read_text(encoding="utf-8")
        control = CONTROL.read_text(encoding="utf-8")

        self.assertEqual([], plugin_paths)
        self.assertNotIn("extra_gazebo_args", launch)
        self.assertNotIn("gazebo_ros_control:", control)
        self.assertIn("pid: {p: 30.0, i: 0.01, d: 20.0}", control)

    def test_robot_and_world_match_approved_official_resource_baseline(self):
        # The project owner confirmed these repository files as the official
        # baseline.  URDF must never change.  For math.world, only the
        # time-related <physics> parameters are approved for adjustment (see
        # docs/changes); the model part of the world stays locked.  The hash
        # below is the world state after the physics step was set to 0.003 s
        # with max_update_rate 200 (realtime factor ~0.6).
        self.assertEqual(
            "d54efc16a412266712b6661dd60951e9d5d2519864b4782be9959904f2be8d26",
            _content_hash(URDF),
        )
        self.assertEqual(
            "48045b11c21da45803593222ab18297da0222e42de8ae0acf8838016a34b892e",
            _content_hash(WORLD),
        )


if __name__ == "__main__":
    unittest.main()
