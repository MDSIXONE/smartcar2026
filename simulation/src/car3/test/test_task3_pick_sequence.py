#!/usr/bin/env python3
"""Regression tests for task-three pickup sequencing."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_DIR / "scripts" / "task3_pick_deliver.py"
EXECUTE_LAUNCH = PACKAGE_DIR / "launch" / "task3_execute.launch"
PREPARE_LAUNCH = PACKAGE_DIR / "launch" / "task3_prepare.launch"
GAZEBO_LAUNCH = PACKAGE_DIR / "launch" / "gazebo.launch"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "task3_pick_deliver_pick_under_test", SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Task3PickSequenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_module()

    def _task(self, physical_success, fallback_enabled=False):
        task = self.module.PickDeliverTask.__new__(self.module.PickDeliverTask)
        task.arm_grasp = [0.0] * 5
        task.arm_carry = [10.0] * 5
        task.arm_grasp_duration = 2.0
        task.arm_carry_duration = 2.5
        task.physical_grasp_settle = 0.25
        task.attachment_fallback_enabled = fallback_enabled
        task.events = []
        task._status = lambda message: task.events.append(("status", message))
        task._set_gripper = (
            lambda position: task.events.append(("gripper", position))
        )
        task._wait_for_grasp_state = lambda *_args: True
        task._wall_pause = (
            lambda duration, reason: task.events.append(
                ("pause", duration, reason)
            )
        )
        task._move_arm = (
            lambda positions, duration, guard_wrist_rotation=False:
            task.events.append(
                (
                    "arm",
                    list(positions),
                    duration,
                    guard_wrist_rotation,
                )
            )
        )
        task._publish_arm_target = (
            lambda positions, duration:
            task.events.append(
                ("async_arm", list(positions), duration)
            )
        )
        task._physical_grasp_succeeded = (
            lambda: task.events.append(("physical_check",)) or physical_success
        )
        task._set_cargo_to_tcp = (
            lambda: task.events.append(("align_fallback",))
        )
        task._request_fallback_attachment = (
            lambda: task.events.append(("attach_fallback",)) or True
        )
        task._set_navigation_mode = (
            lambda mode, reason: task.events.append(("navigation", mode))
        )
        task.clear_costmaps = (
            lambda: task.events.append(("clear_costmaps",))
        )
        task.carry_mode_pub = SimpleNamespace(
            publish=lambda message: task.events.append(
                ("carry", message.data)
            )
        )
        return task

    def test_enabled_attachment_closes_once_without_physical_probe(self):
        task = self._task(
            physical_success=True,
            fallback_enabled=True,
        )

        result = self.module.PickDeliverTask._pick(task)

        self.assertTrue(result)
        self.assertEqual(
            [("gripper", 1.0), ("gripper", 0.76)],
            [event for event in task.events if event[0] == "gripper"],
        )
        self.assertNotIn(("physical_check",), task.events)
        self.assertEqual(
            [
                ("arm", [0.0] * 5, 2.0, True),
                ("arm", [10.0] * 5, 2.5, False),
            ],
            [event for event in task.events if event[0] == "arm"],
        )
        self.assertNotIn(
            ("async_arm", [10.0] * 5, 2.5),
            task.events,
        )

    def test_enabled_attachment_aligns_before_the_only_close(self):
        task = self._task(
            physical_success=False,
            fallback_enabled=True,
        )

        result = self.module.PickDeliverTask._pick(task)

        self.assertTrue(result)
        self.assertLess(
            task.events.index(("align_fallback",)),
            task.events.index(("gripper", 0.76)),
        )
        self.assertLess(
            task.events.index(("gripper", 0.76)),
            task.events.index(("attach_fallback",)),
        )
        self.assertNotIn(("physical_check",), task.events)

    def test_failed_official_attachment_retries_without_carry_mode(self):
        task = self._task(
            physical_success=False,
            fallback_enabled=True,
        )
        task._request_fallback_attachment = (
            lambda: task.events.append(("attach_fallback",)) or False
        )

        result = self.module.PickDeliverTask._pick(task)

        self.assertFalse(result)
        self.assertNotIn(("physical_check",), task.events)
        self.assertNotIn(("navigation", "laser_avoidance"), task.events)
        self.assertNotIn(("carry", True), task.events)
        self.assertFalse(hasattr(task, "fallback_hold_timer"))

    def test_disabled_attachment_uses_direct_reference_pick_sequence(self):
        task = self._task(
            physical_success=True,
            fallback_enabled=False,
        )

        result = self.module.PickDeliverTask._pick(task)

        self.assertTrue(result)
        self.assertNotIn(("attach_fallback",), task.events)
        self.assertNotIn(("align_fallback",), task.events)
        self.assertIn(("physical_check",), task.events)
        self.assertEqual(
            [
                ("arm", [0.0] * 5, 2.0, True),
                ("arm", [10.0] * 5, 2.5, False),
            ],
            [event for event in task.events if event[0] == "arm"],
        )

    def test_disabled_attachment_reports_failed_physical_pick_for_retry(self):
        task = self._task(
            physical_success=False,
            fallback_enabled=False,
        )

        result = self.module.PickDeliverTask._pick(task)

        self.assertFalse(result)
        self.assertIn(("physical_check",), task.events)
        self.assertNotIn(("attach_fallback",), task.events)
        self.assertNotIn(("navigation", "laser_avoidance"), task.events)
        self.assertNotIn(("carry", True), task.events)

    def test_clears_grasp_phase_laser_ghost_before_carry_navigation(self):
        task = self._task(
            physical_success=True,
            fallback_enabled=True,
        )

        result = self.module.PickDeliverTask._pick(task)

        self.assertTrue(result)
        carry_pose = ("arm", [10.0] * 5, 2.5, False)
        self.assertIn(("clear_costmaps",), task.events)
        self.assertLess(
            task.events.index(carry_pose),
            task.events.index(("clear_costmaps",)),
        )
        self.assertLess(
            task.events.index(("clear_costmaps",)),
            task.events.index(("navigation", "laser_avoidance")),
        )

    def test_launches_enable_official_attachment_by_default(self):
        execute_root = ET.parse(EXECUTE_LAUNCH).getroot()
        prepare_root = ET.parse(PREPARE_LAUNCH).getroot()
        gazebo_root = ET.parse(GAZEBO_LAUNCH).getroot()

        execute_args = {
            arg.get("name"): arg.get("default")
            for arg in execute_root.findall("arg")
        }
        prepare_args = {
            arg.get("name"): arg.get("default")
            for arg in prepare_root.findall("arg")
        }
        gazebo_args = {
            arg.get("name"): arg.get("default")
            for arg in gazebo_root.findall("arg")
        }

        self.assertEqual(
            "true", execute_args["attachment_fallback_enabled"]
        )
        self.assertEqual(
            "true", prepare_args["grasp_attachment_enabled"]
        )
        self.assertEqual(
            "true", gazebo_args["grasp_attachment_enabled"]
        )

    def test_wrist_guard_rejects_collision_rotation_not_small_tracking_error(self):
        check = self.module.PickDeliverTask._wrist_rotation_is_safe

        self.assertTrue(check(-0.02, 0.0, 0.035))
        self.assertFalse(check(-0.12, 0.0, 0.035))
        self.assertTrue(check(-3.13, 3.13, 0.035))

    def test_arm_publisher_uses_nearest_equivalent_multiturn_targets(self):
        task = self.module.PickDeliverTask.__new__(
            self.module.PickDeliverTask
        )
        published = []
        task.arm_pub = SimpleNamespace(
            publish=lambda message: published.append(message)
        )
        task._wall_pause = lambda *_args: None
        current_positions = [
            2.0 * math.pi + 0.2,
            -2.0 * math.pi - 0.5,
            4.0 * math.pi + 1.28,
            -4.0 * math.pi + 1.7,
            2.0 * math.pi,
        ]
        task._latest_arm_joint_sample = lambda: (1, current_positions)
        desired_positions = [0.3, 1.5, 0.28, 1.3, 0.0]

        self.module.PickDeliverTask._publish_arm_target(
            task, desired_positions, 0.5
        )

        self.assertEqual(3, len(published))
        for current, actual, desired in zip(
            current_positions,
            published[-1].points[0].positions,
            desired_positions,
        ):
            self.assertAlmostEqual(
                0.0,
                math.atan2(
                    math.sin(actual - desired),
                    math.cos(actual - desired),
                ),
                places=6,
            )
            self.assertLessEqual(abs(actual - current), math.pi)

    def test_unguarded_arm_motion_waits_for_simulation_time(self):
        task = self.module.PickDeliverTask.__new__(
            self.module.PickDeliverTask
        )
        events = []
        task._publish_arm_target = (
            lambda positions, duration:
            events.append(("publish", list(positions), duration))
        )
        task._wait_for_sim_duration = (
            lambda duration, context:
            events.append(("sim_wait", duration, context))
        )
        task._wall_pause = (
            lambda duration, context:
            events.append(("wall_wait", duration, context))
        )

        result = self.module.PickDeliverTask._move_arm(
            task, [0.0] * 5, 2.0
        )

        self.assertTrue(result)
        self.assertIn(("sim_wait", 2.2, "moving arm"), events)
        self.assertFalse(
            any(event[0] == "wall_wait" for event in events)
        )


if __name__ == "__main__":
    unittest.main()
