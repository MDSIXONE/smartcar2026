#!/usr/bin/env python3
"""Regression test for the arm fold-to-carry pose completing before the
base starts moving after official attachment."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_DIR / "scripts" / "task3_pick_deliver.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "task3_pick_deliver_carry_sequence_under_test", SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Task3CarrySequenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_module()

    def _task(self):
        task = self.module.PickDeliverTask.__new__(
            self.module.PickDeliverTask
        )
        task.arm_grasp = [0.0] * 5
        task.arm_carry = [10.0] * 5
        task.arm_grasp_duration = 2.0
        task.arm_carry_duration = 2.5
        task.physical_grasp_settle = 0.25
        task.attachment_fallback_enabled = True
        task.events = []
        task._status = (
            lambda message: task.events.append(("status", message))
        )
        task._set_gripper = (
            lambda position: task.events.append(("gripper", position))
        )
        task._wait_for_grasp_state = lambda *_args: True
        task._wall_pause = lambda *_args: None
        task._move_arm = (
            lambda positions, duration, guard_wrist_rotation=False:
            task.events.append(
                (
                    "blocking_arm",
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
        task._set_cargo_to_tcp = (
            lambda: task.events.append(("align",))
        )
        task._request_fallback_attachment = (
            lambda: task.events.append(("attach",)) or True
        )
        task._set_navigation_mode = (
            lambda mode, _reason:
            task.events.append(("navigation", mode))
        )
        task.clear_costmaps = (
            lambda: task.events.append(("clear_costmaps",))
        )
        task.carry_mode_pub = SimpleNamespace(
            publish=lambda message:
            task.events.append(("carry", message.data))
        )
        return task

    def test_attachment_finishes_blocking_fold_before_navigation(self):
        task = self._task()

        result = self.module.PickDeliverTask._pick(task)

        self.assertTrue(result)
        self.assertIn(
            ("blocking_arm", [10.0] * 5, 2.5, False),
            task.events,
        )
        self.assertNotIn(
            ("async_arm", [10.0] * 5, 2.5),
            task.events,
        )
        self.assertLess(
            task.events.index(("attach",)),
            task.events.index(("blocking_arm", [10.0] * 5, 2.5, False)),
        )
        self.assertLess(
            task.events.index(("blocking_arm", [10.0] * 5, 2.5, False)),
            task.events.index(("clear_costmaps",)),
        )
        self.assertLess(
            task.events.index(("clear_costmaps",)),
            task.events.index(("navigation", "laser_avoidance")),
        )
        self.assertLess(
            task.events.index(("navigation", "laser_avoidance")),
            task.events.index(("carry", True)),
        )
        statuses = [
            event[1]
            for event in task.events
            if event[0] == "status"
        ]
        self.assertTrue(
            any(
                "先恢复携带姿势，再开始底盘导航" in message
                for message in statuses
            )
        )


if __name__ == "__main__":
    unittest.main()
