#!/usr/bin/env python3
"""Regression tests for recognition and physical-pick recovery loops."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_DIR / "scripts" / "task3_pick_deliver.py"
PREPARE_LAUNCH = PACKAGE_DIR / "launch" / "task3_prepare.launch"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "task3_pick_deliver_retry_under_test", SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Task3RetryStateMachineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_module()

    def test_recognition_restarts_left_upper_right_until_target_is_found(self):
        task = self.module.PickDeliverTask.__new__(
            self.module.PickDeliverTask
        )
        target = {"name": "right", "display_name": "右侧"}
        outcomes = [None, None, target]
        events = []
        task.category = "food"
        task.recognition_retry_delay = 0.25
        task._find_and_align_target_once = (
            lambda: events.append(("scan_cycle",)) or outcomes.pop(0)
        )
        task._status = lambda message: events.append(("status", message))
        task._wall_pause = (
            lambda duration, reason: events.append(
                ("pause", duration, reason)
            )
        )

        result = self.module.PickDeliverTask._find_and_align_target(task)

        self.assertIs(result, target)
        self.assertEqual(
            3,
            sum(event == ("scan_cycle",) for event in events),
        )
        self.assertEqual(
            2,
            sum(
                event[0] == "pause"
                and event[2] == "restarting visual recognition"
                for event in events
            ),
        )

    def test_missing_and_non_target_views_advance_clockwise_in_place(self):
        task = self.module.PickDeliverTask.__new__(
            self.module.PickDeliverTask
        )
        anchor_x = -1.510723
        anchor_y = -0.445771
        task.search_regions = [
            {
                "name": name,
                "display_name": name,
                "observation_goal": [
                    anchor_x,
                    anchor_y,
                    math.pi - index * math.pi / 2.0,
                ],
                "fallback_observation_goal": [
                    anchor_x + index * 0.05,
                    anchor_y - index * 0.03,
                    math.pi - index * math.pi / 2.0,
                ],
            }
            for index, name in enumerate(("left", "upper", "right"))
        ]
        task.category = "food"
        target = task.search_regions[2]
        outcomes = [None, None, target]
        attempts = []
        statuses = []
        task._search_target_at_observation = (
            lambda region, goal, phase, index:
            attempts.append((region["name"], list(goal), phase, index))
            or outcomes.pop(0)
        )
        task._status = statuses.append

        result = task._find_and_align_target_once()

        self.assertEqual("right", result["name"])
        self.assertEqual(3, len(attempts))
        for _name, goal, phase, _index in attempts:
            self.assertAlmostEqual(anchor_x, goal[0])
            self.assertAlmostEqual(anchor_y, goal[1])
            self.assertEqual("旋转搜索", phase)
        self.assertAlmostEqual(
            -math.pi / 2.0,
            attempts[1][1][2] - attempts[0][1][2],
        )
        self.assertAlmostEqual(
            -math.pi / 2.0,
            attempts[2][1][2] - attempts[1][1][2],
        )
        self.assertTrue(
            any("顺时针旋转90度" in message for message in statuses)
        )

    def test_rotation_miss_falls_back_to_previous_three_observation_poses(self):
        task = self.module.PickDeliverTask.__new__(
            self.module.PickDeliverTask
        )
        primary = [
            [-1.510723, -0.445771, math.pi - index * math.pi / 2.0]
            for index in range(3)
        ]
        fallback = [
            [-1.510723, -0.445771, 3.141157],
            [-1.395771, -0.419277, 1.570361],
            [-1.359277, -0.524229, -0.000436],
        ]
        task.search_regions = [
            {
                "name": name,
                "display_name": name,
                "observation_goal": primary[index],
                "fallback_observation_goal": fallback[index],
            }
            for index, name in enumerate(("left", "upper", "right"))
        ]
        task.category = "food"
        target = task.search_regions[1]
        outcomes = [None, None, None, None, target]
        attempts = []
        statuses = []
        task._search_target_at_observation = (
            lambda region, goal, phase, index:
            attempts.append((region["name"], list(goal), phase, index))
            or outcomes.pop(0)
        )
        task._status = statuses.append

        result = task._find_and_align_target_once()

        self.assertIs(target, result)
        self.assertEqual(
            [entry[1] for entry in attempts],
            primary + fallback[:2],
        )
        self.assertEqual(
            [entry[2] for entry in attempts],
            ["旋转搜索"] * 3 + ["旧版观察位复查"] * 2,
        )
        self.assertTrue(
            any("切换旧版左/中/右观察位复查" in message for message in statuses)
        )

    def test_terminal_progress_messages_cover_each_recognition_step(self):
        source = SCRIPT.read_text(encoding="utf-8")

        self.assertIn("到达夹取区", source)
        self.assertIn("开始识别第%s个物块", source)
        self.assertIn("不是目标", source)
        self.assertIn("顺时针旋转90度", source)
        self.assertIn("切换旧版左/中/右观察位复查", source)

    def test_failed_pick_recovers_then_reacquires_before_delivery(self):
        task = self.module.PickDeliverTask.__new__(
            self.module.PickDeliverTask
        )
        region = {"name": "upper", "display_name": "上方"}
        pick_results = [False, True]
        events = []
        task.cargo_name = "苹果"
        task.grasp_retry_delay = 0.30
        task._find_and_align_target = (
            lambda: events.append(("recognise",)) or region
        )
        task._start_arm_control = lambda: events.append(("arm_start",))
        task._pick = (
            lambda: events.append(("pick",)) or pick_results.pop(0)
        )
        task._recover_after_failed_pick = (
            lambda: events.append(("recover",))
        )
        task._status = lambda message: events.append(("status", message))
        task._wall_pause = (
            lambda duration, reason: events.append(
                ("pause", duration, reason)
            )
        )

        result = self.module.PickDeliverTask._acquire_cargo(task)

        self.assertIs(result, region)
        self.assertEqual(2, events.count(("recognise",)))
        self.assertEqual(2, events.count(("pick",)))
        self.assertEqual(2, events.count(("arm_start",)))
        self.assertEqual(1, events.count(("recover",)))
        self.assertLess(events.index(("recover",)), events.index(("recognise",), 1))

    def test_failed_pick_recovery_opens_gripper_and_restores_initial_arm_pose(self):
        task = self.module.PickDeliverTask.__new__(
            self.module.PickDeliverTask
        )
        task.arm_initial = [0.0, -0.5, 1.28, 1.7, 0.0]
        task.arm_recovery_duration = 2.5
        events = []
        task.nav = SimpleNamespace(
            cancel_all_goals=lambda: events.append(("cancel_nav",))
        )
        task.cmd_pub = SimpleNamespace(
            publish=lambda _message: events.append(("stop_base",))
        )
        task.carry_mode_pub = SimpleNamespace(
            publish=lambda message: events.append(("carry", message.data))
        )
        task._select_cmd_vel_source = (
            lambda source, reason: events.append(("source", source))
        )
        task._set_navigation_mode = (
            lambda mode, reason: events.append(("navigation", mode))
        )
        task._set_gripper = (
            lambda position: events.append(("gripper", position))
        )
        task._move_arm = (
            lambda positions, duration: events.append(
                ("arm", list(positions), duration)
            )
        )
        task._stop_arm_control = lambda: events.append(("arm_stop",))
        task._status = lambda message: events.append(("status", message))

        self.module.PickDeliverTask._recover_after_failed_pick(task)

        self.assertIn(("gripper", 1.0), events)
        self.assertIn(("arm", task.arm_initial, 2.5), events)
        self.assertIn(("arm_stop",), events)
        self.assertIn(("navigation", "main_legacy"), events)
        self.assertIn(("carry", False), events)
        self.assertLess(
            events.index(("gripper", 1.0)),
            events.index(("arm", task.arm_initial, 2.5)),
        )
        self.assertLess(
            events.index(("arm", task.arm_initial, 2.5)),
            events.index(("arm_stop",)),
        )

    def test_recovery_reads_the_exact_prepare_arm_start_pose(self):
        root = ET.parse(PREPARE_LAUNCH).getroot()
        shared_pose = next(
            param
            for param in root.findall("param")
            if param.get("name") == "/sim_task3/arm_initial_pose"
        )
        source = SCRIPT.read_text(encoding="utf-8")

        self.assertEqual("$(arg arm_start_pose)", shared_pose.get("value"))
        self.assertIn('"/sim_task3/arm_initial_pose"', source)


if __name__ == "__main__":
    unittest.main()
