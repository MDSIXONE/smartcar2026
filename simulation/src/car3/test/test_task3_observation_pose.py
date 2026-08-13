#!/usr/bin/env python3
"""Regression tests for left/upper/right camera observation rotations."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
TASK_SCRIPT = PACKAGE_DIR / "scripts" / "task3_pick_deliver.py"


def _load_task_module():
    spec = importlib.util.spec_from_file_location(
        "task3_pick_deliver_observation_test", TASK_SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Task3ObservationPoseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_task_module()

    def _task_at(self, x, y, yaw):
        quaternion = self.module.transformations.quaternion_from_euler(
            0.0, 0.0, yaw
        )
        task = self.module.PickDeliverTask.__new__(self.module.PickDeliverTask)
        task.observation_position_tolerance = 0.12
        task.observation_yaw_tolerance = 0.12
        task.latest_odom = SimpleNamespace(
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=x, y=y),
                    orientation=SimpleNamespace(
                        x=quaternion[0],
                        y=quaternion[1],
                        z=quaternion[2],
                        w=quaternion[3],
                    ),
                )
            )
        )
        return task

    def test_position_without_camera_yaw_is_not_an_observation_pose(self):
        task = self._task_at(-1.3958, -0.4193, math.pi)
        upper_goal = [-1.3958, -0.4193, math.pi / 2.0]
        self.assertFalse(task._observation_pose_reached(upper_goal))

    def test_position_and_camera_yaw_reach_observation_pose(self):
        task = self._task_at(-1.3958, -0.4193, math.pi / 2.0 + 0.04)
        upper_goal = [-1.3958, -0.4193, math.pi / 2.0]
        self.assertTrue(task._observation_pose_reached(upper_goal))

    def test_yaw_comparison_wraps_across_pi(self):
        task = self._task_at(-1.5107, -0.4458, -math.pi + 0.03)
        left_goal = [-1.5107, -0.4458, math.pi]
        self.assertTrue(task._observation_pose_reached(left_goal))

    def test_visual_search_uses_the_complete_observation_pose(self):
        source = TASK_SCRIPT.read_text(encoding="utf-8")
        search_body = source[
            source.index("def _search_target_at_observation"):
            source.index("def _observation_pose_reached")
        ]
        move_body = source[
            source.index("def _move_base"):
            source.index("def _start_arm_control")
        ]
        self.assertIn("observation_pose=True", search_body)
        self.assertIn(
            "observation_pose and self._observation_pose_reached(goal)",
            move_body,
        )

    def test_search_configuration_rejects_translation_between_views(self):
        acceptance = {
            "center_x": [0.47, 0.535],
            "center_y": [0.745, 0.790],
            "width": [0.065, 0.115],
            "height": [0.120, 0.175],
        }
        regions = [
            {
                "name": name,
                "display_name": name,
                "observation_goal": [-1.5, -0.45, math.pi - index * math.pi / 2.0],
                "fallback_observation_goal": [
                    -1.5 + index * 0.05,
                    -0.45 - index * 0.03,
                    math.pi - index * math.pi / 2.0,
                ],
                "grasp_target": [0.5, 0.757, 0.085, 0.146],
                "grasp_acceptance": acceptance,
            }
            for index, name in enumerate(("left", "upper", "right"))
        ]
        parsed = self.module.PickDeliverTask._read_search_regions(regions)
        self.assertEqual(["left", "upper", "right"], [
            region["name"] for region in parsed
        ])

        regions[1]["observation_goal"][0] += 0.01
        with self.assertRaisesRegex(
            self.module.rospy.ROSException,
            "keep one XY and turn clockwise 90 degrees",
        ):
            self.module.PickDeliverTask._read_search_regions(regions)


if __name__ == "__main__":
    unittest.main()
