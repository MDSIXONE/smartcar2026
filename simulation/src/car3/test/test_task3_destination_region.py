#!/usr/bin/env python3
"""Regression tests for completing at the requested factory pose."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import unittest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
TASK_SCRIPT = PACKAGE_DIR / "scripts" / "task3_pick_deliver.py"


class Task3DestinationRegionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location(
            "task3_pick_deliver_destination_test", TASK_SCRIPT
        )
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def _food_task_at(self, x, y, yaw):
        task = self.module.PickDeliverTask.__new__(
            self.module.PickDeliverTask
        )
        task.destination_position_tolerance = 0.05
        task.destination_yaw_tolerance = 0.10
        task.latest_odom = SimpleNamespace(
            pose=SimpleNamespace(
                pose=SimpleNamespace(
                    position=SimpleNamespace(x=x, y=y),
                    orientation=SimpleNamespace(
                        x=0.0,
                        y=0.0,
                        z=math.sin(yaw / 2.0),
                        w=math.cos(yaw / 2.0),
                    ),
                )
            )
        )
        return task

    def test_factory_regions_match_the_blue_markings(self):
        self.assertEqual(
            self.module.WAREHOUSE_REGIONS["food"],
            (0.75, 1.25, -3.23, -2.73),
        )
        self.assertEqual(
            self.module.WAREHOUSE_REGIONS["daily"],
            (0.75, 1.25, -1.75, -1.25),
        )
        self.assertEqual(
            self.module.WAREHOUSE_REGIONS["electronics"],
            (2.30, 2.80, -2.47, -1.97),
        )

    def test_pose_near_requested_point_and_yaw_is_delivered(self):
        goal = self.module.WAREHOUSES["food"][1]
        task = self._food_task_at(1.03, -2.99, -math.pi / 2.0 + 0.04)
        self.assertTrue(task._destination_pose_reached(goal))

    def test_old_rectangle_entry_pose_is_not_delivered_early(self):
        goal = self.module.WAREHOUSES["food"][1]
        task = self._food_task_at(0.8177, -2.7484, 0.0)
        self.assertFalse(task._destination_pose_reached(goal))

    def test_wrong_yaw_at_target_point_is_not_delivered(self):
        goal = self.module.WAREHOUSES["food"][1]
        task = self._food_task_at(goal[0], goal[1], 0.0)
        self.assertFalse(task._destination_pose_reached(goal))

    def test_delivery_requires_final_position_and_yaw_after_pickup(self):
        source = TASK_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("def _destination_pose_reached", source)
        self.assertIn(
            "destination_pose=True",
            source,
        )
        self.assertIn(
            "Destination pose reached within",
            source,
        )
        self.assertNotIn('"visual", "destination region reached"', source)


if __name__ == "__main__":
    unittest.main()
