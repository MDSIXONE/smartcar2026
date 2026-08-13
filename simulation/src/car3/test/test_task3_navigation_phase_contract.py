#!/usr/bin/env python3

import pathlib
import unittest
import xml.etree.ElementTree as ET


PACKAGE_DIR = pathlib.Path(__file__).resolve().parents[1]
WORKSPACE_DIR = PACKAGE_DIR.parents[1]
TASK_SCRIPT = PACKAGE_DIR / "scripts" / "task3_pick_deliver.py"
PREPARE_LAUNCH = PACKAGE_DIR / "launch" / "task3_prepare.launch"
PLANNER_HEADER = (
    WORKSPACE_DIR / "src" / "cym_planner" / "include" / "cym_planner.h"
)
PLANNER_SOURCE = (
    WORKSPACE_DIR / "src" / "cym_planner" / "src" / "cym_planner.cpp"
)
PLANNER_CONFIG = (
    WORKSPACE_DIR / "src" / "cym_planner" / "config" / "cym_planner_params.json"
)
NAV_LAUNCH = WORKSPACE_DIR / "src" / "gazebo_nav" / "launch" / "gazebo_nav.launch"
MOVE_BASE_CONFIG = (
    WORKSPACE_DIR / "src" / "gazebo_nav" / "launch" / "config" / "move_base"
)


class Task3NavigationPhaseContractTest(unittest.TestCase):
    def test_pickup_is_the_only_navigation_upgrade_boundary(self):
        source = TASK_SCRIPT.read_text(encoding="utf-8")
        first_main = source.index('self._set_navigation_mode(\n            "main_legacy"')
        pickup_complete = source.index('"pickup complete: carrying cube to factory"')
        first_laser = source.rfind(
            'self._set_navigation_mode(\n            "laser_avoidance"',
            0,
            pickup_complete,
        )

        self.assertLess(first_main, pickup_complete)
        self.assertGreaterEqual(first_laser, 0)
        self.assertNotIn("simple_follow", source)

    def test_main_legacy_has_a_separate_controller_path(self):
        header = PLANNER_HEADER.read_text(encoding="utf-8")
        source = PLANNER_SOURCE.read_text(encoding="utf-8")

        self.assertIn("computeMainLegacyCommands", header)
        self.assertIn('normalized_mode == "main_legacy"', source)
        self.assertIn(
            "return computeMainLegacyCommands(\n"
            "        cmd_vel, laser_avoidance_enabled_.load());",
            source,
        )
        self.assertIn("main_legacy_linear_x_kd_", source)
        self.assertIn("isCostmapPathBlocked(", source)

    def test_main_legacy_uses_origin_main_control_values(self):
        config = PLANNER_CONFIG.read_text(encoding="utf-8")
        expected_values = (
            '"navigation_mode": "main_legacy"',
            '"main_legacy_target_distance": 0.2',
            '"main_legacy_linear_x_gain": 10.0',
            '"main_legacy_linear_x_kd": 0.05',
            '"main_legacy_angular_gain": 14.0',
            '"main_legacy_max_vel_x": 14.0',
            '"main_legacy_max_vel_theta": 20.5',
            '"main_legacy_final_yaw_gain": 12.0',
            '"main_legacy_final_yaw_max_vel": 10.2',
            '"main_legacy_final_yaw_tolerance": 0.10',
            '"main_legacy_final_linear_x_gain": 1.5',
            '"main_legacy_obstacle_lookahead_distance": 0.8',
            '"main_legacy_obstacle_cost_threshold": 253',
        )
        for expected in expected_values:
            self.assertIn(expected, config)

    def test_post_pickup_uses_laser_vehicle_projection_before_line_following(self):
        header = PLANNER_HEADER.read_text(encoding="utf-8")
        source = PLANNER_SOURCE.read_text(encoding="utf-8")
        config = PLANNER_CONFIG.read_text(encoding="utf-8")

        self.assertIn('"laser_projection_step": 0.03', config)
        self.assertIn("laser_projection_step_", header)
        self.assertIn("checkLaserPathProjection", header)
        self.assertIn(
            "projectionTouchesLaser",
            source,
        )
        self.assertIn(
            "local_x >= footprint_min_x_",
            source,
        )
        self.assertIn(
            "local_x <= footprint_max_x_",
            source,
        )
        self.assertIn(
            "local_y >= footprint_min_y_",
            source,
        )
        self.assertIn(
            "local_y <= footprint_max_y_",
            source,
        )
        self.assertIn(
            "std::ceil(projected_distance / laser_projection_step_)",
            source,
        )
        self.assertIn(
            "STOP: laser touches projected vehicle footprint",
            source,
        )
        main_controller = source[
            source.index("bool CymPlanner::computeMainLegacyCommands"):
            source.index("bool CymPlanner::computeVelocityCommands")
        ]
        self.assertIn("if(use_laser_projection)", main_controller)
        self.assertIn("copyFreshLaserPoints", main_controller)
        self.assertIn("checkLaserPathProjection", main_controller)
        self.assertIn("else\n    {\n", main_controller)
        self.assertIn("isCostmapPathBlocked", main_controller)

    def test_post_pickup_dispatches_before_deprecated_rollout_code(self):
        source = PLANNER_SOURCE.read_text(encoding="utf-8")
        velocity_commands = source[
            source.index("bool CymPlanner::computeVelocityCommands"):
            source.index("bool CymPlanner::isGoalReached")
        ]
        dispatch_index = velocity_commands.index(
            "return computeMainLegacyCommands("
        )
        old_rollout_index = velocity_commands.index(
            "std::vector<LaserPoint> laser_points;"
        )
        self.assertLess(dispatch_index, old_rollout_index)

    def test_costmaps_and_move_base_match_origin_main(self):
        local_common = (
            MOVE_BASE_CONFIG / "local_costmap_common.yaml"
        ).read_text(encoding="utf-8")
        global_common = (
            MOVE_BASE_CONFIG / "global_costmap_common.yaml"
        ).read_text(encoding="utf-8")
        nav_root = ET.parse(NAV_LAUNCH).getroot()
        move_base = nav_root.find("./node[@name='move_base']")
        params = {
            param.get("name"): param.get("value")
            for param in move_base.findall("param")
        }

        self.assertIn(
            "footprint: [[0.18, -0.12], [0.18, 0.12], "
            "[-0.18, 0.12], [-0.18, -0.12]]",
            local_common,
        )
        self.assertIn("inflation_radius: 0.07", local_common)
        self.assertIn("cost_scaling_factor: 4.0", local_common)
        self.assertIn(
            "footprint: [[0.20, -0.14], [0.20, 0.14], "
            "[-0.20, 0.14], [-0.20, -0.14]]",
            global_common,
        )
        self.assertIn("inflation_radius: 0.23", global_common)
        self.assertIn("cost_scaling_factor: 0.05", global_common)
        self.assertEqual(params["planner_frequency"], "0.0")

    def test_prepare_uses_main_style_arm_initialization(self):
        root = ET.parse(PREPARE_LAUNCH).getroot()
        launch_args = {
            arg.get("name"): arg.get("default") for arg in root.findall("arg")
        }
        include = root.find("include")
        include_args = {
            arg.get("name"): arg.get("value") for arg in include.findall("arg")
        }

        self.assertEqual(launch_args["gripper_park_position"], "1.0")
        self.assertEqual(include_args["direct_initial_configuration"], "false")


if __name__ == "__main__":
    unittest.main()
