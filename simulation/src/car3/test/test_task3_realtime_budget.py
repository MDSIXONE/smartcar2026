#!/usr/bin/env python3

import ast
import pathlib
import unittest
import xml.etree.ElementTree as ET


PACKAGE_DIR = pathlib.Path(__file__).resolve().parents[1]
WORKSPACE_DIR = PACKAGE_DIR.parents[1]
WORLD = PACKAGE_DIR / "world" / "math.world"
URDF = PACKAGE_DIR / "urdf" / "car3.urdf"
PREPARE_LAUNCH = PACKAGE_DIR / "launch" / "task3_prepare.launch"
TASK_SCRIPT = PACKAGE_DIR / "scripts" / "task3_pick_deliver.py"
TASK_CONFIG = PACKAGE_DIR / "config" / "task3_vision.yaml"
PLANNER_CONFIG = (
    WORKSPACE_DIR / "src" / "cym_planner" / "config" / "cym_planner_params.json"
)
PLANNER_SOURCE = WORKSPACE_DIR / "src" / "cym_planner" / "src" / "cym_planner.cpp"
GLOBAL_COSTMAP = (
    WORKSPACE_DIR
    / "src"
    / "gazebo_nav"
    / "launch"
    / "config"
    / "move_base"
    / "global_costmap_common.yaml"
)
LOCAL_COSTMAP = (
    WORKSPACE_DIR
    / "src"
    / "gazebo_nav"
    / "launch"
    / "config"
    / "move_base"
    / "local_costmap_common.yaml"
)


class Task3RealtimeBudgetTest(unittest.TestCase):
    def test_world_uses_200hz_max_update_rate(self):
        # 时间参数（physics 段）由负责人授权可按需调整：物理步长 0.003 s
        # （约 333 Hz），max_update_rate 200 钳制实时率到约 0.6；world 中
        # 的模型部分仍保持官方基线，见 SHA-256 回归。
        root = ET.parse(WORLD).getroot()
        physics = root.find(".//world/physics")
        self.assertIsNotNone(physics)
        self.assertAlmostEqual(float(physics.findtext("max_step_size")), 0.003)
        self.assertAlmostEqual(
            float(physics.findtext("real_time_update_rate")), 200.0
        )
        self.assertAlmostEqual(float(physics.findtext("real_time_factor")), 1.0)
        self.assertEqual(root.findtext(".//world/scene/shadows"), "1")

    def test_robot_keeps_official_sensor_and_controller_settings(self):
        root = ET.parse(URDF).getroot()
        sensors = {
            sensor.get("name"): sensor
            for sensor in root.findall(".//gazebo/sensor")
        }
        rgb = sensors["rgb_camera"]
        depth = sensors["depth_camera"]
        laser = sensors["head_hokuyo_sensor"]
        self.assertEqual(rgb.findtext("update_rate"), "20")
        self.assertEqual(rgb.findtext("visualize"), "true")
        self.assertEqual(depth.findtext("always_on"), "true")
        self.assertEqual(depth.findtext("update_rate"), "20")
        self.assertEqual(
            depth.find("./plugin/alwaysOn").text, "true"
        )
        self.assertAlmostEqual(
            float(laser.findtext("./ray/scan/horizontal/min_angle")), -1.57
        )
        self.assertAlmostEqual(
            float(laser.findtext("./ray/scan/horizontal/max_angle")), 1.57
        )
        planar = root.find(".//gazebo/plugin[@name='planar_controller']")
        self.assertIsNotNone(planar)
        self.assertIsNone(planar.find("cmdTimeout"))

        planner = PLANNER_CONFIG.read_text(encoding="utf-8")
        self.assertIn('"main_legacy_target_distance": 0.2', planner)
        self.assertIn('"main_legacy_max_vel_x": 13.5', planner)
        self.assertIn('"main_legacy_max_vel_theta": 20.5', planner)
        self.assertIn('"laser_projection_step": 0.03', planner)
        self.assertIn('"carry_speed_scale": 1.00', planner)
        planner_source = PLANNER_SOURCE.read_text(encoding="utf-8")
        self.assertIn("Self returns are necessarily inside", planner_source)
        self.assertIn(
            "std::ceil(projected_distance / laser_projection_step_)",
            planner_source,
        )
        self.assertIn(
            "laser touches projected vehicle footprint", planner_source
        )
        global_footprint = next(
            line
            for line in GLOBAL_COSTMAP.read_text(encoding="utf-8").splitlines()
            if line.startswith("footprint:")
        )
        local_footprint = next(
            line
            for line in LOCAL_COSTMAP.read_text(encoding="utf-8").splitlines()
            if line.startswith("footprint:")
        )
        self.assertEqual(
            global_footprint,
            "footprint: [[0.20, -0.14], [0.20, 0.14], "
            "[-0.20, 0.14], [-0.20, -0.14]]",
        )
        self.assertEqual(
            local_footprint,
            "footprint: [[0.18, -0.12], [0.18, 0.12], "
            "[-0.18, 0.12], [-0.18, -0.12]]",
        )

    def test_fast_launch_and_task_have_wall_clock_guards(self):
        launch = ET.parse(PREPARE_LAUNCH).getroot()
        args = {arg.get("name"): arg.get("default") for arg in launch.findall("arg")}
        self.assertEqual(args["gui"], "true")
        self.assertEqual(args["rviz"], "true")

        source = TASK_SCRIPT.read_text(encoding="utf-8")
        self.assertIn("task_wall_budget", source)
        self.assertIn("time.monotonic()", source)
        self.assertIn("observation_pose", source)
        self.assertIn("_observation_pose_reached", source)
        self.assertIn("Odometry", source)
        self.assertIn("RTF preflight", source)
        config = TASK_CONFIG.read_text(encoding="utf-8")
        self.assertIn("task_wall_budget_strict: false", config)

    def test_arm_trajectory_wait_is_simulation_time_based(self):
        source = TASK_SCRIPT.read_text(encoding="utf-8")

        self.assertIn("def _wait_for_sim_duration(", source)
        self.assertIn(
            '_wait_for_sim_duration(duration + 0.20, "moving arm")',
            source,
        )
        self.assertIn("required_sim_duration = duration + 0.20", source)
        self.assertNotIn(
            '_wall_pause(duration + 0.20, "moving arm")',
            source,
        )

    def test_retrying_acquisition_is_not_killed_by_advisory_wall_budget(self):
        source = TASK_SCRIPT.read_text(encoding="utf-8")
        tree = ast.parse(source)
        policy_node = next(
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "wall_budget_should_abort"
        )
        policy_module = ast.Module(body=[policy_node], type_ignores=[])
        ast.fix_missing_locations(policy_module)
        namespace = {}
        exec(compile(policy_module, str(TASK_SCRIPT), "exec"), namespace)
        should_abort = namespace["wall_budget_should_abort"]

        self.assertFalse(should_abort(-1.0, False))
        self.assertTrue(should_abort(-1.0, True))
        self.assertFalse(should_abort(1.0, True))

    def test_rtf_guard_warns_by_default_and_can_be_strict(self):
        source = TASK_SCRIPT.read_text(encoding="utf-8")
        tree = ast.parse(source)
        policy_nodes = [
            node
            for node in tree.body
            if isinstance(node, ast.FunctionDef)
            and node.name
            in {
                "realtime_factor_is_acceptable",
                "realtime_preflight_should_abort",
            }
        ]
        self.assertEqual(
            {node.name for node in policy_nodes},
            {
                "realtime_factor_is_acceptable",
                "realtime_preflight_should_abort",
            },
        )
        guard_module = ast.Module(body=policy_nodes, type_ignores=[])
        ast.fix_missing_locations(guard_module)
        namespace = {}
        exec(compile(guard_module, str(TASK_SCRIPT), "exec"), namespace)
        guard = namespace["realtime_factor_is_acceptable"]
        should_abort = namespace["realtime_preflight_should_abort"]

        self.assertTrue(guard(0.9500, 0.9500, 0.0050))
        self.assertTrue(guard(0.9495, 0.9500, 0.0050))
        self.assertFalse(guard(0.9440, 0.9500, 0.0050))
        self.assertFalse(should_abort(0.9320, 0.9500, 0.0050, False))
        self.assertTrue(should_abort(0.9320, 0.9500, 0.0050, True))
        self.assertFalse(should_abort(0.9500, 0.9500, 0.0050, True))
        self.assertGreaterEqual(
            source.count("realtime_preflight_should_abort("),
            2,
        )
        config = TASK_CONFIG.read_text(encoding="utf-8")
        self.assertIn('rospy.get_param("~rtf_minimum", 0.30)', source)
        self.assertIn("rtf_minimum: 0.30", config)
        self.assertIn("rtf_measurement_tolerance: 0.005", config)
        self.assertIn("rtf_preflight_strict: false", config)

    def test_destination_has_one_long_attempt_until_exact_factory_pose(self):
        source = TASK_SCRIPT.read_text(encoding="utf-8")
        config = TASK_CONFIG.read_text(encoding="utf-8")

        self.assertIn("destination_nav_timeout: 75.0", config)
        self.assertIn("destination_nav_attempts: 1", config)
        self.assertIn("self.destination_nav_timeout", source)
        self.assertIn("self.destination_nav_attempts", source)
        self.assertIn("attempt_timeout=self.destination_nav_timeout", source)
        self.assertIn("attempts=self.destination_nav_attempts", source)
        self.assertIn("destination_position_tolerance: 0.08", config)
        self.assertIn("destination_yaw_tolerance: 0.10", config)
        self.assertIn("destination_pose=True", source)
        self.assertNotIn('"visual", "destination region reached"', source)


if __name__ == "__main__":
    unittest.main()
