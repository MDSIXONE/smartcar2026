#!/usr/bin/env python3
"""Regression contract for a single Gazebo /cmd_vel publisher."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE_DIR.parent
GAZEBO_LAUNCH = PACKAGE_DIR / "launch" / "gazebo.launch"
NAV_LAUNCH = WORKSPACE_SRC / "gazebo_nav" / "launch" / "gazebo_nav.launch"
TASK_SCRIPT = PACKAGE_DIR / "scripts" / "task3_pick_deliver.py"
ARBITER_SCRIPT = PACKAGE_DIR / "scripts" / "cmd_vel_arbiter.py"
CMAKE = PACKAGE_DIR / "CMakeLists.txt"


class Task3CmdVelArbiterTest(unittest.TestCase):
    def test_gazebo_receives_commands_from_one_arbiter(self):
        launch = ET.parse(GAZEBO_LAUNCH).getroot()
        arbiter = next(
            node
            for node in launch.findall("node")
            if node.get("name") == "cmd_vel_arbiter"
        )
        self.assertEqual(arbiter.get("pkg"), "car3")
        self.assertEqual(arbiter.get("type"), "cmd_vel_arbiter.py")

        source = ARBITER_SCRIPT.read_text(encoding="utf-8")
        self.assertIn('Publisher("/cmd_vel"', source)
        self.assertIn('"/sim_task3/navigation_cmd_vel"', source)
        self.assertIn('"/sim_task3/visual_cmd_vel"', source)
        self.assertIn('"/sim_task3/cmd_vel_source"', source)

    def test_navigation_and_visual_nodes_do_not_publish_cmd_vel_directly(self):
        nav_launch = ET.parse(NAV_LAUNCH).getroot()
        move_base = next(
            node
            for node in nav_launch.findall("node")
            if node.get("name") == "move_base"
        )
        remaps = {
            remap.get("from"): remap.get("to")
            for remap in move_base.findall("remap")
        }
        self.assertEqual(
            remaps.get("cmd_vel"), "/sim_task3/navigation_cmd_vel"
        )

        task_source = TASK_SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn('Publisher("/cmd_vel"', task_source)
        self.assertIn('"/sim_task3/visual_cmd_vel"', task_source)
        self.assertIn('"/sim_task3/cmd_vel_source"', task_source)

    def test_arbiter_is_installed_with_the_package(self):
        cmake = CMAKE.read_text(encoding="utf-8")
        self.assertIn("scripts/cmd_vel_arbiter.py", cmake)


if __name__ == "__main__":
    unittest.main()
