#!/usr/bin/env python3

import pathlib
import sys
import types
import unittest


BRIDGE_DIR = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(BRIDGE_DIR))
sys.modules.setdefault("rospy", types.ModuleType("rospy"))

import sim_bridge


class Properties:
    time_step = 0.001
    max_update_rate = 1000.0
    gravity = object()
    ode_config = object()


class PhysicsRateControllerTest(unittest.TestCase):
    def test_restores_the_rate_read_during_initialization(self):
        controller = sim_bridge.GazeboPhysicsRateController(100.0)
        properties = Properties()
        calls = []
        controller._connect = lambda: None
        controller._get_physics = lambda: properties
        controller._set_physics = lambda *args: calls.append(args) or types.SimpleNamespace(
            success=True, status_message=""
        )

        controller.reduce_for_idle()
        controller.restore_for_task()

        self.assertEqual(calls[0][1], 100.0)
        self.assertEqual(calls[1][1], 1000.0)
        self.assertIs(controller.original_properties, properties)


if __name__ == "__main__":
    unittest.main()
