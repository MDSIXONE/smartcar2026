#!/usr/bin/env python3
"""Regression tests for portable loading of the official scene-label meshes."""

import importlib.util
from pathlib import Path
import unittest
from xml.etree import ElementTree


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_DIR / "scripts" / "scene_sign_compat.py"
WORLD = PACKAGE_DIR / "world" / "math.world"
GAZEBO_LAUNCH = PACKAGE_DIR / "launch" / "gazebo.launch"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "scene_sign_compat_under_test", SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SceneSignCompatTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = _load_module()

    def test_missing_official_absolute_paths_get_equivalent_overlays(self):
        world_xml = WORLD.read_text(encoding="utf-8")

        overlays = self.module.build_missing_sign_overlays(
            world_xml,
            resource_exists=lambda _path: False,
        )

        self.assertEqual(
            [
                "wall_electronics__resource_compat",
                "wall_daily__resource_compat",
                "wall_food__resource_compat",
            ],
            [overlay.model_name for overlay in overlays],
        )
        expected_meshes = [
            "wall_Electronics.obj",
            "wall_Daily.obj",
            "wall_Food.obj",
        ]
        for overlay, expected_mesh in zip(overlays, expected_meshes):
            sdf = ElementTree.fromstring(overlay.sdf_xml)
            model = sdf.find("model")
            self.assertIsNotNone(model)
            self.assertIsNone(model.find("pose"))
            self.assertEqual(
                "model://sign/meshes/" + expected_mesh,
                model.findtext(".//visual/geometry/mesh/uri"),
            )
            self.assertEqual(overlay.model_name, model.get("name"))

        # Compatibility loading must not rewrite the official world source.
        self.assertEqual(world_xml, WORLD.read_text(encoding="utf-8"))

    def test_resolvable_official_resources_need_no_overlay(self):
        overlays = self.module.build_missing_sign_overlays(
            WORLD.read_text(encoding="utf-8"),
            resource_exists=lambda _path: True,
        )

        self.assertEqual([], overlays)

    def test_gazebo_launch_starts_resource_compatibility_node(self):
        root = ElementTree.parse(GAZEBO_LAUNCH).getroot()
        nodes = {
            node.get("name"): node
            for node in root.findall("node")
        }

        self.assertIn("scene_sign_compat", nodes)
        self.assertEqual(
            "scene_sign_compat.py",
            nodes["scene_sign_compat"].get("type"),
        )
        self.assertEqual(
            "$(find car3)/world/math.world",
            nodes["scene_sign_compat"].find("param[@name='world_path']").get(
                "value"
            ),
        )


if __name__ == "__main__":
    unittest.main()
