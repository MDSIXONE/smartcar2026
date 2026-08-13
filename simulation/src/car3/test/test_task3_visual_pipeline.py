#!/usr/bin/env python3
"""Regression tests for camera-only task-three pickup selection."""

import importlib.util
from pathlib import Path
import threading
import time
import unittest

import numpy as np
import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
TASK_SCRIPT = PACKAGE_DIR / "scripts" / "task3_pick_deliver.py"
VISION_CONFIG = PACKAGE_DIR / "config" / "task3_vision.yaml"
RVIZ_CONFIG = PACKAGE_DIR / "rviz" / "navigation.rviz"
VISION_DIR = PACKAGE_DIR / "models" / "vision"
TASK_LAUNCH = PACKAGE_DIR / "launch" / "task3_execute.launch"


class Task3VisualPipelineTest(unittest.TestCase):
    def test_runtime_package_contains_exactly_one_yolo_weight(self):
        weights = sorted(
            path.name
            for path in VISION_DIR.iterdir()
            if path.suffix.lower() in {
                ".bin", ".ckpt", ".engine", ".onnx", ".pb",
                ".pt", ".pth", ".tflite", ".weights",
            }
        )
        self.assertEqual(["cube_yolov5_best.onnx"], weights)
        self.assertGreater(
            (VISION_DIR / "cube_yolov5_best.onnx").stat().st_size,
            0,
        )
        launch = TASK_LAUNCH.read_text(encoding="utf-8")
        self.assertIn(
            "$(find car3)/models/vision/cube_yolov5_best.onnx",
            launch,
        )

    def test_runtime_source_does_not_read_gazebo_cube_positions(self):
        source = TASK_SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn("/gazebo/get_model_state", source)
        self.assertNotIn("GetModelState", source)
        self.assertNotIn("_cube_poses", source)
        self.assertNotIn("cube_world_poses", source)
        self.assertNotIn("time.time()", source)
        self.assertIn("rospy.Time.now()", source)
        search_body = source[
            source.index("def _search_target_at_observation"):
            source.index("def _observation_pose_reached")
        ]
        self.assertLess(
            search_body.index("_quick_classify_observation"),
            search_body.index("_vision_align"),
        )
        self.assertLess(
            search_body.index("_vision_align"),
            search_body.index("_classify_aligned_cube"),
        )
        quick_body = source[
            source.index("def _quick_classify_observation"):
            source.index("def _inside_grasp_range")
        ]
        self.assertNotIn("vision_scan_center_tolerance", quick_body)
        scan_body = source[
            source.index("def _scan_region"):
            source.index("def _quick_classify_observation")
        ]
        self.assertNotIn("angular.z", scan_body)
        self.assertNotIn("_classify_cube_multiview", source)
        self.assertNotIn("_collect_classification_view", source)
        self.assertNotIn("vision_label_guard", source)

    def test_rviz_camera_shows_realtime_yolo_annotations(self):
        source = TASK_SCRIPT.read_text(encoding="utf-8")
        rviz = RVIZ_CONFIG.read_text(encoding="utf-8")

        self.assertIn(
            '"/sim_task3/vision/debug_image", Image, queue_size=1',
            source,
        )
        self.assertIn("cv2.rectangle(annotated", source)
        self.assertIn("detection[\"confidence\"]", source)
        self.assertIn(
            "Image Topic: /sim_task3/vision/debug_image",
            rviz,
        )
        self.assertIn("Name: YOLO Realtime Detection", rviz)
        self.assertNotIn("Image Topic: /camera/rgb/image_raw", rviz)

    def test_debug_heartbeat_keeps_processing_frames_after_search_stops(self):
        spec = importlib.util.spec_from_file_location(
            "task3_pick_deliver_heartbeat", TASK_SCRIPT
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        detector = module.PickDeliverTask.__new__(module.PickDeliverTask)
        detector.image_lock = threading.Lock()
        detector.latest_image = np.zeros((8, 12, 3), dtype=np.uint8)
        detector.latest_image_sequence = 7
        detector.vision_debug_region = {"name": "upper"}
        detector.vision_debug_last_sequence = 6
        detector.vision_debug_last_publish_wall = time.monotonic() - 1.0
        detector.vision_debug_rate = 5.0
        detector.vision_inference_lock = threading.Lock()
        calls = []
        detector._detect_unlocked = (
            lambda image, region: calls.append((image.shape, region["name"]))
        )

        module.PickDeliverTask._vision_debug_tick(detector, None)
        module.PickDeliverTask._vision_debug_tick(detector, None)

        self.assertEqual([((8, 12, 3), "upper")], calls)
        self.assertEqual(7, detector.vision_debug_last_sequence)

    def test_visual_alignment_uses_lateral_and_forward_motion_without_yaw(self):
        spec = importlib.util.spec_from_file_location(
            "task3_pick_deliver_translation_servo", TASK_SCRIPT
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        task = module.PickDeliverTask.__new__(module.PickDeliverTask)
        task.vision_lateral_gain = 0.45
        task.vision_min_lateral = 0.15
        task.vision_max_lateral = 0.25
        task.vision_forward_gain = 0.45
        task.vision_min_forward = 0.15
        task.vision_max_forward = 0.25

        move_left = task._visual_servo_command(0.10, 0.20)
        self.assertGreater(move_left.linear.y, 0.0)
        self.assertEqual(move_left.linear.x, 0.0)
        self.assertEqual(move_left.angular.z, 0.0)

        move_right = task._visual_servo_command(-0.10, 0.20)
        self.assertLess(move_right.linear.y, 0.0)
        self.assertEqual(move_right.angular.z, 0.0)

        move_forward = task._visual_servo_command(0.01, 0.20)
        self.assertEqual(move_forward.linear.y, 0.0)
        self.assertGreater(move_forward.linear.x, 0.0)
        self.assertEqual(move_forward.angular.z, 0.0)

        # A detection may be just outside grasp acceptance (as little as
        # 0.005 image heights) while its proportional correction is far below
        # the measured wheel static-friction threshold.  It must still move.
        tiny_forward = task._visual_servo_command(0.01, 0.0136)
        self.assertEqual(tiny_forward.linear.y, 0.0)
        self.assertEqual(tiny_forward.linear.x, 0.15)

    def test_search_order_and_grasp_calibration_are_complete(self):
        config = yaml.safe_load(VISION_CONFIG.read_text(encoding="utf-8"))
        self.assertGreaterEqual(config["vision_quick_classify_frames"], 3)
        self.assertGreaterEqual(config["vision_quick_min_confidence"], 0.85)
        self.assertGreater(config["vision_quick_classify_timeout"], 0.0)
        self.assertGreaterEqual(config["vision_classify_stable_frames"], 5)
        self.assertGreater(config["vision_classify_timeout"], 0.0)
        regions = config["vision_search_regions"]
        self.assertEqual(
            [region["name"] for region in regions],
            ["left", "upper", "right"],
        )
        anchor_x, anchor_y, anchor_yaw = regions[0]["observation_goal"]
        for index, region in enumerate(regions):
            x, y, yaw = region["observation_goal"]
            self.assertAlmostEqual(x, anchor_x, places=6)
            self.assertAlmostEqual(y, anchor_y, places=6)
            expected_yaw = anchor_yaw - index * np.pi / 2.0
            yaw_error = np.arctan2(
                np.sin(yaw - expected_yaw),
                np.cos(yaw - expected_yaw),
            )
            self.assertAlmostEqual(yaw_error, 0.0, places=4)
        self.assertEqual(
            [region["fallback_observation_goal"] for region in regions],
            [
                [-1.510723, -0.445771, 3.141157],
                [-1.395771, -0.419277, 1.570361],
                [-1.359277, -0.524229, -0.000436],
            ],
        )
        for region in regions:
            self.assertEqual(len(region["observation_goal"]), 3)
            self.assertEqual(len(region["fallback_observation_goal"]), 3)
            self.assertEqual(len(region["recorded_bbox_px"]), 4)
            self.assertEqual(len(region["grasp_target"]), 4)
            self.assertEqual(
                set(region["grasp_acceptance"]),
                {"center_x", "center_y", "width", "height"},
            )
            self.assertGreaterEqual(region["grasp_target"][1], 0.757)
            self.assertEqual(
                region["grasp_acceptance"]["center_y"],
                [0.745, 0.790],
            )

if __name__ == "__main__":
    unittest.main()
