#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

import importlib.util
import os
import unittest


SCRIPT_PATH = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "scripts", "production_camera_ocr.py"))
SPEC = importlib.util.spec_from_file_location(
    "production_camera_ocr_under_test", SCRIPT_PATH)
CAMERA_OCR = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CAMERA_OCR)


class FakeCamera(object):
    def __init__(self, opened):
        self.opened = opened
        self.released = False
        self.read_count = 0

    def isOpened(self):
        return self.opened

    def set(self, _property, _value):
        return True

    def read(self):
        self.read_count += 1
        return True, object()

    def release(self):
        self.released = True


class ProductionCameraOcrTest(unittest.TestCase):
    def test_open_camera_retries_after_two_transient_open_failures(self):
        cameras = [
            FakeCamera(False),
            FakeCamera(False),
            FakeCamera(True),
        ]
        calls = []
        original_video_capture = CAMERA_OCR.cv2.VideoCapture

        def fake_video_capture(*args):
            calls.append(args)
            return cameras[len(calls) - 1]

        CAMERA_OCR.cv2.VideoCapture = fake_video_capture
        try:
            camera = CAMERA_OCR.open_camera(
                "/dev/ucar_video", 640, 480, 2,
                open_timeout=1.0, retry_interval=0.0)
        finally:
            CAMERA_OCR.cv2.VideoCapture = original_video_capture

        self.assertIs(camera, cameras[2])
        self.assertEqual(3, len(calls))
        self.assertTrue(cameras[0].released)
        self.assertTrue(cameras[1].released)
        self.assertEqual(2, cameras[2].read_count)


if __name__ == "__main__":
    unittest.main()
