#!/usr/bin/env python3
"""Regression tests for qrcode_scanner's process-local API item cache."""

import importlib.util
import json
import os
import sys
import threading
import types
import unittest


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))


def load_scanner_module():
    """Load the scanner without requiring a running ROS master."""
    names = ("cv2", "numpy", "rospy", "sensor_msgs", "sensor_msgs.msg",
             "std_msgs", "std_msgs.msg")
    original_modules = dict((name, sys.modules.get(name)) for name in names)
    fake_rospy = types.ModuleType("rospy")
    fake_rospy.loginfo = lambda *args, **kwargs: None
    fake_rospy.logwarn = lambda *args, **kwargs: None
    fake_rospy.logwarn_throttle = lambda *args, **kwargs: None
    fake_sensor_msgs = types.ModuleType("sensor_msgs")
    fake_sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    fake_sensor_msgs_msg.Image = object
    fake_std_msgs = types.ModuleType("std_msgs")
    fake_std_msgs_msg = types.ModuleType("std_msgs.msg")
    fake_std_msgs_msg.Int8 = object
    fake_std_msgs_msg.String = object
    sys.modules.update({
        "cv2": types.ModuleType("cv2"),
        "numpy": types.ModuleType("numpy"),
        "rospy": fake_rospy,
        "sensor_msgs": fake_sensor_msgs,
        "sensor_msgs.msg": fake_sensor_msgs_msg,
        "std_msgs": fake_std_msgs,
        "std_msgs.msg": fake_std_msgs_msg,
    })
    try:
        path = os.path.join(PACKAGE_ROOT, "scripts", "qrcode_scanner.py")
        spec = importlib.util.spec_from_file_location("qrcode_scanner_cache_test", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        for name, module in original_modules.items():
            if module is None:
                del sys.modules[name]
            else:
                sys.modules[name] = module


class QRCodeScannerCacheTest(unittest.TestCase):

    def test_image_callback_publishes_all_codes_from_one_frame(self):
        module = load_scanner_module()
        scanner = object.__new__(module.QRCodeScanner)
        scanner.scan_enabled = True
        scanner.image_message_to_bgr = lambda _message: "frame"
        scanner.decode = lambda _image: ["http://qr.example/a",
                                         "http://qr.example/b"]
        scanner.last_text = ""
        scanner.last_publish_time = 0.0
        scanner.same_code_cooldown_sec = 1.0
        scanner.api_enabled = False
        published = []
        scanner.result_pub = types.SimpleNamespace(publish=published.append)

        scanner.image_callback(object())

        self.assertEqual(
            published,
            ["http://qr.example/a", "http://qr.example/b"])

    def test_opencv_decoder_returns_all_codes(self):
        module = load_scanner_module()
        scanner = object.__new__(module.QRCodeScanner)
        scanner.wechat_detector = None

        class Detector(object):
            def detectAndDecodeMulti(self, _image):
                return True, ("a", "b"), None, None

        scanner.open_cv_detector = Detector()

        self.assertEqual(scanner.decode(object()), ["a", "b"])

    def test_successful_url_lookup_is_republished_from_process_cache(self):
        module = load_scanner_module()
        scanner = object.__new__(module.QRCodeScanner)
        scanner.api_item_cache = {}
        scanner.api_inflight = set()
        scanner.api_lock = threading.Lock()
        scanner.api_timeout_sec = 5.0
        published = []
        scanner.api_result_pub = types.SimpleNamespace(publish=published.append)
        requests = []

        class Response(object):
            def read(self):
                return b'{"code": 200, "result": "\xe8\x8b\xb9\xe6\x9e\x9c"}'

            def getcode(self):
                return 200

            def __enter__(self):
                return self

            def __exit__(self, exc_type, exc_value, traceback):
                return False

        class ImmediateThread(object):
            def __init__(self, target, args):
                self.target = target
                self.args = args
                self.daemon = False

            def start(self):
                self.target(*self.args)

        def fake_urlopen(url, timeout):
            requests.append((url, timeout))
            return Response()

        original_urlopen = module.urlopen
        original_thread = module.threading.Thread
        module.urlopen = fake_urlopen
        module.threading.Thread = ImmediateThread
        try:
            scanner.query_api_async(" http://qr.example/a ")
            scanner.query_api_async("http://qr.example/a")
        finally:
            module.urlopen = original_urlopen
            module.threading.Thread = original_thread

        self.assertEqual(requests, [("http://qr.example/a", 5.0)])
        self.assertEqual(scanner.api_item_cache, {"http://qr.example/a": "苹果"})
        self.assertEqual(len(published), 2)
        self.assertFalse(json.loads(published[0]).get("cached", False))
        self.assertTrue(json.loads(published[1])["cached"])
        self.assertEqual(json.loads(published[1])["response"]["result"], "苹果")


if __name__ == "__main__":
    unittest.main()
