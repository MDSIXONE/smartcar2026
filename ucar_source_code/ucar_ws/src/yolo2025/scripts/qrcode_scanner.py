#!/home/ucar/myenv/bin/python3
"""ROS QR-code scanner with an optional HTTP item-name lookup.

Real-field QR codes carry a full URL; when api_enabled the decoded URL
itself is GET-requested and the JSON reply ({"code": 200, "result": "<item>"})
is published on /qr_api_result for the task node to match item names.
"""

import json
import os
import threading
import time
from urllib.request import urlopen

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8, String


class QRCodeScanner(object):
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.scan_enabled = rospy.get_param("~scan_enabled", True)
        self.api_enabled = rospy.get_param("~api_enabled", False)
        self.api_timeout_sec = float(rospy.get_param("~api_timeout_sec", 5.0))
        self.same_code_cooldown_sec = float(rospy.get_param("~same_code_cooldown_sec", 1.0))

        self.open_cv_detector = cv2.QRCodeDetector()
        self.wechat_detector = self._create_wechat_detector()
        self.last_text = ""
        self.last_publish_time = 0.0
        self.api_inflight = set()
        # Process-local successful URL -> item-name cache.  It deliberately
        # lives on this node instance so a qrcode_scanner restart starts a new
        # mission with no retained API result.
        self.api_item_cache = {}
        self.api_lock = threading.Lock()

        self.result_pub = rospy.Publisher("/qr_result", String, queue_size=1)
        self.api_result_pub = rospy.Publisher("/qr_api_result", String, queue_size=1)
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)
        self.enable_sub = rospy.Subscriber("/qrcode_start_flag", Int8, self.enable_callback, queue_size=1)

        rospy.loginfo(
            "QR scanner ready: image=%s, enabled=%s, api_enabled=%s, wechat=%s",
            self.image_topic, self.scan_enabled, self.api_enabled, self.wechat_detector is not None)

    def _create_wechat_detector(self):
        model_dir = rospy.get_param("~wechat_model_dir", "/home/ucar/myenv/qr_file")
        model_files = ["detect.prototxt", "detect.caffemodel", "sr.prototxt", "sr.caffemodel"]
        model_paths = [os.path.join(model_dir, name) for name in model_files]
        if not hasattr(cv2, "wechat_qrcode_WeChatQRCode") or not all(os.path.isfile(path) for path in model_paths):
            rospy.logwarn("WeChat QR model is unavailable; using OpenCV QRCodeDetector only.")
            return None
        try:
            return cv2.wechat_qrcode_WeChatQRCode(*model_paths)
        except Exception as error:
            rospy.logwarn("Cannot initialize WeChat QR detector: %s", error)
            return None

    def enable_callback(self, message):
        self.scan_enabled = message.data != 0
        rospy.loginfo("QR scanning %s", "enabled" if self.scan_enabled else "disabled")

    def image_callback(self, message):
        if not self.scan_enabled:
            return
        try:
            image = self.image_message_to_bgr(message)
        except ValueError as error:
            rospy.logwarn_throttle(2.0, "QR image conversion failed: %s", error)
            return

        decoded_text = self.decode(image)
        if not decoded_text:
            return

        now = time.monotonic()
        if decoded_text == self.last_text and now - self.last_publish_time < self.same_code_cooldown_sec:
            return
        self.last_text = decoded_text
        self.last_publish_time = now
        self.result_pub.publish(decoded_text)
        rospy.loginfo("QR detected: %s", decoded_text)

        if self.api_enabled:
            self.query_api_async(decoded_text)

    @staticmethod
    def image_message_to_bgr(message):
        """Convert common ROS 8-bit image encodings without cv_bridge.

        ROS Melodic on this car supplies a Python-2-only cv_bridge binary, while
        the WeChat detector requires the Python-3 OpenCV environment.  The USB
        camera publishes rgb8, so direct NumPy conversion keeps those runtimes
        independent.
        """
        encoding = message.encoding.lower()
        if encoding not in ("rgb8", "bgr8", "mono8"):
            raise ValueError("unsupported image encoding: %s" % message.encoding)
        channels = 1 if encoding == "mono8" else 3
        required_size = message.step * message.height
        pixels = np.frombuffer(message.data, dtype=np.uint8)
        if pixels.size < required_size:
            raise ValueError("image payload is shorter than height * step")
        rows = pixels[:required_size].reshape((message.height, message.step))
        if encoding == "mono8":
            return cv2.cvtColor(rows[:, :message.width], cv2.COLOR_GRAY2BGR)
        image = rows[:, :message.width * channels].reshape((message.height, message.width, channels))
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image

    def decode(self, image):
        if self.wechat_detector is not None:
            try:
                results, _ = self.wechat_detector.detectAndDecode(image)
                if results:
                    for result in results:
                        if result:
                            return result.strip()
            except Exception as error:
                rospy.logwarn_throttle(2.0, "WeChat QR detection failed: %s", error)
        try:
            text, _, _ = self.open_cv_detector.detectAndDecode(image)
            return text.strip() if text else ""
        except cv2.error as error:
            rospy.logwarn_throttle(2.0, "OpenCV QR detection failed: %s", error)
            return ""

    def query_api_async(self, decoded_text):
        url = decoded_text.strip()
        with self.api_lock:
            cached_item = self.api_item_cache.get(url)
            if cached_item is not None:
                response = {
                    "qr_text": decoded_text,
                    "key": "",
                    "ok": True,
                    "cached": True,
                    "response": {"code": 200, "result": cached_item},
                }
            elif url in self.api_inflight:
                return
            else:
                self.api_inflight.add(url)
                response = None
        if response is not None:
            rospy.loginfo("QR API cache %s -> result=%s", url, cached_item)
            self.api_result_pub.publish(
                json.dumps(response, ensure_ascii=False, sort_keys=True))
            return
        worker = threading.Thread(target=self.query_api, args=(decoded_text, url))
        worker.daemon = True
        worker.start()

    def query_api(self, decoded_text, url):
        response = {"qr_text": decoded_text, "key": "", "ok": False}
        try:
            with urlopen(url, timeout=self.api_timeout_sec) as http_response:
                parsed = json.loads(http_response.read().decode("utf-8"))
                response.update({"ok": True, "http_status": http_response.getcode(), "response": parsed})
                item = parsed.get("result")
                if item:
                    with self.api_lock:
                        self.api_item_cache[url] = item
                rospy.loginfo("QR API %s -> code=%s result=%s", url, parsed.get("code"), parsed.get("result"))
        except Exception as error:
            response["error"] = str(error)
            rospy.logwarn("QR API lookup for %s failed: %s", url, error)
        finally:
            with self.api_lock:
                self.api_inflight.discard(url)
        self.api_result_pub.publish(json.dumps(response, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    rospy.init_node("qrcode_scanner")
    QRCodeScanner()
    rospy.spin()
