#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Capture a fixed ROS-camera sequence into a YOLO dataset directory.

The node is intentionally opt-in: it never starts with the normal 2026 launch.
When invoked it uses the existing ``/usb_cam`` stream, records fresh frames only,
and creates matching empty label files ready for later annotation.
"""

from __future__ import print_function

import json
import os
import threading
import time

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_srvs.srv import Empty


class YoloDatasetCapture(object):
    def __init__(self):
        self.output_root = os.path.abspath(os.path.expanduser(
            rospy.get_param("~output_root", "~/.ros/ucar_2026_extra_yolo_dataset")))
        self.image_topic = str(rospy.get_param(
            "~image_topic", "/usb_cam/image_raw"))
        self.capture_count = int(rospy.get_param("~capture_count", 300))
        self.capture_interval = float(rospy.get_param("~capture_interval", 0.5))
        self.train_ratio = float(rospy.get_param("~train_ratio", 0.8))
        self.image_wait_timeout = float(rospy.get_param(
            "~image_wait_timeout", 4.0))
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 95))
        self.manage_camera = bool(rospy.get_param("~manage_camera", True))
        self.stop_camera_after_capture = bool(rospy.get_param(
            "~stop_camera_after_capture", True))
        self.camera_start_service = str(rospy.get_param(
            "~camera_start_service", "/usb_cam/start_capture"))
        self.camera_stop_service = str(rospy.get_param(
            "~camera_stop_service", "/usb_cam/stop_capture"))
        self.camera_service_timeout = float(rospy.get_param(
            "~camera_service_timeout", 5.0))

        if self.capture_count <= 0:
            raise ValueError("capture_count must be positive")
        if self.capture_interval <= 0.0:
            raise ValueError("capture_interval must be positive")
        if not 0.0 < self.train_ratio < 1.0:
            raise ValueError("train_ratio must be between 0 and 1")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be in 1..100")

        self.bridge = CvBridge()
        self.condition = threading.Condition()
        self.latest_image = None
        self.latest_stamp = None
        self.image_sequence = 0
        self.subscriber = rospy.Subscriber(
            self.image_topic, Image, self.image_callback, queue_size=1)

    def image_callback(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(2.0, "YOLO_DATASET_CAPTURE cv_bridge: %s", exc)
            return
        with self.condition:
            self.latest_image = image.copy()
            self.latest_stamp = (message.header.stamp.secs,
                                 message.header.stamp.nsecs)
            self.image_sequence += 1
            self.condition.notify_all()

    def call_camera_service(self, service_name, action):
        rospy.wait_for_service(service_name, timeout=self.camera_service_timeout)
        rospy.ServiceProxy(service_name, Empty)()
        rospy.loginfo("YOLO_DATASET_CAPTURE_CAMERA_%s service=%s", action,
                      service_name)

    def prepare_directories(self):
        directories = {}
        for split in ("train", "val"):
            image_directory = os.path.join(self.output_root, "images", split)
            label_directory = os.path.join(self.output_root, "labels", split)
            if not os.path.isdir(image_directory):
                os.makedirs(image_directory)
            if not os.path.isdir(label_directory):
                os.makedirs(label_directory)
            directories[split] = (image_directory, label_directory)
        return directories

    def wait_for_fresh_image(self, previous_sequence):
        deadline = time.time() + self.image_wait_timeout
        with self.condition:
            while (self.image_sequence <= previous_sequence and
                   not rospy.is_shutdown()):
                remaining = deadline - time.time()
                if remaining <= 0.0:
                    raise RuntimeError("no fresh image on %s within %.1f s" % (
                        self.image_topic, self.image_wait_timeout))
                self.condition.wait(min(remaining, 0.1))
            if self.latest_image is None:
                raise RuntimeError("camera returned no image")
            return self.image_sequence, self.latest_image.copy(), self.latest_stamp

    def split_for_index(self, zero_based_index):
        # Interleave splits while retaining the requested exact proportion.
        # 300 captures at the default 0.8 ratio produces exactly 240/60.
        before = int(zero_based_index * self.train_ratio)
        after = int((zero_based_index + 1) * self.train_ratio)
        return "train" if after > before else "val"

    def write_capture(self, directories, zero_based_index, image, stamp):
        split = self.split_for_index(zero_based_index)
        image_directory, label_directory = directories[split]
        stem = "capture_%06d" % (zero_based_index + 1)
        image_path = os.path.join(image_directory, stem + ".jpg")
        label_path = os.path.join(label_directory, stem + ".txt")
        if not cv2.imwrite(image_path, image,
                           [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]):
            raise RuntimeError("cannot write image %s" % image_path)
        # An empty matching label file explicitly records that annotation is
        # pending, while preserving the standard YOLO images/labels pairing.
        open(label_path, "wb").close()
        return {
            "index": zero_based_index + 1,
            "split": split,
            "image": os.path.relpath(image_path, self.output_root),
            "label": os.path.relpath(label_path, self.output_root),
            "stamp": stamp,
        }

    def run(self):
        directories = self.prepare_directories()
        started_camera = False
        records = []
        try:
            if self.manage_camera:
                self.call_camera_service(self.camera_start_service, "START")
                started_camera = True

            previous_sequence = 0
            next_capture_time = time.time()
            for index in range(self.capture_count):
                delay = next_capture_time - time.time()
                if delay > 0.0:
                    time.sleep(delay)
                previous_sequence, image, stamp = self.wait_for_fresh_image(
                    previous_sequence)
                record = self.write_capture(directories, index, image, stamp)
                records.append(record)
                rospy.loginfo(
                    "YOLO_DATASET_CAPTURED index=%d/%d split=%s image=%s",
                    record["index"], self.capture_count, record["split"],
                    record["image"])
                next_capture_time += self.capture_interval
        finally:
            if started_camera and self.stop_camera_after_capture:
                try:
                    self.call_camera_service(self.camera_stop_service, "STOP")
                except Exception as exc:
                    rospy.logerr("YOLO_DATASET_CAPTURE_CAMERA_STOP_FAILED: %s", exc)

        manifest_path = os.path.join(self.output_root, "capture_manifest.json")
        with open(manifest_path, "wb") as manifest_file:
            manifest_file.write(json.dumps({
                "image_topic": self.image_topic,
                "capture_count": self.capture_count,
                "capture_interval_seconds": self.capture_interval,
                "train_ratio": self.train_ratio,
                "records": records,
            }, indent=2, sort_keys=True))
            manifest_file.write("\n")
        rospy.loginfo("YOLO_DATASET_CAPTURE_COMPLETE count=%d root=%s",
                      len(records), self.output_root)


def main():
    rospy.init_node("yolo_dataset_capture")
    try:
        YoloDatasetCapture().run()
    except Exception as exc:
        rospy.logfatal("YOLO_DATASET_CAPTURE_FAILED: %s", exc)
        raise


if __name__ == "__main__":
    main()
