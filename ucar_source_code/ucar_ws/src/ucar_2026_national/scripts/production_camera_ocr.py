#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Persistent native-camera bridge for the vehicle's live_ppocr.py engine.

The ROS mission is Python 2.  This helper deliberately runs in the vehicle's
Python 3 OCR environment, owns the V4L2 device after usb_cam is stopped, and
exchanges one JSON object per line over stdin/stdout.
"""

from __future__ import print_function

import argparse
import importlib.util
import json
import os
import re
import sys
import time

import cv2
import numpy as np


def emit(payload):
    sys.stdout.write(json.dumps(payload, ensure_ascii=True) + "\n")
    sys.stdout.flush()


def load_live_module(path):
    module_directory = os.path.dirname(os.path.abspath(path))
    if module_directory not in sys.path:
        sys.path.insert(0, module_directory)
    spec = importlib.util.spec_from_file_location("ucar_live_ppocr", path)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load OCR module %s" % path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def video_source(device):
    real_path = os.path.realpath(device)
    match = re.match(r"^/dev/video([0-9]+)$", real_path)
    if match:
        return int(match.group(1))
    return device


def open_camera(device, width, height, warmup_frames,
                open_timeout=8.0, retry_interval=0.25):
    source = video_source(device)
    cap_v4l2 = getattr(cv2, "CAP_V4L2", 200)
    deadline = time.monotonic() + max(0.0, float(open_timeout))
    attempts = 0

    while True:
        for force_v4l2 in (True, False):
            attempts += 1
            try:
                camera = (
                    cv2.VideoCapture(source, cap_v4l2)
                    if force_v4l2 else cv2.VideoCapture(source))
            except TypeError:
                camera = cv2.VideoCapture(source)

            if camera.isOpened():
                if (hasattr(cv2, "CAP_PROP_FOURCC") and
                        hasattr(cv2, "VideoWriter_fourcc")):
                    camera.set(
                        cv2.CAP_PROP_FOURCC,
                        cv2.VideoWriter_fourcc(*"YUYV"))
                camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
                    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)

                warmup_ok = True
                for _index in range(max(1, warmup_frames)):
                    ok, frame = camera.read()
                    if not ok or frame is None:
                        warmup_ok = False
                        break
                if warmup_ok:
                    return camera

            camera.release()

        if time.monotonic() >= deadline:
            raise RuntimeError(
                "cannot open camera %s after %d attempts in %.1f s" % (
                    device, attempts, float(open_timeout)))
        time.sleep(max(0.0, float(retry_interval)))


def capture_result(camera, engine, live_module, command, side, mirror):
    input_path = command.get("input")
    if input_path:
        frame = cv2.imread(input_path, cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError("cannot read ROS camera frame %s" % input_path)
    else:
        if camera is None:
            raise RuntimeError("capture command requires an input image")
        ok, frame = camera.read()
        if not ok or frame is None:
            raise RuntimeError("camera frame capture failed")
    if mirror:
        frame = cv2.flip(frame, 1)
    output_path = command["output"]
    output_directory = os.path.dirname(os.path.abspath(output_path))
    if output_directory and not os.path.isdir(output_directory):
        os.makedirs(output_directory)
    if not cv2.imwrite(output_path, frame):
        raise RuntimeError("cannot save image %s" % output_path)

    boxes = engine.ocr(frame, side=side)
    items = []
    confidence_limit = float(command.get(
        "minimum_confidence", live_module.CONF_MIN))
    for box in boxes:
        confidence = float(box["conf"])
        if confidence < confidence_limit:
            continue
        quad = np.asarray(box["pts"], dtype=np.float32)
        items.append({
            "quad": quad.tolist(),
            "text": box["text"],
            "confidence": confidence,
        })
    raw_text = "".join(item["text"] for item in items)
    name, probability, probabilities = live_module.classify(raw_text)

    detection = None
    if items:
        all_points = np.concatenate(
            [np.asarray(item["quad"], dtype=np.float32) for item in items],
            axis=0)
        left = float(np.min(all_points[:, 0]))
        top = float(np.min(all_points[:, 1]))
        right = float(np.max(all_points[:, 0]))
        bottom = float(np.max(all_points[:, 1]))
        text = name or raw_text
        confidence = (
            float(probability) * 100.0 if name else
            sum(item["confidence"] for item in items) / len(items) * 100.0)
        detection = {
            "text": text,
            "raw_text": raw_text,
            "confidence": confidence,
            "bbox": [left, top, right - left, bottom - top],
            "classification_probabilities": probabilities,
            "items": items,
        }
    return {
        "ok": True,
        "image_path": output_path,
        "width": int(frame.shape[1]),
        "height": int(frame.shape[0]),
        "detection": detection,
    }


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ocr-module", required=True)
    parser.add_argument("--device", required=True)
    parser.add_argument("--det", required=True)
    parser.add_argument("--rec", required=True)
    parser.add_argument("--keys", required=True)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--side", type=int, default=640)
    parser.add_argument("--warmup-frames", type=int, default=8)
    parser.add_argument("--open-timeout", type=float, default=8.0)
    parser.add_argument("--ros-image-input", action="store_true")
    parser.add_argument("--mirror", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    live_module = load_live_module(args.ocr_module)
    engine = live_module.PPOcr(args.det, args.rec, args.keys)
    engine.set_whitelist(live_module.WHITELIST)
    camera = None
    if not args.ros_image_input:
        camera = open_camera(
            args.device, args.width, args.height, args.warmup_frames,
            open_timeout=args.open_timeout)
    emit({
        "ready": True,
        "device": args.device,
        "mode": "ros_image" if args.ros_image_input else "native_camera",
        "cv2_version": cv2.__version__,
        "candidates": live_module.CANDIDATES,
    })
    try:
        for raw_line in sys.stdin:
            try:
                command = json.loads(raw_line)
                name = command.get("command")
                if name == "capture":
                    emit(capture_result(
                        camera, engine, live_module, command,
                        args.side, args.mirror))
                elif name == "close":
                    emit({"ok": True, "closed": True})
                    break
                else:
                    emit({"ok": False, "error": "unknown command"})
            except Exception as exc:
                emit({"ok": False, "error": str(exc)})
    finally:
        if camera is not None:
            camera.release()
    return 0


if __name__ == "__main__":
    sys.exit(main())
