#!/usr/bin/env python3
"""Find, visually align with, pick, and deliver one labelled cube.

The task visits the left observation point, then turns clockwise in 90-degree
steps to search upper and right with the trained YOLOv5 model.  The requested
class and its image-space box are the only inputs used to choose a bay and
align the base for grasping.  Gazebo cube positions are intentionally never
read by this runtime node.
"""

import math
import os
import threading
import time
from collections import deque

import actionlib
import cv2
import numpy as np
import rospy
import tf.transformations as transformations
from actionlib_msgs.msg import GoalStatus
from controller_manager_msgs.srv import (
    ListControllers,
    SwitchController,
    SwitchControllerRequest,
)
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetLinkState, SetModelState
from dynamic_reconfigure.client import Client as DynamicReconfigureClient
from geometry_msgs.msg import Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import Empty, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
# Model names are retained only for the post-close Gazebo attachment fallback.
# They are never queried to locate a cube or choose a pickup bay.
CATEGORY_CUBE = {"food": "cube_0", "daily": "cube_1", "electronics": "cube_2"}
CATEGORY_ALIASES = {
    "food": "food", "foods": "food", "食品": "food", "食品类": "food",
    "daily": "daily", "daily_necessities": "daily", "日用品": "daily",
    "electronics": "electronics", "electronic": "electronics", "电子": "electronics",
    "电子产品": "electronics",
}
ITEM_CATEGORIES = {
    "可乐": "food", "牛奶": "food", "面包": "food", "饼干": "food", "苹果": "food",
    "香蕉": "food", "零食": "food", "饮料": "food",
    "牙刷": "daily", "毛巾": "daily", "纸巾": "daily", "肥皂": "daily",
    "洗发水": "daily", "水杯": "daily",
    "手机": "electronics", "平板": "electronics", "耳机": "electronics",
    "键盘": "electronics", "鼠标": "electronics", "相机": "electronics",
    "充电器": "electronics",
}
WAREHOUSES = {
    "food": ("食品加工车间", (1.00, -2.98, -math.pi / 2.0)),
    "daily": ("日用品加工车间", (1.00, -1.50, math.pi / 2.0)),
    "electronics": ("电子产品生产车间", (2.55, -2.22, 0.0)),
}
WAREHOUSE_REGIONS = {
    "food": (0.75, 1.25, -3.23, -2.73),
    "daily": (0.75, 1.25, -1.75, -1.25),
    "electronics": (2.30, 2.80, -2.47, -1.97),
}

def clamp(value, lower, upper):
    return max(lower, min(value, upper))


def realtime_factor_is_acceptable(measured, minimum, tolerance):
    """Accept bounded sampling jitter without hiding a genuinely slow simulator."""
    return measured + tolerance >= minimum


def wall_budget_should_abort(remaining, strict):
    """Keep the match-time budget advisory unless strict timing is requested."""
    return remaining <= 0.0 and bool(strict)


def realtime_preflight_should_abort(measured, minimum, tolerance, strict):
    """Only make a slow RTF sample fatal when strict enforcement is requested."""
    return bool(strict) and not realtime_factor_is_acceptable(
        measured,
        minimum,
        tolerance,
    )


def quaternion_from_yaw(yaw):
    x, y, z, w = transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=x, y=y, z=z, w=w)


class PickDeliverTask:
    def __init__(self):
        rospy.init_node("task3_pick_deliver")
        self.task_wall_started = time.monotonic()
        self.task_sim_started = rospy.Time.now()

        self.cargo_item = str(rospy.get_param("~cargo_item", "")).strip()
        requested_category = str(rospy.get_param("~cargo_category", "auto")).strip().lower()
        self.category = self._resolve_category(requested_category, self.cargo_item)
        self.cargo_name = str(rospy.get_param("~cargo_name", "")).strip()
        if not self.cargo_name:
            self.cargo_name = self.cargo_item or self.category
        self.cargo_model = CATEGORY_CUBE[self.category]
        self.destination_name, self.destination = WAREHOUSES[self.category]

        self.arm_grasp = self._pose_param(
            "~arm_grasp_pose", [-0.0001, 1.5000, 0.2800, 1.3000, 0.0000]
        )
        self.arm_carry = self._pose_param(
            "~arm_carry_pose", [-0.0001, 0.0000, -1.7200, -0.5000, 0.0000]
        )
        self.arm_initial = self._pose_param(
            "/sim_task3/arm_initial_pose",
            [-0.0001, -0.4999, 1.2800, 1.7000, 0.0000],
        )
        self.arm_grasp_duration = float(rospy.get_param("~arm_grasp_duration", 2.0))
        self.arm_carry_duration = float(rospy.get_param("~arm_carry_duration", 2.5))
        self.arm_recovery_duration = float(
            rospy.get_param("~arm_recovery_duration", 2.5)
        )
        self.recognition_retry_delay = float(
            rospy.get_param("~recognition_retry_delay", 0.50)
        )
        self.grasp_retry_delay = float(
            rospy.get_param("~grasp_retry_delay", 0.50)
        )
        self.physical_grasp_settle = float(
            rospy.get_param("~physical_grasp_settle", 0.25)
        )
        self.attachment_fallback_enabled = bool(
            rospy.get_param("~attachment_fallback_enabled", False)
        )
        self.grasp_wrist_rotation_limit = float(
            rospy.get_param("~grasp_wrist_rotation_limit", 0.035)
        )
        self.grasp_guard_history_samples = int(
            rospy.get_param("~grasp_guard_history_samples", 6)
        )
        self.grasp_guard_poll_interval = float(
            rospy.get_param("~grasp_guard_poll_interval", 0.01)
        )
        self.grasp_guard_recovery_duration = float(
            rospy.get_param("~grasp_guard_recovery_duration", 0.25)
        )
        self.task_wall_budget = float(rospy.get_param("~task_wall_budget", 145.0))
        self.task_wall_budget_strict = bool(
            rospy.get_param("~task_wall_budget_strict", False)
        )
        self.task_wall_budget_warning_sent = False
        self.rtf_preflight_duration = float(
            rospy.get_param("~rtf_preflight_duration", 5.0)
        )
        self.rtf_minimum = float(rospy.get_param("~rtf_minimum", 0.30))
        self.rtf_measurement_tolerance = float(
            rospy.get_param("~rtf_measurement_tolerance", 0.005)
        )
        self.rtf_preflight_strict = bool(
            rospy.get_param("~rtf_preflight_strict", False)
        )
        self.nav_timeout = float(rospy.get_param("~nav_timeout", 30.0))
        self.nav_attempts = int(rospy.get_param("~nav_attempts", 2))
        self.destination_nav_timeout = float(
            rospy.get_param("~destination_nav_timeout", 75.0)
        )
        self.destination_nav_attempts = int(
            rospy.get_param("~destination_nav_attempts", 1)
        )
        self.observation_position_tolerance = float(
            rospy.get_param("~observation_position_tolerance", 0.12)
        )
        self.observation_yaw_tolerance = float(
            rospy.get_param("~observation_yaw_tolerance", 0.12)
        )
        self.destination_position_tolerance = float(
            rospy.get_param("~destination_position_tolerance", 0.08)
        )
        self.destination_yaw_tolerance = float(
            rospy.get_param("~destination_yaw_tolerance", 0.10)
        )
        if self.task_wall_budget <= 0.0:
            raise rospy.ROSException("task_wall_budget must be positive")
        if self.rtf_measurement_tolerance < 0.0:
            raise rospy.ROSException(
                "rtf_measurement_tolerance must be non-negative"
            )
        if self.nav_attempts <= 0:
            raise rospy.ROSException("nav_attempts must be positive")
        if self.destination_nav_timeout <= 0.0:
            raise rospy.ROSException("destination_nav_timeout must be positive")
        if self.destination_nav_attempts <= 0:
            raise rospy.ROSException("destination_nav_attempts must be positive")
        if self.destination_position_tolerance <= 0.0:
            raise rospy.ROSException(
                "destination_position_tolerance must be positive"
            )
        if self.destination_yaw_tolerance <= 0.0:
            raise rospy.ROSException("destination_yaw_tolerance must be positive")
        if self.physical_grasp_settle < 0.0:
            raise rospy.ROSException("physical_grasp_settle must be non-negative")
        if self.arm_recovery_duration <= 0.0:
            raise rospy.ROSException("arm_recovery_duration must be positive")
        if self.recognition_retry_delay < 0.0:
            raise rospy.ROSException("recognition_retry_delay must be non-negative")
        if self.grasp_retry_delay < 0.0:
            raise rospy.ROSException("grasp_retry_delay must be non-negative")
        if self.grasp_wrist_rotation_limit <= 0.0:
            raise rospy.ROSException("grasp_wrist_rotation_limit must be positive")
        if self.grasp_guard_history_samples <= 0:
            raise rospy.ROSException("grasp_guard_history_samples must be positive")
        if self.grasp_guard_poll_interval <= 0.0:
            raise rospy.ROSException("grasp_guard_poll_interval must be positive")
        if self.grasp_guard_recovery_duration <= 0.0:
            raise rospy.ROSException("grasp_guard_recovery_duration must be positive")
        self.camera_topic = rospy.get_param("~camera_topic", "/camera/rgb/image_raw")
        self.vision_model_path = os.path.abspath(
            os.path.expanduser(str(rospy.get_param("~vision_model_path")))
        )
        self.vision_label_template_dir = os.path.abspath(
            os.path.expanduser(str(rospy.get_param("~vision_label_template_dir")))
        )
        self.vision_class_names = [
            str(name) for name in rospy.get_param(
                "~vision_class_names", ["food", "daily", "electronics"]
            )
        ]
        if self.category not in self.vision_class_names:
            raise rospy.ROSException(
                "Requested category %s is absent from vision_class_names=%s"
                % (self.category, self.vision_class_names)
            )
        self.target_class_id = self.vision_class_names.index(self.category)
        self.vision_confidence = float(
            rospy.get_param("~vision_confidence_threshold", 0.20)
        )
        self.vision_nms = float(rospy.get_param("~vision_nms_threshold", 0.45))
        self.vision_input_size = int(rospy.get_param("~vision_input_size", 640))
        self.vision_scan_timeout = float(rospy.get_param("~vision_scan_timeout", 8.0))
        self.vision_quick_classify_frames = int(
            rospy.get_param("~vision_quick_classify_frames", 5)
        )
        self.vision_quick_classify_timeout = float(
            rospy.get_param("~vision_quick_classify_timeout", 1.5)
        )
        self.vision_quick_min_confidence = float(
            rospy.get_param("~vision_quick_min_confidence", 0.75)
        )
        self.vision_classify_stable_frames = int(
            rospy.get_param("~vision_classify_stable_frames", 7)
        )
        self.vision_classify_timeout = float(
            rospy.get_param("~vision_classify_timeout", 5.0)
        )
        self.vision_template_min_score = float(
            rospy.get_param("~vision_template_min_score", 0.30)
        )
        self.vision_template_min_margin = float(
            rospy.get_param("~vision_template_min_margin", 0.08)
        )
        self.vision_align_timeout = float(rospy.get_param("~vision_align_timeout", 25.0))
        self.vision_lost_timeout = float(rospy.get_param("~vision_lost_timeout", 2.5))
        self.vision_align_stable_frames = int(
            rospy.get_param("~vision_align_stable_frames", 5)
        )
        self.vision_forward_gain = float(rospy.get_param("~vision_forward_gain", 0.45))
        self.vision_lateral_gain = float(rospy.get_param("~vision_lateral_gain", 0.45))
        self.vision_min_forward = float(
            rospy.get_param("~vision_min_forward_speed", 0.15)
        )
        self.vision_max_forward = float(
            rospy.get_param("~vision_max_forward_speed", 0.25)
        )
        self.vision_min_lateral = float(
            rospy.get_param("~vision_min_lateral_speed", 0.15)
        )
        self.vision_max_lateral = float(
            rospy.get_param("~vision_max_lateral_speed", 0.25)
        )
        if not 0.0 < self.vision_min_forward <= self.vision_max_forward:
            raise rospy.ROSException(
                "visual forward speed limits must satisfy 0 < min <= max"
            )
        if not 0.0 < self.vision_min_lateral <= self.vision_max_lateral:
            raise rospy.ROSException(
                "visual lateral speed limits must satisfy 0 < min <= max"
            )
        self.search_regions = self._read_search_regions(
            rospy.get_param("~vision_search_regions")
        )
        self.vision_debug_rate = float(
            rospy.get_param("~vision_debug_rate", 5.0)
        )
        if self.vision_debug_rate <= 0.0:
            raise rospy.ROSException("vision_debug_rate must be positive")

        self.arm_pub = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
        self.gripper_pub = rospy.Publisher("/gripper_controller/command", Float64, queue_size=1)
        self.cmd_pub = rospy.Publisher(
            "/sim_task3/visual_cmd_vel", Twist, queue_size=1
        )
        self.cmd_vel_source_pub = rospy.Publisher(
            "/sim_task3/cmd_vel_source", String, queue_size=1, latch=True
        )
        self.vision_debug_pub = rospy.Publisher(
            "/sim_task3/vision/debug_image", Image, queue_size=1
        )
        self.status_pub = rospy.Publisher("/sim_task3/status", String, queue_size=10, latch=True)
        self.done_pub = rospy.Publisher("/sim_task3/done", Bool, queue_size=1, latch=True)
        self.carry_mode_pub = rospy.Publisher(
            "/sim_task3/carry_mode", Bool, queue_size=1, latch=True
        )
        # The local planner receives the task phase explicitly: the pickup
        # route uses the stable direct follower, while transport of a held
        # cube uses direct laser trajectory avoidance.
        self.navigation_mode_pub = rospy.Publisher(
            "/sim_task3/navigation_mode", String, queue_size=1, latch=True
        )
        # task3_prepare keeps the parked pose through Gazebo configuration,
        # not through arm control.  This topic asks that holder to stop before
        # arm_controller takes ownership at the pickup bay.
        self.arm_control_enabled_pub = rospy.Publisher(
            "/sim_task3/arm_control_enabled", Bool, queue_size=1, latch=False
        )

        self.image_lock = threading.Lock()
        self.vision_inference_lock = threading.Lock()
        self.arm_joint_state_lock = threading.Lock()
        self.latest_image = None
        self.latest_image_sequence = 0
        # Stay idle during the real-time-factor preflight.  The first normal
        # detector call selects a region and enables the continuous RViz feed.
        self.vision_debug_region = None
        self.vision_debug_last_sequence = -1
        self.vision_debug_last_publish_wall = 0.0
        self.latest_odom = None
        self.latest_arm_joint_positions = None
        self.latest_arm_joint_sequence = 0
        self.grasp_state = "UNKNOWN"
        rospy.Subscriber(self.camera_topic, Image, self._camera_callback, queue_size=1)
        rospy.Subscriber("/grasp_attach/state", String, self._grasp_state_callback, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=1)
        rospy.Subscriber(
            "/joint_states", JointState, self._joint_state_callback, queue_size=10
        )

        self.nav = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.switch_controllers = rospy.ServiceProxy(
            "/controller_manager/switch_controller", SwitchController
        )
        self.list_controllers = rospy.ServiceProxy(
            "/controller_manager/list_controllers", ListControllers
        )
        self.get_link = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        self.set_model = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        self.check_physical_grasp = rospy.ServiceProxy(
            "/grasp_attach/check_physical", Trigger
        )
        self.attach_fallback = rospy.ServiceProxy(
            "/grasp_attach/attach", Trigger
        )
        self.global_obstacle_layer_client = None
        self.cmd_vel_source = None
        self._load_vision_model()
        self.vision_debug_timer = rospy.Timer(
            rospy.Duration(1.0 / self.vision_debug_rate),
            self._vision_debug_tick,
        )
        # A fresh task must never inherit low-speed mode from an earlier run.
        self.carry_mode_pub.publish(Bool(data=False))
        self.done_pub.publish(Bool(data=False))
        self._select_cmd_vel_source(
            "navigation", "task startup and move_base control"
        )
        self._set_navigation_mode(
            "main_legacy",
            "before pickup: origin/main CymPlanner for observation bays",
            configure_global_obstacle_layer=False,
        )

    @staticmethod
    def _resolve_category(requested, item):
        if requested and requested != "auto":
            category = CATEGORY_ALIASES.get(requested)
            if category is None:
                raise rospy.ROSException(
                    "Unknown cargo_category '%s'; use food/食品, daily/日用品, or electronics/电子产品"
                    % requested
                )
            return category
        candidate = str(item).strip().lower()
        category = CATEGORY_ALIASES.get(candidate) or ITEM_CATEGORIES.get(candidate)
        if category is None:
            raise rospy.ROSException(
                "Cannot infer category for cargo_item '%s'; pass cargo_category explicitly" % item
            )
        return category

    @staticmethod
    def _pose_param(name, default):
        pose = rospy.get_param(name, default)
        if isinstance(pose, str):
            pose = [value.strip() for value in pose.split(",") if value.strip()]
        if not isinstance(pose, (list, tuple)) or len(pose) != len(ARM_JOINTS):
            raise rospy.ROSException("%s must contain five joint angles" % name)
        return [float(value) for value in pose]

    @staticmethod
    def _read_search_regions(regions):
        if not isinstance(regions, list) or len(regions) != 3:
            raise rospy.ROSException(
                "vision_search_regions must contain left, upper, and right entries"
            )
        parsed = []
        for region in regions:
            if not isinstance(region, dict):
                raise rospy.ROSException("each vision search region must be a mapping")
            missing = [
                key for key in (
                    "name", "display_name", "observation_goal",
                    "fallback_observation_goal", "grasp_target",
                    "grasp_acceptance",
                )
                if key not in region
            ]
            if missing:
                raise rospy.ROSException(
                    "vision region is missing keys: %s" % ", ".join(missing)
                )
            goal = [float(value) for value in region["observation_goal"]]
            fallback_goal = [
                float(value)
                for value in region["fallback_observation_goal"]
            ]
            target = [float(value) for value in region["grasp_target"]]
            if (
                len(goal) != 3
                or len(fallback_goal) != 3
                or len(target) != 4
            ):
                raise rospy.ROSException(
                    "%s observation/fallback/grasp target dimensions are invalid"
                    % region["name"]
                )
            acceptance = {}
            for key in ("center_x", "center_y", "width", "height"):
                values = [
                    float(value) for value in region["grasp_acceptance"].get(key, [])
                ]
                if len(values) != 2 or values[0] > values[1]:
                    raise rospy.ROSException(
                        "%s grasp_acceptance.%s must be [minimum, maximum]"
                        % (region["name"], key)
                    )
                acceptance[key] = values
            parsed.append({
                "name": str(region["name"]),
                "display_name": str(region["display_name"]),
                "observation_goal": goal,
                "fallback_observation_goal": fallback_goal,
                "grasp_target": target,
                "grasp_acceptance": acceptance,
            })
        expected_names = ["left", "upper", "right"]
        if [region["name"] for region in parsed] != expected_names:
            raise rospy.ROSException(
                "vision_search_regions order must be exactly left, upper, right"
            )
        anchor_x, anchor_y, anchor_yaw = parsed[0]["observation_goal"]
        for index, region in enumerate(parsed):
            x, y, yaw = region["observation_goal"]
            expected_yaw = anchor_yaw - index * math.pi / 2.0
            yaw_error = math.atan2(
                math.sin(yaw - expected_yaw),
                math.cos(yaw - expected_yaw),
            )
            if (
                abs(x - anchor_x) > 1e-6
                or abs(y - anchor_y) > 1e-6
                or abs(yaw_error) > 1e-3
            ):
                raise rospy.ROSException(
                    "vision_search_regions must keep one XY and turn clockwise "
                    "90 degrees from left to upper to right"
                )
        return parsed

    def _load_vision_model(self):
        if not os.path.isfile(self.vision_model_path):
            raise rospy.ROSException(
                "YOLOv5 ONNX model does not exist: %s" % self.vision_model_path
            )
        if not os.path.isdir(self.vision_label_template_dir):
            raise rospy.ROSException(
                "visual label template directory does not exist: %s"
                % self.vision_label_template_dir
            )
        try:
            import onnxruntime as ort
        except ImportError as error:
            raise rospy.ROSException(
                "onnxruntime is required; from the workspace root run "
                "'python3 -m pip install -r requirements-vision.txt'"
            ) from error

        options = ort.SessionOptions()
        options.intra_op_num_threads = max(1, int(rospy.get_param("~vision_cpu_threads", 2)))
        options.inter_op_num_threads = 1
        self.vision_session = ort.InferenceSession(
            self.vision_model_path,
            sess_options=options,
            providers=["CPUExecutionProvider"],
        )
        inputs = self.vision_session.get_inputs()
        outputs = self.vision_session.get_outputs()
        if len(inputs) != 1 or not outputs:
            raise rospy.ROSException("unexpected YOLOv5 ONNX input/output signature")
        self.vision_input_name = inputs[0].name
        self.vision_output_name = outputs[0].name
        template_files = {
            "food": "Food.png",
            "daily": "Daily_Necessities.png",
            "electronics": "Electronics.png",
        }
        self.vision_label_templates = []
        for class_name in self.vision_class_names:
            if class_name not in template_files:
                raise rospy.ROSException(
                    "no visual label template configured for class %s" % class_name
                )
            path = os.path.join(
                self.vision_label_template_dir, template_files[class_name]
            )
            template = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if template is None:
                raise rospy.ROSException(
                    "cannot read visual label template: %s" % path
                )
            template = cv2.resize(
                template, (128, 128), interpolation=cv2.INTER_AREA
            )
            self.vision_label_templates.append(
                cv2.threshold(
                    template,
                    0,
                    255,
                    cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU,
                )[1]
            )
        warmup = np.zeros(
            (1, 3, self.vision_input_size, self.vision_input_size), dtype=np.float32
        )
        output = self.vision_session.run(
            [self.vision_output_name], {self.vision_input_name: warmup}
        )[0]
        if output.ndim != 3 or output.shape[-1] != 5 + len(self.vision_class_names):
            raise rospy.ROSException(
                "YOLOv5 output shape %s does not match %d configured classes"
                % (output.shape, len(self.vision_class_names))
            )
        self._status(
            "YOLOv5 vision ready: %s; classes=%s"
            % (os.path.basename(self.vision_model_path), ",".join(self.vision_class_names))
        )

    @staticmethod
    def _image_message_to_bgr(message):
        encoding = str(message.encoding).lower()
        channels = {
            "bgr8": 3,
            "rgb8": 3,
            "bgra8": 4,
            "rgba8": 4,
            "mono8": 1,
        }.get(encoding)
        if channels is None:
            raise ValueError("unsupported camera encoding %s" % message.encoding)
        row_bytes = int(message.width) * channels
        if int(message.step) < row_bytes:
            raise ValueError("camera image step is shorter than one pixel row")
        raw = np.frombuffer(message.data, dtype=np.uint8)
        needed = int(message.height) * int(message.step)
        if raw.size < needed:
            raise ValueError("camera image data is truncated")
        rows = raw[:needed].reshape((int(message.height), int(message.step)))
        pixels = rows[:, :row_bytes].reshape(
            (int(message.height), int(message.width), channels)
        )
        if encoding == "bgr8":
            return pixels.copy()
        if encoding == "rgb8":
            return pixels[:, :, ::-1].copy()
        if encoding == "bgra8":
            return pixels[:, :, :3].copy()
        if encoding == "rgba8":
            return pixels[:, :, [2, 1, 0]].copy()
        return cv2.cvtColor(pixels, cv2.COLOR_GRAY2BGR)

    def _camera_callback(self, message):
        try:
            image = self._image_message_to_bgr(message)
        except (ValueError, TypeError) as error:
            rospy.logwarn_throttle(2.0, "Camera frame rejected: %s" % error)
            return
        with self.image_lock:
            self.latest_image = image
            self.latest_image_sequence += 1

    def _grasp_state_callback(self, message):
        self.grasp_state = message.data

    def _odom_callback(self, message):
        self.latest_odom = message

    def _joint_state_callback(self, message):
        if not all(name in message.name for name in ARM_JOINTS):
            return
        positions = [
            message.position[message.name.index(name)]
            for name in ARM_JOINTS
        ]
        with self.arm_joint_state_lock:
            self.latest_arm_joint_positions = positions
            self.latest_arm_joint_sequence += 1

    def _status(self, text):
        rospy.loginfo(text)
        self.status_pub.publish(String(data=text))

    def _select_cmd_vel_source(self, source, reason):
        if source not in ("navigation", "visual"):
            raise rospy.ROSException("unsupported cmd_vel source: %s" % source)
        self.cmd_vel_source_pub.publish(String(data=source))
        if source != self.cmd_vel_source:
            previous = self.cmd_vel_source or "unset"
            self.cmd_vel_source = source
            self._status(
                "Velocity control source=%s (previous=%s; %s)"
                % (source, previous, reason)
            )

    def _set_global_obstacle_layer(self, enabled):
        """Keep the origin/main global laser obstacle layer enabled."""
        try:
            if self.global_obstacle_layer_client is None:
                self.global_obstacle_layer_client = DynamicReconfigureClient(
                    "/move_base/global_costmap/obstacle_layer", timeout=5.0
                )
            self.global_obstacle_layer_client.update_configuration(
                {"enabled": bool(enabled)}
            )
        except Exception as error:
            raise rospy.ROSException(
                "cannot set global laser obstacle layer to %s: %s"
                % (enabled, error)
            )

    def _set_navigation_mode(self, mode, reason, configure_global_obstacle_layer=True):
        """Latch the planner and global-map mode at a task-phase boundary."""
        if mode not in ("main_legacy", "laser_avoidance"):
            raise rospy.ROSException("unsupported navigation mode: %s" % mode)
        if configure_global_obstacle_layer:
            # Both phases retain the global laser obstacle layer so the global
            # planner can build a detour after the local planner returns false.
            # The post-pickup local predicate itself consumes /scan directly.
            self._set_global_obstacle_layer(True)
        self.navigation_mode_pub.publish(String(data=mode))
        self._status(
            "Navigation mode=%s; global laser obstacle layer=enabled (%s)"
            % (mode, reason)
        )

    def _wall_elapsed(self):
        return time.monotonic() - self.task_wall_started

    def _wall_remaining(self):
        return self.task_wall_budget - self._wall_elapsed()

    def _check_wall_budget(self, context):
        remaining = self._wall_remaining()
        if remaining > 0.0:
            return
        if wall_budget_should_abort(
            remaining,
            self.task_wall_budget_strict,
        ):
            self.nav.cancel_all_goals()
            self.cmd_pub.publish(Twist())
            raise rospy.ROSException(
                "task wall-clock budget %.1f seconds exceeded during %s"
                % (self.task_wall_budget, context)
            )
        if not self.task_wall_budget_warning_sent:
            warning = (
                "Task wall-clock budget %.1f seconds exceeded during %s; "
                "continuing retries because task_wall_budget_strict=false"
                % (self.task_wall_budget, context)
            )
            rospy.logwarn(warning)
            self.status_pub.publish(String(data=warning))
            self.task_wall_budget_warning_sent = True

    def _wall_pause(self, duration, context="task execution"):
        deadline = time.monotonic() + max(0.0, float(duration))
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._check_wall_budget(context)
            time.sleep(max(0.0, min(0.05, deadline - time.monotonic())))
        self._check_wall_budget(context)

    def _wait_for_sim_duration(self, duration, context="task execution"):
        """Wait for a trajectory's Gazebo-time duration, independent of RTF."""
        required = max(0.0, float(duration))
        started = rospy.Time.now()
        previous = started
        while not rospy.is_shutdown():
            current = rospy.Time.now()
            if current < previous:
                # Gazebo can reset /clock when the world is restarted.
                started = current
            if (current - started).to_sec() >= required:
                break
            self._check_wall_budget(context)
            time.sleep(0.02)
            previous = current
        self._check_wall_budget(context)

    def _wait_for_services(self):
        for name in (
            "/gazebo/get_link_state",
            "/gazebo/set_model_state",
            "/controller_manager/switch_controller",
            "/controller_manager/list_controllers",
            "/move_base/clear_costmaps",
            "/move_base/global_costmap/obstacle_layer/set_parameters",
            "/grasp_attach/check_physical",
            "/grasp_attach/attach",
        ):
            self._status("Waiting for %s" % name)
            while not rospy.is_shutdown():
                self._check_wall_budget("waiting for %s" % name)
                try:
                    rospy.wait_for_service(name, timeout=0.5)
                    break
                except rospy.ROSException:
                    continue
        self._status("Waiting for move_base")
        while not rospy.is_shutdown():
            self._check_wall_budget("waiting for move_base")
            if self.nav.wait_for_server(rospy.Duration(0.5)):
                break

    def _verify_realtime_factor(self):
        self._status(
            "RTF preflight: measuring Gazebo for %.1f wall-clock seconds"
            % self.rtf_preflight_duration
        )
        wall_started = time.monotonic()
        sim_started = rospy.Time.now().to_sec()
        deadline = wall_started + self.rtf_preflight_duration
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            self._check_wall_budget("RTF preflight")
            time.sleep(max(0.0, min(0.05, deadline - time.monotonic())))
        wall_elapsed = max(time.monotonic() - wall_started, 1e-6)
        sim_elapsed = rospy.Time.now().to_sec() - sim_started
        measured_rtf = sim_elapsed / wall_elapsed
        if realtime_preflight_should_abort(
            measured_rtf,
            self.rtf_minimum,
            self.rtf_measurement_tolerance,
            self.rtf_preflight_strict,
        ):
            raise rospy.ROSException(
                "RTF preflight failed: measured %.4f, required %.3f "
                "(measurement tolerance %.3f)"
                % (
                    measured_rtf,
                    self.rtf_minimum,
                    self.rtf_measurement_tolerance,
                )
            )
        if not realtime_factor_is_acceptable(
            measured_rtf,
            self.rtf_minimum,
            self.rtf_measurement_tolerance,
        ):
            warning = (
                "RTF preflight warning: measured %.4f, recommended minimum "
                "%.3f (measurement tolerance %.3f); continuing because "
                "rtf_preflight_strict=false"
                % (
                    measured_rtf,
                    self.rtf_minimum,
                    self.rtf_measurement_tolerance,
                )
            )
            rospy.logwarn(warning)
            self.status_pub.publish(String(data=warning))
            return
        if measured_rtf < self.rtf_minimum:
            self._status(
                "RTF preflight passed within measurement tolerance: "
                "real_time_factor=%.4f, minimum=%.3f, tolerance=%.3f"
                % (
                    measured_rtf,
                    self.rtf_minimum,
                    self.rtf_measurement_tolerance,
                )
            )
        else:
            self._status(
                "RTF preflight passed: real_time_factor=%.4f (target %.3f)"
                % (measured_rtf, self.rtf_minimum)
            )

    def _latest_frame(self, after_sequence):
        with self.image_lock:
            if (
                self.latest_image is None
                or self.latest_image_sequence == after_sequence
            ):
                return None
            return self.latest_image_sequence, self.latest_image.copy()

    def _letterbox(self, image):
        height, width = image.shape[:2]
        scale = min(
            float(self.vision_input_size) / float(width),
            float(self.vision_input_size) / float(height),
        )
        resized_width = int(round(width * scale))
        resized_height = int(round(height * scale))
        resized = cv2.resize(
            image, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR
        )
        pad_x = (self.vision_input_size - resized_width) // 2
        pad_y = (self.vision_input_size - resized_height) // 2
        canvas = np.full(
            (self.vision_input_size, self.vision_input_size, 3), 114, dtype=np.uint8
        )
        canvas[
            pad_y:pad_y + resized_height, pad_x:pad_x + resized_width
        ] = resized
        rgb = canvas[:, :, ::-1].transpose((2, 0, 1))
        blob = np.ascontiguousarray(rgb, dtype=np.float32) / 255.0
        return blob[np.newaxis, :], scale, pad_x, pad_y

    @staticmethod
    def _ordered_quad(points):
        points = points.reshape(-1, 2).astype(np.float32)
        sums = points.sum(axis=1)
        differences = np.diff(points, axis=1).reshape(-1)
        return np.asarray(
            [
                points[np.argmin(sums)],
                points[np.argmin(differences)],
                points[np.argmax(sums)],
                points[np.argmax(differences)],
            ],
            dtype=np.float32,
        )

    def _template_classify(self, image, box):
        """Rectify the bright printed face and compare it with known labels."""
        x1, y1, x2, y2 = [float(value) for value in box]
        width = x2 - x1
        height = y2 - y1
        x1 = max(0, int(round(x1 - 0.20 * width)))
        y1 = max(0, int(round(y1 - 0.20 * height)))
        x2 = min(image.shape[1], int(round(x2 + 0.20 * width)))
        y2 = min(image.shape[0], int(round(y2 + 0.20 * height)))
        crop = image[y1:y2, x1:x2]
        if crop.size == 0:
            return None, 0.0, 0.0
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        mask = cv2.threshold(
            cv2.GaussianBlur(gray, (3, 3), 0),
            205,
            255,
            cv2.THRESH_BINARY,
        )[1]
        contours = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )[0]
        candidates = []
        minimum_area = 0.04 * crop.shape[0] * crop.shape[1]
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < minimum_area:
                continue
            polygon = cv2.approxPolyDP(
                contour, 0.035 * cv2.arcLength(contour, True), True
            )
            if len(polygon) == 4:
                candidates.append((area, polygon))
        if not candidates:
            return None, 0.0, 0.0

        _, polygon = max(candidates, key=lambda item: item[0])
        destination = np.asarray(
            [[0, 0], [127, 0], [127, 127], [0, 127]],
            dtype=np.float32,
        )
        rectified = cv2.warpPerspective(
            gray,
            cv2.getPerspectiveTransform(
                self._ordered_quad(polygon), destination
            ),
            (128, 128),
        )
        binary = cv2.threshold(
            rectified,
            0,
            255,
            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU,
        )[1]
        scores = [
            float(
                np.corrcoef(
                    binary.reshape(-1), template.reshape(-1)
                )[0, 1]
            )
            for template in self.vision_label_templates
        ]
        class_id = int(np.argmax(scores))
        ranked = sorted(scores, reverse=True)
        score = ranked[0]
        margin = ranked[0] - ranked[1]
        if (
            not math.isfinite(score)
            or score < self.vision_template_min_score
            or margin < self.vision_template_min_margin
        ):
            return None, score, margin
        return class_id, score, margin

    def _detect(self, image, region):
        self.vision_debug_region = region
        with self.vision_inference_lock:
            return self._detect_unlocked(image, region)

    def _detect_unlocked(self, image, region):
        blob, scale, pad_x, pad_y = self._letterbox(image)
        output = self.vision_session.run(
            [self.vision_output_name], {self.vision_input_name: blob}
        )[0]
        rows = np.squeeze(output, axis=0)
        if rows.shape[0] <= 5 + len(self.vision_class_names):
            rows = rows.transpose()

        image_height, image_width = image.shape[:2]
        candidates = []
        for row in rows:
            objectness = float(row[4])
            if objectness < self.vision_confidence:
                continue
            class_scores = row[5:5 + len(self.vision_class_names)]
            class_id = int(np.argmax(class_scores))
            confidence = objectness * float(class_scores[class_id])
            if confidence < self.vision_confidence:
                continue

            center_x, center_y, box_width, box_height = [
                float(value) for value in row[:4]
            ]
            x1 = clamp((center_x - box_width / 2.0 - pad_x) / scale, 0.0, image_width - 1.0)
            y1 = clamp((center_y - box_height / 2.0 - pad_y) / scale, 0.0, image_height - 1.0)
            x2 = clamp((center_x + box_width / 2.0 - pad_x) / scale, 0.0, image_width - 1.0)
            y2 = clamp((center_y + box_height / 2.0 - pad_y) / scale, 0.0, image_height - 1.0)
            if x2 <= x1 or y2 <= y1:
                continue
            candidates.append({
                "yolo_class_id": class_id,
                "yolo_class_name": self.vision_class_names[class_id],
                "confidence": confidence,
                "box_px": [x1, y1, x2, y2],
                "nms_box": [
                    int(round(x1)), int(round(y1)),
                    int(round(x2 - x1)), int(round(y2 - y1)),
                ],
            })

        detections = []
        indices = cv2.dnn.NMSBoxes(
            [item["nms_box"] for item in candidates],
            [item["confidence"] for item in candidates],
            self.vision_confidence,
            self.vision_nms,
        ) if candidates else []
        for index in np.asarray(indices).reshape(-1):
            item = candidates[int(index)]
            class_id, template_score, template_margin = self._template_classify(
                image, item["box_px"]
            )
            item["template_class_id"] = class_id
            item["template_score"] = template_score
            item["template_margin"] = template_margin
            item["template_class_name"] = (
                self.vision_class_names[class_id]
                if class_id is not None
                else "uncertain"
            )
            if class_id is None:
                class_id = item["yolo_class_id"]
            item["class_id"] = class_id
            item["class_name"] = self.vision_class_names[class_id]
            x1, y1, x2, y2 = item["box_px"]
            item["center_x"] = ((x1 + x2) / 2.0) / float(image_width)
            item["center_y"] = ((y1 + y2) / 2.0) / float(image_height)
            item["width"] = (x2 - x1) / float(image_width)
            item["height"] = (y2 - y1) / float(image_height)
            detections.append(item)
        detections.sort(key=lambda item: item["confidence"], reverse=True)
        self._publish_vision_debug(image, detections, region)
        return detections

    def _publish_vision_debug(self, image, detections, region):
        annotated = image.copy()
        image_height, image_width = annotated.shape[:2]
        target = region["grasp_target"]
        target_x = int(round(target[0] * image_width))
        target_y = int(round(target[1] * image_height))
        target_w = int(round(target[2] * image_width))
        target_h = int(round(target[3] * image_height))
        cv2.rectangle(
            annotated,
            (target_x - target_w // 2, target_y - target_h // 2),
            (target_x + target_w // 2, target_y + target_h // 2),
            (255, 255, 0),
            2,
        )
        cv2.drawMarker(
            annotated,
            (target_x, target_y),
            (255, 255, 0),
            markerType=cv2.MARKER_CROSS,
            markerSize=18,
            thickness=2,
        )
        for detection in detections:
            x1, y1, x2, y2 = [int(round(value)) for value in detection["box_px"]]
            colour = (
                (0, 220, 0)
                if detection["class_id"] == self.target_class_id
                else (0, 150, 255)
            )
            cv2.rectangle(annotated, (x1, y1), (x2, y2), colour, 2)
            cv2.putText(
                annotated,
                "%s (template:%.2f yolo:%s %.2f)"
                % (
                    detection["template_class_name"],
                    detection["template_score"],
                    detection["yolo_class_name"],
                    detection["confidence"],
                ),
                (x1, max(18, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                colour,
                2,
                cv2.LINE_AA,
            )
        message = Image()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "camera_rgb_optical_frame"
        message.height = image_height
        message.width = image_width
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = image_width * 3
        message.data = annotated.tobytes()
        self.vision_debug_pub.publish(message)
        self.vision_debug_last_publish_wall = time.monotonic()

    def _vision_debug_tick(self, _event):
        """Keep annotated RViz frames live outside visual-control loops."""
        if (
            time.monotonic() - self.vision_debug_last_publish_wall
            < 1.0 / self.vision_debug_rate
        ):
            return
        with self.image_lock:
            if (
                self.latest_image is None
                or self.latest_image_sequence == self.vision_debug_last_sequence
            ):
                return
            sequence = self.latest_image_sequence
            image = self.latest_image.copy()
            region = self.vision_debug_region
        if region is None:
            return
        if not self.vision_inference_lock.acquire(False):
            return
        try:
            self._detect_unlocked(image, region)
            self.vision_debug_last_sequence = sequence
        except Exception as error:
            rospy.logwarn_throttle(
                2.0, "Continuous YOLO debug frame failed: %s" % error
            )
        finally:
            self.vision_inference_lock.release()

    def _scan_region(self, region):
        deadline = rospy.Time.now() + rospy.Duration(self.vision_scan_timeout)
        last_sequence = -1
        seen_frames = 0
        try:
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                frame = self._latest_frame(last_sequence)
                if frame is None:
                    self._wall_pause(0.02, "camera region scan")
                    continue
                last_sequence, image = frame
                detections = self._detect(image, region)
                if not detections:
                    self.cmd_pub.publish(Twist())
                    self._wall_pause(0.02, "camera region scan")
                    continue

                cube = detections[0]
                seen_frames += 1
                self.cmd_pub.publish(Twist())
                self._status(
                    "Camera acquired a cube in %s region "
                    "(YOLO raw=%s, confidence=%.3f); starting observation classification"
                    % (
                        region["display_name"],
                        cube["yolo_class_name"],
                        cube["confidence"],
                    )
                )
                return cube
        finally:
            self.cmd_pub.publish(Twist())
        self._status(
            "No cube acquired in %s region; detector frames=%d"
            % (region["display_name"], seen_frames)
        )
        return None

    def _quick_classify_observation(self, region, initial_detection):
        """Return a class after repeated high-confidence YOLO observations.

        This fast path is allowed to skip a confidently non-target region while
        the vehicle is still at its observation pose.  The printed-face
        template is intentionally not used here: in the distant observation
        view it occupies too few pixels and caused long, unnecessary close
        approaches even when the retrained YOLO model was stable.  Low
        confidence, class changes, lost tracking, or timeout returns None so
        the existing close-range alignment and seven-frame template
        verification remains the safe fallback.
        """
        deadline = rospy.Time.now() + rospy.Duration(
            self.vision_quick_classify_timeout
        )
        last_sequence = -1
        tracked_center = (
            initial_detection["center_x"],
            initial_detection["center_y"],
        )
        votes = []
        try:
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                frame = self._latest_frame(last_sequence)
                if frame is None:
                    self._wall_pause(0.02, "observation classification")
                    continue
                last_sequence, image = frame
                detections = self._detect(image, region)
                if not detections:
                    votes = []
                    self._wall_pause(0.02, "observation classification")
                    continue
                cube = min(
                    detections,
                    key=lambda item: (
                        (item["center_x"] - tracked_center[0]) ** 2
                        + (item["center_y"] - tracked_center[1]) ** 2
                    ),
                )
                tracking_distance = math.hypot(
                    cube["center_x"] - tracked_center[0],
                    cube["center_y"] - tracked_center[1],
                )
                tracked_center = (cube["center_x"], cube["center_y"])
                yolo_id = cube["yolo_class_id"]
                reliable = (
                    tracking_distance <= 0.25
                    and cube["confidence"] >= self.vision_quick_min_confidence
                )
                if not reliable:
                    votes = []
                    self._wall_pause(0.02, "observation classification")
                    continue
                if votes and votes[-1]["yolo_class_id"] != yolo_id:
                    votes = []
                votes.append(cube)
                if len(votes) >= self.vision_quick_classify_frames:
                    selected = max(
                        votes, key=lambda item: item["confidence"]
                    )
                    self._status(
                        "Observation camera classified %s region as %s "
                        "(YOLO %d/%d, confidence %.3f..%.3f)"
                        % (
                            region["display_name"],
                            selected["yolo_class_name"],
                            len(votes),
                            self.vision_quick_classify_frames,
                            min(item["confidence"] for item in votes),
                            max(item["confidence"] for item in votes),
                        )
                    )
                    return selected
                self._wall_pause(0.02, "observation classification")
        finally:
            self.cmd_pub.publish(Twist())
        self._status(
            "Observation classification uncertain in %s; "
            "using close-range verification"
            % region["display_name"]
        )
        return None

    @staticmethod
    def _inside_grasp_range(detection, region):
        acceptance = region["grasp_acceptance"]
        values = {
            "center_x": detection["center_x"],
            "center_y": detection["center_y"],
            "width": detection["width"],
            "height": detection["height"],
        }
        return all(
            acceptance[key][0] <= value <= acceptance[key][1]
            for key, value in values.items()
        )

    @staticmethod
    def _ensure_minimum_speed(value, minimum):
        if value == 0.0 or abs(value) >= minimum:
            return value
        return math.copysign(minimum, value)

    def _visual_servo_command(self, horizontal_error, distance_error):
        """Translate toward the grasp view without changing the base yaw."""
        command = Twist()
        command.linear.y = clamp(
            self.vision_lateral_gain * horizontal_error,
            -self.vision_max_lateral,
            self.vision_max_lateral,
        )
        if abs(horizontal_error) > 0.020:
            command.linear.y = self._ensure_minimum_speed(
                command.linear.y, self.vision_min_lateral
            )
        else:
            command.linear.y = 0.0

        # Approach only after lateral error is small enough that the gripper
        # remains aimed at the selected cube.  Twist.angular stays zero.
        if abs(horizontal_error) <= 0.080:
            command.linear.x = clamp(
                self.vision_forward_gain * distance_error,
                -self.vision_max_forward,
                self.vision_max_forward,
            )
            # _vision_align calls this only while the detection is outside the
            # recorded grasp acceptance range.  A small non-zero correction
            # must still clear the base's static-friction deadzone; otherwise
            # a box just outside the range can stall forever at a few mm/s.
            if distance_error != 0.0:
                command.linear.x = self._ensure_minimum_speed(
                    command.linear.x, self.vision_min_forward
                )
        return command

    def _vision_align(self, region, initial_detection):
        self._select_cmd_vel_source(
            "visual", "camera servo at %s observation" % region["display_name"]
        )
        target = region["grasp_target"]
        deadline = rospy.Time.now() + rospy.Duration(self.vision_align_timeout)
        last_seen = rospy.Time.now()
        last_sequence = -1
        last_horizontal_error = target[0] - initial_detection["center_x"]
        tracked_center = (
            initial_detection["center_x"],
            initial_detection["center_y"],
        )
        stable_frames = 0
        self._status(
            "Visual servo active in %s region; target box=(%.3f, %.3f, %.3f, %.3f)"
            % (
                region["display_name"],
                target[0], target[1], target[2], target[3],
            )
        )
        try:
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                frame = self._latest_frame(last_sequence)
                if frame is None:
                    self._wall_pause(0.02, "visual alignment")
                    continue
                last_sequence, image = frame
                detections = self._detect(image, region)
                if detections:
                    detection = min(
                        detections,
                        key=lambda item: (
                            (item["center_x"] - tracked_center[0]) ** 2
                            + (item["center_y"] - tracked_center[1]) ** 2
                        ),
                    )
                    tracking_distance = math.hypot(
                        detection["center_x"] - tracked_center[0],
                        detection["center_y"] - tracked_center[1],
                    )
                    if tracking_distance > 0.25:
                        detection = None
                else:
                    detection = None
                if detection is None:
                    stable_frames = 0
                    missing_for = (rospy.Time.now() - last_seen).to_sec()
                    if missing_for >= self.vision_lost_timeout:
                        self._status(
                            "Visual alignment lost the selected cube for %.1f seconds"
                            % missing_for
                        )
                        return False
                    command = Twist()
                    if missing_for >= 0.5 and last_horizontal_error != 0.0:
                        command.linear.y = math.copysign(
                            self.vision_min_lateral, last_horizontal_error
                        )
                    self.cmd_pub.publish(command)
                    self._wall_pause(0.02, "visual alignment")
                    continue

                last_seen = rospy.Time.now()
                tracked_center = (
                    detection["center_x"],
                    detection["center_y"],
                )
                horizontal_error = target[0] - detection["center_x"]
                vertical_error = target[1] - detection["center_y"]
                height_error = target[3] - detection["height"]
                last_horizontal_error = horizontal_error

                if self._inside_grasp_range(detection, region):
                    stable_frames += 1
                    self.cmd_pub.publish(Twist())
                    if stable_frames >= self.vision_align_stable_frames:
                        self._status(
                            "Visual grasp range reached in %s: "
                            "box=(%.3f, %.3f, %.3f, %.3f), confidence=%.3f"
                            % (
                                region["display_name"],
                                detection["center_x"],
                                detection["center_y"],
                                detection["width"],
                                detection["height"],
                                detection["confidence"],
                            )
                        )
                        return True
                    self._wall_pause(0.02, "visual alignment")
                    continue

                stable_frames = 0
                distance_error = vertical_error + 0.45 * height_error
                command = self._visual_servo_command(
                    horizontal_error,
                    distance_error,
                )
                self.cmd_pub.publish(command)
                rospy.loginfo_throttle(
                    1.0,
                    "Vision servo %s: cx=%.3f cy=%.3f w=%.3f h=%.3f "
                    "cmd=(forward=%.3f, lateral=%.3f, yaw=0.000)"
                    % (
                        region["name"],
                        detection["center_x"],
                        detection["center_y"],
                        detection["width"],
                        detection["height"],
                        command.linear.x,
                        command.linear.y,
                    ),
                )
                self._wall_pause(0.02, "visual alignment")
        finally:
            self.cmd_pub.publish(Twist())
            self._wall_pause(0.05, "stopping visual alignment")
            self._select_cmd_vel_source(
                "navigation", "visual alignment finished"
            )
        self._status("Visual alignment timed out in %s region" % region["display_name"])
        return False

    def _classify_aligned_cube(self, region):
        """Classify only after the box is inside the recorded grasp range.

        The label occupies too few pixels at the observation pose, and lateral
        views introduce strong perspective distortion.  The recorded
        bottom-centre grasp view is the reliable classification domain.
        """
        deadline = rospy.Time.now() + rospy.Duration(self.vision_classify_timeout)
        last_sequence = -1
        votes = []
        target_x, target_y = region["grasp_target"][:2]
        try:
            while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                frame = self._latest_frame(last_sequence)
                if frame is None:
                    self._wall_pause(0.02, "grasp-view classification")
                    continue
                last_sequence, image = frame
                detections = self._detect(image, region)
                if not detections:
                    votes = []
                    self._wall_pause(0.02, "grasp-view classification")
                    continue
                cube = min(
                    detections,
                    key=lambda item: (
                        (item["center_x"] - target_x) ** 2
                        + (item["center_y"] - target_y) ** 2
                    ),
                )
                if (
                    not self._inside_grasp_range(cube, region)
                    or cube["template_class_id"] is None
                ):
                    votes = []
                    self._wall_pause(0.02, "grasp-view classification")
                    continue
                votes.append(cube)
                if len(votes) >= self.vision_classify_stable_frames:
                    break
                self._wall_pause(0.02, "grasp-view classification")
        finally:
            self.cmd_pub.publish(Twist())

        if len(votes) < self.vision_classify_stable_frames:
            self._status(
                "Aligned classification timed out in %s region"
                % region["display_name"]
            )
            return None

        counts = {}
        raw_counts = {}
        for vote in votes:
            template_id = vote["template_class_id"]
            counts[template_id] = counts.get(template_id, 0) + 1
            raw_id = vote["yolo_class_id"]
            raw_counts[raw_id] = raw_counts.get(raw_id, 0) + 1
        voted_class_id, support = max(
            counts.items(), key=lambda item: item[1]
        )
        if support <= len(votes) // 2:
            self._status(
                "Aligned classification in %s has no majority: %s"
                % (region["display_name"], counts)
            )
            return None
        voted_cube = max(
            (
                vote for vote in votes
                if vote["template_class_id"] == voted_class_id
            ),
            key=lambda vote: vote["template_score"],
        )
        raw_class_id, raw_support = max(
            raw_counts.items(), key=lambda item: item[1]
        )
        self._status(
            "Grasp-view camera classified %s region as %s "
            "(template vote=%d/%d score=%.3f margin=%.3f; "
            "YOLO raw=%s %d/%d)"
            % (
                region["display_name"],
                voted_cube["template_class_name"],
                support,
                len(votes),
                voted_cube["template_score"],
                voted_cube["template_margin"],
                self.vision_class_names[raw_class_id],
                raw_support,
                len(votes),
            )
        )
        return voted_cube

    def _search_target_at_observation(
        self, region, goal, phase, index
    ):
        ordinal = ("一", "二", "三")[index]
        if not self._move_base(
            goal,
            "%s第%s个物块观察位" % (phase, ordinal),
            observation_pose=True,
        ):
            self._status(
                "%s：未到达第%s个物块观察位（%s），继续搜索"
                % (phase, ordinal, region["display_name"])
            )
            return None

        # move_base can keep publishing for a short time after reporting
        # success.  Relinquish navigation before camera steering starts.
        self.nav.cancel_all_goals()
        self.cmd_pub.publish(Twist())
        self._wall_pause(0.20, "stopping at visual observation pose")
        prefix = "到达夹取区；" if phase == "旋转搜索" and index == 0 else ""
        self._status(
            prefix + "%s：开始识别第%s个物块（%s）"
            % (phase, ordinal, region["display_name"])
        )
        detection = self._scan_region(region)
        if detection is None:
            self._status(
                "%s：第%s个位置没有识别到物块"
                % (phase, ordinal)
            )
            return None

        quick_classified = self._quick_classify_observation(
            region, detection
        )
        if (
            quick_classified is not None
            and quick_classified["class_id"] != self.target_class_id
        ):
            self._status(
                "%s：第%s个物块不是目标 %s"
                % (phase, ordinal, self.category)
            )
            return None
        if quick_classified is not None:
            detection = quick_classified
            self._status(
                "%s：第%s个物块可能是 %s，进入近距离确认"
                % (phase, ordinal, self.category)
            )
        if not self._vision_align(region, detection):
            self._status(
                "%s：第%s个物块未能完成视觉对准"
                % (phase, ordinal)
            )
            return None
        classified = self._classify_aligned_cube(region)
        if classified is None:
            self._status(
                "%s：第%s个物块近距离分类未确认"
                % (phase, ordinal)
            )
            return None
        if classified["class_id"] != self.target_class_id:
            self._status(
                "%s：第%s个物块不是目标 %s"
                % (phase, ordinal, self.category)
            )
            return None
        self._status(
            "%s：第%s个物块已确认为目标 %s（%s）"
            % (phase, ordinal, self.category, region["display_name"])
        )
        return region

    def _find_and_align_target_once(self):
        search_passes = (
            ("旋转搜索", "observation_goal"),
            ("旧版观察位复查", "fallback_observation_goal"),
        )
        ordinals = ("一", "二", "三")
        for phase, goal_key in search_passes:
            if phase == "旧版观察位复查":
                self._status(
                    "旋转搜索三个方向均未确认目标；"
                    "切换旧版左/中/右观察位复查"
                )
            for index, region in enumerate(self.search_regions):
                result = self._search_target_at_observation(
                    region,
                    region[goal_key],
                    phase,
                    index,
                )
                if result is not None:
                    return result
                if phase == "旋转搜索" and index < 2:
                    self._status(
                        "旋转搜索：第%s个未确认目标；顺时针旋转90度，"
                        "准备识别第%s个物块"
                        % (ordinals[index], ordinals[index + 1])
                    )
                elif phase == "旧版观察位复查" and index < 2:
                    self._status(
                        "旧版观察位复查：第%s个未确认目标；"
                        "前往第%s个旧版观察位"
                        % (ordinals[index], ordinals[index + 1])
                    )
        return None

    def _find_and_align_target(self):
        scan_cycle = 0
        while not rospy.is_shutdown():
            scan_cycle += 1
            region = self._find_and_align_target_once()
            if region is not None:
                return region
            self._status(
                "YOLOv5 did not confirm %s in rotation plus legacy-pose "
                "scan cycle %d; restarting recognition from the left region"
                % (self.category, scan_cycle)
            )
            self._wall_pause(
                self.recognition_retry_delay,
                "restarting visual recognition",
            )
        raise rospy.ROSInterruptException(
            "visual recognition stopped during ROS shutdown"
        )

    def _observation_pose_reached(self, goal):
        if self.latest_odom is None:
            return False
        pose = self.latest_odom.pose.pose
        position_error = math.hypot(
            pose.position.x - goal[0], pose.position.y - goal[1]
        )
        orientation = pose.orientation
        current_yaw = transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )[2]
        yaw_error = math.atan2(
            math.sin(current_yaw - goal[2]),
            math.cos(current_yaw - goal[2]),
        )
        return (
            position_error <= self.observation_position_tolerance
            and abs(yaw_error) <= self.observation_yaw_tolerance
        )

    def _destination_pose_reached(self, goal):
        if self.latest_odom is None:
            return False
        pose = self.latest_odom.pose.pose
        position_error = math.hypot(
            pose.position.x - goal[0], pose.position.y - goal[1]
        )
        orientation = pose.orientation
        current_yaw = transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )[2]
        yaw_error = math.atan2(
            math.sin(current_yaw - goal[2]),
            math.cos(current_yaw - goal[2]),
        )
        return (
            position_error <= self.destination_position_tolerance
            and abs(yaw_error) <= self.destination_yaw_tolerance
        )

    def _move_base(
        self,
        goal,
        description,
        observation_pose=False,
        destination_pose=False,
        attempt_timeout=None,
        attempts=None,
    ):
        self._select_cmd_vel_source(
            "navigation", "move_base navigation to %s" % description
        )
        message = MoveBaseGoal()
        message.target_pose.header.frame_id = "map"
        message.target_pose.pose.position.x = goal[0]
        message.target_pose.pose.position.y = goal[1]
        message.target_pose.pose.orientation = quaternion_from_yaw(goal[2])
        self._status(
            "Navigating to %s: (%.4f, %.4f, %.4f)" % (description, goal[0], goal[1], goal[2])
        )
        terminal_states = {
            GoalStatus.PREEMPTED,
            GoalStatus.ABORTED,
            GoalStatus.REJECTED,
            GoalStatus.RECALLED,
            GoalStatus.LOST,
        }
        nav_attempt_timeout = (
            self.nav_timeout if attempt_timeout is None else float(attempt_timeout)
        )
        nav_attempts = self.nav_attempts if attempts is None else int(attempts)
        for attempt in range(1, nav_attempts + 1):
            self._check_wall_budget("navigation to %s" % description)
            message.target_pose.header.stamp = rospy.Time.now()
            self.nav.send_goal(message)
            attempt_deadline = time.monotonic() + nav_attempt_timeout
            position_samples = 0
            while not rospy.is_shutdown() and time.monotonic() < attempt_deadline:
                self._check_wall_budget("navigation to %s" % description)
                state = self.nav.get_state()
                destination_reached = (
                    destination_pose and self._destination_pose_reached(goal)
                )
                if state == GoalStatus.SUCCEEDED and not destination_pose:
                    return True
                observation_reached = (
                    observation_pose and self._observation_pose_reached(goal)
                )
                if observation_reached or destination_reached:
                    position_samples += 1
                    if position_samples >= 3:
                        self._select_cmd_vel_source(
                            "visual",
                            "requested destination pose reached"
                            if destination_reached
                            else "observation pose reached",
                        )
                        self.nav.cancel_all_goals()
                        self.cmd_pub.publish(Twist())
                        if destination_reached:
                            self._status(
                                "Destination pose reached within %.2f m and "
                                "%.2f rad"
                                % (
                                    self.destination_position_tolerance,
                                    self.destination_yaw_tolerance,
                                )
                            )
                        else:
                            self._status(
                                "Observation pose reached within %.2f m and %.2f rad; "
                                "fine cube alignment is delegated to visual servo"
                                % (
                                    self.observation_position_tolerance,
                                    self.observation_yaw_tolerance,
                                )
                            )
                        return True
                else:
                    position_samples = 0
                if state in terminal_states:
                    self._status(
                        "move_base attempt %d returned state %d; retrying"
                        % (attempt, state)
                    )
                    break
                self._wall_pause(0.05, "navigation to %s" % description)
            else:
                self._status(
                    "move_base attempt %d timed out after %.1f wall-clock "
                    "seconds; retrying" % (attempt, nav_attempt_timeout)
                )
            self.nav.cancel_all_goals()
            try:
                self.clear_costmaps()
            except rospy.ServiceException:
                pass
            self._wall_pause(0.25, "navigation retry")
        return False

    def _start_arm_control(self):
        # Stop the Gazebo-only parked-pose holder before a real controller
        # claims the same joints.  Publishing more than once avoids a lost
        # transient connection when task3_execute is launched in a new shell.
        for _ in range(3):
            self.arm_control_enabled_pub.publish(Bool(data=True))
            self._wall_pause(0.08, "enabling arm control")
        self._wall_pause(0.25, "enabling arm control")

        for _ in range(12):
            states = {
                controller.name: controller.state
                for controller in self.list_controllers().controller
            }
            needed = [
                name for name in ("arm_controller", "gripper_controller")
                if states.get(name) != "running"
            ]
            if not needed:
                self._status("Pickup pose reached: arm/gripper controllers enabled")
                return
            request = SwitchControllerRequest()
            request.start_controllers = needed
            request.strictness = SwitchControllerRequest.STRICT
            request.start_asap = True
            request.timeout = 2.0
            if self.switch_controllers(request).ok:
                self._wall_pause(0.15, "starting arm controllers")
                continue
            self._wall_pause(0.20, "starting arm controllers")
        states = {
            controller.name: controller.state
            for controller in self.list_controllers().controller
        }
        raise rospy.ROSException(
            "arm/gripper controllers could not be started; states=%s" % states
        )

    def _stop_arm_control(self):
        for _ in range(12):
            states = {
                controller.name: controller.state
                for controller in self.list_controllers().controller
            }
            running = [
                name for name in ("arm_controller", "gripper_controller")
                if states.get(name) == "running"
            ]
            if not running:
                self._status(
                    "Arm/gripper controllers stopped for recognition navigation"
                )
                return
            request = SwitchControllerRequest()
            request.stop_controllers = running
            request.strictness = SwitchControllerRequest.STRICT
            request.start_asap = True
            request.timeout = 2.0
            if self.switch_controllers(request).ok:
                self._wall_pause(0.15, "stopping arm controllers")
                continue
            self._wall_pause(0.20, "stopping arm controllers")
        states = {
            controller.name: controller.state
            for controller in self.list_controllers().controller
        }
        raise rospy.ROSException(
            "arm/gripper controllers could not be stopped; states=%s" % states
        )

    def _publish_arm_target(self, positions, duration):
        message = JointTrajectory()
        message.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        # Gazebo may report a bounded revolute joint on any equivalent
        # multi-turn branch.  Keep every command on the branch nearest the
        # current feedback; forcing a canonical [-pi, pi] target can otherwise
        # turn a small pose change into a several-revolution trajectory.
        sample = self._latest_arm_joint_sample()
        if sample is None or len(sample[1]) != len(positions):
            point.positions = list(positions)
        else:
            current_positions = sample[1]
            point.positions = [
                current + math.atan2(
                    math.sin(desired - current),
                    math.cos(desired - current),
                )
                for current, desired in zip(current_positions, positions)
            ]
        point.velocities = [0.0] * len(ARM_JOINTS)
        point.time_from_start = rospy.Duration(duration)
        message.points = [point]
        for _ in range(3):
            self.arm_pub.publish(message)
            self._wall_pause(0.05, "publishing arm trajectory")

    @staticmethod
    def _wrist_rotation_is_safe(current, expected, limit):
        error = math.atan2(
            math.sin(current - expected),
            math.cos(current - expected),
        )
        return abs(error) <= limit

    def _latest_arm_joint_sample(self):
        with self.arm_joint_state_lock:
            if self.latest_arm_joint_positions is None:
                return None
            return (
                self.latest_arm_joint_sequence,
                list(self.latest_arm_joint_positions),
            )

    def _move_arm(self, positions, duration, guard_wrist_rotation=False):
        self._publish_arm_target(positions, duration)
        if not guard_wrist_rotation:
            self._wait_for_sim_duration(duration + 0.20, "moving arm")
            return True

        stable_history = deque(maxlen=self.grasp_guard_history_samples)
        last_sequence = -1
        sim_started = rospy.Time.now()
        previous_sim_time = sim_started
        required_sim_duration = duration + 0.20
        while not rospy.is_shutdown():
            current_sim_time = rospy.Time.now()
            if current_sim_time < previous_sim_time:
                sim_started = current_sim_time
            if (
                current_sim_time - sim_started
            ).to_sec() >= required_sim_duration:
                break
            previous_sim_time = current_sim_time
            sample = self._latest_arm_joint_sample()
            if sample is not None and sample[0] != last_sequence:
                last_sequence, current = sample
                if self._wrist_rotation_is_safe(
                    current[4],
                    positions[4],
                    self.grasp_wrist_rotation_limit,
                ):
                    stable_history.append(current)
                elif stable_history:
                    hold = list(stable_history[0])
                    hold[4] = positions[4]
                    self._status(
                        "Grasp descent wrist guard stopped collision rotation: "
                        "joint5=%.3f rad, limit=%.3f rad; backing off to the "
                        "recent pre-contact approach pose"
                        % (
                            current[4],
                            self.grasp_wrist_rotation_limit,
                        )
                    )
                    self._publish_arm_target(
                        hold, self.grasp_guard_recovery_duration
                    )
                    self._wait_for_sim_duration(
                        self.grasp_guard_recovery_duration + 0.15,
                        "relieving grasp contact",
                    )
                    return False
            self._wall_pause(
                self.grasp_guard_poll_interval,
                "monitoring grasp wrist rotation",
            )
        return True

    def _set_gripper(self, position):
        for _ in range(3):
            self.gripper_pub.publish(Float64(data=position))
            self._wall_pause(0.08, "moving gripper")

    def _wait_for_grasp_state(self, wanted, timeout):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.grasp_state == wanted:
                return True
            self._wall_pause(0.05, "waiting for gripper state")
        return self.grasp_state == wanted

    def _set_cargo_to_tcp(self):
        tcp = self.get_link("car3::tcp_link", "world")
        if not tcp.success:
            raise rospy.ROSException("could not read TCP pose")
        state = ModelState()
        state.model_name = self.cargo_model
        state.pose = tcp.link_state.pose
        state.twist = tcp.link_state.twist
        state.reference_frame = "world"
        if not self.set_model(state).success:
            raise rospy.ROSException("could not align target cube with TCP")

    def _physical_grasp_succeeded(self):
        try:
            result = self.check_physical_grasp()
        except rospy.ServiceException as error:
            self._status("Physical grasp check failed: %s" % error)
            return False
        if result.success:
            self._status("Normal physics grasp confirmed: %s" % result.message)
            return True
        self._status("Normal physics grasp was not retained: %s" % result.message)
        return False

    def _request_fallback_attachment(self):
        try:
            result = self.attach_fallback()
        except rospy.ServiceException as error:
            self._status("Attachment fallback service failed: %s" % error)
            return False
        if result.success:
            self._status("Attachment fallback activated: %s" % result.message)
            return True
        self._status("Attachment fallback rejected: %s" % result.message)
        return False

    def _recover_after_failed_pick(self):
        self._status(
            "Physical grasp was not retained; opening gripper and restoring "
            "the initial arm pose before recognition restarts"
        )
        self._select_cmd_vel_source(
            "visual", "stopping base for failed-pick recovery"
        )
        self.nav.cancel_all_goals()
        self.cmd_pub.publish(Twist())
        self.carry_mode_pub.publish(Bool(data=False))
        self._set_navigation_mode(
            "main_legacy", "failed pickup: return to recognition navigation"
        )
        self._set_gripper(1.0)
        self._move_arm(self.arm_initial, self.arm_recovery_duration)
        self._stop_arm_control()
        self._select_cmd_vel_source(
            "navigation", "initial arm pose restored for recognition retry"
        )
        self._status(
            "Initial arm pose restored; restarting visual recognition"
        )

    def _pick(self):
        self._status("Opening gripper and moving camera/gripper to the visually aligned cube")
        self._set_gripper(1.0)
        self._wait_for_grasp_state("IDLE", 1.0)
        self._move_arm(
            self.arm_grasp,
            self.arm_grasp_duration,
            guard_wrist_rotation=True,
        )

        if not self.attachment_fallback_enabled:
            # Match the reference physical-gripper sequence.  The intermediate
            # lift/check is not part of the official attachment path.
            self._set_gripper(0.76)
            self._wall_pause(
                self.physical_grasp_settle, "settling normal physics grasp"
            )
            self._move_arm(self.arm_carry, self.arm_carry_duration)
            if not self._physical_grasp_succeeded():
                self._status(
                    "Closed gripper reached the carry pose without retaining "
                    "the cube"
                )
                return False
        else:
            self._status(
                "Aligning cargo with TCP for direct official attachment"
            )
            self._set_cargo_to_tcp()
            self._set_gripper(0.76)
            self._wall_pause(
                self.physical_grasp_settle,
                "settling direct official attachment",
            )
            if not self._request_fallback_attachment():
                self._status(
                    "Official attachment failed; retrying the pickup"
                )
                return False
            # The attached cube follows the TCP; fold to the carry pose and
            # wait for it to finish before navigation starts, so the moving
            # arm cannot be misread as an obstacle by the laser scanner.
            self._move_arm(self.arm_carry, self.arm_carry_duration)
            self._status(
                "吸附完成；先恢复携带姿势，再开始底盘导航"
            )
        # Upgrade navigation only after the selected physical or official
        # attachment path is secure and the arm has fully reached the carry
        # pose on both paths.
        # The cube can cross the laser field while the arm folds.  In the
        # global, non-rolling costmap that transient hit is stored in map
        # coordinates; later rays are stopped by the carried cube and cannot
        # necessarily raytrace through the old cell.  Clear that grasp-phase
        # ghost only after the cargo has reached its laser-safe carry pose.
        self.clear_costmaps()
        self._status(
            "携带姿势已就绪；已清除夹取阶段的全局代价地图激光残影"
        )
        self._set_navigation_mode(
            "laser_avoidance", "pickup complete: carrying cube to factory"
        )
        self.carry_mode_pub.publish(Bool(data=True))
        self._status(
            "Cargo is held; arm is in carry pose before origin/main path "
            "following starts"
        )
        return True

    def _acquire_cargo(self):
        pick_attempt = 0
        while not rospy.is_shutdown():
            region = self._find_and_align_target()
            self._status(
                "%s was selected from camera recognition in %s region"
                % (self.cargo_name, region["display_name"])
            )
            self._start_arm_control()
            pick_attempt += 1
            if self._pick():
                return region
            self._status(
                "Pickup attempt %d failed physical verification; retrying "
                "recognition and alignment" % pick_attempt
            )
            self._recover_after_failed_pick()
            self._wall_pause(
                self.grasp_retry_delay,
                "restarting after failed pickup",
            )
        raise rospy.ROSInterruptException(
            "cargo acquisition stopped during ROS shutdown"
        )

    def run(self):
        self._wait_for_services()
        self._set_navigation_mode(
            "main_legacy",
            "before pickup: origin/main CymPlanner for observation bays",
        )
        if self.task_sim_started.to_sec() <= 0.0:
            self.task_sim_started = rospy.Time.now()
        self._verify_realtime_factor()
        self._acquire_cargo()
        if not self._move_base(
            self.destination,
            self.destination_name,
            destination_pose=True,
            attempt_timeout=self.destination_nav_timeout,
            attempts=self.destination_nav_attempts,
        ):
            raise rospy.ROSException("cannot reach %s" % self.destination_name)
        self.cmd_pub.publish(Twist())
        self._check_wall_budget("publishing completion")
        wall_elapsed = self._wall_elapsed()
        sim_elapsed = max(
            0.0, (rospy.Time.now() - self.task_sim_started).to_sec()
        )
        effective_rtf = sim_elapsed / max(wall_elapsed, 1e-6)
        result = (
            "%s delivered to %s; gripper remains closed; "
            "wall=%.1fs sim=%.1fs effective_RTF=%.3f"
            % (
                self.cargo_name,
                self.destination_name,
                wall_elapsed,
                sim_elapsed,
                effective_rtf,
            )
        )
        self._status("DONE: " + result)
        self.done_pub.publish(Bool(data=True))
        rospy.spin()


if __name__ == "__main__":
    try:
        PickDeliverTask().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as error:
        rospy.logfatal("task3_pick_deliver failed: %s", error)
        raise
