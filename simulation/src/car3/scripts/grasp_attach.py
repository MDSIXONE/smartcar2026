#!/usr/bin/env python3
"""Official r_joint-triggered Gazebo grasp follower with task compatibility."""

import threading

import rospy
import tf.transformations as T
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import (
    GetLinkState,
    GetModelState,
    GetWorldProperties,
    SetModelState,
)
from geometry_msgs.msg import Point, Pose, Quaternion, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, Float64, String
from std_srvs.srv import Trigger, TriggerResponse


def _qv_mult(q, v):
    qv = T.quaternion_multiply(
        T.quaternion_multiply(q, [v[0], v[1], v[2], 0.0]),
        T.quaternion_conjugate(q),
    )
    return (qv[0], qv[1], qv[2])


class GraspAttach:
    def __init__(self):
        self._state_lock = threading.RLock()
        self.close_threshold = rospy.get_param(
            "~gripper_close_threshold", 0.8
        )
        self.open_threshold = rospy.get_param(
            "~gripper_open_threshold", 0.89
        )
        self.gripper_link = rospy.get_param(
            "~gripper_link", "car3::tcp_link"
        )
        self.object_models = rospy.get_param(
            "~object_models", ["cube_0", "cube_1", "cube_2"]
        )
        self.attachment_enabled = bool(
            rospy.get_param("~attachment_enabled", True)
        )
        self.update_rate = float(rospy.get_param("~update_rate", 100.0))
        self.check_rate = float(rospy.get_param("~check_rate", 2.0))
        self.obj_half_x = float(rospy.get_param("~object_half_x", 0.02))
        self.obj_half_y = float(rospy.get_param("~object_half_y", 0.02))
        self.obj_half_z = float(rospy.get_param("~object_half_z", 0.02))
        self.physical_check_half_x = float(
            rospy.get_param("~physical_check_half_x", 0.06)
        )
        self.physical_check_half_y = float(
            rospy.get_param("~physical_check_half_y", 0.06)
        )
        self.physical_check_half_z = float(
            rospy.get_param("~physical_check_half_z", 0.08)
        )

        self.state = "IDLE"
        self.current_object = None
        self.r_joint_pos = None
        self.commanded_position = None
        self.offset_pos = None
        self.offset_quat = None
        self.grasp_success = False
        self._follow_timer = None

        rospy.loginfo("等待 Gazebo 服务...")
        rospy.wait_for_service("/gazebo/get_link_state")
        rospy.wait_for_service("/gazebo/get_model_state")
        rospy.wait_for_service("/gazebo/get_world_properties")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.get_link_srv = rospy.ServiceProxy(
            "/gazebo/get_link_state", GetLinkState
        )
        self.get_model_srv = rospy.ServiceProxy(
            "/gazebo/get_model_state", GetModelState
        )
        self.get_world_srv = rospy.ServiceProxy(
            "/gazebo/get_world_properties", GetWorldProperties
        )
        self.set_model_srv = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState
        )
        self._model_list = []
        self._model_list_stamp = rospy.Time(0)
        rospy.loginfo("Gazebo 服务已就绪")

        self.dist_pub = rospy.Publisher("~distance", Float32, queue_size=5)
        self.ready_pub = rospy.Publisher("~ready", Bool, queue_size=5)
        self.offset_pub = rospy.Publisher("~offset", Point, queue_size=5)
        self.state_pub = rospy.Publisher("~state", String, queue_size=5)
        self.attach_service = rospy.Service(
            "~attach", Trigger, self._attach_cb
        )
        self.physical_check_service = rospy.Service(
            "~check_physical", Trigger, self._check_physical_cb
        )
        self.gripper_pub = rospy.Publisher(
            "/gripper_controller/command", Float64, queue_size=1
        )
        self.joint_sub = rospy.Subscriber(
            "/joint_states", JointState, self._joint_cb
        )
        self.command_sub = rospy.Subscriber(
            "/gripper_controller/command",
            Float64,
            self._command_cb,
            queue_size=1,
        )
        self._check_timer = rospy.Timer(
            rospy.Duration(1.0 / self.check_rate), self._check_cb
        )

        rospy.loginfo(
            "grasp_attach 就绪 "
            "(close<%.2f, open>%.2f, obj=(%.3f,%.3f,%.3f), "
            "models=%s, attachment_enabled=%s, check=%.1fHz, "
            "follow=official_set_model_state@%.1fHz)",
            self.close_threshold,
            self.open_threshold,
            self.obj_half_x,
            self.obj_half_y,
            self.obj_half_z,
            self.object_models,
            self.attachment_enabled,
            self.check_rate,
            self.update_rate,
        )

    def _joint_cb(self, msg):
        if "r_joint" not in msg.name:
            return
        self.r_joint_pos = msg.position[msg.name.index("r_joint")]
        self._tick_state()

    def _command_cb(self, msg):
        with self._state_lock:
            self.commanded_position = msg.data
        self._tick_state()

    def _tick_state(self):
        with self._state_lock:
            if not self.attachment_enabled or self.r_joint_pos is None:
                return
            if (
                self.state == "IDLE"
                and self.r_joint_pos < self.close_threshold
            ):
                self._do_grasp()
            elif (
                self.state == "GRASPING"
                and self.commanded_position is not None
                and self.commanded_position > self.open_threshold
            ):
                self._do_release()

    def _gripper_is_closed(self):
        return (
            self.commanded_position is not None
            and self.commanded_position < self.close_threshold
        ) or (
            self.r_joint_pos is not None
            and self.r_joint_pos < self.close_threshold
        )

    def _attach_cb(self, _request):
        with self._state_lock:
            if not self.attachment_enabled:
                return TriggerResponse(
                    success=False,
                    message="official r_joint attachment is disabled",
                )
            if self.state == "GRASPING":
                return TriggerResponse(
                    success=True,
                    message="official set_model_state attachment already active",
                )
            if not self._gripper_is_closed():
                return TriggerResponse(
                    success=False,
                    message="gripper is not commanded closed",
                )
            if not self._do_grasp():
                return TriggerResponse(
                    success=False,
                    message="no object is inside the official TCP box",
                )
            return TriggerResponse(
                success=True,
                message="official set_model_state attachment activated",
            )

    def _check_physical_cb(self, _request):
        with self._state_lock:
            if not self._gripper_is_closed():
                return TriggerResponse(
                    success=False,
                    message="gripper is not commanded closed",
                )
            best = self._find_closest_in_box(
                self.physical_check_half_x,
                self.physical_check_half_y,
                self.physical_check_half_z,
            )
            if best is None:
                return TriggerResponse(
                    success=False,
                    message="cube did not remain inside the lifted gripper",
                )
            return TriggerResponse(
                success=True,
                message="%s remained inside the lifted gripper" % best[0],
            )

    def _get_gripper_pose(self):
        try:
            gripper = self.get_link_srv(self.gripper_link, "world")
            if not gripper.success:
                return None
            return gripper.link_state.pose
        except rospy.ServiceException:
            return None

    def _refresh_model_list(self):
        now = rospy.Time.now()
        if (now - self._model_list_stamp).to_sec() < 2.0:
            return
        try:
            response = self.get_world_srv()
            self._model_list = response.model_names
            self._model_list_stamp = now
        except rospy.ServiceException:
            pass

    def _get_model_offset(self, model_name, gripper_pose=None):
        self._refresh_model_list()
        if model_name not in self._model_list:
            return None
        if gripper_pose is None:
            gripper_pose = self._get_gripper_pose()
            if gripper_pose is None:
                return None

        g_pos = gripper_pose.position
        g_q = [
            gripper_pose.orientation.x,
            gripper_pose.orientation.y,
            gripper_pose.orientation.z,
            gripper_pose.orientation.w,
        ]
        try:
            obj = self.get_model_srv(model_name, "world")
            if not obj.success:
                return None
            o_pos = obj.pose.position
            o_q = [
                obj.pose.orientation.x,
                obj.pose.orientation.y,
                obj.pose.orientation.z,
                obj.pose.orientation.w,
            ]
        except rospy.ServiceException:
            return None

        world_dp = (
            o_pos.x - g_pos.x,
            o_pos.y - g_pos.y,
            o_pos.z - g_pos.z,
        )
        local = _qv_mult(T.quaternion_inverse(g_q), world_dp)
        distance = sum(value * value for value in world_dp) ** 0.5
        return (local, distance, o_q)

    def _find_closest_in_box(
        self, half_x=None, half_y=None, half_z=None
    ):
        half_x = self.obj_half_x if half_x is None else half_x
        half_y = self.obj_half_y if half_y is None else half_y
        half_z = self.obj_half_z if half_z is None else half_z
        gripper_pose = self._get_gripper_pose()
        if gripper_pose is None:
            return None

        best = None
        best_distance = float("inf")
        for model_name in self.object_models:
            result = self._get_model_offset(model_name, gripper_pose)
            if result is None:
                continue
            (px, py, pz), distance, object_quaternion = result
            in_box = (
                abs(px) <= half_x
                and abs(py) <= half_y
                and abs(pz) <= half_z
            )
            if in_box and distance < best_distance:
                best_distance = distance
                best = (
                    model_name,
                    px,
                    py,
                    pz,
                    distance,
                    object_quaternion,
                )
        return best

    def _check_cb(self, _event):
        best_distance = float("inf")
        best_px = best_py = best_pz = 0.0
        best_in_box = False
        for model_name in self.object_models:
            result = self._get_model_offset(model_name)
            if result is None:
                continue
            (px, py, pz), distance, _ = result
            if distance < best_distance:
                best_distance = distance
                best_px, best_py, best_pz = px, py, pz
                best_in_box = (
                    abs(px) <= self.obj_half_x
                    and abs(py) <= self.obj_half_y
                    and abs(pz) <= self.obj_half_z
                )
        self.dist_pub.publish(Float32(data=best_distance))
        self.ready_pub.publish(Bool(data=best_in_box))
        self.offset_pub.publish(
            Point(x=best_px, y=best_py, z=best_pz)
        )
        with self._state_lock:
            state = self.state
        self.state_pub.publish(String(data=state))

    def _do_grasp(self):
        best = self._find_closest_in_box()
        if best is None:
            return False
        model_name, px, py, pz, _distance, object_quaternion = best

        gripper_pose = self._get_gripper_pose()
        if gripper_pose is None:
            return False
        g_q = [
            gripper_pose.orientation.x,
            gripper_pose.orientation.y,
            gripper_pose.orientation.z,
            gripper_pose.orientation.w,
        ]

        self.current_object = model_name
        self.offset_pos = (px, py, pz)
        self.offset_quat = tuple(
            T.quaternion_multiply(
                T.quaternion_inverse(g_q), object_quaternion
            )
        )
        self.grasp_success = True
        self.state = "GRASPING"

        # Keep the official echo only after the physical joint has actually
        # crossed the close threshold.  Explicit task attachment can happen
        # slightly earlier from the commanded value.
        if (
            self.r_joint_pos is not None
            and self.r_joint_pos < self.close_threshold
        ):
            self.gripper_pub.publish(Float64(data=self.r_joint_pos))
        if self._follow_timer is None:
            self._follow_timer = rospy.Timer(
                rospy.Duration(1.0 / self.update_rate), self._follow_cb
            )
        return True

    def _do_release(self):
        if self._follow_timer is not None:
            self._follow_timer.shutdown()
            self._follow_timer = None
        self.offset_pos = None
        self.offset_quat = None
        self.current_object = None
        self.grasp_success = False
        self.state = "IDLE"

    def _follow_cb(self, _event):
        with self._state_lock:
            if (
                self.state != "GRASPING"
                or self.offset_pos is None
                or self.offset_quat is None
                or self.current_object is None
            ):
                return
            offset_pos = tuple(self.offset_pos)
            offset_quat = tuple(self.offset_quat)
            current_object = self.current_object

        try:
            gripper = self.get_link_srv(self.gripper_link, "world")
            if not gripper.success:
                return
            g_pos = gripper.link_state.pose.position
            g_q = [
                gripper.link_state.pose.orientation.x,
                gripper.link_state.pose.orientation.y,
                gripper.link_state.pose.orientation.z,
                gripper.link_state.pose.orientation.w,
            ]
        except rospy.ServiceException:
            return

        rotated = _qv_mult(g_q, offset_pos)
        target_position = Point(
            x=g_pos.x + rotated[0],
            y=g_pos.y + rotated[1],
            z=g_pos.z + rotated[2],
        )
        target_quaternion = T.quaternion_multiply(g_q, offset_quat)
        state = ModelState()
        state.model_name = current_object
        state.pose = Pose(
            position=target_position,
            orientation=Quaternion(
                x=target_quaternion[0],
                y=target_quaternion[1],
                z=target_quaternion[2],
                w=target_quaternion[3],
            ),
        )
        state.twist = Twist()
        state.reference_frame = "world"

        with self._state_lock:
            if (
                self.state != "GRASPING"
                or self.current_object != current_object
                or self.offset_pos != offset_pos
                or tuple(self.offset_quat) != offset_quat
            ):
                return
            try:
                self.set_model_srv(state)
            except rospy.ServiceException:
                pass


if __name__ == "__main__":
    rospy.init_node("grasp_attach")
    GraspAttach()
    rospy.spin()
