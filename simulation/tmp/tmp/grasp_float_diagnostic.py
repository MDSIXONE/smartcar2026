#!/usr/bin/env python3
"""Capture the first pickup's joint and cube/gripper motion at 50 Hz."""

import math
import sys
import time

import rospy
from gazebo_msgs.msg import LinkStates, ModelStates
from sensor_msgs.msg import JointState
from std_msgs.msg import String


ARM_NAMES = [
    "arm_joint1",
    "arm_joint2",
    "arm_joint3",
    "arm_joint4",
    "arm_joint5",
    "r_joint",
]
LINK_NAMES = [
    "car3::tcp_link",
    "car3::r_out_link",
    "car3::l_out_link",
]


class Capture:
    def __init__(self):
        self.status = ""
        self.joints = None
        self.models = None
        self.links = None
        self.started = None
        rospy.Subscriber("/sim_task3/status", String, self._status, queue_size=10)
        rospy.Subscriber("/joint_states", JointState, self._joints, queue_size=20)
        rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self._models, queue_size=10
        )
        rospy.Subscriber(
            "/gazebo/link_states", LinkStates, self._links, queue_size=10
        )

    def _status(self, message):
        self.status = message.data
        if self.started is None and "Opening gripper" in self.status:
            self.started = time.monotonic()

    def _joints(self, message):
        values = dict(zip(message.name, message.position))
        if all(name in values for name in ARM_NAMES):
            self.joints = [values[name] for name in ARM_NAMES]

    def _models(self, message):
        self.models = dict(zip(message.name, message.pose))

    def _links(self, message):
        self.links = dict(zip(message.name, message.pose))

    @staticmethod
    def _distance(first, second):
        return math.sqrt(
            (first.position.x - second.position.x) ** 2
            + (first.position.y - second.position.y) ** 2
            + (first.position.z - second.position.z) ** 2
        )

    def run(self):
        print(
            "wall,sim,status,"
            + ",".join(ARM_NAMES)
            + ",cube_z,tcp_z,cube_tcp,cube_right,cube_left",
            flush=True,
        )
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.started is not None:
                elapsed = time.monotonic() - self.started
                if (
                    self.joints is not None
                    and self.models is not None
                    and self.links is not None
                    and "cube_2" in self.models
                    and all(name in self.links for name in LINK_NAMES)
                ):
                    cube = self.models["cube_2"]
                    tcp = self.links["car3::tcp_link"]
                    right = self.links["car3::r_out_link"]
                    left = self.links["car3::l_out_link"]
                    values = [
                        "%.4f" % elapsed,
                        "%.4f" % rospy.Time.now().to_sec(),
                        self.status.replace(",", ";"),
                    ]
                    values.extend("%.6f" % value for value in self.joints)
                    values.extend(
                        [
                            "%.6f" % cube.position.z,
                            "%.6f" % tcp.position.z,
                            "%.6f" % self._distance(cube, tcp),
                            "%.6f" % self._distance(cube, right),
                            "%.6f" % self._distance(cube, left),
                        ]
                    )
                    print(",".join(values), flush=True)
                if elapsed >= 14.0:
                    return
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("grasp_float_diagnostic", anonymous=True)
    try:
        Capture().run()
    except rospy.ROSInterruptException:
        sys.exit(0)
