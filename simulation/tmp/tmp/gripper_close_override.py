#!/usr/bin/env python3
"""Diagnostic-only: replace the first 0.76 close command with a tighter one."""

import time

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class CloseOverride:
    def __init__(self):
        self.position = float(rospy.get_param("~position", 0.70))
        self.triggered = False
        self.publisher = rospy.Publisher(
            "/gripper_controller/command", Float64, queue_size=1
        )
        rospy.Subscriber("/joint_states", JointState, self._joint_state)

    def _joint_state(self, message):
        if self.triggered or "r_joint" not in message.name:
            return
        value = message.position[message.name.index("r_joint")]
        if value > 0.80:
            return
        self.triggered = True
        rospy.loginfo(
            "Overriding first physical close from %.3f to %.3f",
            value,
            self.position,
        )
        for _ in range(10):
            self.publisher.publish(Float64(data=self.position))
            time.sleep(0.02)


if __name__ == "__main__":
    rospy.init_node("gripper_close_override")
    CloseOverride()
    rospy.spin()
