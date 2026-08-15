#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Route exactly one motion source to the chassis command topic."""

import rospy
import threading

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse


class CmdVelOwner(object):
    """The selected owner is the only source forwarded to the chassis."""

    def __init__(self):
        self.lock = threading.RLock()
        self.owner = "mission"
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel")
        self.mission_topic = rospy.get_param(
            "~mission_topic", "/cmd_vel/navigation")
        self.lane_topic = rospy.get_param("~lane_topic", "/cmd_vel/lane")
        self.output = rospy.Publisher(self.output_topic, Twist, queue_size=1)
        self.status = rospy.Publisher(
            "/cmd_vel_owner/state", String, queue_size=1, latch=True)
        rospy.Subscriber(
            self.mission_topic, Twist, self.mission_cb, queue_size=1)
        rospy.Subscriber(self.lane_topic, Twist, self.lane_cb, queue_size=1)
        self.switch = rospy.Service(
            "/cmd_vel_owner/set_lane_mode", SetBool, self.set_lane_mode)
        self.publish_owner()

    def publish_owner(self):
        self.status.publish(String(data=self.owner))

    def mission_cb(self, command):
        with self.lock:
            if self.owner == "mission":
                self.output.publish(command)

    def lane_cb(self, command):
        with self.lock:
            if self.owner == "lane":
                self.output.publish(command)

    def set_lane_mode(self, request):
        requested_owner = "lane" if request.data else "mission"
        with self.lock:
            if self.owner != requested_owner:
                self.owner = requested_owner
                self.publish_owner()
                rospy.loginfo("CMD_VEL_OWNER owner=%s", self.owner)
            return SetBoolResponse(True, self.owner)


if __name__ == "__main__":
    rospy.init_node("cmd_vel_owner")
    CmdVelOwner()
    rospy.spin()
