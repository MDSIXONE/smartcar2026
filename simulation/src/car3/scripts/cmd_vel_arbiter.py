#!/usr/bin/env python3
"""Expose one Gazebo cmd_vel publisher and switch between task controllers."""

import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


NAVIGATION_SOURCE = "navigation"
VISUAL_SOURCE = "visual"
VALID_SOURCES = {NAVIGATION_SOURCE, VISUAL_SOURCE}


class CmdVelArbiter:
    def __init__(self):
        rospy.init_node("cmd_vel_arbiter")
        self._lock = threading.Lock()
        self._selected = NAVIGATION_SOURCE
        self._output = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber(
            "/sim_task3/navigation_cmd_vel",
            Twist,
            self._navigation_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/sim_task3/visual_cmd_vel",
            Twist,
            self._visual_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            "/sim_task3/cmd_vel_source",
            String,
            self._source_callback,
            queue_size=1,
        )
        rospy.loginfo(
            "cmd_vel arbiter ready; selected source=%s", self._selected
        )

    def _forward(self, source, command):
        with self._lock:
            if self._selected == source:
                self._output.publish(command)

    def _navigation_callback(self, command):
        self._forward(NAVIGATION_SOURCE, command)

    def _visual_callback(self, command):
        self._forward(VISUAL_SOURCE, command)

    def _source_callback(self, message):
        requested = str(message.data).strip().lower()
        if requested not in VALID_SOURCES:
            rospy.logerr("cmd_vel arbiter rejected unknown source=%s", requested)
            return
        with self._lock:
            if requested == self._selected:
                return
            # Stop the previous source before accepting commands from the next.
            self._output.publish(Twist())
            previous = self._selected
            self._selected = requested
        rospy.loginfo(
            "cmd_vel source switched: %s -> %s", previous, requested
        )


if __name__ == "__main__":
    CmdVelArbiter()
    rospy.spin()
