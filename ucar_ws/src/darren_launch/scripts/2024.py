#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#------------------------可用------------
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool 

if __name__ == '__main__':

      
    rospy.init_node("task_control")
    rospy.loginfo("Starting task_control node")
    while True:
        cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 1
        cmd_vel_msg.linear.y = 0
        cmd_vel_msg.linear.z = 0
        cmd_vel_msg.angular.x = 0
        cmd_vel_msg.angular.y = 0
        cmd_vel_msg.angular.y = 0
        cmd_vel_pub.publish(cmd_vel_msg)
    rospy.spin()
        