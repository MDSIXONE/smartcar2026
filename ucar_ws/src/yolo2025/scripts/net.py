#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32,Int8
from cv_bridge import CvBridge
import cv2
import fcntl
import time
from std_msgs.msg import String as ROSString

def icb(ms):
    print(ms.data)
rospy.init_node('cb')
image_sub = rospy.Subscriber("/cb", Int8,icb)
rospy.spin()