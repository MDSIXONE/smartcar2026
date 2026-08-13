#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32,Int8
from cv_bridge import CvBridge
import cv2
import fcntl
import time
print(cv2.__version__)
