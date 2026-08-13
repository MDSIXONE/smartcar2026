#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8,Int32
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
import time
import time
last=0
count=0
pref=str(int(time.time()))

class ROSImageReader:
    def __init__(self, topic_name="/usb_cam/image_raw"):
        self.bridge = CvBridge()
        self.current_frame = None
        self.frame_ready = False
        # 订阅图像话题
        self.sub = rospy.Subscriber(topic_name, Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)



    def image_callback(self, msg):
        global count,last
        try:
            # 将ROS图像消息转为OpenCV格式 (BGR)
            if abs(time.time()-last)<0.25:
                return
            last=time.time()
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            self.current_frame = cv_image
            self.frame_ready = True
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            count=count+1
            print(count)
            filename = "/home/ucar/pout/cam_out_"+pref+"_"+str(count)+".png"
            with open(filename, "w") as f:
                
                cv2.imwrite(filename, cv_image)#写入临时路径

        except CvBridgeError as e:
            rospy.logerr(f"Image conversion failed: {e}")
            self.frame_ready = False

    def read(self):
        """模拟cv2.VideoCapture.read()的接口"""
        if self.frame_ready and self.current_frame is not None:
            return True, self.current_frame.copy()  # 返回拷贝避免数据竞争
        return False, None
    
   


if __name__ == "__main__":
    rospy.init_node('imux')
    reader = ROSImageReader(topic_name="/usb_cam/image_raw")  # 根据实际话题调整

    rate = rospy.Rate(4)

    while not rospy.is_shutdown():
        ret,frame = reader.read()
        if not ret:
            rate.sleep()
            continue

        #handle(frame,reader)

        rate.sleep()
    
