#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import cv2
import re
import numpy as np
import time
from pyzbar.pyzbar import decode

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class image_converter:
    def __init__(self):
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("/code_image", Image, queue_size=1)
        self.qrdata_pub = rospy.Publisher("/qr_data", String, queue_size=1)
        self.nav_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.tts_pub = rospy.Publisher("/voice/xf_tts_topic", String, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/image", Image, self.callback)

        self.qrdata = ['','','','']
        self.last_qr1 = 0
        self.last_qr2 = 0
        self.pub_flag = True

    def decode(self,data):

        self.last_qr2 = self.last_qr1 #k-2的数据

        self.last_qr1 = data #k-1的数据

        if(not cmp(data, self.last_qr1)) and (not cmp(data, self.last_qr2)) and self.pub_flag:
            
            msg = String()
            msg.data = "任务已领取"
            self.tts_pub.publish(msg)
            
            target=PoseStamped()
            target.header.frame_id='map' #设置自己的目标
            target.pose.position.x=float(data[0])
            target.pose.position.y=float(data[1])
            target.pose.orientation.z=float(data[2])
            target.pose.orientation.w=float(data[3])

            self.nav_pub.publish(target)

            self.pub_flag = False

            print("ok")

    def callback(self,data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        for barcode in decode(cv_image):
            
            codeData = barcode.data
            self.qrdata = re.findall(r"\-?\d+\.?\d*",codeData) #当前的数据

            self.decode(self.qrdata)

            print(self.qrdata)
            print(time.strftime("%H:%M:%S-")+codeData)
            self.qrdata_pub.publish(codeData)
            pts = np.array([barcode.polygon],np.int32)
            pts = pts.reshape((-1,1,2))
            cv2.polylines(cv_image,[pts],True,(255,0,255),5)
            pts2 = barcode.rect
            cv2.putText(cv_image,time.strftime("%H:%M:%S-")+codeData,(pts2[0],pts2[1]),cv2.FONT_HERSHEY_SIMPLEX,0.9,(255,0,255),2)

        cv2.waitKey(1)
        
        
        # 再将opencv格式额数据转换成ros image格式的数据发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print e

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("lax_qr_zbar")
        rospy.loginfo("Starting qr_zbar node")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down qr_zbar node."
        cv2.destroyAllWindows()


