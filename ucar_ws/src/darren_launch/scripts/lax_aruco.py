#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import numpy as np
import time
import cv2
import cv2.aruco as aruco

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Int8


#相机纠正参数
dist=np.array(([[-0.295393, 0.073008, -0.001659, 0.000923, 0.000000]]))

mtx=np.array([[416.55154,   0.     , 322.70817],
 	     [   0.     , 413.70392, 244.50991],
 	     [   0.     ,   0.     ,   1.     ]])

#pos_reg为人物识别停车点
# pos_reg = np.array([0.151, -2.344, 0.507, 0.862])
# pos_reg = np.array([0.025, -2.434, 0.700, 0.714])
pos_reg = np.array([0.05, -2.390, 0.703, 0.711])

font = cv2.FONT_HERSHEY_SIMPLEX

class image_converter:
    def __init__(self):
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("/aruco_decode_image", Image, queue_size=1)
        self.qrdata_pub = rospy.Publisher("/aruco_id", String, queue_size=1)
        self.nav_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.tts_pub = rospy.Publisher("/voice/xf_tts_topic", String, queue_size=1)

        self.image_sub = rospy.Subscriber("/image", Image, self.callback)
        self.tts_flag_sub = rospy.Subscriber("/tts_over_flag", Int8, self.ttsCallback)

        self.bridge = CvBridge()

        self.last_aruco1 = "set_a"
        self.last_aruco2 = "set_b"
        self.pub_flag = True
        self.flag = 0
        self.arrive_b = 0
        self.tts_flag = 0

    def decode(self, data):

        self.last_aruco2 = self.last_aruco1  # k-2的数据
        self.last_aruco1 = data  # k-1的数据

        if (not cmp(data, self.last_aruco1)) and (not cmp(data, self.last_aruco2)) and self.pub_flag:
            if data == "[[0]]":
                msg = String()
                msg.data = "U-CAR本次运输的菜品是蔬菜"
                self.tts_pub.publish(msg)
                while not self.flag:
                    if self.tts_flag == 1:
                        target = PoseStamped()
                        target.header.frame_id = 'map'  # 设置自己的目标
                        target.pose.position.x = pos_reg[0]
                        target.pose.position.y = pos_reg[1]
                        target.pose.orientation.z = pos_reg[2]
                        target.pose.orientation.w = pos_reg[3]
                        self.nav_pub.publish(target)
                        self.flag = 1
                self.pub_flag = False

            if data == "[[1]]":
                msg = String()
                msg.data = "U-CAR本次运输的菜品是水果"
                self.tts_pub.publish(msg)
                while not self.flag:
                    if self.tts_flag == 1:
                        target = PoseStamped()
                        target.header.frame_id = 'map'  # 设置自己的目标
                        target.pose.position.x = pos_reg[0]
                        target.pose.position.y = pos_reg[1]
                        target.pose.orientation.z = pos_reg[2]
                        target.pose.orientation.w = pos_reg[3]
                        self.nav_pub.publish(target)
                        self.flag = 1
                self.pub_flag = False

            if data == "[[2]]":
                msg = String()
                msg.data = "U-CAR本次运输的菜品是肉类"
                self.tts_pub.publish(msg)
                while not self.flag:
                    if self.tts_flag == 1:
                        target = PoseStamped()
                        target.header.frame_id = 'map'  # 设置自己的目标
                        target.pose.position.x = pos_reg[0]
                        target.pose.position.y = pos_reg[1]
                        target.pose.orientation.z = pos_reg[2]
                        target.pose.orientation.w = pos_reg[3]
                        self.nav_pub.publish(target)
                        self.flag = 1
                self.pub_flag = False


    def arrive_B_Callback(self, data):
        self.arrive_b = data.data

    def ttsCallback(self, data):
        self.tts_flag = data.data

    def callback(self, data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        parameters = aruco.DetectorParameters_create()

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:

            rvec, tvec = aruco.estimatePoseSingleMarkers(corners, 0.05, mtx, dist)
            # 估计每个标记的姿态并返回值rvet和tvec ---不同
            # from camera coeficcients
            (rvec - tvec).any()  # get rid of that nasty numpy value array error

            for i in range(rvec.shape[0]):
                aruco.drawAxis(cv_image, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.03)
                aruco.drawDetectedMarkers(cv_image, corners)

            self.decode(str(ids))

            ###### DRAW ID #####
            cv2.putText(cv_image, "Id: " + str(ids), (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        else:
            ##### DRAW "NO IDS" #####
            cv2.putText(cv_image, "No Ids", (0, 64), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


        cv2.waitKey(1)

        # 再将opencv格式额数据转换成ros image格式的数据发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("lax_aruco")
        rospy.loginfo("Starting lax_aruco node")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down lax_aruco node.")
        cv2.destroyAllWindows()
