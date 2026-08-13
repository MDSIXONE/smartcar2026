#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int8
from sensor_msgs.msg import Image
import cv2
import os
from sensor_msgs.msg import Imu
import math


# 巡逻点
# waypoints=[
#     # (-0.878, -1.276, 1.000, 0.007),#入坡点
#     # (-2.632, -1.092, -1.000, 0.018),#房间1(F)
#     # (-4.692, -1.067, -0.910, 0.414),#房间1内拍点
#     # (-2.282, -1.081,  0.017, 1.000),#出坡上坡点
#     # (-0.305, -1.013, -0.619, 0.785),#出坡下坡点
#     (-0.568, -3.343, -0.359, 0.933),#房间2入拍点(E)
#     # (-3.537, -3.074, -1.000, 0.020),#房间3入拍点(D)
#     # (-4.964, -3.067, -1.000, 0.008),#房间3内拍点
#     # (-1.799, -5.079,  0.029, 1.000),#房间5入拍点(C)    
#     # (-3.623, -5.104, -1.000, 0.022),#房间4入拍点(B)
#     # (-2.662, -2.864, 0.008, 1.000),#房间2反拍点

#     (-0.169, -0.225, 0.676, 0.737) #end
#     # (0.011, 0.006, 0.731, 0.682) #end
#     # (0.808, -0.195, 1.000, -0.018)  #end
#     # (0.776, -0.294, 0.910, -0.414)
#     # (0.783, -0.064, -0.698, 0.716)
# ]

# # 巡逻点
# waypoints=[
#     (-0.664, -3.290,-0.371, 0.928), #3房间
#     (-3.709, -3.112,0.813, 0.582), #5房间
#     (-1.956, -5.220,0.006, 1.000), #4房间
#     (-3.668, -5.263,-1.000, 0.007), #2房间
#     (-0.834, -1.246,1.000, 0.005), #入坡点
#     (-2.431, -1.265,1.000, 0.005), #下坡点
#     (-4.848, -1.257,-0.918, 0.396), #1房间内拍摄点
#     (-2.399, -1.255,0.009, 1.000), #出房间入坡点
#     (-0.847, -1.242,0.039, 0.999),#出房间下坡点
#     (-0.167, -0.174,0.710, 0.704) #入库点
#     # (-0.878, -1.276, 1.000, 0.007),#入坡点
#     # (-2.632, -1.092, -1.000, 0.018),#房间1(F)
#     # (-4.692, -1.067, -0.910, 0.414),#房间1内拍点
#     # (-2.282, -1.081,  0.017, 1.000),#出坡上坡点
#     # (-0.305, -1.013, -0.619, 0.785),#出坡下坡点
#     # (-0.568, -3.343, -0.359, 0.933),#房间2入拍点(E)
#     # (-3.537, -3.074, -1.000, 0.020),#房间3入拍点(D)
#     # (-4.964, -3.067, -1.000, 0.008),#房间3内拍点
#     # (-1.799, -5.079,  0.029, 1.000),#房间5入拍点(C)    
#     # (-3.623, -5.104, -1.000, 0.022),#房间4入拍点(B)
#     # (-2.662, -2.864, 0.008, 1.000),#房间2反拍点

#     # (-0.169, -0.225, 0.676, 0.737) #end
#     # (0.011, 0.006, 0.731, 0.682) #end
#     # (0.808, -0.195, 1.000, -0.018)  #end
#     # (0.776, -0.294, 0.910, -0.414)
#     # (0.783, -0.064, -0.698, 0.716)
# ]
# 巡逻点
waypoints=[
    (-0.742, -3.354,-0.324, 0.946), #E房间
    (-3.782, -3.162,-0.999, 0.055), #D房间
    (-2.319, -5.381,0.011, 1.000), #C房间
    (-3.761, -5.223,-0.979, 0.202), #B房间
    (-0.860, -1.230,-1.000, 0.001), #入坡点
    (-2.399, -1.269,-1.000, 0.002), #下坡点
    (-2.751, -1.834,1.000, 0.000), #F房间内拍摄点left
    # (-2.455, -0.710,-1.000, 0.001),#F房间内拍摄点right
    (-2.384, -1.264,0.017, 1.000), #出房间入坡点
    (-0.871, -1.237,0.024, 1.000),#出房间下坡点
    (-0.190, -0.234,0.694, 0.720) #入库点
]
def goal_pose(pose):
    goal_pose = MoveBaseGoal()

    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0]
    goal_pose.target_pose.pose.position.y = pose[1]
    goal_pose.target_pose.pose.orientation.z = pose[2]
    goal_pose.target_pose.pose.orientation.w = pose[3]

    return goal_pose
def function_judge_num(room):
    result = ''
    maize_num = 0
    cucumber_num = 0
    watermelon_num = 0
    max_num = 0
    for item in room:
        if item == 'watermelon':
            watermelon_num += 1
        elif item == 'maizeExposed':
            maize_num += 1
        elif item == 'twoMaizes':
            maize_num += 2
        elif item == 'twoExposedMaizes':
            maize_num += 2
        elif item == 'cucumber':
            cucumber_num += 1
        elif item == 'threeCucumbers':
            cucumber_num += 3
    if maize_num > cucumber_num and maize_num > watermelon_num:
        result = '玉米'
        max_num = maize_num
    elif cucumber_num > maize_num and cucumber_num > watermelon_num:
        result = '黄瓜'
        max_num = cucumber_num
    elif watermelon_num > maize_num and watermelon_num > cucumber_num:
        result = '西瓜'
        max_num = watermelon_num
    return result,max_num


def function_judge(room):
    result = ''
    find_index = -1        # 水稻 小麦 玉米 黄瓜
    if {'rice'}.issubset(room):
        result = '水稻'
        find_index = 0
    elif {'wheat'}.issubset(room) or {'maturewheat'}.issubset(room):
        result = '小麦'
        find_index = 1
    elif {'maize'}.issubset(room):  #set(room).issubset({'...','...','...','...'})  子集
        result = '玉米'
        find_index = 2
    elif {'cucumberStem'}.issubset(room) or {'cucumberAndStem'}.issubset(room):
        result = '黄瓜'
        find_index = 3
    return result, find_index

def judge_room():
    # 在下面的代码行中使用断点来调试脚本。
    filename = '/home/ucar/ucar_ws/src/image/result.txt'
    result = ['', '', '', '', '']
    room_find_flag = [0, 0, 0, 0]  # 水稻 小麦 玉米 黄瓜
    num = 0
    with open(filename, 'r') as file:
        line = file.readline()
        photo_D = line.strip().split(' ')
        line = file.readline()
        photo_E = line.strip().split(' ')
        line = file.readline()
        photo_C = line.strip().split(' ')
        line = file.readline()
        photo_B = line.strip().split(' ')
        line = file.readline()
        photo_F = line.strip().split(' ')        
    ## 判断B房间
    result[0], find_index = function_judge(photo_B)
    if find_index != -1:
        room_find_flag[find_index] = 1
    ## 判断C房间
    result[1], find_index = function_judge(photo_C)
    if find_index != -1:
        room_find_flag[find_index] = 1
    ## 判断D房间
    result[2], find_index = function_judge(photo_D)
    if find_index != -1:
        room_find_flag[find_index] = 1
    ## 判断E房间
    result[3], find_index = function_judge(photo_E)
    if find_index != -1:
        room_find_flag[find_index] = 1
    ## 判断F房间
    result[4], num = function_judge_num(photo_F)
    if num == 0:
        num = 10
    try:
        f1 = result.index('')
        if room_find_flag.index(0) == 0:  # 水稻 小麦 玉米 黄瓜
            if {'水稻'}.issubset(result) and {'小麦'}.issubset(result) and {'玉米'}.issubset(result):
                result[f1] = '黄瓜'
            elif {'水稻'}.issubset(result) and {'小麦'}.issubset(result) and {'黄瓜'}.issubset(result):
                result[f1] = '玉米'
            elif {'水稻'}.issubset(result) and {'玉米'}.issubset(result) and {'黄瓜'}.issubset(result):
                result[f1] = '小麦'
            elif {'小麦'}.issubset(result) and {'玉米'}.issubset(result) and {'黄瓜'}.issubset(result):
                result[f1] = '水稻'
        elif room_find_flag.index(0) == 1:
            if {'水稻'}.issubset(result) and {'小麦'}.issubset(result) and {'玉米'}.issubset(result):
                result[f1] = '黄瓜'
            elif {'水稻'}.issubset(result) and {'小麦'}.issubset(result) and {'黄瓜'}.issubset(result):
                result[f1] = '玉米'
            elif {'水稻'}.issubset(result) and {'玉米'}.issubset(result) and {'黄瓜'}.issubset(result):
                result[f1] = '小麦'
            elif {'小麦'}.issubset(result) and {'玉米'}.issubset(result) and {'黄瓜'}.issubset(result):
                result[f1] = '水稻'   
        elif room_find_flag.index(0) == 2:
            if {'水稻'}.issubset(result) and {'小麦'}.issubset(result) and {'玉米'}.issubset(result):
                result[f1] = '黄瓜'
            elif {'水稻'}.issubset(result) and {'小麦'}.issubset(result) and {'黄瓜'}.issubset(result):
                result[f1] = '玉米'
            elif {'水稻'}.issubset(result) and {'玉米'}.issubset(result) and {'黄瓜'}.issubset(result):
                result[f1] = '小麦'
            elif {'小麦'}.issubset(result) and {'玉米'}.issubset(result) and {'黄瓜'}.issubset(result):
                result[f1] = '水稻' 
        elif room_find_flag.index(0) == 3:
            if {'水稻'}.issubset(result) and {'小麦'}.issubset(result) and {'玉米'}.issubset(result):
                result[f1] = '黄瓜'
            elif {'水稻'}.issubset(result) and {'小麦'}.issubset(result) and {'黄瓜'}.issubset(result):
                result[f1] = '玉米'
            elif {'水稻'}.issubset(result) and {'玉米'}.issubset(result) and {'黄瓜'}.issubset(result):
                result[f1] = '小麦'
            elif {'小麦'}.issubset(result) and {'玉米'}.issubset(result) and {'黄瓜'}.issubset(result):
                result[f1] = '水稻'                                          
        print(result)
    except:
        print(result)
    final_result = '任务完成,B区域种植的作物为' + str(result[0]) + ',C区域种植的作物为' + str(result[1]) + ',D区域种植的作物为' + str(result[2]) + ',E区域种植的作物为' + str(result[3]) + ',F区域存放的果实为' + str(result[4]) + ',数量为' + str(num) + '个' + '。'
    return final_result

class task_control:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.cmd_vel_sub = rospy.Subscriber("/teb_cmd_vel", Twist, self.cmd_vel_callback)

        self.tts_pub = rospy.Publisher("/voice/xf_tts_topic", String, queue_size=100)

        # self.task_start_sub = rospy.Subscriber("/task_start_flag", Int8 , self.callback_task_start)
        self.task_start_sub = rospy.Subscriber("/awake_flag", Int8 , self.callback_task_start)
        self.yolo_start_pub = rospy.Publisher("/yolo_start_flag", Int8, queue_size=1)

        self.usb_cam_start_pub = rospy.Publisher("/usb_cam/start_flag", Int8, queue_size=1)
        self.usb_cam_image_finish = rospy.Subscriber("/usb_cam/finish_flag",Int8, self.callback_image_finish)

        self.imuflag_sub = rospy.Subscriber("/pub", Int8, self.imu_flag)

        self.change_inflation_pub = rospy.Publisher("/change_inflation_flag", Int8, queue_size=1)
        # self.yolo_over_sub = rospy.Subscriber("/yolo_over_flag", Int8 , self.callback_yolo_over)
        self.sub_ = rospy.Subscriber('/usb_cam/image_raw', Image, self.img_callback, ('args'))

        self.ramp_start = 0
        self.image_finish_flag = 0
        self.task_start_flag = 0
        self.imu_sub_flag = 0
        # self.yolo_over_flag = 0
        self.rate1 = 150
        self.angular_turn_msg = 3.0
        self.image_msg = String()
        self.cv_img = 0
        
        # for i in range(1000):
        #     self.usb_cam_start_pub.publish(1)

        self.rate = rospy.Rate(self.rate1)
        # self.cap = cv2.VideoCapture("/dev/video0")
        # ret, frame = self.cap.read()

    # def is_yolo_over():
    #     return self.yolo_over_flag

    def change_inflation():
        self.change_inflation_pub.publish(1)

    def yolo_start(self):
        self.yolo_start_pub.publish(1)

    def usb_cam_start(self,nums,times):
        for i in range(times):
            self.usb_cam_start_pub.publish(nums)

    # def callback_yolo_over(self, msg):
    #     self.yolo_over_flag = msg.data
    def callback_image_finish(self,msg):
        self.image_finish_flag = msg.data

    def get_image_finish_flag(self):
        return self.image_finish_flag

    def callback_task_start(self, msg):
        self.task_start_flag = msg.data

    def get_task_start_flag(self):
        return self.task_start_flag

    def imu_flag(self,msg):
        # if(msg.data==1):
        #     self.imu_sub_flag=1
        # else:
        #     self.imu_sub_flag=0
        self.imu_sub_flag = msg.data
        rospy.loginfo("--------%d--------",self.imu_sub_flag)

    def img_callback(self,ros_img_msg, args):

        # print(args)
        assert isinstance(ros_img_msg, Image)
        self.cv_img = np.frombuffer(ros_img_msg.data, dtype=np.uint8).reshape(ros_img_msg.height, ros_img_msg.width, -1)
        self.cv_img = cv2.cvtColor(self.cv_img, cv2.COLOR_RGB2BGR)
        self.cv_img = cv2.flip(self.cv_img,1)
        cv2.imwrite("/home/ucar/ucar_ws/src/image/" + str(1) + "_" + str(1) + ".jpg",self.cv_img)
        # cv2.imshow("cv_img", self.cv_img)
        # cv2.waitKey(1) 

    def save_img(self, index, num, num_wait):
        for i in range(num_wait):
        #     ret, frame = self.cap.read()
        # frame = cv2.flip(frame,1)   ##图像左右颠倒
            cv2.imwrite("/home/ucar/ucar_ws/src/image/" + str(index) + "_" + str(num) + ".jpg", frame)

    def stop(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0
        cmd_vel_msg.linear.y = 0
        cmd_vel_msg.linear.z = 0
        cmd_vel_msg.angular.x = 0
        cmd_vel_msg.angular.y = 0
        cmd_vel_msg.angular.y = 0
        self.cmd_vel_pub.publish(cmd_vel_msg)

    def cmd_vel_callback(self, msg):
        # rospy.loginfo("!!!!!!!!!!!!!!!%d!!!!!!!!!!!!!!",self.imu_sub_flag)
        # if(self.imu_sub_flag==1):
        #     cmd_vel_msg = Twist()
        #     cmd_vel_msg.linear.x = 0.5
        #     cmd_vel_msg.linear.y = 0
        #     cmd_vel_msg.linear.z = 0
        #     cmd_vel_msg.angular.x = 0
        #     cmd_vel_msg.angular.y = 0
        #     cmd_vel_msg.angular.y = 0
        #     self.cmd_vel_pub.publish(cmd_vel_msg)
        #     rospy.loginfo("go straight")
        # elif(self.imu_sub_flag==0):
        #     self.cmd_vel_pub.publish(msg)
        if(self.imu_sub_flag==1):
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.5
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.linear.z = 0
            cmd_vel_msg.angular.x = 0
            cmd_vel_msg.angular.y = 0
            cmd_vel_msg.angular.y = 0
            self.cmd_vel_pub.publish(cmd_vel_msg)
            self.ramp_start = 1
            rospy.loginfo("go straight")
        # elif(self.imu_sub_flag == 2 and self.ramp_start== 1 ):
        #     self.stop()
        #     self.imu_sub_flag = 0         
        #     self.ramp_start = 0
        else:
            self.cmd_vel_pub.publish(msg)
        # self.cmd_vel_pub.publish(msg)
        # cmd_vel_msg = Twist()
        # cmd_vel_msg.linear.x = 0.8
        # cmd_vel_msg.linear.y = 0
        # cmd_vel_msg.linear.z = 0
        # cmd_vel_msg.angular.x = 0
        # cmd_vel_msg.angular.y = 0
        # cmd_vel_msg.angular.y = 0
        # self.cmd_vel_pub.publish(cmd_vel_msg)
        
    def rotate(self, angle):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0
        if angle <= 180:
            cmd_vel_msg.angular.z = -self.angular_turn_msg
        else:
            angle = 360 - angle
            cmd_vel_msg.angular.z = self.angular_turn_msg

        angular_duration = angle / self.angular_turn_msg / 180.0 * 3.1415926
        ticks = int(angular_duration * self.rate1)
        rospy.loginfo(ticks)
        for i in range(ticks):
            self.cmd_vel_pub.publish(cmd_vel_msg)
            self.rate.sleep()

        cmd_vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(Twist())

    def imu_cb(self,imu_data):
        # Read the quaternion of the robot IMU
        x = imu_data.orientation.x
        y = imu_data.orientation.y
        z = imu_data.orientation.z
        w = imu_data.orientation.w
        # rospy.loginfo("---%f---,---%f---,---%f---,---%f---",x,y,z,w)
    
        # Read the angular velocity of the robot IMU
        w_x = imu_data.angular_velocity.x
        w_y = imu_data.angular_velocity.y
        w_z = imu_data.angular_velocity.z
        rospy.loginfo("---%f---,---%f---,---%f---",w_x,w_y,w_z)
        # Read the linear acceleration of the robot IMU
        a_x = imu_data.linear_acceleration.x
        a_y = imu_data.linear_acceleration.y
        a_z = imu_data.linear_acceleration.z
    
        # Convert Quaternions to Euler-Angles
        rpy_angle = [0, 0, 0]
        rpy_angle[0] = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        rpy_angle[1] = math.asin(2 * (w * y - z * x))
        rpy_angle[2] = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        
        return

    def cap_release(self):
        self.cap.release()

    def report_result(self,msg):
        self.tts_pub.publish(msg)


if __name__ == '__main__':
    try:
        # os.system("rosrun ucar_yolo darknet_ucar.py")

        # 初始化ros节点
        rospy.init_node("task_control")
        rospy.loginfo("Starting task_control node")

        #创建MoveBaseAction client
        client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
        #等待MoveBaseAction server启动
        client.wait_for_server()
        task = task_control()
        
        cm_times=10
        while True:
            if task.get_task_start_flag():
                for index, pose in enumerate(waypoints, 1):
                    goal=goal_pose(pose)
                    client.send_goal(goal)
                    client.wait_for_result()  # <180 RIGHT
                    if index == 1:
                        task.usb_cam_start(20,1000)
                        while(1):
                            if(task.image_finish_flag):
                                task.image_finish_flag = 0
                                break
                        task.rotate(110)
                        task.usb_cam_start(21,1000)
                        while(1):
                            if(task.image_finish_flag):
                                task.image_finish_flag = 0
                                break
                        task.rotate(70)
                        task.usb_cam_start(22,1000)
                        while(1):
                            if(task.image_finish_flag):
                                task.image_finish_flag = 0
                                break        
                        task.rotate(70)
                        task.usb_cam_start(23,1000)
                        while(1):
                            if(task.image_finish_flag):
                                task.image_finish_flag = 0
                                break                                                 
                    elif index == 2:
                        task.usb_cam_start(10,1000)
                        while(1):
                            if(task.image_finish_flag):
                                task.image_finish_flag = 0
                                break
                        task.rotate(80)
                        task.usb_cam_start(11,1000)
                        while(1):
                            if(task.image_finish_flag):
                                task.image_finish_flag = 0
                                break
                        task.rotate(185)
                        task.usb_cam_start(12,1000)
                        while(1):
                            if(task.image_finish_flag):
                                task.image_finish_flag = 0
                                break
                    elif index == 3:
                        task.usb_cam_start(30,1000)
                        while(1):
                            if(task.image_finish_flag):
                                task.image_finish_flag = 0
                                break
                        task.rotate(300)
                        task.usb_cam_start(31,1000)
                        while(1):
                            if(task.image_finish_flag):
                                task.image_finish_flag = 0
                                break
                        task.rotate(280)
                        task.usb_cam_start(32,1000)
                        while(1):
                            if(task.image_finish_flag):
                                task.image_finish_flag = 0
                                break
                    elif index == 4:
                        task.usb_cam_start(40,1500)
                        while(1):
                            if(task.image_finish_flag):
                                task.image_finish_flag = 0
                                break
                        task.rotate(60)
                        task.usb_cam_start(41,1500)
                        while(1):
                            if(task.image_finish_flag):
                                task.image_finish_flag = 0
                                break
                        task.rotate(60)
                        task.usb_cam_start(42,1500)
                        while(1):
                            if(task.image_finish_flag):
                                task.image_finish_flag = 0
                                break
                    elif index == 5:
                        pass
                    elif index == 6:
                        pass
                    elif index == 7:
                        task.usb_cam_start(50,1500)
                        while(1):
                            if(task.image_finish_flag):
                                task.image_finish_flag = 0
                                break
                        task.rotate(140)
                        task.usb_cam_start(51,1500)
                        while(1):
                            if(task.image_finish_flag):
                                task.image_finish_flag = 0
                                break
                        task.rotate(90)
                        task.usb_cam_start(52,1500)
                        while(1):
                            if(task.image_finish_flag):
                                task.image_finish_flag = 0
                                break
                    elif index == 8:
                        pass
                    elif index == 9:
                        task.yolo_start()           
                    elif index == 10:  #12
                        # while True:
                        #     if task.yolo_over_flag:
                        #         break
                        # task.stop()
                        os.system("rosnode kill /base_driver")
                        tts_result = judge_room()
                        rospy.loginfo(tts_result)
                        task.report_result(tts_result)
                break
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down task_control node.")