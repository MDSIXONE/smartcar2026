#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int8
import cv2
import os

# 巡逻点
waypoints=[
    # (0.819, 2.741, 0.713, 0.701),
    # (0.733, 3.984, 0.975, 0.223),
    (2.800, 4.246, -0.225,  0.974), #B
    (3.932, 2.626, -0.353,  0.979), #C
    (2.874, 1.449,  0.284,  0.936), #D
    # (0.808, -0.195, 1.000, -0.018)  #end
    (0.776, -0.294, 0.910, -0.414)
    #(0.733, -0.064, -0.698, 0.716)

]
# waypoints=[
#     # (0.819, 2.741, 0.713, 0.701),
#     # (0.733, 3.984, 0.975, 0.223),
#     (3.737, 4.185, 0.479,  0.878), #B
#     (3.655, 2.643, -0.062,  0.998), #C
#     (3.089, 1.059, 0.030,  1.000), #D
#     (0.808, -0.195, 1.000, -0.018)  #end
#     # (0.776, -0.294, 0.910, -0.414)
# ]
# waypoints=[
#     # (0.819, 2.741, 0.713, 0.701),
#     # (0.733, 3.984, 0.975, 0.223),
#     (3.328, 2.610, 0.072,  0.997), #C
#     (4.207, 4.075, 0.004,  1.000), #B
#     (3.459, 0.453, 0.259,  0.966), #D
#     (0.816, -0.206, 0.877, -0.481)  #end
# ]

def goal_pose(pose):
    goal_pose = MoveBaseGoal()

    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0]
    goal_pose.target_pose.pose.position.y = pose[1]
    goal_pose.target_pose.pose.orientation.z = pose[2]
    goal_pose.target_pose.pose.orientation.w = pose[3]

    return goal_pose

def function_judge(room):
    result = ''
    find_index = -1
    if {'food'}.issubset(room):
        result= '餐厅'
        find_index=0
    elif {'tableware'}.issubset(room):
        result= '餐厅'
        find_index=0
    elif {'bed'}.issubset(room):
        result= '卧室'
        find_index=1
    elif {'person', 'pet'}.issubset(room):
        result = '卧室'
        find_index=1

    elif {'sofa'}.issubset(room):
        result = '客厅'
        find_index=2
    elif {'tv'}.issubset(room) and set(room).issubset({'sofa','pet','chair','tv'}):
        result = '客厅'
        find_index=2
    return result, find_index
    #  if {'food','tv'}.issubset(room) or {'food'}.issubset(room):
    #     result= '餐厅'
    #     find_index=0
    # if {'tableware'}.issubset(room) or {'tableware','tv'}.issubset(room):
    #     result= '餐厅'
    #     find_index=0
    # if {'bed'}.issubset(room) or {'bed','tv'}.issubset(room):
    #     result= '卧室'
    #     find_index=1
    # if {'person', 'pet'}.issubset(room) or {'person', 'pet','tv'}.issubset(room):
    #     result = '卧室'
    #     find_index=1
    # # if {'tv'}.issubset(room):
    # #     result = '客厅'
    # #     find_index=2
    # if {'sofa'}.issubset(room):
    #     result = '客厅'
    #     find_index=2
    # return result, find_index



def judge_room():
    # 在下面的代码行中使用断点来调试脚本。
    filename = '/home/ucar/ucar_ws/src/image/result.txt'
    result = ['', '', '']
    room_find_flag = [0, 0, 0]  # canteen,bedroom,livingroom
    with open(filename, 'r') as file:
        line = file.readline()
        photo_1 = line.strip().split(' ')
        # photo_1 = [photo1[0], photo1[1], photo1[2]]
        line = file.readline()
        photo_2 = line.strip().split(' ')
        # photo_2 = [photo2[0], photo2[1], photo2[2]]
        line = file.readline()
        photo_3 = line.strip().split(' ')
        # photo_3 = [photo3[0], photo3[1], photo3[2]]
    ## 判断B房间
    result[0], find_index = function_judge(photo_1)
    if find_index != -1:
        room_find_flag[find_index] = 1
    ## 判断C房间
    result[1], find_index = function_judge(photo_2)
    if find_index != -1:
        room_find_flag[find_index] = 1
    ## 判断D房间
    result[2], find_index = function_judge(photo_3)
    if find_index != -1:
        room_find_flag[find_index] = 1
    try:
        f1 = result.index('')
        if room_find_flag.index(0) == 0:
            result[f1] = '餐厅'
            # room_find_flag[0]=1
        if room_find_flag.index(0) == 1:
            result[f1] = '卧室'
            # room_find_flag[1]=1
        if room_find_flag.index(0) == 2:
            result[f1] = '客厅'
            # room_find_flag[2]=1
        f1 = result.index('')
        if room_find_flag.index(0) == 0:
            result[f1] = '餐厅'
            # room_find_flag[0]=1
        if room_find_flag.index(0) == 1:
            result[f1] = '卧室'
            # room_find_flag[1]=1
        if room_find_flag.index(0) == 2:
            result[f1] = '客厅'
            # room_find_flag[2]=1
        f1 = result.index('')
        if room_find_flag.index(0) == 0:
            result[f1] = '餐厅'
            # room_find_flag[0]=1
        if room_find_flag.index(0) == 1:
            result[f1] = '卧室'
            # room_find_flag[1]=1
        if room_find_flag.index(0) == 2:
            result[f1] = '客厅'
            # room_find_flag[2]=1
              
        print(result)
    except:
        print(result)
# ####
#     try:
#         f1 = result.index('')
#         if room_find_flag.index(0) == 0:
#             result[f1] = '餐厅'
#             room_find_flag[0]=1
#         if room_find_flag.index(0) == 1:
#             result[f1] = '卧室'
#             room_find_flag[1]=1
#         if room_find_flag.index(0) == 2:
#             result[f1] = '客厅'
#             room_find_flag[2]=1
#         print(result)
#     except:
#         print(result)

#     try:
#         f1 = result.index('')
#         if room_find_flag.index(0) == 0:
#             result[f1] = '餐厅'
#             room_find_flag[0]=1
#         if room_find_flag.index(0) == 1:
#             result[f1] = '卧室'
#             room_find_flag[1]=1
#         if room_find_flag.index(0) == 2:
#             result[f1] = '客厅'
#             room_find_flag[2]=1
#         print(result)
#     except:
#         print(result)
# ###
    final_result = '任务完成,B房间为' + result[0] + ',C房间为' + result[1] + ',D房间为' + result[2] + '。'
    return final_result

class task_control:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.cmd_vel_sub = rospy.Subscriber("/teb_cmd_vel", Twist, self.cmd_vel_callback)

        self.tts_pub = rospy.Publisher("/voice/xf_tts_topic", String, queue_size=100)

        # self.task_start_sub = rospy.Subscriber("/task_start_flag", Int8 , self.callback_task_start)
        self.task_start_sub = rospy.Subscriber("/awake_flag", Int8 , self.callback_task_start)
        self.yolo_start_pub = rospy.Publisher("/yolo_start_flag", Int8, queue_size=1)

        self.change_inflation_pub = rospy.Publisher("/change_inflation_flag", Int8, queue_size=1)
        # self.yolo_over_sub = rospy.Subscriber("/yolo_over_flag", Int8 , self.callback_yolo_over)

        self.task_start_flag = 0
        # self.yolo_over_flag = 0
        self.rate1 = 150
        self.angular_turn_msg = 3.0

        self.rate = rospy.Rate(self.rate1)
        self.cap = cv2.VideoCapture("/dev/video0")
        ret, frame = self.cap.read()

    # def is_yolo_over():
    #     return self.yolo_over_flag

    def change_inflation(self):
        self.change_inflation_pub.publish(1)

    def yolo_start(self):
        self.yolo_start_pub.publish(1)

    # def callback_yolo_over(self, msg):
    #     self.yolo_over_flag = msg.data
    
    def callback_task_start(self, msg):
        self.task_start_flag = msg.data

    def get_task_start_flag(self):
        return self.task_start_flag

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
        self.cmd_vel_pub.publish(msg)
        
    def rotate(self, angle):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0
        if angle <= 180:
            cmd_vel_msg.angular.z = -self.angular_turn_msg
        else:
            angle = 360 - angle
            cmd_vel_msg.angular.z = self.angular_turn_msg

        angular_duration = angle / self.angular_turn_msg / 180 * 3.14159
        ticks = int(angular_duration * self.rate1)
        rospy.loginfo(ticks)
        for i in range(ticks):
            self.cmd_vel_pub.publish(cmd_vel_msg)
            self.rate.sleep()

        cmd_vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(Twist())

    def save_img(self, index, num, num_wait):
        for i in range(num_wait):
            ret, frame = self.cap.read()
            frame = cv2.flip(frame,1)   ##图像左右颠倒
        cv2.imwrite("/home/ucar/ucar_ws/src/image/" + str(index) + "_" + str(num) + ".jpg", frame)

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
                    client.wait_for_result()
                    if index == 1:
                        task.save_img(index, 0, cm_times)
                        task.rotate(270)
                        task.save_img(index, 1, cm_times)
                        task.rotate(270)
                        task.save_img(index, 2, cm_times)
                        task.rotate(290)
                        task.save_img(index, 3, cm_times)
                    elif index == 2:
                        task.save_img(index, 0, cm_times)
                        task.rotate(270)
                        task.save_img(index, 1, cm_times)
                        task.rotate(270)
                        task.save_img(index, 2, cm_times)
                        task.rotate(290)
                        task.save_img(index, 3, cm_times)
                    elif index == 3:
                        # task.rotate(290)
                        task.save_img(index, 0, cm_times)
                        task.rotate(90)
                        task.save_img(index, 1, cm_times)
                        task.rotate(90)
                        task.save_img(index, 2, cm_times)
                        task.rotate(60)
                        task.save_img(index, 3, cm_times)
                        task.yolo_start()
                    # if index == 1:
                    #     task.save_img(index, 0, 10)
                    #     # rospy.loginfo('旋转开始')
                    #     task.rotate(180)
                    #     # # task.rotate(180)
                    #     # rospy.loginfo('旋转结束')
                    # elif index == 2:
                    #     task.save_img(index, 0, 10)
                    #     task.rotate(250)
                    #     task.save_img(index, 1, 10)
                    #     task.rotate(250)
                    #     task.save_img(index, 2, 10)
                    # elif index == 3:
                    #     task.save_img(index, 0, 10)
                    #     task.rotate(200)
                    #     task.save_img(index, 1, 10)
                    #     task.yolo_start()
                    elif index == 4:
                        # while True:
                        #     if task.yolo_over_flag:
                        #         break
                        # task.stop()
                        os.system("rosnode kill /base_driver")
                        tts_result = judge_room()
                        rospy.loginfo(tts_result)
                        task.report_result(tts_result)
                task.cap_release()
                break
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down task_control node.")
