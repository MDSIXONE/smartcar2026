#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import numpy as np
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Bool 
from sensor_msgs.msg import Image
import cv2
import os
from sensor_msgs.msg import Imu
import math
waypoints_dict = {
    "point1": (1.600, 0.400, 0.000, 1.000),  #
    "point2": (1.700, 0.600, 1.000, 0.000),  # 
    "point3": (0.300, 0.650, 1.000, 0.000),  # 二维码识别点 )
    "point4": (0.400, 1.100, 0.000, 1.000),
    "point5": (1.700, 1.200, 0.000, 1.000),
    "point6": (0.400, 2.100, 0.000, 1.000),  #识别货物点
    "point7": (2.000, 4.200, 0.000, 1.000),
    "point8": (2.800, 4.250, 0.707, 0.707)
}
points_list=["none","point1","point2","point3","point4","point5","point6","point7","point8"]
task_start_flag=0             
def switch(case):
    cases = waypoints_dict
    return cases.get(case, 'default pose')


def goal_pose(pose):
    goal_pose = MoveBaseGoal()

    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0]
    goal_pose.target_pose.pose.position.y = pose[1]
    goal_pose.target_pose.pose.orientation.z = pose[2]
    goal_pose.target_pose.pose.orientation.w = pose[3]

    return goal_pose
def callback_task_start(msg):
        task_start_flag = msg.data
def get_task_start_flag():
        return True
if __name__ == '__main__':
     rospy.init_node("navigatiobn_test")
     rospy.loginfo("Starting navigation_test node")
     client=actionlib.SimpleActionClient('move_base',MoveBaseAction)#建立server导航actionlib客户端server
     client.wait_for_server()
     task_start_sub = rospy.Subscriber("/awake_flag", Int8 , callback_task_start)
     index=0
     while True: 
        if get_task_start_flag():

            while index < 8:
                index=index+1
                rospy.loginfo('Task Start')
                pose = switch(points_list[index])
                goal=goal_pose(pose)
                client.send_goal(goal)
                client.wait_for_result()  # <180 RIGHT
                if index==3:
                    rospy.sleep(2)#执行扫描二维码操作
            
                elif index==6:
                    rospy.sleep(2)
                    rospy.loginfo("detect goods")
                rospy.loginfo("i have reached point")




                
     rospy.spin()