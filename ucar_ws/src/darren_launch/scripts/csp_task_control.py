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
    (-0.565, -3.216,-0.582, 0.814),#E房间
    # (-0.742, -3.354,-0.324, 0.946), #E房间
    (-3.782, -3.162,-0.999, 0.055), #D房间
    (-2.319, -5.381,0.011, 1.000), #C房间
    (-3.707, -5.280,-1.000, 0.019),#B房间
    # (-3.610, -5.198,-1.000, 0.015), #B房间
    (-0.860, -1.230,-1.000, 0.001), #入坡点
    (-2.399, -1.269,-1.000, 0.002), #下坡点
    # (-2.990, -1.874,0.969, 0.247), #F房间左拍摄点 斜着
    (-2.937, -1.735,-1.000, 0.012),#F房间左拍摄点 正着
    (-2.908, -0.643,1.000, 0.006),#F房间右拍摄点 正着
    # (-2.798, -0.794,-0.986, 0.169), #F房间右拍摄点 斜着
    # (-2.751, -1.834,1.000, 0.000), #F房间内拍摄点left
    # (-2.985, -1.168,-1.000, 0.004),#F房间内拍摄点middle
    # (-3.784, -2.165,1.000, 0.007),#F房间内左移
    # (-4.957, -1.651,-0.951, 0.310),#F房间最内拍摄点
    # (-5.101, -2.055,-0.969, 0.245),#F房间最内拍摄点
    # (-2.455, -0.710,-1.000, 0.001),#F房间内拍摄点right
    # (-3.497, -2.117,0.077, 0.997),#F房间出来缓冲点
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
        elif item == 'twoHorizontalCucumber':
            cucumber_num += 2
        elif item == 'twoVerticalCucumber':
            cucumber_num += 2
    max_num = max(maize_num,cucumber_num,watermelon_num)
    if(max_num == maize_num):
        result = '玉米'
    elif(max_num == cucumber_num):
        result = '黄瓜'
    elif(max_num == watermelon_num):
        result = '西瓜'
    else:result = '玉米'
    # if maize_num > cucumber_num and maize_num > watermelon_num:
    #     result = '玉米'
    #     max_num = maize_num
    # elif cucumber_num > maize_num and cucumber_num > watermelon_num:
    #     result = '黄瓜'
    #     max_num = cucumber_num
    # elif watermelon_num > maize_num and watermelon_num > cucumber_num:
    #     result = '西瓜'
    #     max_num = watermelon_num
    return result,max_num

def function_judge_vegetation(room):
    result = ''
    find_index = -1        # 水稻 小麦 玉米 黄瓜
    rice_num = 0
    wheat_num = 0
    maize_num = 0
    cucumberandStem_num = 0
    max_num = 0
    for item in room:
        if item == 'rice':
            rice_num += 1
        elif item == 'wheat' or item == 'matureWheat':
            wheat_num += 1
        elif item == 'maize':
            maize_num += 1
        elif item == 'cucumberStem' or item == 'cucumberAndStem':
            cucumberandStem_num += 1
    max_num = max(rice_num,wheat_num,maize_num,cucumberandStem_num)
    if(max_num == maize_num):
        result = '玉米'
        find_index = 2
    elif(max_num == cucumberandStem_num):
        result = '黄瓜'
        find_index = 3
    elif(max_num == wheat_num):
        result = '小麦'
        find_index = 1
    elif(max_num == rice_num):
        result = '水稻'
        find_index = 0
    else:
        result = '黄瓜'
    # if {'rice'}.issubset(room):
    #     result = '水稻'
    #     find_index = 0
    # elif {'wheat'}.issubset(room) or {'maturewheat'}.issubset(room):
    #     result = '小麦'
    #     find_index = 1
    # elif {'maize'}.issubset(room):  #set(room).issubset({'...','...','...','...'})  子集
    #     result = '玉米'
    #     find_index = 2
    # elif {'cucumberStem'}.issubset(room) or {'cucumberAndStem'}.issubset(room):
    #     result = '黄瓜'
    #     find_index = 3
    return result, find_index

# def function_judge_vegetation(room):
#     result = ''
#     find_index = -1        # 水稻 小麦 玉米 黄瓜
#     if {'rice'}.issubset(room):
#         result = '水稻'
#         find_index = 0
#     elif {'wheat'}.issubset(room) or {'maturewheat'}.issubset(room):
#         result = '小麦'
#         find_index = 1
#     elif {'maize'}.issubset(room):  #set(room).issubset({'...','...','...','...'})  子集
#         result = '玉米'
#         find_index = 2
#     elif {'cucumberStem'}.issubset(room) or {'cucumberAndStem'}.issubset(room):
#         result = '黄瓜'
#         find_index = 3
#     return result, find_index

def judge_vegetation_room(first,second):
    result = ''
    find_index = -1        # 水稻 小麦 玉米 黄瓜
    rice_number = 0
    wheat_number = 0
    maize_number = 0
    cucumber_number = 0
    max_number = 0
    rice_prob = 0.0
    wheat_prob = 0.0
    maize_prob = 0.0
    cucumber_prob = 0.0
    max_prob = 0.0
    file_path = "/home/ucar/ucar_ws/src/txt_total/"+str(first)+"_"+str(second)+".txt"
    fileHandler = open(file_path, 'r')
    lines  =  fileHandler.readlines()
    # while True:
        
    #     # if not line:
    #     #     break
    #     if line == 'rice':
    #         rice_number = rice_number + 1
    #     elif line == 'wheat' or line == 'matureWheat':
    #         wheat_number = wheat_number + 1
    #     elif line == 'maize':
    #         maize_number = maize_number + 1
    #     elif line == 'cucumberStem' or line == 'cucumberAndStem':
    #         cucumber_number = cucumber_number + 1
    #     else:
    #         break
    #     # print(line.strip())
    for ann in lines:
        ann = ann.strip('\n')       #去除文本中的换行符
        line_list = ann.split(':')
        if line_list[0] == 'rice':
            rice_number = rice_number + 1
            rice_prob = rice_prob + float(line_list[1])
        elif line_list[0] == 'wheat' or line_list[0] == 'matureWheat':
            wheat_number = wheat_number + 1
            wheat_prob = wheat_prob + float(line_list[1])
        elif line_list[0] == 'maize':
            maize_number = maize_number + 1
            maize_prob = maize_prob + float(line_list[1])
        elif line_list[0] == 'cucumberStem' or line_list[0] == 'cucumberAndStem':
            cucumber_number = cucumber_number + 1
            cucumber_prob = cucumber_prob + float(line_list[1])
        else:
            break
        rospy.loginfo("%s:%f",str(line_list[0]),float(line_list[1]))

    fileHandler.close()

    max_number = max(rice_number,wheat_number,maize_number,cucumber_number)
    
    if rice_number != 0:
        rice_prob = rice_prob/rice_number
    else:
        rice_prob = 0
    if wheat_number != 0:
        wheat_prob = wheat_prob/wheat_number
    else:
        wheat_prob = 0
    if maize_number != 0:
        maize_prob = maize_prob/maize_number
    else:
        maize_prob = 0
    if cucumber_number != 0:
        cucumber_prob = cucumber_prob/cucumber_number
    else:
        cucumber_prob = 0
    max_prob = max(rice_prob,wheat_prob,maize_prob,cucumber_prob)
    rospy.loginfo("########%f########",max_prob)

    if(abs(max_prob - rice_prob)<1e-06):
        result = '水稻'
        find_index = 0
    elif(abs(max_prob - wheat_prob)<1e-06):
        result = '小麦'
        find_index = 1
    elif(abs(max_prob - maize_prob)<1e-06):
        result = '玉米'
        find_index = 2
    elif(abs(max_prob - cucumber_prob)<1e-06):
        result = '黄瓜'
        find_index = 3
    else:
        result = '水稻'
        find_index = -1

    if(max_prob == rice_prob):
        if(max_number - rice_number > 2):
            if(max_number == rice_number):
                result = '水稻'
                find_index = 0
            elif(max_number == wheat_number):
                result = '小麦'
                find_index = 1
            elif(max_number == maize_number):
                result = '玉米'
                find_index = 2
            elif(max_number == cucumber_number):
                result = '黄瓜'
                find_index = 3
            else:
                result = '玉米'
                find_index = -1
    elif(max_prob == wheat_prob):
        if(max_number - wheat_number > 2):
            if(max_number == rice_number):
                result = '水稻'
                find_index = 0
            elif(max_number == wheat_number):
                result = '小麦'
                find_index = 1
            elif(max_number == maize_number):
                result = '玉米'
                find_index = 2
            elif(max_number == cucumber_number):
                result = '黄瓜'
                find_index = 3
            else:
                result = '玉米'
                find_index = -1
    elif(max_prob == maize_prob):
        if(max_number - maize_number > 2):
            if(max_number == rice_number):
                result = '水稻'
                find_index = 0
            elif(max_number == wheat_number):
                result = '小麦'
                find_index = 1
            elif(max_number == maize_number):
                result = '玉米'
                find_index = 2
            elif(max_number == cucumber_number):
                result = '黄瓜'
                find_index = 3
            else:
                result = '玉米'
                find_index = -1
    elif(max_prob == cucumber_prob):
        if(max_number - cucumber_number > 2):
            if(max_number == rice_number):
                result = '水稻'
                find_index = 0
            elif(max_number == wheat_number):
                result = '小麦'
                find_index = 1
            elif(max_number == maize_number):
                result = '玉米'
                find_index = 2
            elif(max_number == cucumber_number):
                result = '黄瓜'
                find_index = 3
            else:
                result = '玉米'
                find_index = -1                                 

    # if(max_number == rice_number):
    #     result = '水稻'
    #     find_index = 0
    # elif(max_number == wheat_number):
    #     result = '小麦'
    #     find_index = 1
    # elif(max_number == maize_number):
    #     result = '玉米'
    #     find_index = 2
    # elif(max_number == cucumber_number):
    #     result = '黄瓜'
    #     find_index = 3
    # else:
    #     result = '玉米'
    #     find_index = -1

    return result, find_index, max_number

def judge_room():
    # 在下面的代码行中使用断点来调试脚本。
    filename = '/home/ucar/ucar_ws/src/image/result.txt'
    result = ['', '', '', '', '']
    room_find_flag = [0, 0, 0, 0]  # 水稻 小麦 玉米 黄瓜
    maxNumForRooms = [0, 0, 0, 0]
    roomIndex = [0, 0]
    correctResult = [0 ,0, 0]
    num = 0
    clearTargetValue = 0
    clearTargetIndex = 0
    j = 0
    x = ''
    item = ''
    with open(filename, 'r') as file:
        # line = file.readline()
        # photo_D = line.strip().split(' ')
        # line = file.readline()
        # photo_E = line.strip().split(' ')
        # line = file.readline()
        # photo_C = line.strip().split(' ')
        # line = file.readline()
        # photo_B = line.strip().split(' ')
        line = file.readline()
        photo_F = line.strip().split(' ')        
    ## 判断B房间
    result[0], find_index, maxNumForRooms[0] = judge_vegetation_room(1,4)
    if find_index != -1:
        room_find_flag[find_index] = 1
    ## 判断C房间
    result[1], find_index, maxNumForRooms[1] = judge_vegetation_room(1,3)
    if find_index != -1:
        room_find_flag[find_index] = 1
    ## 判断D房间
    result[2], find_index, maxNumForRooms[2] = judge_vegetation_room(1,2)
    if find_index != -1:
        room_find_flag[find_index] = 1
    ## 判断E房间
    result[3], find_index, maxNumForRooms[3] = judge_vegetation_room(1,1)
    if find_index != -1:
        room_find_flag[find_index] = 1
    ## 消除异己
    set_list = set(result)
    if(len(set_list) != len(result)):
        repeat_class = [x for x in result if result.count(x) > 1][0]
        for i in range(4):
            if repeat_class == result[i]:
                roomIndex[j] = i
                j = j + 1
                if j > 1:
                    break
        clearTargetValue = min(maxNumForRooms[roomIndex[0]],maxNumForRooms[roomIndex[1]])
        if(clearTargetValue == maxNumForRooms[roomIndex[0]]):
            clearTargetIndex = roomIndex[0]
        else:
            clearTargetIndex = roomIndex[1]
        j = 0
        for i in range(4):
            if i == clearTargetIndex:
                continue
            correctResult[j] = result[i]
            j = j + 1
            if j > 3:
                break
        result[clearTargetIndex] = [item for item in ['水稻', '黄瓜', '玉米', '小麦'] if not item in correctResult][0]
    else:
        pass
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
        self.yolo_vegetation_start_pub = rospy.Publisher("/yolo_vegetation_start_flag", Int8, queue_size=1)

        # self.usb_cam_start_pub = rospy.Publisher("/usb_cam/start_flag", Int8, queue_size=1)
        # self.usb_cam_image_finish = rospy.Subscriber("/usb_cam/finish_flag",Int8, self.callback_image_finish)
        self.darknet_yolo = rospy.Publisher("/darknet_yolo", Int8, queue_size=1)
        self.imuflag_sub = rospy.Subscriber("/pub", Int8, self.imu_flag)
        self.yolo_fruit_start_pub = rospy.Publisher("/yolo_vegetation_over_flag", Int8, queue_size=1)
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
        self.rate = rospy.Rate(self.rate1)

    # def is_yolo_over():
    #     return self.yolo_over_flag

    def change_inflation():
        self.change_inflation_pub.publish(1)

    def yolo_vegetation_start(self):
        self.yolo_vegetation_start_pub.publish(1)

    def yolo_fruit_start(self,times):
        for i in range(times):
            self.yolo_fruit_start_pub.publish(1)
        # self.yolo_vegetation_start_pub.publish(1)

    def usb_cam_start(self,nums,times):
        for i in range(times):
            self.usb_cam_start_pub.publish(nums)
    
    def darknet_yolo_write(self,nums,times):
        for i in range(times):
            self.darknet_yolo.publish(nums)

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
        self.imu_sub_flag = msg.data
        rospy.loginfo("--------%d--------",self.imu_sub_flag)

    def img_callback(self,ros_img_msg, args):

        # print(args)
        assert isinstance(ros_img_msg, Image)
        self.cv_img = np.frombuffer(ros_img_msg.data, dtype=np.uint8).reshape(ros_img_msg.height, ros_img_msg.width, -1)
        self.cv_img = cv2.cvtColor(self.cv_img, cv2.COLOR_RGB2BGR)
        self.cv_img = cv2.flip(self.cv_img,1)
        # cv2.imwrite("/home/ucar/ucar_ws/src/image/" + str(1) + "_" + str(1) + ".jpg",self.cv_img)
        # cv2.imshow("cv_img", self.cv_img)
        # cv2.waitKey(1) 

    def save_img(self, index, num, num_wait):
        for i in range(num_wait):
        #     ret, frame = self.cap.read()
        # frame = cv2.flip(frame,1)   ##图像左右颠倒
            cv2.imwrite("/home/ucar/ucar_ws/src/image/" + str(index) + "_" + str(num) + ".jpg", self.cv_img)
    
    def save_img_vegetation(self, index, num, num_wait):
        for i in range(num_wait):
        #     ret, frame = self.cap.read()
        # frame = cv2.flip(frame,1)   ##图像左右颠倒
            cv2.imwrite("/home/ucar/ucar_ws/src/image_vegetation/" + str(index) + "_" + str(num) + ".jpg", self.cv_img)

    def save_img_fruits(self, index, num, num_wait):
        for i in range(num_wait):
        #     ret, frame = self.cap.read()
        # frame = cv2.flip(frame,1)   ##图像左右颠倒
            cv2.imwrite("/home/ucar/ucar_ws/src/image_fruits/" + str(index) + "_" + str(num) + ".jpg", self.cv_img)

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
        else:
            self.cmd_vel_pub.publish(msg)
        
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
                        task.darknet_yolo_write(11,100) 
                        # task.save_img_vegetation(2,0,50)
                        task.rotate(110)
                        # task.save_img_vegetation(2,1,50)
                        task.rotate(70)
                        # task.save_img_vegetation(2,2,50)
                        task.rotate(15)
                        # task.save_img_vegetation(2,3,50)
                        task.darknet_yolo_write(0,300) 
                    elif index == 2:
                        task.darknet_yolo_write(12,100) 
                        # task.save_img_vegetation(1,0,50)
                        task.rotate(90)
                        # task.save_img_vegetation(1,1,50)
                        task.rotate(185)
                        # task.save_img_vegetation(1,2,50)
                        task.rotate(330)
                        task.darknet_yolo_write(0,300) 
                        task.rotate(320)
                    elif index == 3:
                        task.darknet_yolo_write(13,100) 
                        # task.save_img_vegetation(3,0,50)
                        task.rotate(80)
                        # task.save_img_vegetation(3,1,50)
                        task.rotate(220)
                        # task.save_img_vegetation(3,2,50)
                        task.rotate(330)
                        task.darknet_yolo_write(0,300) 
                        task.rotate(270)
                    elif index == 4:
                        task.darknet_yolo_write(14,100) 
                        # task.save_img_vegetation(4,0,50)
                        task.rotate(280)
                        # task.save_img_vegetation(4,1,50)
                        task.rotate(140)
                        # task.save_img_vegetation(4,2,50)
                        task.rotate(30)
                        task.darknet_yolo_write(0,300) 
                        task.rotate(20)
                    elif index == 5:
                        pass
                    elif index == 6:
                        pass
                    elif index == 7:
                        pass
                        task.save_img_fruits(5,0,70)
                        # task.rotate(150)
                        # task.save_img_fruits(5,1,60)
                        # task.rotate(110)
                        # task.save_img_fruits(5,2,60)
                        # task.rotate(120)
                    elif index == 8:
                        task.save_img_fruits(5,1,60)
                        # task.rotate(240)
                        task.rotate(210) #斜着
                        task.save_img_fruits(5,2,60)
                        task.rotate(240)
                        task.save_img_fruits(5,3,60)
                        # task.rotate(60)
                    elif index == 9:
                        task.yolo_fruit_start(50)
                    elif index == 10:
                        pass
                    elif index == 11:  #12
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