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
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
import cv2
import os
from sensor_msgs.msg import Imu
import math
# import serial
# 巡逻点
waypoints=[
     (0.575, 0.468, 0.712, 0.702),#进入通道
     (1.071, 1.417, -0.007, 1.000), #恐怖分子识别
     (0.587, 0.688,-0.708, 0.706), #返回通道
    #  (0.064, 0.001,1.000, 0.011), #入坡点
       (0.031, 0.015,1.000, -0.013), #入坡点1
    #  (0.075, -0.010,1.000, -0.013), #入坡点2
    #  (-1.645, -0.027,1.000, 0.005), #下坡点
     (-1.545, -0.027,1.000, 0.005), #下坡点
     (-2.234, -1.641,-0.714, 0.700), #急救包识别
     (-2.154, 1.245,0.709, 0.705), #救援物品点
     (-1.612, 0.011,0.002, 1.000), #返回入坡点
     (0.031, 0.015,0.007,1.000),#返回下坡点
     (1.053, -0.507,0.004, 1.000),#slam通道1
     (1.567, -0.307,0.710, 0.704),#slam通道2
     (2.091, 0.028,-0.023, 1.000) #入库点
   
 ]

target_item_list = ['jinggun','fangdanyi','cuileiwasi']
target_report_list = ['警棍','防弹衣','催泪瓦斯']

def goal_pose(pose):
    goal_pose = MoveBaseGoal()

    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0]
    goal_pose.target_pose.pose.position.y = pose[1]
    goal_pose.target_pose.pose.orientation.z = pose[2]
    goal_pose.target_pose.pose.orientation.w = pose[3]

    return goal_pose


def read_labels_from_file(file_path):
    labels = []  # 用于存储label信息的列表
    center_value_list = []
    with open(file_path, 'r') as file:
        for line in file:
            # 每行的格式是 "label x y w h confidence \n"
            parts = line.strip().split(' ')
            label = parts[0]  # 获取label信息
            center_value = float(parts[1])
            center_value_list.append(center_value)
            labels.append(label)  # 将label添加到列表中
    return labels, center_value_list

# def read_labels_from_file(file_path):
#     labels = []  # 用于存储label信息的列表
#     with open(file_path, 'r') as file:
#         for line in file:
#             # 每行的格式是 "label x y w h confidence \n"
#             parts = line.strip().split(' ')
#             label = parts[0]  # 获取label信息
#             x1 = int(parts[1])  # 获取左上角x坐标
#             y1 = int(parts[2])  # 获取左上角y坐标
#             w = int(parts[3])   # 获取检测框宽度
#             h = int(parts[4])   # 获取检测框高度
#             cx = x1 + w / 2     # 计算中心点x坐标
#             labels.append((label, cx))  # 将label和中心点x坐标添加到列表中
#     return labels

def judge_kongbufenzi_num(label):
    if  label == "kongbufenzi1":
       return 1
    elif label == "kongbufenzi2":
       return 2 
    elif label == "kongbufenzi3":
       return 3 
def jude_target_item(num):
    if num == 1 :
        return target_item_list[0]
    if num == 2 :
        return target_item_list[1]
    if num == 3 :
        return target_item_list[2]

def calculate_angle(x, image_width=640, horizontal_fov=124.8):
        image_center_x = image_width / 2
        dx = x - image_center_x
        angle_per_pixel = horizontal_fov / image_width
        angle = dx * angle_per_pixel
        return angle
         
class task_control:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.cmd_vel_sub = rospy.Subscriber("/teb_cmd_vel", Twist, self.cmd_vel_callback)

        self.tts_pub = rospy.Publisher("/voice/xf_tts_topic", String, queue_size=100)

        # self.task_start_sub = rospy.Subscriber("/task_start_flag", Int8 , self.callback_task_start)
        self.task_start_sub = rospy.Subscriber("/awake_flag", Int8 , self.callback_task_start)
        self.yolo_start_pub = rospy.Publisher("/yolo_start_flag", Int8, queue_size = 1)
        self.yolo_obj_start_pub = rospy.Publisher("/yolo_obj_start_flag", Int8, queue_size = 1)
        # self.usb_cam_start_pub = rospy.Publisher("/usb_cam/start_flag", Int8, queue_size=1)
        # self.usb_cam_image_finish = rospy.Subscriber("/usb_cam/finish_flag",Int8, self.callback_image_finish)
        self.darknet_yolo = rospy.Publisher("/darknet_yolo", Int8, queue_size=1)
        self.imuflag_sub = rospy.Subscriber("/pub", Int8, self.imu_flag)

        self.change_inflation_pub = rospy.Publisher("/change_inflation_flag", Int8, queue_size=1)
        self.yolo_over_sub = rospy.Subscriber("/yolo_over_flag", Int8 , self.callback_yolo_over)

        self.sub_ = rospy.Subscriber('/usb_cam/image_raw', Image, self.img_callback, ('args'))
        #-------------------------
        # self.task_completed_sub = rospy.Subscriber('/task_completed', Bool, self.task_completed_callback)
        # self.control_pub = rospy.Publisher('/start_following', Bool, queue_size=1)
        # self.task_completed = False
        
        self.start_mission_pub = rospy.Publisher('/start_mission', Int32, queue_size=1)
        self.mission_complete_sub = rospy.Subscriber('/mission_complete', Int32, self.mission_complete_callback)
        self.mission_complete = False
        
        

        #-------------------------

        self.ramp_start = 0
        self.image_finish_flag = 0
        self.task_start_flag = 0
        self.imu_sub_flag = 0
        self.yolo_over_flag = 0
        self.rate1 = 150
        self.angular_turn_msg = 3.0
        self.image_msg = String()
        self.cv_img = 0
        self.rate = rospy.Rate(self.rate1)
        self.kongbufenzi_result = "/home/ucar/ucar_ws/src/image/1_1.txt"
        self.obj_result = "/home/ucar/ucar_ws/src/image/"

        
    #------------------------
    def mission_complete_callback(self, msg):
        if msg.data == 1:
            self.mission_complete = True

    def start_mission(self):
        # 发布1到/start_mission话题
        self.start_mission_pub.publish(1)
        #rospy.loginfo("靠近任务开始...")
        
        # 等待任务完成信号
        while not self.mission_complete:
            rospy.sleep(0.1)
        
        rospy.loginfo("靠近任务完成")
        self.mission_complete = False  # 重置任务完成标志
    # def send_control_toggle(self, start):
    #     self.start_following_pub.publish(Bool(start))

    # def task_complete_callback(self, msg):
    #     if msg.data:
    #         self.task_completed = True

    # def follow_lane_and_wait(self):
    #     self.task_completed = False
    #     self.send_control_toggle(True)

    #     while not rospy.is_shutdown() and not self.task_completed:
    #         rospy.sleep(0.1)

    #     self.send_control_toggle(False)
    #     rospy.loginfo("Lane following task completed.")
    #-----------------------

    # def is_yolo_over():
    #     return self.yolo_over_flag
    #---------------------识别对准标靶------------------
    
    
    def find_target(self, target_label):
        """
        寻找指定标靶的函数
        target_label: 标靶的标签
        """
        for _ in range(6):  # 360度转一圈，每次转60度
            obj = jude_target_item(numofkongbufenzi)
            task.save_img(1, 1, 1)
            task.yolo_kongbufenzi_start()
            while not task.yolo_over_flag:
                    rospy.sleep(0.1)
            if task.yolo_over_flag == 1:
               task.yolo_over_flag = 0
               labels = read_labels_from_file(task.kongbufenzi_result)
               for obj, cx in labels:
                  if obj in labels :
                     rospy.loginfo(f"找到了标靶：{obj}")    
                     #angle = calculate_angle(cx)
                     #self.rotate(angle)  # 旋转到对准目标
                     return True
                  else : 
                     task.rotate(60)       
        return False
        
    
    #---------------------------------------------------

    def change_inflation(self):
        self.change_inflation_pub.publish(1)

    def yolo_kongbufenzi_start(self):
        self.yolo_start_pub.publish(1)

    def yolo_obj_start(self,times):
        for i in range(times):
            self.yolo_obj_start_pub.publish(1)

    def usb_cam_start(self,nums,times):
        for i in range(times):
            self.usb_cam_start_pub.publish(nums)
    
    def darknet_yolo_write(self,nums,times):
        for i in range(times):
            self.darknet_yolo.publish(nums)

    def callback_yolo_over(self, msg):
        self.yolo_over_flag = msg.data
        
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
    
    def save_img_obj(self, index, num, num_wait):
        for i in range(num_wait):
        #     ret, frame = self.cap.read()
        # frame = cv2.flip(frame,1)   ##图像左右颠倒
            cv2.imwrite("/home/ucar/ucar_ws/src/image_obj/" + str(index) + "_" + str(num) + ".jpg", self.cv_img)
    
    # def start_obj_recognition(self):
    #     # 发布目标识别开始的信号
    #     self.yolo_obj_start_pub.publish(1)
    #     # 循环直到目标识别任务结束
    #     while not self.yolo_over_flag:
    #         rospy.sleep(1)  # 每隔一秒执行一次
    #         self.save_img_obj(2, 0, 1)  # 调用save_img_obj方法拍照，这里num_wait设置为1表示每次循环拍照一次
    #     rospy.loginfo("目标识别任务结束。")
    
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
        
    # def rotate(self, angle):
    #     cmd_vel_msg = Twist()
    #     cmd_vel_msg.linear.x = 0
    #     if angle <= 180:
    #         cmd_vel_msg.angular.z = -self.angular_turn_msg
    #     else:
    #         angle = 360 - angle
    #         cmd_vel_msg.angular.z = self.angular_turn_msg

    #     angular_duration = angle / self.angular_turn_msg / 180.0 * 3.1415926
    #     ticks = int(angular_duration * self.rate1)
    #     rospy.loginfo(ticks)
    #     for i in range(ticks):
    #         self.cmd_vel_pub.publish(cmd_vel_msg)
    #         self.rate.sleep()

    #     cmd_vel_msg.angular.z = 0
    #     self.cmd_vel_pub.publish(Twist())
    def rotate(self, angle):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0
        # angle=1.3*angle   
        if angle >= 0:
            if angle <= 180:
                cmd_vel_msg.angular.z = -self.angular_turn_msg
            else:
                angle = 360 - angle
                cmd_vel_msg.angular.z = self.angular_turn_msg
        else:
            angle = -angle  # 将负值角度转换为正值
            if angle <= 180:
                cmd_vel_msg.angular.z = self.angular_turn_msg
            else:
                angle = 360 - angle
                cmd_vel_msg.angular.z = -self.angular_turn_msg

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

    def perform_task_at_index_2(self):
        self.save_img(1, 1, 1)
        self.yolo_kongbufenzi_start()
        while not self.yolo_over_flag:
            rospy.sleep(0.1)
        if self.yolo_over_flag == 1:
            labels, center_value_list = read_labels_from_file(self.kongbufenzi_result)
            if labels:  # 检查labels是否为空
                label = labels[0]
                numofkongbufenzi = judge_kongbufenzi_num(label)
                report1 = '恐怖分子的数量为{}个'.format(numofkongbufenzi)
                self.report_result(report1)
                self.yolo_over_flag = 0
                return True  # 成功完成任务
            else:
                rospy.logwarn("识别到的内容为空，重新到达拍照点...")
                self.yolo_over_flag = 0
                return False  # 识别内容为空，重新开始
        else:
            rospy.logwarn("YOLO 识别未完成，等待中...")
            return False  # 识别未完成，重新开始

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
        # ser = serial.Serial('/dev/ttyUSB1',9600,timeout=0.5)
        # ser.isOpen()
        while True:
            if task.get_task_start_flag():
                for index, pose in enumerate(waypoints, 1):
                    goal=goal_pose(pose)
                    client.send_goal(goal)
                    client.wait_for_result()  # <180 RIGHT
                    if index == 1:
                        #self.follow_lane_and_wait()
                        
                        pass
                                   
                    elif index == 2:
                        while index == 2:
                            if task.perform_task_at_index_2():
                                labels, center_value_list = read_labels_from_file(task.kongbufenzi_result)
                                label = labels[0]
                                numofkongbufenzi = judge_kongbufenzi_num(label)
                                break   
                        # rospy.sleep(0.5)
                        # task.save_img(1,1,1)
                        # task.yolo_kongbufenzi_start()
                        # while not task.yolo_over_flag:
                        #     rospy.sleep(0.1)
                        # if task.yolo_over_flag == 1:
                        #     labels, center_value_list = read_labels_from_file(task.kongbufenzi_result)
                        #     if labels :
                        #         label = labels[0]
                        #         numofkongbufenzi = judge_kongbufenzi_num(label)
                        #         report1 = '恐怖分子的数量为{}个'.format(numofkongbufenzi)
                        #         task.report_result(report1)
                        #         task.yolo_over_flag = 0
                        # else: 
                        #     rospy.loginfo("YOLO 识别未完成，等待中...")
                    elif index == 3:
                        pass
                        # task.save_img_vegetation(3,0,50)
                        # task.rotate(60)
                        # task.save_img_vegetation(3,1,50)
                        # task.rotate(260)
                        # task.save_img_vegetation(3,2,50)
                        # task.rotate(270)
                        # task.save_img_vegetation(3,3,50)
                        # task.rotate(220)
                        # ser.write(b's')# tingzhi
                        # ser.close()
                    elif index == 4:
                        pass
                        # task.save_img_vegetation(4,0,50)
                        # task.rotate(300)
                        # task.save_img_vegetation(4,1,50)
                        # task.rotate(90)
                        # task.save_img_vegetation(4,2,50)
                        # task.rotate(60)
                        # task.save_img_vegetation(4,3,50)
                        # task.rotate(40)
                    elif index == 5:
                         pass
                    elif index == 6:
                        task.report_result('我已取到急救包')
                    elif index == 7:
                        
                        for _ in range(6):
                            #angle = angle + 60
                            obj = jude_target_item(numofkongbufenzi)
                            task.save_img(1, 1, 1)
                            task.yolo_kongbufenzi_start()
                            while not task.yolo_over_flag:
                                rospy.sleep(0.1)
                            if task.yolo_over_flag == 1:
                                task.yolo_over_flag = 0
                                labels, center_value_list = read_labels_from_file(task.kongbufenzi_result)
                                if obj in labels :
                                    rospy.loginfo(f"找到了标靶：{obj}")
                                    order = labels.index(obj)
                                    report_index = target_item_list.index(obj)
                                    report_item = target_report_list[report_index]
                                    x_center = center_value_list[order]
                                    rospy.loginfo(f'{x_center}')
                                    angle_offset = calculate_angle(x_center)
                                    # rospy.loginfo(f'rotate{angle_offset}')
                                    task.rotate(int(angle_offset))
                                    rospy.loginfo(f'rotate{angle_offset}')
                                    rospy.sleep(2)
                                     # 再次检测目标对象是否在中心位置
                                    # task.save_img(1, 1, 1)
                                    # task.yolo_kongbufenzi_start()
            
                                    # while not task.yolo_over_flag:
                                    #     rospy.sleep(0.1)
                
                                    # if task.yolo_over_flag == 1:
                                    #     task.yolo_over_flag = 0
                                    #     labels, center_value_list = read_labels_from_file(task.kongbufenzi_result)
                
                                    #     if obj in labels:
                                    #         order = labels.index(obj)
                                    #         x_center = center_value_list[order]
                                    #         if abs(x_center-320) < 20:
                                    #             rospy.loginfo("标靶已对准")
                                    #             # 启动任务和报告结果
                                    #             task.start_mission()
                                    #             report2 = "我已取到{}".format(report_item)
                                    #             task.report_result(report2)
                                    #             rospy.sleep(2)
                                    #             break
                                    #     else:
                                    #         rospy.loginfo("未能找到标靶，继续尝试")                     
                                    
                                    task.start_mission()
                                    report2 = "我已取到{}".format(report_item)
                                    task.report_result(report2)
                                    rospy.sleep(2)
                                    rospy.logwarn("Both left and right lines are boundary lines. Stopping the vehicle.")
                                    break
                                else : 
                                    task.rotate(60)
                                    rospy.sleep(1)
                                    rospy.loginfo("rotate 60")                                  
                    #     task.save_img_fruits(5,0,50)
                    #     task.rotate(150)
                    #     task.save_img_fruits(5,1,60)
                    #     task.rotate(110)
                    #     task.save_img_fruits(5,2,60)
                    #     task.rotate(120)
                    elif index == 8:
                        pass
                    elif index == 9:
                        pass
                    #     task.save_img_fruits(5,3,60)
                    #     task.rotate(100)
                    #     task.save_img_fruits(5,4,60)
                    #     task.rotate(150)
                    elif index == 10:
                         pass
                    elif index == 11:
                        pass
                    #     task.yolo_vegetation_start(20)
                    elif index == 12:
                      #   pass
                    #elif index == 13:
                        
                    #elif index == 5:  #12
                        # while True:
                        #     if task.yolo_over_flag:
                        #         break
                        # task.stop()
                        os.system("rosnode kill /base_driver")
                        # tts_result = judge_room()
                        # rospy.loginfo(tts_result)
                        task.report_result('已完成人质救援工作，请快速增派支援进行人质救援')
                        #task.report_result(tts_result)
                break
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down task_control node.")