#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Int32
from std_msgs.msg import Bool 
from sensor_msgs.msg import Image
import cv2
import os
from sensor_msgs.msg import Imu
import math

waypoints_dict = {
    "point1": (1.000, 1.250, 0.000, 1.000),  # 进入通道(0.500, 0.625, 0.707, 0.707)
    "point2": (1.000, 1.250, 0.000, 1.000),  # 恐怖分子识别
    "point3": (0.000, 0.000, 1.000, 0.000),  # 返回通道(0.500, 0.625, -0.707, 0.707)
    "point4": (0.000, 0.000, 1.000, 0.000),  # 入坡点
    "point5": (0.000, 0.000, 1.000, 0.000),  # 入坡点1
    "point6": (-1.685, 0.200, 1.000, 0.000),  # 下坡点
    "point7": (-2.400, -1.900, -0.707, 0.707),  # 急救包识别
    "point8": (-2.400, 0.000, 0.707, 0.707),  # 救援物品点1 (-1.750, -1.000, 0.000, 1.000)靠近急救包
    "point9": (-1.750, 1.300, 0.000, 1.000), #救援物品点2 (-1.750, 1.000, 0.000, 1.000)
    "point10": (-1.820, 0.000, -0.707, 0.707),  # 救援物品点3 (-2.00, -0.200, 1.000, 0.000)中点
    "point11": (-1.820, -0.200, 0.000, 1.000),  # 入坡点
    "point12": (0.000, 0.000, -0.707, 0.707),  # 返回巡线  
    "point13": (2.000, 0.000, 0.000, 1.000)  # 巡线入库点
}
points_list = ["none","point1","point2","point3","point4","point5","point6","point7","point8","point9","point10","point11","point12","point13"]
target_item_list = ['jinggun','fangdanyi','cuileiwasi']
target_report_list = ['警棍','防弹衣','催泪瓦斯']

def switch(case):
    cases = waypoints_dict
    return cases.get(case, 'default pose')

def swap_last_two(tup):
    # 调换后两位数据
    new_tup = tup[:-2] + (tup[-1], tup[-2])
    return new_tup

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

def read_info_from_file(char, file_path):
    with open(file_path, 'r') as file:
        for line in file:
            parts = line.strip().split(' ')
            label = parts[0]
            if label == char:
                return parts
            else:
                continue 

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
# def calculate_angle(x, image_width=640, horizontal_fov=124.8):
#     # 图像中心的x坐标
#     image_center_x = image_width / 2.0   
#     # 偏移量dx
#     dx = x - image_center_x  
#     # 焦距f的计算
#     f = (image_width / 2.0) / math.tan(math.radians(horizontal_fov / 2.0))  
#     # 计算角度（使用反正切函数）
#     angle = math.degrees(math.atan(dx / f))  
#     return angle

class task_control:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.cmd_vel_sub = rospy.Subscriber("/teb_cmd_vel", Twist, self.cmd_vel_callback)

        self.tts_pub = rospy.Publisher("/voice/xf_tts_topic", String, queue_size=100)

        # self.task_start_sub = rospy.Subscriber("/task_start_flag", Int8 , self.callback_task_start)
        self.task_start_sub = rospy.Subscriber("/awake_flag", Int8 , self.callback_task_start)
        self.yolo_start_pub = rospy.Publisher("/yolo_start_flag", Int8, queue_size=1)

        # self.usb_cam_start_pub = rospy.Publisher("/usb_cam/start_flag", Int8, queue_size=1)
        # self.usb_cam_image_finish = rospy.Subscriber("/usb_cam/finish_flag",Int8, self.callback_image_finish)
        self.darknet_yolo = rospy.Publisher("/darknet_yolo", Int8, queue_size=1)
        self.imuflag_sub = rospy.Subscriber("/pub", Int8, self.imu_flag)

        self.change_inflation_pub = rospy.Publisher("/change_inflation_flag", Int8, queue_size=1)
        self.yolo_over_sub = rospy.Subscriber("/yolo_over_flag", Int8 , self.callback_yolo_over)
        self.sub_ = rospy.Subscriber('/usb_cam/image_raw', Image, self.img_callback, ('args'))
        #-------------------------
        self.task_completed_sub = rospy.Subscriber('/task_completed', Bool, self.task_completed_callback)
        self.start_following_pub = rospy.Publisher('/start_following', Bool, queue_size=1)
        self.task_completed = False
        
        self.start_mission_pub = rospy.Publisher('/start_mission', Int32, queue_size=1)
        self.mission_complete_sub = rospy.Subscriber('/mission_complete', Int32, self.mission_complete_callback)
        self.mission_complete = False
        self.mission_second_sub = rospy.Subscriber('/mission_second', Int32, self.mission_second_callback)
        self.mission_second = False
        
        # self.current_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_current_pose)
        self.current_pose_sub = rospy.Subscriber('/robot_pose', Pose, self.update_current_pose)

        self.current_pose = None
       
       
        
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
    def mission_second_callback(self, msg):
        if msg.data:
            self.mission_second = True
    
    def update_current_pose(self, msg):
            # self.current_pose = msg.pose.pose
            self.current_pose = msg
    def calculate_new_pose(self, angle_offset):
        if self.current_pose is None:
            rospy.logwarn("Current pose is not available yet.")
            return None
        else:
        # 获取当前的四元数朝向
            current_orientation = self.current_pose.orientation
            current_orientation_list = [current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w]

        # 将四元数转换为欧拉角
            (roll, pitch, yaw) = euler_from_quaternion(current_orientation_list)

        # 计算新的朝向
            new_yaw = yaw + math.radians(angle_offset)

        # 将新的欧拉角转换回四元数
            new_orientation_quat = quaternion_from_euler(roll, pitch, new_yaw)

        # 创建新的目标姿态
            new_pose = Pose()
            new_pose.position = self.current_pose.position
            new_pose.orientation.x = new_orientation_quat[0]
            new_pose.orientation.y = new_orientation_quat[1]
            new_pose.orientation.z = new_orientation_quat[2]
            new_pose.orientation.w = new_orientation_quat[3]

            return new_pose 
    
    def send_new_goal(self, new_pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = new_pose

        client.send_goal(goal)
        client.wait_for_result()    
    
    def send_control_toggle(self, start):
        self.start_following_pub.publish(Bool(start))

    def task_completed_callback(self, msg):
        if msg.data:
            self.task_completed = True

    def follow_lane_and_wait(self):
        self.task_completed = False
        self.send_control_toggle(True)

        while not rospy.is_shutdown() and not self.task_completed:
            rospy.sleep(0.1)

        self.send_control_toggle(False)
        rospy.loginfo("Lane following task completed.")
    
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
        
        #rospy.loginfo("靠近任务完成")
        self.mission_complete = False  # 重置任务完成标志
    #-----------------------

    # def is_yolo_over():
    #     return self.yolo_over_flag

    def change_inflation(self):
        self.change_inflation_pub.publish(1)

    def yolo_kongbufenzi_start(self):
            self.yolo_start_pub.publish(1)

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
    def save_and_process(self):
        self.save_img(1,1,1)
        self.yolo_kongbufenzi_start()
        while not self.yolo_over_flag:
            rospy.sleep(0.1)
        self.yolo_over_flag = 0


    def terrorists_detect(self):
            global numofkongbufenzi
            rospy.sleep(0.5)
            task.save_img(1,1,1)
            task.yolo_kongbufenzi_start()
            while not task.yolo_over_flag:
                rospy.sleep(0.1)
            if task.yolo_over_flag == 1:
                labels, center_value_list = read_labels_from_file(task.kongbufenzi_result)
                if  any(label.startswith('kongbufenzi') for label in labels):
                    label = next(label for label in labels if label.startswith('kongbufenzi')) 
                    numofkongbufenzi = judge_kongbufenzi_num(label)
                    report1 = '恐怖分子的数量为{}个'.format(numofkongbufenzi)
                    task.report_result(report1)
                    task.yolo_over_flag = 0
                    return True
                else:
                    rospy.loginfo("未识别到恐怖分子，重新识别")
                    rospy.sleep(1)
                    return False
            else:
                rospy.loginfo("yolo 正在识别....")        
    def recognize_and_ajust(self, times, rotate_angle):
        for _ in range(times):
            self.save_and_process()    
            labels, center_value_list = read_labels_from_file(task.kongbufenzi_result)
            if obj in labels:
                j = 0
                for i in range(10):
                    j = j + 1
                    self.save_and_process()    
                    labels, center_value_list = read_labels_from_file(task.kongbufenzi_result)
                    order = labels.index(obj)
                    x_center = center_value_list[order]
                    rospy.loginfo(f"x_center为{x_center}")
                    if abs(x_center - 320) < 20  or j==5 or i==9:
                        j = 0
                        rospy.loginfo(f"我已经对准了标靶：{obj}")
                        self.start_mission()
                        if self.mission_second:
                            rospy.loginfo("进入mission_second")
                            self.mission_second = False
                            continue
                        else:
                            self.report_result(f"我已取到{report_item}")
                            rospy.sleep(1)
                            return True 
                    else:
                        angle_offset = -calculate_angle(x_center)
                        angle_offset = max(-25, min(25, angle_offset))
                        rospy.loginfo(f'{x_center}')
                        new_pose = task.calculate_new_pose(angle_offset)
                        if new_pose:
                            task.send_new_goal(new_pose)
                            rospy.sleep(1)
                            rospy.loginfo(f"{angle_offset}")
            else:
                new_pose = task.calculate_new_pose(rotate_angle)
                if new_pose:
                        task.send_new_goal(new_pose)
                        rospy.loginfo("rotate 45") 
                rospy.sleep(1)
        rospy.logwarn("未能完成任务")
        return False      

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
        while True:
            if task.get_task_start_flag():
                index = 0                                 
                while index < 13:
                    index = index + 1  
                    pose = switch(points_list[index])
                    goal=goal_pose(pose)
                    client.send_goal(goal)
                    client.wait_for_result()  # <180 RIGHT
                    if index == 1:                       
                        continue                              
                    elif index == 2:
                        if task.terrorists_detect():
                            rospy.sleep(2)
                            pose = (1.000, 1.250, 1.000, 0.000)
                            goal=goal_pose(pose)
                            client.send_goal(goal)
                            client.wait_for_result()
                            rospy.sleep(1)
                            continue
                        else: 
                            index = 1
                            continue
                    elif index == 3:
                        continue
                    elif index == 4:
                        continue
                    elif index == 5:
                        continue
                    elif index == 6:
                        continue
                    elif index == 7:
                        task.report_result('我已取到急救包')
                        rospy.sleep(1)
                        pose = (-2.400, -1.900, 0.000, 1.000)# (-2.400, -2.000, 0.000, 1.000)
                        goal=goal_pose(pose)
                        client.send_goal(goal)
                        client.wait_for_result()
                        continue
                    elif index == 8:
                        try:
                            global obj, report_item
                            obj = jude_target_item(numofkongbufenzi)
                            report_index = target_item_list.index(obj)
                            report_item = target_report_list[report_index]
                            if task.recognize_and_ajust(8, -45):#(6,-45)顺时针旋转45
                                index = 10
                            else:
                                continue
                        except ValueError as e:
                            index = 7
                    elif index == 9:
                        try:
                            if task.recognize_and_ajust(8,-45):#(6,+45)逆时针旋转45
                                index = 10
                            else:
                                continue
                        except ValueError as e:
                            index = 8                        
                    elif index == 10:
                        try:
                            if task.recognize_and_ajust(8,-45):
                                continue
                            else:
                                pose = (-1.05, -1.25, -0.707, 0.707)
                                goal=goal_pose(pose)
                                client.send_goal(goal)
                                client.wait_for_result()
                                task.recognize_and_ajust(8,-45)
                                pose = (-1.45, -1.95, 1.000, 0.000)
                                goal=goal_pose(pose)
                                client.send_goal(goal)
                                client.wait_for_result()
                                task.recognize_and_ajust(8,-45) 
                                continue                             
                        except ValueError as e:
                            index = 9                          
                    elif index == 11:
                        continue                
                    elif index == 12:
                        task.follow_lane_and_wait()
                        continue                  
                    elif index == 13:
                        os.system("rosnode kill /base_driver")
                        task.report_result('已完成人质营救工作，请快速增派支援进行人质救援')
                break                        
        rospy.spin()
    except KeyboardInterrupt:
        os.system("killall node")
        print("Shutting down task_control node.")