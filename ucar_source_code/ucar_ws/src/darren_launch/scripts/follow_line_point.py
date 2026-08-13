#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LaneFollower:
    def __init__(self):
        rospy.init_node('lane_follower_node')

        #self.image_sub = rospy.Subscriber('/image', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
        self.bridge = CvBridge()

        self.frame_center = 320  # 假设图像为640x480，中心x坐标
        self.width = None  # 初始化宽度变量
        # PD控制器增益
        self.Kp = 0.005  # 比例增益
        self.Kd = 0.0002  # 微分增益

        # 最大控制信号值
        self.max_control_signal = 1

        # 用于微分控制的先前误差变量
        self.prev_error = 0.0

        # 控制车道跟随的标志
        self.follow_lane_flag = False

         # 角速度消息设定
        self.angular_turn_msg = 0.5  # 设定转弯角速度
        self.rate = rospy.Rate(10)  # 设定控制频率
        
        # Initialize the video capture
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
    # def image_callback(self, msg):
    #     if self.follow_lane_flag:
    #         self.follow_lane(msg)
            
    def image_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            rospy.logerr("Failed to capture image")
            return

        if self.follow_lane_flag:
            self.follow_lane(frame)
        else:
            return

    # def follow_lane(self, msg):
    #     try:
    #         if msg.encoding == "bgr8":
    #             img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    #         elif msg.encoding == "rgb8":
    #             img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
    #             img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    #         else:
    #             rospy.logerr("Unsupported encoding: {}".format(msg.encoding))
    #             return
    #     except CvBridgeError as e:
    #         rospy.logerr("CvBridge Error: {0}".format(e))
    #         return
    def follow_lane(self, img):
        img = cv2.flip(img, 1)
        mid_points = self.process_image(img)

        if len(mid_points) > 0:
            rospy.loginfo("Mid Points: {}".format(mid_points))
            chosen_mid_point = self.choose_mid_point(mid_points, method="average_all")
            deviation = self.frame_center - chosen_mid_point
            rospy.loginfo("Frame Center: {}, Lane Center: {}, Deviation: {}".format(self.frame_center, chosen_mid_point, deviation))
            control_signal = self.control_car(deviation)
            self.publish_control(control_signal)
            rospy.loginfo("Control Signal: {}".format(control_signal))
            self.detect_bottom_turning_points(mid_points)
            
           
        else:
            rospy.logwarn("No mid points detected. Skipping control.")

    def process_image(self, img):
        height, self.width, _ = img.shape
        lower_half = img[height // 2:, :]

        # 转换为灰度图像
        gray = cv2.cvtColor(lower_half, cv2.COLOR_BGR2GRAY)

        # 应用二值化
        _, mask = cv2.threshold(gray, 190, 255, cv2.THRESH_BINARY)

        mid_points = self.find_mid_points(mask, height)
        return mid_points

    def find_mid_points(self, mask, height):
        left_points = []
        right_points = []
        mid_points = []

        # 处理底部30行
        start_row = mask.shape[0] - 10
        end_row = max(mask.shape[0] - 31, -1)
        for i in range(start_row, end_row, -1):
            row = mask[i]
            mid_index = mask.shape[1] // 2  # 中心列索引

            # 初始化位置
            left_end = mid_index
            right_start = mid_index

            left_found = False
            right_found = False

            # 在左侧寻找第一个黑到白到黑的序列
            for j in range(mid_index - 2, 0, -1):
                if row[j] == 0 and row[j - 1] == 255:  # 找到黑到白转变
                    for k in range(j, 0, -1):
                        if k - 1 >= 0 and row[k] == 255 and row[k - 1] == 0:  # 找到白到黑转变
                            left_end = k
                            left_found = True
                            break
                    break

            # 在右侧寻找第一个黑到白到黑的序列
            for j in range(mid_index + 2, mask.shape[1] - 1):
                if row[j] == 0 and j + 1 < mask.shape[1] and row[j + 1] == 255:  # 找到黑到白转变
                    for k in range(j, mask.shape[1] - 1):
                        if k + 1 < mask.shape[1] and row[k] == 255 and row[k + 1] == 0:  # 找到白到黑转变
                            right_start = k
                            right_found = True
                            break
                    break

            # 使用第一个白色像素或边界，如果只找到一侧
            if left_found and not right_found:
                right_start = mask.shape[1] - 1  # 使用右边界
            elif right_found and not left_found:
                left_end = 0  # 使用左边界

            # 更新基于找到位置的点
            left_points.append((left_end, i + height // 2))
            right_points.append((right_start, i + height // 2))
            mid_points.append(((left_end + right_start) // 2, i + height // 2))

            # 设置剩余像素为黑色
            row[left_end:right_start] = 0
            mask[i] = row

        return mid_points

    def choose_mid_point(self, mid_points, method="average_all"):
        if len(mid_points) == 0:
             return None

        if method == "average_all":
            # 使用前五个中点的平均值
            average_point = int(np.mean([mp[0] for mp in mid_points[:5]]))
            return average_point
        else:
            return mid_points[0][0]  # 如果方法未知，默认使用第一个点


    def control_car(self, deviation):
        error = deviation
        d_error = error - self.prev_error
        self.prev_error = error

        control_signal = self.Kp * error + self.Kd * d_error
        control_signal = np.clip(control_signal, -self.max_control_signal, self.max_control_signal)

        return control_signal

    def publish_control(self, control_signal):
        twist = Twist()
        twist.linear.x = 0.3  # 常量速度
        twist.angular.z = control_signal
        self.cmd_vel_pub.publish(twist)

    def detect_and_transition(self, mid_points):
        # 检测左拐点
        left_turn_detected = self.detect_left_turn(mid_points)
        if left_turn_detected:
            self.state = self.STATE_LEFT_TURN_WAIT
            self.turn_wait_counter = self.turn_wait_duration
            rospy.loginfo("Detected left turn point. Waiting to turn left...")

        # 检测右拐点
        right_turn_detected = self.detect_right_turn(mid_points)
        if right_turn_detected:
            self.state = self.STATE_RIGHT_TURN_WAIT
            self.turn
            self.turn_wait_counter = self.turn_wait_duration
            rospy.loginfo("Detected right turn point. Waiting to turn right...")

    def detect_bottom_turning_points(self, mid_points):
        sm = 30  # 阈值范围
        left_down = right_down = False
        left_down1 = right_down1 = None

        for i in range(len(mid_points) - 10):
            if (mid_points[i][0] > 0 and mid_points[i + 3][0] > 0 and mid_points[i + 5][0] == 0 and mid_points[i + 10][0] == 0):
                left_down = True
                left_down1 = (i - min(i, sm), mid_points[i - min(i, sm)][0])
            if (mid_points[i][0] < self.width and mid_points[i + 3][0] < self.width and mid_points[i + 5][0] == self.width and mid_points[i + 10][0] == width):
                right_down = True
                right_down1 = (i - min(i, sm), mid_points[i - min(i, sm)][0])

        # 处理左下角和右下角转折点的逻辑
        if left_down:
            rospy.loginfo("Detected left lower turning point at {}".format(left_down1))
            # 处理左下角转折点逻辑
            for i in range(20):
                #self.cmd_vel_pub.publish(cmd_vel_msg)
                self.rate.sleep()
            self.rotate(90)

            # 添加你需要的动作或消息发布
        if right_down:
            rospy.loginfo("Detected right lower turning point at {}".format(right_down1))
            # 处理右下角转折点逻辑
            for i in range(20):
                #self.cmd_vel_pub.publish(cmd_vel_msg)
                self.rate.sleep()
            self.rotate(-90)
            # 添加你需要的动作或消息发布

    def rotate(self, angle):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0

        # Determine direction based on angle
        if angle >= 0:
            cmd_vel_msg.angular.z = -self.angular_turn_msg  # Counter-clockwise
        else:
            angle = -angle  # Convert negative angle to positive
            cmd_vel_msg.angular.z = self.angular_turn_msg  # Clockwise

        # Calculate duration and ticks
        angular_duration = angle / self.angular_turn_msg / 180.0 * 3.1415926
        ticks = int(angular_duration * self.rate1)

        rospy.loginfo(f"Executing rotation for {angle} degrees")

        # Publish control message for specified number of ticks
        for i in range(ticks):
            self.cmd_vel_pub.publish(cmd_vel_msg)
            self.rate.sleep()

        # Stop rotation
        cmd_vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(cmd_vel_msg)


    def start_following(self):
        self.follow_lane_flag = True

    def stop_following(self):
        self.follow_lane_flag = False
        self.publish_stop_command()

    # def run(self):
    #     rospy.spin()
    def run(self):
        rate = rospy.Rate(20)  # 10 Hz
        while not rospy.is_shutdown():
            self.image_callback()
            rate.sleep()

    def publish_stop_command(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Stop command published.")

if __name__ == '__main__':
    lane_follower = LaneFollower()

    # 示例用法：开始巡线跟随
    lane_follower.start_following()

    lane_follower.run()
