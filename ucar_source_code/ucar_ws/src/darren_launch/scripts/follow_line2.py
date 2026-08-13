#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#------------------------可用------------

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LaneFollower:
    def __init__(self):
        rospy.init_node('lane_follower_node')

        self.image_sub = rospy.Subscriber('/image', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.image_pub = rospy.Publisher('/processed_image', Image, queue_size=1)
        self.bridge = CvBridge()

        self.frame_center = 320  # Assuming a 640x480 image, center x-coordinate

        # PD controller gains
        self.Kp = 0.005  # Proportional gain
        self.Kd = 0  # Derivative gain

        # Maximum control signal valuess
        self.max_control_signal = 1.0

        # Variables to track previous error for derivative control
        self.prev_error = 0.0
        
         

        # Flag to control lane followings
        self.follow_lane_flag = False

    def image_callback(self, msg):
        if self.follow_lane_flag:
            self.follow_lane(msg)
        else:
            rospy.loginfo("Lane following is disabled. Skipping image processing.")
            return

    def follow_lane(self, msg):
        try:
            if msg.encoding == "bgr8":
                img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            elif msg.encoding == "rgb8":
                img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            else:
                rospy.logerr("Unsupported encoding: {}".format(msg.encoding))
                return
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        mid_points, left_points, right_points,img_with_lines = self.process_image(img)

        if len(mid_points) > 0:
            #rospy.loginfo("Mid Points: {}".format(mid_points))
            
            chosen_mid_point = self.choose_mid_point(mid_points, method="average")
            deviation = self.frame_center - chosen_mid_point

            rospy.loginfo("Frame Center: {}, Lane Center: {}, Deviation: {}".format(self.frame_center, chosen_mid_point, deviation))

            control_signal = self.control_car(deviation)
            rospy.loginfo("Control Signal: {}".format(control_signal))

            self.publish_control(control_signal)

            # try:
            #      self.image_pub.publish(self.bridge.cv2_to_imgmsg(img_with_lines, "bgr8"))
            # except CvBridgeError as e:
            #      rospy.logerr("CvBridge Error: {0}".format(e))

            if self.is_boundary_line(left_points) and self.is_boundary_line(right_points):
                rospy.logwarn("Both left and right lines are boundary lines. Stopping the vehicle.")
                self.stop_following()
        else:
            
            rospy.logwarn("No mid points detected. Skipping control. No line counter: {}".format(self.no_line_counter))
            
            

    def process_image(self, img):
        height, width, _ = img.shape
        #lower_half = img[height *21// 24:height *22// 24, :]
        lower_half = img[height *19// 24:height *20// 24, :]

        frameBGR = cv2.GaussianBlur(lower_half, (7, 7), 0)
        hsv = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2HSV)

        colorLow = np.array([0, 0, 200])
        colorHigh = np.array([180, 30, 222])
        # colorLow = np.array([0, 0, 221])
        # colorHigh = np.array([180, 30, 255])
        mask = cv2.inRange(hsv, colorLow, colorHigh)

        kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernal)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)

        left_points = []
        right_points = []
        mid_points = []

        for i in range(mask.shape[0] - 1, 0, -1):
            row = mask[i]
            mid_index = mask.shape[1] // 2

            # 从中间向左边找左边线
            left_indices = np.where(row[:mid_index] == 255)[0][::-1]  # 反向索引找到最右边的白色像素
            if left_indices.size > 0:
                left_points.append((left_indices[0], i + height // 2))
            else:
                left_points.append((0, i + height // 2))

            # 从中间向右边找右边线
            right_indices = np.where(row[mid_index:] == 255)[0]
            if right_indices.size > 0:
                right_points.append((right_indices[0] + mid_index, i + height // 2))
            else:
                right_points.append((mask.shape[1] - 1, i + height // 2))

            # 如果左右边线都找到了，跳出循环
            if left_indices.size > 0 and right_indices.size > 0:
                break

        left_points = np.array(left_points)
        right_points = np.array(right_points)
        mid_points = (left_points + right_points) // 2

        # for point in mid_points:
        #     cv2.circle(img, (point[0], point[1]), 1, (0, 255, 0), 2)  # Green for mid points
        # for point in left_points:
        #     cv2.circle(img, (point[0], point[1]), 1, (255, 0, 0), 2)  # Blue for left points
        # for point in right_points:
        #     cv2.circle(img, (point[0], point[1]), 1, (0, 0, 255), 2)  # Red for right points

        return mid_points, left_points, right_points, img

    def is_boundary_line(self, points):
        if len(points) == 0:
            return False
        if all(point[0] == 0 or point[0] == 639 for point in points):
            return True
        return False

    def choose_mid_point(self, mid_points, method="first"):
        if len(mid_points) == 0:
            return None
        if method == "first":
            num_points = min(5, len(mid_points))
            average_point = int(np.mean(mid_points[:num_points, 0]))
            return average_point
        else:
            return mid_points[0][0]

    def control_car(self, deviation):
        error = deviation
        d_error = error - self.prev_error
        self.prev_error = error

        control_signal = self.Kp * error + self.Kd * d_error
        control_signal = np.clip(control_signal, -self.max_control_signal, self.max_control_signal)

        return control_signal

    def publish_control(self, control_signal):
        twist = Twist()
        twist.linear.x = 0.25  # Constant speed
        twist.angular.z = control_signal
        self.cmd_vel_pub.publish(twist)

    def publish_stop_command(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Stop command published.")

    def start_following(self):
        self.follow_lane_flag = True

    def stop_following(self):
        self.follow_lane_flag = False
        self.publish_stop_command()
        
        
    def run(self):
        rospy.spin()
        
        
if __name__ == '__main__':
    lane_follower = LaneFollower()

    # Example usage: start following lane
    lane_follower.start_following()

    lane_follower.run()
