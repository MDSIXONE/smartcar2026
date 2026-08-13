#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool 
action_list = ["L","L","L","L","S"]
count = 0
class LaneFollower:
    def __init__(self):
        rospy.init_node('lane_follower_node')
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=20)
        self.control_sub = rospy.Subscriber('/start_following', Bool, self.control_callback)
        self.task_completed_pub = rospy.Publisher('/task_completed', Bool, queue_size=1)  # Add this line
        self.bridge = CvBridge()
        self.frame_center = 320  # Assuming a 320x160 image, center x-coordinate

        # PD controller gains
        self.Kp = 0.005  # Proportional gain
        self.Kd = 0.002  # Derivative gain

        # Maximum control signal values
        self.max_control_signal = 2.0

        # Variables to track previous error for derivative control
        self.prev_error = 0.0

        # Flag to control lane following
        self.follow_lane_flag = False

        # Initialize the video capture
        # self.cap = cv2.VideoCapture(0)
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.angular_turn_msg = 3.0
        self.rate1 = 150
        self.rate = rospy.Rate(self.rate1)
    # def image_callback(self):
    #     ret, frame = self.cap.read()
    #     if not ret:
    #         rospy.logerr("Failed to capture image")
    #         return

    #     if self.follow_lane_flag:
    #         self.follow_lane(frame)
    #     else:
    #         return
    def image_callback(self, msg):
        if self.follow_lane_flag:
            self.follow_lane(msg)
        else:
            #rospy.loginfo("Lane following is disabled. Skipping image processing.")
            return
        
    # def follow_lane(self, img):
            # img = cv2.flip(img, 1)
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
            img = cv2.flip(img, 1)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return
        mid_points, left_points, right_points, img_with_lines = self.process_image(img)
        if len(mid_points) > 0:
            chosen_mid_point = self.choose_mid_point(mid_points, method="average")
            deviation = self.frame_center - chosen_mid_point
            rospy.loginfo("Frame Center: {}, Lane Center: {}, Deviation: {}".format(self.frame_center, chosen_mid_point, deviation))
            control_signal = self.control_car(deviation)
            rospy.loginfo("Control Signal: {}".format(control_signal))
            self.publish_control(control_signal)
            if self.detect_horizontal_line(img):
                rospy.sleep(1)
                global count
                self.choose_action(action_list[count])
                rospy.loginfo({count})
                count = count + 1
            # if self.is_boundary_line(left_points) and self.is_boundary_line(right_points):
            #     rospy.logwarn("Both left and right lines are boundary lines. Stopping the vehicle.")
            #     self.stop_following()
            #     self.task_completed_pub.publish(True)
            #     rospy.loginfo("Task completed message published.")
        else:
            rospy.logwarn("No mid points detected. Skipping control.")
            
    def process_image(self, img):
        height, width, _ = img.shape
        lower_half = img[height * 19 // 24:height * 21 // 24, :]

        gray = cv2.cvtColor(lower_half, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)

        left_points = []
        right_points = []
        mid_points = []

        for i in range(binary.shape[0] - 1, 0, -1):
            row = binary[i]
            mid_index = binary.shape[1] // 2

            left_indices = np.where(row[:mid_index] == 255)[0][::-1]
            if left_indices.size > 0:
                left_points.append((left_indices[0], i + height // 2))
            else:
                left_points.append((0, i + height // 2))

            right_indices = np.where(row[mid_index:] == 255)[0]
            if right_indices.size > 0:
                right_points.append((right_indices[0] + mid_index, i + height // 2))
            else:
                right_points.append((binary.shape[1] - 1, i + height // 2))

            if np.sum(row) > (0.5 * width * 255):
                rospy.logwarn("Crossing line detected. Stopping the vehicle.")
                # self.stop_following()
                # self.task_completed_pub.publish(True)
                return mid_points, left_points, right_points, binary

            if left_indices.size > 0 and right_indices.size > 0:
                break

        left_points = np.array(left_points)
        right_points = np.array(right_points)
        mid_points = (left_points + right_points) // 2

        return mid_points, left_points, right_points, binary

    def is_boundary_line(self, points):
        if len(points) == 0:
            return False
        if all(point[0] == 0 or point[0] == 319 for point in points):
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
        
    def detect_horizontal_line(self, img):
        height, width, _ = img.shape
        lower_half = img[height * 19 // 24:height * 21 // 24, :]

        gray = cv2.cvtColor(lower_half, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)

        for row in binary:
            if np.sum(row) > (0.5 * width * 255):

                return True
        return False
    
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

    def choose_action(self,char):
        if char == "L":
            self.rotate(-90)
            rospy.logwarn("Turn left")
        elif char == "R":
            self.rotate(90)
            rospy.logwarn("Turn right")
        else:
            self.stop_following()
            self.task_completed_pub.publish(True)
            rospy.logwarn("Stop")

    def control_car(self, deviation):
        error = deviation
        d_error = error - self.prev_error
        self.prev_error = error

        control_signal = self.Kp * error + self.Kd * d_error
        control_signal = np.clip(control_signal, -self.max_control_signal, self.max_control_signal)

        return control_signal

    def publish_control(self, control_signal):
        twist = Twist()
        twist.linear.x = 0.3 # Constant speed
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

    def control_callback(self, msg):
        if msg.data:
            self.start_following()
        else:
            self.stop_following()

    def run(self):
        rospy.spin()
        rate = rospy.Rate(20)  # 10 Hz
        while not rospy.is_shutdown():
            self.image_callback()
            rate.sleep()


if __name__ == '__main__':
    lane_follower = LaneFollower()
    lane_follower.run()

