#!/usr/bin/env python3
# -*- coding: utf-8 -*-


#----------灰度、二值化、直角


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
        self.binary_image_pub = rospy.Publisher('/binary_image', Image, queue_size=1)

        self.bridge = CvBridge()

        self.frame_center = 320  # Assuming a 640x480 image, center x-coordinate

        # PD controller gains
        self.Kp = 0.005  # Proportional gain
        self.Kd = 0  # Derivative gain

        # Maximum control signal values
        self.max_control_signal = 1.0

        # Variables to track previous error for derivative control
        self.prev_error = 0.0

        # Flag to control lane followings
        self.follow_lane_flag = False

        # Parameters for path prediction
        self.previous_mid_points = []
        self.prediction_length = 5

        # Flags to detect turn and stop
        self.is_turning = False
        self.is_crossing_line_detected = False

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

        mid_points, left_points, right_points, binary_image = self.process_image(img)

        if len(mid_points) > 0:
            chosen_mid_point = self.choose_mid_point(mid_points, method="average")
            deviation = self.frame_center - chosen_mid_point

            rospy.loginfo("Frame Center: {}, Lane Center: {}, Deviation: {}".format(self.frame_center, chosen_mid_point, deviation))

            if self.is_turning:
                rospy.loginfo("Executing turn maneuver.")
                self.execute_turn()
            elif self.is_crossing_line_detected:
                rospy.logwarn("Crossing line detected. Stopping the vehicle.")
                self.stop_following()
            else:
                control_signal = self.control_car(deviation)
                rospy.loginfo("Control Signal: {}".format(control_signal))
                self.publish_control(control_signal)

        else:
            rospy.logwarn("No mid points detected. Skipping control.")

    def process_image(self, img):
        height, width, _ = img.shape

        # Convert entire image to grayscale and binary
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)

        # Extract regions for specific detection tasks
        lower_half = binary[height * 19 // 24:height * 21 // 24, :]
        full_height = binary[height * 14 // 24:height * 21 // 24, :]

        left_points = []
        right_points = []
        mid_points = []

        for i in range(lower_half.shape[0] - 1, 0, -1):
            row = lower_half[i]
            mid_index = lower_half.shape[1] // 2

            left_indices = np.where(row[:mid_index] == 255)[0][::-1]
            if left_indices.size > 0:
                left_points.append((left_indices[0], i + height * 19 // 24))
            else:
                left_points.append((0, i + height * 19 // 24))

            right_indices = np.where(row[mid_index:] == 255)[0]
            if right_indices.size > 0:
                right_points.append((right_indices[0] + mid_index, i + height * 19 // 24))
            else:
                right_points.append((lower_half.shape[1] - 1, i + height * 19 // 24))

            if left_indices.size > 0 and right_indices.size > 0:
                break

        left_points = np.array(left_points)
        right_points = np.array(right_points)
        mid_points = (left_points + right_points) // 2

        # Detect crossing line in the full height region
        if np.sum(full_height) > (0.5 * width * full_height.shape[0] * 255):
            rospy.logwarn("Crossing line detected.")
            self.is_crossing_line_detected = True

        # Check for potential turn
        if self.is_potential_turn(mid_points, (left_points, right_points)):
            rospy.loginfo("Turn detected, executing turn maneuver.")
            self.is_turning = True
            self.is_crossing_line_detected = False  # Reset crossing line detection

        return mid_points, left_points, right_points, lower_half

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

    def is_potential_turn(self, mid_points, line_points):
        left_points, right_points = line_points
        if len(mid_points) < self.prediction_length:
            return False

        recent_mid_points = np.array(mid_points[-self.prediction_length:])
        x_coords = recent_mid_points[:, 0]
        y_coords = recent_mid_points[:, 1]

        poly_fit = np.polyfit(y_coords, x_coords, 2)
        curve = np.polyval(poly_fit, y_coords)

        curve_diff = np.diff(curve)
        if np.all(curve_diff > 0) or np.all(curve_diff < 0):
            return True
        return False

    def execute_turn(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5  # Adjust the turn speed and direction as needed
        self.cmd_vel_pub.publish(twist)
        rospy.sleep(1.5)  # Adjust the sleep time to complete the turn
        self.is_turning = False  # Reset the turning flag

if __name__ == '__main__':
    lane_follower = LaneFollower()

    lane_follower.start_following()

    lane_follower.run()
