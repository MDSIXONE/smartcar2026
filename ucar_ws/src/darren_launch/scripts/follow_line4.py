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

        self.image_sub = rospy.Subscriber('/image', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        #self.mask_image_pub = rospy.Publisher('/mask_image', Image, queue_size=1)
       # self.image_pub = rospy.Publisher('/processed_image', Image, queue_size=1)
        self.bridge = CvBridge()

        self.frame_center = 320  # Assuming a 640x480 image, center x-coordinate

        # PD controller gains
        self.Kp = 0.01  # Proportional gain
        self.Kd = 0.005  # Derivative gain

        # Maximum control signal value
        self.max_control_signal = 0.8

        # Variables to track previous error for derivative control
        self.prev_error = 0.0

        # Flag to control lane following
        self.follow_lane_flag = False

       
        

    def image_callback(self, msg):
        if self.follow_lane_flag:
            self.follow_lane(msg)

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

        mid_points = self.process_image(img)

        if len(mid_points) > 0:
            rospy.loginfo("Mid Points: {}".format(mid_points))
            chosen_mid_point = self.choose_mid_point(mid_points, method="average_all")
            deviation = self.frame_center - chosen_mid_point
            rospy.loginfo("Frame Center: {}, Lane Center: {}, Deviation: {}".format(self.frame_center, chosen_mid_point, deviation))
            control_signal = self.control_car(deviation)
            self.publish_control(control_signal)
            rospy.loginfo("Control Signal: {}".format(control_signal))
        else:
            rospy.logwarn("No mid points detected. Skipping control.")

    def process_image(self, img):
        height, width, _ = img.shape
        lower_half = img[height // 2:, :]

        # Apply Gaussian blur
        frameBGR = cv2.GaussianBlur(lower_half, (3, 3), 0)
        
        # Convert BGR to RGB
        rgb = cv2.cvtColor(frameBGR, cv2.COLOR_BGR2RGB)

        # Define color range in RGB
        colorLow = np.array([180, 180, 180])
        colorHigh = np.array([255, 255, 255])
        mask = cv2.inRange(rgb, colorLow, colorHigh)

        # Morphological operations
        #kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        #mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Publish mask image for visualization
        # try:
        #     mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        #     self.mask_image_pub.publish(self.bridge.cv2_to_imgmsg(mask_color, "bgr8"))
        # except CvBridgeError as e:
        #     rospy.logerr("CvBridge Error: {0}".format(e))

        mid_points = self.find_mid_points(mask, height)

        # Draw mid points on the original image
        # for mid_point in mid_points:
        #     cv2.circle(img, mid_point, 5, (0, 0, 255), -1)  # Draw red circles for mid points

        # try:
        #     # Convert the image back to ROS Image message and publish
        #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        # except CvBridgeError as e:
        #     rospy.logerr("CvBridge Error: {0}".format(e))

        return mid_points

    def find_mid_points(self, mask, height):
        left_points = []
        right_points = []
        mid_points = []

        # Process only the bottom 30 rows
        start_row = mask.shape[0] - 10
        end_row = max(mask.shape[0] - 31, -1)
        for i in range(start_row, end_row, -1):
            row = mask[i]
            mid_index = mask.shape[1] // 2  # Center column index

            # Initialize positions
            left_end = mid_index
            right_start = mid_index

            left_found = False
            right_found = False

            # Search left side for first black to white to black sequence
            for j in range(mid_index - 2, 0, -1):
                if row[j] == 0 and row[j - 1] == 255:  # Found black to white transition
                    for k in range(j, 0, -1):
                        if k - 1 >= 0 and row[k] == 255 and row[k - 1] == 0:  # Found white to black transition
                            left_end = k
                            left_found = True
                            break
                    break

            # Search right side for first black to white to black sequence
            for j in range(mid_index + 2, mask.shape[1] - 1):
                if row[j] == 0 and j + 1 < mask.shape[1] and row[j + 1] == 255:  # Found black to white transition
                    for k in range(j, mask.shape[1] - 1):
                        if k + 1 < mask.shape[1] and row[k] == 255 and row[k + 1] == 0:  # Found white to black transition
                            right_start = k
                            right_found = True
                            break
                    break

            # Use the first white pixel or the boundary if only one side found
            if left_found and not right_found:
                right_start = mask.shape[1] - 1  # Use right boundary
            elif right_found and not left_found:
                left_end = 0  # Use left boundary

            # Update points based on found positions
            left_points.append((left_end, i + height // 2))
            right_points.append((right_start, i + height // 2))
            mid_points.append(((left_end + right_start) // 2, i + height // 2))

            # Set remaining pixels to black
            row[left_end:right_start] = 0
            mask[i] = row

        return mid_points

    def choose_mid_point(self, mid_points, method="average_all"):
        if len(mid_points) == 0:
            return None

        if method == "average_all":
            average_point = int(np.mean([mp[0] for mp in mid_points]))  # Use the average of all mid points
            return average_point
        else:
            return mid_points[0][0]  # Default to first point if method is unknown

    def control_car(self, deviation):
        error = deviation
        d_error = error - self.prev_error
        self.prev_error = error

        control_signal = self.Kp * error + self.Kd * d_error
        control_signal = np.clip(control_signal, -self.max_control_signal, self.max_control_signal)

        return control_signal

    def publish_control(self, control_signal):
        twist = Twist()
        twist.linear.x = 0.3  # Constant speed
        twist.angular.z = control_signal
        self.cmd_vel_pub.publish(twist)

    def start_following(self):
        self.follow_lane_flag = True

    def stop_following(self):
        self.follow_lane_flag = False
        self.publish_stop_command()

    def run(self):
        rospy.spin()

    def publish_stop_command(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Stop command published.")

if __name__ == '__main__':
    lane_follower = LaneFollower()

    # Example usage: start following lane
    lane_follower.start_following()

    lane_follower.run()