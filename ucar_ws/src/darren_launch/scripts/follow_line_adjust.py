#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class LaneDetector:
    def __init__(self):
        rospy.init_node('lane_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/lane_detection/image_processed", Image, queue_size=1)
        self.rate = rospy.Rate(10)  # Adjust as needed
        self.mm = 320  # Starting scan position
        #self.cap = cv2.VideoCapture(0)  # Initialize video capture if needed

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        
        processed_image, lane_center, _ = self.detect_lane(cv_image, self.mm)
        
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(processed_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

        self.rate.sleep()

    def detect_lane(self, img, SX):
        # Gaussian blur
        frame_bgr = cv2.GaussianBlur(img, (7, 7), 0)
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        
        # Define color range for lane detection
        # color_low = np.array([16, 45, 65])
        # color_high = np.array([44, 225, 225])
        color_low = np.array([0, 0, 200])
        color_high = np.array([180, 30, 222])
        # Create a mask to isolate lane colors
        mask = cv2.inRange(hsv, color_low, color_high)
        
        # Morphological operations to clean up the mask
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        left_lane = np.array([])
        right_lane = np.array([])
        left_boundary = np.array([])
        right_boundary = np.array([])
        
        # Loop through rows from bottom to top
        for i in range(480, 1, -1):
            # Scan left lane
            for j in range(SX + 1, 0, -1):
                if mask[i - 1][j] == 0 and mask[i - 1][j - 1] == 255:
                    left_lane = np.append(left_lane, j)
                    left_boundary = np.append(left_boundary, j)
                    break
                elif j == 1:
                    left_lane = np.append(left_lane, 0)
                    left_boundary = np.append(left_boundary, 0)
            
            # Scan right lane
            for j1 in range(SX + 2, 639, 1):
                if mask[i - 1][j1] == 0 and mask[i - 1][j1 + 1] == 255:
                    right_lane = np.append(right_lane, j1)
                    right_boundary = np.append(right_boundary, j1)
                    break
                elif j1 == 638:
                    right_lane = np.append(right_lane, 639)
                    right_boundary = np.append(right_boundary, 639)
            
            # Update SX for the next row
            SX = int((left_lane[480 - i] + right_lane[480 - i]) / 2)
        
        # Calculate lane center and draw on the image
        lane_center = (left_boundary + right_boundary) / 2
        
        # Detect turning points
        left_up = left_down = right_up = right_down = False
        sm = 30  # Smoothing factor for turning point detection
        
        for i in range(len(left_boundary) - 10):
            # Detect left turning point up
            if left_lane[i] == 0 and left_lane[i + 3] == 0 and left_lane[i + 5] > 0 and left_lane[i + 10] > 0:
                left_up = True
                left_up_index = i + min(len(left_lane[i+2:]), sm)
                left_up_point = (int(lane_center[left_up_index]), 479 - left_up_index)
            
            # Detect left turning point down
            if left_lane[i] > 0 and left_lane[i + 3] > 0 and left_lane[i + 5] == 0 and left_lane[i + 10] == 0:
                left_down = True
                left_down_index = i - min(i, sm)
                left_down_point = (int(lane_center[left_down_index]), 479 - left_down_index)
            
            # Detect right turning point up
            if right_lane[i] == 639 and right_lane[i + 3] == 639 and right_lane[i + 5] < 639 and right_lane[i + 10] <= 639:
                right_up = True
                right_up_index = i + min(len(left_lane[i+2:]), sm)
                right_up_point = (int(lane_center[right_up_index]), 479 - right_up_index)
            
            # Detect right turning point down
            if right_lane[i] < 639 and right_lane[i + 3] < 639 and right_lane[i + 5] == 639 and right_lane[i + 10] == 639:
                right_down = True
                right_down_index = i - min(i, sm)
                right_down_point = (int(lane_center[right_down_index]), 479 - right_down_index)
        
        # Draw turning points on the image
        if left_up:
            cv2.circle(img, left_up_point, 5, (0, 255, 0), -1)
        if left_down:
            cv2.circle(img, left_down_point, 5, (0, 255, 0), -1)
        if right_up:
            cv2.circle(img, right_up_point, 5, (0, 255, 0), -1)
        if right_down:
            cv2.circle(img, right_down_point, 5, (0, 255, 0), -1)
        
        return img, lane_center, SX

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        ld = LaneDetector()
        ld.run()
    except rospy.ROSInterruptException:
        pass
