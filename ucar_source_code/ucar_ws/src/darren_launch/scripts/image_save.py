# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import numpy as np
# import cv2
# import rospy
# from sensor_msgs.msg import Image


# def img_callback(ros_img_msg, args):

#     # print(args)
#     assert isinstance(ros_img_msg, Image)
#     cv_img = np.frombuffer(ros_img_msg.data, dtype=np.uint8).reshape(ros_img_msg.height, ros_img_msg.width, -1)
#     cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
#     cv_img = cv2.flip(cv_img,1)
#     cv2.imwrite("/home/ucar/ucar_ws/src/image/" + str(1) + "_" + str(1) + ".jpg", cv_img)
#     cv2.imshow("cv_img", cv_img)
#     cv2.waitKey(1)


# def main():
#     rospy.init_node('photo_save', anonymous=True)

#     # ('args'): img_callback args
#     sub_ = rospy.Subscriber('/usb_cam/image_raw', Image, img_callback, ('args'))
#     rospy.spin()


# if __name__ == '__main__':
#     main()
