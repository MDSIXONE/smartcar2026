#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8,Int32
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
import rospy
from sensor_msgs.msg import Imu
import tf
import numpy as np
from math import degrees
lidar_distance=-1
imu_degree=-1000
imu_degree_enter=-1000
imu_degree_prev=-1000
prev_time = None
def imu_callback(data):

    global prev_time
    global imu_degree,imu_degree_enter,imu_degree_prev
    # 获取当前时间
    current_time = rospy.Time.now()
    if prev_time is None:
        prev_time = current_time
        return
    
    # 计算时间差（秒）
    dt = (current_time - prev_time).to_sec()
    prev_time = current_time
    
    try:
        # 从四元数获取欧拉角（航向角）
        quaternion = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]  # 航向角（绕Z轴的旋转）
        

        
        # 打印结果


        imu_degree=degrees(yaw)
        print(imu_degree)
        if imu_degree_enter<-900:
            imu_degree_enter=imu_degree
            imu_degree_prev=imu_degree
        while abs(imu_degree_prev-imu_degree)>270:#如果发送了大跳变（270度以上的差别），说明可能是从-180跳到180
            #print(str(imu_degree)+"  "+str(imu_degree_prev))
            if imu_degree>imu_degree_prev:
                imu_degree=imu_degree-360#一圈
            elif imu_degree<imu_degree_prev:#血的教训！这里要用else if否则就会无限循环...
                imu_degree=imu_degree+360#一圈 
                #从而保证连续性！          
                #注意 用的是while 可能会加好几圈   
        imu_degree_prev=imu_degree
        #print(f"Yaw (heading): {imu_degree:.2f}°")
    except Exception as e:
        rospy.logerr(f"Error processing IMU data: {str(e)}")





# 使用示例
if __name__ == "__main__":
    rospy.init_node('imux')
    rospy.Subscriber('/imu', Imu, imu_callback)

    rate = rospy.Rate(10)  # 30Hz处理频率
    while not rospy.is_shutdown():

        rate.sleep()
    
