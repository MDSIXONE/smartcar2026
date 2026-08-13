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
lidar_distance=-1
class ROSImageReader:
    def __init__(self, topic_name="/usb_cam/image_raw"):
        self.bridge = CvBridge()
        self.current_frame = None
        self.frame_ready = False
        # 订阅图像话题
        self.sub = rospy.Subscriber(topic_name, Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.lidar = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        rospy.loginfo(f"Waiting for images on topic {topic_name}...")
    def lidar_callback(self,msg):
        global lidar_distance
        front_angle_range = (-15, 15)  # 单位：度
    
    # 2. 转换到弧度并计算索引
        angle_min = msg.angle_min  # 起始角度（弧度）
        angle_inc = msg.angle_increment  # 角度增量（弧度）
    
    # 计算索引范围
        idx_min = int((front_angle_range[0] * 3.1416/180 - angle_min) / angle_inc)
        idx_max = int((front_angle_range[1] * 3.1416/180 - angle_min) / angle_inc)
    
    # 3. 提取有效距离（忽略无效值）
        front_ranges = []
        for i in range(idx_min, idx_max + 1):
            if msg.range_min < msg.ranges[i] < msg.range_max:
                front_ranges.append(msg.ranges[i])
    
    # 4. 计算板子距离（取最小值或平均值）
        if front_ranges:
            min_distance = min(front_ranges)  # 最近点代表板子
            avg_distance = sum(front_ranges) / len(front_ranges)  # 平均距离
            lidar_distance=avg_distance
            #rospy.loginfo(f"板子距离: 最近点={min_distance:.2f}m, 平均距离={avg_distance:.2f}m")
        else:
            rospy.logwarn("正前方未检测到有效障碍物！") 
    def image_callback(self, msg):

        try:
            # 将ROS图像消息转为OpenCV格式 (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            self.current_frame = cv_image
            self.frame_ready = True
        except CvBridgeError as e:
            rospy.logerr(f"Image conversion failed: {e}")
            self.frame_ready = False

    def read(self):
        """模拟cv2.VideoCapture.read()的接口"""
        if self.frame_ready and self.current_frame is not None:
            return True, self.current_frame.copy()  # 返回拷贝避免数据竞争
        return False, None
    
   



speed=0.2
keep=30
start_flag=1
def find_final_goal(frame,reader):
    H, W = frame.shape[:2]  # 获取图像高度和宽度
    start_row = int(7 * H / 8)  # 从5/6高度处开始
    end_row = H - 1             # 直到图像底部
    bottom_section = frame[start_row:end_row + 1, :]
    gray_bottom = np.dot(bottom_section[..., :3], [0.1140, 0.5870, 0.2989])
    #print("原始图像维度:", gray_bottom.shape)  # 例如 (480, 640, 3)
    #print("原始数据类型:", gray_bottom.dtype)
    _, otsu = cv2.threshold(gray_bottom.astype(np.uint8), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    
    #
    mean_brightness = np.mean(otsu)    
    print(mean_brightness)

####    
kk=-1   

cd=60

def start_flag_callback(msg):
    global start_flag
    start_flag=msg.data
imu_degree_avoid_obstacle=-10000

def clamp(v):
    if v>0.1:
        v=0.1
    if v<-0.1:
        v=-0.1
    return v

stage=0    








prev_time = None
velocity = np.array([0.0, 0.0])  # XY速度
position = np.array([0.0, 0.0])  # XY位置
import rospy
from sensor_msgs.msg import Imu
import tf
import numpy as np
from math import degrees

imu_degree=-1000
imu_degree_enter=-1000
imu_degree_prev=-1000
ring_pass=0
traffic_result=-1#左1 右-1
def traffic_result_cb(data):
    global traffic_result
    traffic_result=data.data






# 使用示例
if __name__ == "__main__":
    rospy.init_node('imux')
    reader = ROSImageReader(topic_name="/usb_cam/image_raw")  # 根据实际话题调整
    start_flag_pub=rospy.Subscriber("/line_start_flag",Int8,start_flag_callback)
    line_end_flag_pub=rospy.Publisher("/line_end_flag",Int8,queue_size=1)
    traffic_result_sub=rospy.Subscriber("/traffic_result",Int8,traffic_result_cb)



    rate = rospy.Rate(10)  # 30Hz处理频率
    while not rospy.is_shutdown():
        ret,frame = reader.read()

        if not ret:
            rate.sleep()
            continue
            
        find_final_goal(frame,reader)
        rate.sleep()
    
