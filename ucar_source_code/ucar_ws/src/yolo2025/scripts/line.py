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
lidar_distance=-1
start_flag=0
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
    
   



speed=0.3
keep=30

def find_final_goal(frame,reader):
    H, W = frame.shape[:2]  # 获取图像高度和宽度
    start_row = int(7 * H / 8)  # 从5/6高度处开始
    end_row = H - 1             # 直到图像底部
    bottom_section = frame[start_row:end_row + 1, :]
    gray_bottom = np.dot(bottom_section[..., :3], [0.1140, 0.5870, 0.2989])
    mean_brightness = np.mean(gray_bottom)    
    print(mean_brightness)
    if mean_brightness>170:
        return True
    
  
    
####    
    gray0 = frame[:, :,1] 
    
   
    
    
    height = gray0.shape[0]
    
    
    start_row = int(height *0.750)#0.70
    gray = gray0[start_row:, :]
 
    gray = gray.astype(np.float32) / 255.0  # 归一化
    gray = (gray ** 6) * 255.0      # 平方后恢复 0~255
    gray = np.clip(gray, 0, 255).astype(np.uint8)  # 限制范围
  
    thresh_value, _ = cv2.threshold(gray, 0,255,cv2.THRESH_OTSU)
  
    
    
    thresh_value=max(50,thresh_value)
    #thresh_value=210
    _, binary_image = cv2.threshold(
        gray,
        thresh_value,           # 使用 OTSU 计算出的阈值
        255,                    # 最大值（白色）
        cv2.THRESH_BINARY       # 普通二值化
    )
    
    
    
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    binary_image = cv2.erode(binary_image, kernel, iterations=1) 
    binary_image = cv2.dilate(binary_image, kernel, iterations=1)
  # iterations 控制腐蚀次数

    thresh=binary_image

    left = np.ones_like(thresh) * 0
    height = gray.shape[0]
    width = gray.shape[1]
    
    le=width//6   
    
    points = np.array([
        [0, height-1],            # 顶点1: 左下角 (x=0, y=height-1)
        [width//2-le, 0],            # 顶点2: 横轴1/3，顶部 (x=width/3, y=0)
           # 顶点3: 中央，顶部 (x=width/2, y=0)
        [width//2, height-1]      # 顶点4: 中央，底部 (x=width/2, y=height-1)
    ], dtype=np.int32)
    cv2.fillPoly(left, [points], color=255)
    
    right = np.ones_like(thresh) * 0
    
    points2 = points = np.array([
        [width-1, height-1],               # 顶点1镜像: 右下角
        [width-1 - width//2+le, 0],           # 顶点2镜像: 横轴2/3，顶部
                   # 顶点3镜像: 仍为中央顶部(对称点)
        [width//2, height-1]               # 顶点4镜像: 仍为中央底部(对称点)
    ])
    cv2.fillPoly(right, [points2], color=255)

    
    and_result1 = cv2.bitwise_and(left, thresh)
    and_result2 = cv2.bitwise_and(right, thresh)
    
    
    
    H, W = and_result1.shape  # 获取图像高度和宽度


    weights = ( np.arange(H) / (H - 1))
    
    white_pixels1 = np.sum((and_result1 > 0) * np.tile(weights[:, np.newaxis], (1, W)))
    white_pixels2 = np.sum((and_result2 > 0) * np.tile(weights[:, np.newaxis], (1, W)))
    rot=(white_pixels1-white_pixels2)
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0.15
    cmd_vel_msg.linear.y = 0
    cmd_vel_msg.linear.z = 0
    cmd_vel_msg.angular.x = 0
    cmd_vel_msg.angular.y = 0
    cmd_vel_msg.angular.z = rot/6000.0

    reader.cmd_vel_pub.publish(cmd_vel_msg)  
####    
    
    return False

def handle(frame,reader):
    global speed
    global keep
    gray0 = frame[:, :,1] 
    
   
    
    
    height = gray0.shape[0]
    
    
    start_row = int(height *0.60)#0.70
    gray = gray0[start_row:, :]
 
    gray = gray.astype(np.float32) / 255.0  # 归一化
    gray = (gray ** 6) * 255.0      # 平方后恢复 0~255
    gray = np.clip(gray, 0, 255).astype(np.uint8)  # 限制范围
  
    thresh_value, _ = cv2.threshold(gray, 0,255,cv2.THRESH_OTSU)
  
    
    
    thresh_value=max(50,thresh_value)
    #thresh_value=210
    _, binary_image = cv2.threshold(
        gray,
        thresh_value,           # 使用 OTSU 计算出的阈值
        255,                    # 最大值（白色）
        cv2.THRESH_BINARY       # 普通二值化
    )
    
    
    
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    binary_image = cv2.erode(binary_image, kernel, iterations=1) 
    binary_image = cv2.dilate(binary_image, kernel, iterations=1)
  # iterations 控制腐蚀次数

    thresh=binary_image

    left = np.ones_like(thresh) * 0
    height = gray.shape[0]
    width = gray.shape[1]
    
    le=width//6   
    
    points = np.array([
        [0, height-1],            # 顶点1: 左下角 (x=0, y=height-1)
        [width//2-le, 0],            # 顶点2: 横轴1/3，顶部 (x=width/3, y=0)
           # 顶点3: 中央，顶部 (x=width/2, y=0)
        [width//2, height-1]      # 顶点4: 中央，底部 (x=width/2, y=height-1)
    ], dtype=np.int32)
    cv2.fillPoly(left, [points], color=255)
    
    right = np.ones_like(thresh) * 0
    
    points2 = points = np.array([
        [width-1, height-1],               # 顶点1镜像: 右下角
        [width-1 - width//2+le, 0],           # 顶点2镜像: 横轴2/3，顶部
                   # 顶点3镜像: 仍为中央顶部(对称点)
        [width//2, height-1]               # 顶点4镜像: 仍为中央底部(对称点)
    ])
    cv2.fillPoly(right, [points2], color=255)

    
    and_result1 = cv2.bitwise_and(left, thresh)
    and_result2 = cv2.bitwise_and(right, thresh)
    
    
    
    H, W = and_result1.shape  # 获取图像高度和宽度


    weights = ( np.arange(H) / (H - 1))
    
    white_pixels1 = np.sum((and_result1 > 0) * np.tile(weights[:, np.newaxis], (1, W)))
    white_pixels2 = np.sum((and_result2 > 0) * np.tile(weights[:, np.newaxis], (1, W)))
    
    
    #white_pixels1 = np.sum(and_result1 >0)

    #white_pixels2 = np.sum(and_result2 >0)
    if keep>0:
        keep=keep-1
        speed=speed+0.03
    if speed>0.15:
        speed=(speed*10+0.15*1)/11
    # print(speed)
    rot=(white_pixels1-white_pixels2)
    real_speed=speed
    if abs(rot)>200:
        real_speed=speed*0.8
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = real_speed
    cmd_vel_msg.linear.y = 0
    cmd_vel_msg.linear.z = 0
    cmd_vel_msg.angular.x = 0
    cmd_vel_msg.angular.y = 0
    cmd_vel_msg.angular.z = rot/3300.0

    reader.cmd_vel_pub.publish(cmd_vel_msg)
def start_flag_callback(msg):
    global start_flag
    start_flag=msg.data

def avoid_obstacle():
    cmd_vel_msg = Twist()
    reader.cmd_vel_pub.publish(cmd_vel_msg)
    rate = rospy.Rate(10)
    for i in range(20*4):   
        cmd_vel_msg.linear.x=lidar_distance-0.4
        reader.cmd_vel_pub.publish(cmd_vel_msg) 
        rate.sleep()    


    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.y = 0.24

    
    for i in range(25):   
        reader.cmd_vel_pub.publish(cmd_vel_msg) 
        rate.sleep()
    for i in range(10):   
        reader.cmd_vel_pub.publish(Twist())   
        rate.sleep()      
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x =0.3
    

    
    for i in range(22):   
        reader.cmd_vel_pub.publish(cmd_vel_msg) 
        rate.sleep()
    for i in range(10):   
        reader.cmd_vel_pub.publish(Twist())      
        rate.sleep()
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x =0
    cmd_vel_msg.linear.y=-0.24
    for i in range(24):   
        reader.cmd_vel_pub.publish(cmd_vel_msg) 
        rate.sleep()
    for i in range(8):   
        reader.cmd_vel_pub.publish(Twist())      
        rate.sleep()
    reader.cmd_vel_pub.publish(Twist())
stage=0    
# 使用示例
if __name__ == "__main__":
    rospy.init_node('line')
    reader = ROSImageReader(topic_name="/usb_cam/image_raw")  # 根据实际话题调整
    start_flag_pub=rospy.Subscriber("/line_start_flag",Int8,start_flag_callback)
    line_end_flag_pub=rospy.Publisher("/line_end_flag",Int8,queue_size=1)
    while(start_flag==0):
        rospy.sleep(0.1)
        continue
    while(lidar_distance<0):
        rospy.sleep(0.1)
        continue   
        
        
    print("start_line_follow")
    rate = rospy.Rate(10)  # 30Hz处理频率
    while not rospy.is_shutdown():
        ret,frame = reader.read()
        if not ret:
            rate.sleep()
            continue
        
        if stage==0:
            if lidar_distance<0.5:#正前方障碍物距离大于0.5m时保持巡线
                stage=1
            handle(frame,reader)
            print(lidar_distance)
        if stage==1:
            avoid_obstacle()
            stage=2
        if stage==2:
            if find_final_goal(frame,reader):
                stage=100#结束！
                line_end_flag_pub.publish(1)
        rate.sleep()
    
