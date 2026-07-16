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
import time
arg=6
start_flag=0
traffic_result=-1#左1 右-1
go_around_island=1#走不走环岛？


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
keep=55
delay=10
seek_time=0
def find_final_goal(frame,reader):
    global arg
    global delay
    global imu_degree,imu_degree_avoid_obstacle,seek_time
    global delay
    mag=-1/15.0;
    if seek_time==0:
        seek_time=time.time()
    H, W = frame.shape[:2]  # 获取图像高度和宽度
    start_row = int(7 * H / 8)  # 从5/6高度处开始
    end_row = H - 1             # 直到图像底部
    bottom_section = frame[start_row:end_row + 1, :]
    gray_bottom = np.dot(bottom_section[..., :3],  [0.1140, 0.5870, 0.2989])
    mean_brightness = np.sum(gray_bottom > 200) #np.mean(gray_bottom)    
    print(mean_brightness)
    if delay>0:
        delay=delay-1
        return False
    if mean_brightness>4000:
        return True
    if abs(seek_time-time.time())>10:
        print("超过10秒了没救了，停吧")
        return True
    
  
    
####    
    gray0 = frame[:, :,1] 
    
   
    
    
    height = gray0.shape[0]
    
    
    start_row = int(height *0.800)#0.70
    gray = gray0[start_row:, :]
 
    gray = gray.astype(np.float32) / 255.0  # 归一化
    gray = (gray ** arg) * 255.0      # 平方后恢复 0~255
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
        [0, height-1+le//2],            # 顶点1: 左下角 (x=0, y=height-1)
        [width//2-le, 0],            # 顶点2: 横轴1/3，顶部 (x=width/3, y=0)
           # 顶点3: 中央，顶部 (x=width/2, y=0)
        [width//2, height-1]      # 顶点4: 中央，底部 (x=width/2, y=height-1)
    ], dtype=np.int32)
    cv2.fillPoly(left, [points], color=255)
    
    right = np.ones_like(thresh) * 0
    
    points2 = points = np.array([
        [width-1, height-1-le//2],               # 顶点1镜像: 右下角
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
    cmd_vel_msg.angular.z = rot/5000.0

    reader.cmd_vel_pub.publish(cmd_vel_msg)  
    return False
####    
kk=-1   

cd=120+30#60
def handle(frame,reader):

    global speed
    global imu_degree,imu_degree_enter,imu_degree_prev,ring_pass
    global keep
    global cd
    global kk
    global traffic_result
    global go_around_island
    if go_around_island==0:
        ring_pass=1
    
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
    keep=0
    if keep>0:
        keep=keep-1
        speed=speed+0.02
    factor=10
    min_speed=0.15
    if go_around_island==0:
        factor=8
        min_speed=0.16
    if speed>min_speed:
        speed=(speed*factor+min_speed*1)/(factor+1)
    # print(speed)
    rot=(white_pixels1-white_pixels2)
    real_speed=speed
    if abs(rot)>200:
        real_speed=speed*1
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = real_speed
    cmd_vel_msg.linear.y = 0
    cmd_vel_msg.linear.z = 0
    cmd_vel_msg.angular.x = 0
    cmd_vel_msg.angular.y = 0
    cmd_vel_msg.angular.z = rot/10000.0
    
    a=0.0#没用
    b=0.15#按照1倍计算的绝对值范围
    c=1.8#如果超出b的范围，超出的部分乘以的系数，这是为了保证差距过大时能及时拉回来
    if cmd_vel_msg.angular.z>b:
        cmd_vel_msg.angular.z=cmd_vel_msg.angular.z*c-b*(c-1)
    if cmd_vel_msg.angular.z<-b:
        cmd_vel_msg.angular.z=cmd_vel_msg.angular.z*c+b*(c-1)    
    
    
    #print(cmd_vel_msg.angular.z)
    if -a<cmd_vel_msg.angular.z and cmd_vel_msg.angular.z<a:
        cmd_vel_msg.angular.z=0
    elif cmd_vel_msg.angular.z>0:
        cmd_vel_msg.angular.z=cmd_vel_msg.angular.z-a
    else:
        cmd_vel_msg.angular.z=cmd_vel_msg.angular.z+a
    #print(cmd_vel_msg.angular.z)
    #print("----")    
    if ring_pass==1:#环岛已过！直接采取初赛策略
        cmd_vel_msg.angular.z = rot/4500.0
    if go_around_island==0:
        cmd_vel_msg.angular.z = rot/4500.0
    
    #print("xxzcasdcas"+str(ring_pass))
    

    
    if cd>0:
        #cd=cd-1
        if ring_pass==1:
            cd =0
            return
        if abs(imu_degree-imu_degree_enter)>40:
            cd=0
            print("开始绕环岛")
        print("和入口的角度差异："+str(abs(imu_degree-imu_degree_enter)))
    
    
    else:

            
        if imu_degree >-900:
            if (imu_degree<imu_degree_enter-270-45) and traffic_result>0: #顺时针减少
                ring_pass=1#已经绕过了，永久禁用偏移
                print("环岛已过")
        if imu_degree >-900:
            if (imu_degree>imu_degree_enter+270+45) and traffic_result<0: #顺时针减少
                ring_pass=1#已经绕过了，永久禁用偏移
                print("环岛已过")        
        if ring_pass==0:             
            
            if kk<-1:
                kk=0.30
            kk=kk+(0.28-kk)*0.6
            if -kk<cmd_vel_msg.angular.z and cmd_vel_msg.angular.z<kk:
                cmd_vel_msg.angular.z=-kk*traffic_result
    
    
    #    cmd_vel_msg.angular.z=cmd_vel_msg.angular.z-0.2
    #if -0.2<cmd_vel_msg.angular.z and cmd_vel_msg.angular.z<0.2:
    #    cmd_vel_msg.angular.z=-0.2
    reader.cmd_vel_pub.publish(cmd_vel_msg)
def start_flag_callback(msg):
    global start_flag
    start_flag=msg.data
imu_degree_avoid_obstacle=-10000

def clamp(v):
    print(v)
    if v>0.5:
        v=0.5
    if v<-0.5:
        v=-0.5
    return v
def avoid_obstacle():
    global imu_degree,imu_degree_avoid_obstacle
    mag=-1/15.0
    
    cmd_vel_msg = Twist()
    reader.cmd_vel_pub.publish(cmd_vel_msg)

    rate = rospy.Rate(10)
    print("终点预期角度/当前角度:"+str(imu_degree_avoid_obstacle)+"/"+str(imu_degree))
    print("角度差异:"+str(imu_degree_avoid_obstacle-imu_degree))
    if abs(imu_degree_avoid_obstacle-imu_degree)>135:
        while imu_degree_avoid_obstacle-imu_degree>135:
            imu_degree_avoid_obstacle=imu_degree_avoid_obstacle-360
        while imu_degree_avoid_obstacle-imu_degree<-135:
            imu_degree_avoid_obstacle=imu_degree_avoid_obstacle+360
    print("修正后的角度差异:"+str(imu_degree_avoid_obstacle-imu_degree))
    for i in range(20):   
        cmd_vel_msg.linear.x=lidar_distance-0.4
        cmd_vel_msg.angular.z =clamp((imu_degree-imu_degree_avoid_obstacle)*mag)
        
        reader.cmd_vel_pub.publish(cmd_vel_msg) 
        rate.sleep()    
    if imu_degree_avoid_obstacle<-9000:
         #如果是单独测试寻线，imu_degree_avoid_obstacle还是没有初始化的情况 那么不依赖起点角度进行角度锁定，所以这里还是使用板子前的角度
        imu_degree_avoid_obstacle=imu_degree
        #imu_degree_avoid_obstacle=145
       
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.y = 0.24
   
    for i in range(12*2):   
        cmd_vel_msg.angular.z =clamp((imu_degree-imu_degree_avoid_obstacle)*mag)
        reader.cmd_vel_pub.publish(cmd_vel_msg) 
        rate.sleep()
        
        
    
    for i in range(10):  
        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z =clamp((imu_degree-imu_degree_avoid_obstacle)*mag) 
        reader.cmd_vel_pub.publish(cmd_vel_msg)   
        rate.sleep()      
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x =0.3
    

    
    for i in range(22):   
        cmd_vel_msg.angular.z =clamp((imu_degree-imu_degree_avoid_obstacle)*mag) 
        reader.cmd_vel_pub.publish(cmd_vel_msg) 
        rate.sleep()
    for i in range(10): 
        cmd_vel_msg = Twist()  
        cmd_vel_msg.angular.z =clamp((imu_degree-imu_degree_avoid_obstacle)*mag) 
        reader.cmd_vel_pub.publish(cmd_vel_msg)      
        rate.sleep()
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x =0
    cmd_vel_msg.linear.y=-0.24
    
    num=14*2
    if traffic_result==-1:
        num=11*2
    for i in range(num):   
        cmd_vel_msg.angular.z =clamp((imu_degree-imu_degree_avoid_obstacle)*mag) 
        reader.cmd_vel_pub.publish(cmd_vel_msg) 
        rate.sleep()
    for i in range(8):
        cmd_vel_msg = Twist()  
        cmd_vel_msg.angular.z =clamp((imu_degree-imu_degree_avoid_obstacle)*mag)     
        reader.cmd_vel_pub.publish(cmd_vel_msg)      
        rate.sleep()
    reader.cmd_vel_pub.publish(Twist())
stage=0    








prev_time = None
velocity = np.array([0.0, 0.0])  # XY速度
position = np.array([0.0, 0.0])  # XY位置
import rospy
from sensor_msgs.msg import Imu
import tf
import numpy as np
from math import degrees

imu_degree=-100000
imu_degree_enter=-1000
imu_degree_prev=-1000
ring_pass=0

def traffic_result_cb(data):
    global traffic_result
    traffic_result=data.data

def imu_callback(data):
    global recorded_start
    global traffic_result
    global prev_time, velocity, position
    global imu_degree,imu_degree_enter,imu_degree_prev
    global imu_degree,imu_degree_avoid_obstacle
    if False and (imu_degree_avoid_obstacle<-9000 and start_flag==0):#如果没有初始化（<-9000） 且没有手动设start_flag为1（正常情况下，start_flag到比赛结尾才会设1）
        #否则说明是手动测试将start_flag设为1 那么不要记录初始角度
        quaternion = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]  # 航向角（绕Z轴的旋转）
    
    
    
    
        imu_degree_avoid_obstacle=degrees(yaw)-90
        print("锁定角度"+str(imu_degree_avoid_obstacle))    
    
    
    
    
    
    
    
    # 获取当前时间
    if start_flag==0:
        return
    current_time = rospy.Time.now()
    
    # 如果是第一次回调，只记录时间
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
        
        # 获取XY加速度（去除重力分量）
        accel_x = data.linear_acceleration.x
        accel_y = data.linear_acceleration.y
        
        # 简单积分计算速度和位置
        velocity[0] += accel_x * dt
        velocity[1] += accel_y * dt
        position[0] += velocity[0] * dt
        position[1] += velocity[1] * dt
        
        # 打印结果


        imu_degree=degrees(yaw)
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





def go_around_island_cb(data):
    global go_around_island
    go_around_island=data.data
    print("是否启用环岛:"+str(go_around_island))




# 使用示例
if __name__ == "__main__":
    rospy.init_node('imux')
    reader = ROSImageReader(topic_name="/usb_cam/image_raw")  # 根据实际话题调整
    start_flag_pub=rospy.Subscriber("/line_start_flag",Int8,start_flag_callback)
    line_end_flag_pub=rospy.Publisher("/line_end_flag",Int8,queue_size=1)
    traffic_result_sub=rospy.Subscriber("/traffic_result",Int8,traffic_result_cb)
    rospy.Subscriber("/go_around_island",Int8,go_around_island_cb)
    rospy.Subscriber('/imu', Imu, imu_callback)
    while(start_flag==0):
        rospy.sleep(0.1)
        continue
    while(lidar_distance<0):
        rospy.sleep(0.1)
        continue   
        
        
    print("start_line_follow")
    rate = rospy.Rate(10)
    while imu_degree<-9000:
        rate.sleep()
    rospy.sleep(1)
    imu_degree_avoid_obstacle=imu_degree
    print("避障角度:"+str(imu_degree_avoid_obstacle))  
    rospy.sleep(1)
      # 30Hz处理频率
    while not rospy.is_shutdown():
        ret,frame = reader.read()
        if not ret:
            rate.sleep()
            continue
        
        if stage==0:
            if lidar_distance<0.55:#正前方障碍物距离大于0.5m时保持巡线
                stage=1
            #print(555)
            handle(frame,reader)
            print("雷达前方距离："+str(lidar_distance))
        if stage==1:
            avoid_obstacle()
            stage=2
        if stage==2:
            if find_final_goal(frame,reader):
                stage=100#结束！
                line_end_flag_pub.publish(1)
        rate.sleep()
    
