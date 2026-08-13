#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32,Int8
from cv_bridge import CvBridge
import cv2
import fcntl
import time
import tf
from std_msgs.msg import String as ROSString
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
freeze=0

def set_freeze(data):
    global freeze,imu_degree,imu_degree_prev,imu_degree_start
    freeze=data
    imu_degree=-100000
    imu_degree_prev=-100000
    imu_degree_start=-100000
lst=[]
arr=[-1]*16
go=0
def rotate(speed):
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0
    cmd_vel_msg.angular.z = speed
    cmd_vel_pub.publish(cmd_vel_msg)
keep=0
set_vel=0
throttle=0
current_speed=0
food_pos={}

forward=0
food_type=0


first=0

def res_cb_ex(msg):
    global lst
    global arr   
    global go
    global keep   
    global set_vel
    global current_speed
    global throttle
    global food_pos
    global forward
    global food_type
    global first
    if freeze==1:
        return
        food_pos={}
        arr=[-1]*16
    if first==0:
        throttle=15#强制转15次以免图片卡在摄像头边缘
        first=1
    if forward ==2:
        return#没必要了
    if go==0:
        return
    a,b=msg.data.split(">")
    data=int(a)    

    
    
    
    
    
      
    


    #yolo_result=int(a)
    food_pos={}
    for item in b.split('|'):
        # 分割字符串获取各部分
        parts = item.split(':')
        
        # 确保有足够的部分（至少4部分：name:id:conf:coords）
        if len(parts) >= 4:
            obj_id = int(parts[1])  # 提取ID
            
            # 处理坐标部分（最后一个部分）
            coords_str = parts[-1].strip('()')
            coords = [float(x) for x in coords_str.split(',')]
            
            food_pos[obj_id] = coords
    if keep>0:

        set_vel=(0) #如果计数>0 那么停车
    else:

        set_vel=(1) #否则 启动      

    if forward>0:
        return#正在前进，不用处理下面的部分
    
    #if not(str(data) in the_filter):
    #    print("非所需求的类型！收到："+str(data)+"需求:"+str(the_filter))
    #    data=-1#不在过滤器中？ 那就当没检测到（-1）
    data=-1#我们不使用可能性最大的那个识别结果，因为如果同时有两个食物，正确的那个哪怕概率低也是应该选择的
    info=""
    for keys in food_pos:
        if (str(keys) in the_filter):
            info=info+str(keys)
            data=keys
    if info=="":
        info="<啥也没有？>"
    if data==-1:
        print("非所需求的类型！收到："+str(info)+"需求:"+str(the_filter))
    print(food_pos)
    print(arr)
        
    if data<0:
        keep=keep-1
        food_pos={}
        arr=[-1]*16#只要有一次检测不到 就重置
        throttle=4#检测不到持续一段时间旋转 以免卡在边角
        return
    keep=5

    #if (msg.data not in lst):
    #    keep=4+1 #没在表里? 保持4次停车来稳定拍摄
    #if keep>0:
    #    keep=keep-1 #计数--

    arr=[data] + arr[:-1]#FIFO
    okay=all(x == arr[0] for x in arr) and (arr[0] >= 0)#如果完全一样 且不是-1
    if not okay:
        return
    print('锁定目标，准备前进')    
    forward=1
    food_type=data     
            
            
lidar_distance =0        
lidar_skip=0            
def lidar_cb(scan_msg):
    global lidar_skip
    global lidar_distance
    if forward ==2:
        return#没必要了
    
    if lidar_skip>0:
        lidar_skip=lidar_skip-1
    else:
        lidar_skip=8#跳过部分扫描 以免不必要的cpu负载
        return
    # 1. 确定正前方角度范围（例如 -15° 到 +15°）
    front_angle_range = (-15, 15)  # 单位：度
    
    # 2. 转换到弧度并计算索引
    angle_min = scan_msg.angle_min  # 起始角度（弧度）
    angle_inc = scan_msg.angle_increment  # 角度增量（弧度）
    
    # 计算索引范围
    idx_min = int((front_angle_range[0] * 3.1416/180 - angle_min) / angle_inc)
    idx_max = int((front_angle_range[1] * 3.1416/180 - angle_min) / angle_inc)
    
    # 3. 提取有效距离（忽略无效值）
    front_ranges = []
    for i in range(idx_min, idx_max + 1):
        if scan_msg.range_min < scan_msg.ranges[i] < scan_msg.range_max:
            front_ranges.append(scan_msg.ranges[i])
    
    # 4. 计算板子距离（取最小值或平均值）
    if front_ranges:
        min_distance = min(front_ranges)  # 最近点代表板子
        avg_distance = sum(front_ranges) / len(front_ranges)  # 平均距离
        lidar_distance=avg_distance
        #rospy.loginfo(f"板子距离: 最近点={min_distance:.2f}m, 平均距离={avg_distance:.2f}m")
    else:
        rospy.logwarn("正前方未检测到有效障碍物！")            
            
            
            
            
            
        
def res_cb(msg):#旧版，遗弃
    global lst
    global arr   
    global go
    global keep   
    global set_vel
    global current_speed
    global throttle
    if go==0:
        return
    print("------------")
    print(msg.data)
    print(arr)
    print(lst)
    print(keep)
    if keep>0:

        set_vel=(0) #如果计数>0 那么停车
    else:

        set_vel=(1) #否则 启动    
    if msg.data<0:
        arr=[-1]*16#只要有一次检测不到 就重置
        throttle=4#检测不到持续一段时间旋转 以免卡在边角
        return

    if (msg.data not in lst):
        keep=4+1 #没在表里? 保持4次停车来稳定拍摄
    if keep>0:
        keep=keep-1 #计数--

    arr=[msg.data] + arr[:-1]#FIFO
    okay=all(x == arr[0] for x in arr) and (arr[0] >= 0)#如果完全一样 且不是-1
    if okay:
        if (arr[0] not in lst) and (len(lst)<3):
            lst.append(arr[0])#不是重复的就记录

from math import degrees
imu_degree=-10000
imu_degree_prev=-10000
 
def imu_callback(data):


    global imu_degree,imu_degree_prev
    global go
    if not go==1:
        return

    
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
        




        imu_degree=degrees(yaw)
        if imu_degree_prev<-900:
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
            
the_filter="0123456789"            
def food_filter_cb(msg):
    global the_filter
    print("收到过滤器："+str(msg.data))
    the_filter=msg.data
def go_cb(msg):
    global go   
    go=msg.data
def yolo_freeze_cb(data):
    set_freeze(data.data)
rospy.init_node('food_finder')
#getter = rospy.Subscriber("/yolo_result", Int32,res_cb)
pub = rospy.Publisher("/yolo_all", Int32,queue_size=2)
cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
yolo_start = rospy.Subscriber("/yolo_start_flag", Int8, go_cb)#和yolo node本体共用启动topic
ex = rospy.Subscriber("/yolo_result_ex", ROSString,res_cb_ex)
rospy.Subscriber('/imu', Imu, imu_callback)
lidar = rospy.Subscriber('/scan', LaserScan, lidar_cb)
filter_type = rospy.Subscriber('/food_filter', ROSString, food_filter_cb)
pub_neo = rospy.Publisher("/yolo_found_and_acquired", Int32,queue_size=2)
pub_yolo_try = rospy.Publisher("/pub_yolo_try", Int32,queue_size=1)
rospy.Subscriber("/yolo_freeze",Int8,yolo_freeze_cb)
import time
import os
import subprocess




while (go==0) and(not rospy.is_shutdown()):
    
    rospy.sleep(0.2)
    continue
print("looking for food...")



full_speed=0.6

imu_degree_start=-10000

while not rospy.is_shutdown():
    rospy.sleep(0.05)
    if forward==0:
        #print("set_vel"+str(set_vel))
        if freeze==1:
            continue
        if set_vel==1:
            current_speed=full_speed
    
        if set_vel==0:   
            if current_speed>0:
                current_speed=0#current_speed-0.05
                
        if throttle>0:
            throttle=throttle-1
            current_speed=full_speed
        pub_yolo_try.publish(int(abs(imu_degree_start-imu_degree)))
        if(imu_degree>-9000):
            if imu_degree_start<-9000:
                imu_degree_start=imu_degree
            if abs(imu_degree_start-imu_degree)>360:#转了一圈还没检测到？减速
                current_speed=current_speed*0.66
            if abs(imu_degree_start-imu_degree)>720:#转了2圈还没检测到？再减速
                current_speed=current_speed*0.66
                #再减速就不动了
        if freeze==0:
            rotate(current_speed)
    
    if forward==1:
        if food_type in food_pos:
            the_pos=food_pos[food_type]
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.25
            if lidar_distance<0.8:
                cmd_vel_msg.linear.x = 0.10
            cmd_vel_msg.angular.z = (the_pos[0]-500)/1000*0.9
            cmd_vel_pub.publish(cmd_vel_msg)        
        else:
            print("目标丢失？？")
            #cmd_vel_msg = Twist()
            #cmd_vel_pub.publish(cmd_vel_msg) 
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.12#丢失也不能停车！ 减速前进 直到够近
            cmd_vel_msg.angular.z = 0
            cmd_vel_pub.publish(cmd_vel_msg)       
            
            
            
        if lidar_distance<0.5:
            print("够近了！停车！")
            forward=2
        else:
            print(f"距离={lidar_distance:.2f}m")
    if forward==2:
        pub_neo.publish(food_type)
        rospy.sleep(1)
    
    #if(len(lst))>=3:
    #    pub.publish(lst[0]+lst[1]*10+lst[2]*100)#拼成三位数
    #    rospy.sleep(1)
    #    rotate(0)
    #    go=0
    #    break    

