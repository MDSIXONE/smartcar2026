#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String as ROSString
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32,Int8

        
import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
lr=0
rospy.init_node('fff')
distance=-2
angle=0
def move(x,y):
    global cmd_vel_pub
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = x
    cmd_vel_msg.linear.y = y  
    cmd_vel_pub.publish(cmd_vel_msg)  
def bd_cb(msg):
    global distance
    global angle
    global lr
    if msg.data.startswith('x'):
        return
 
    a,b,a0,s0,a1,s1=msg.data.split('|')
    print(msg.data)
    distance=float(a)
    angle=float(b)
    lr=float(s0)-float(s1)
rospy.Subscriber("/bd_result", ROSString, bd_cb) 
start = rospy.Publisher("/bd_start", Int32, queue_size=1)
start.publish(1)
cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

adjust=30

while (distance<0) and(not rospy.is_shutdown()):
    print(distance)
    rospy.sleep(0.1)
print(9999)
  
while (angle>-45) and(not rospy.is_shutdown()):
    rospy.sleep(0.1)
    mv=0
    side=0.2
    if adjust>0:
        adjust=adjust-1
        mv=(distance-0.3)
        side=0
    move(mv,side)  
print(8888)
 
while (angle>-120) and(not rospy.is_shutdown()):
    rospy.sleep(0.1)
    move(0.2,0)   
print(7777)

while (angle<0 or (10<angle<150)) and(not rospy.is_shutdown()):
    rospy.sleep(0.1) 
    move(0,-0.2) 
move(0,-0) 


rospy.spin()
