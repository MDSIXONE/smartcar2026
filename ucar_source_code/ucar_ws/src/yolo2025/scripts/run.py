#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32,Int8
from cv_bridge import CvBridge
import cv2
import fcntl
import time
from std_msgs.msg import String as ROSString
count=0
start_flag=0 #yolo识别节点开启标志
saved=0#0表示从未保存过图片 1表示保存过但没有被识别过 2表示保存过且识别过
bridge = CvBridge()
def image_callback(msg):
    global count
    global bridge
    global start_flag
    global saved
    skip=0
    try:
        if start_flag==1:
            #跳过一帧 避免不必要的io
            if skip==1:
                skip=0
                return
            skip=skip+1
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            filename = "/home/ucar/temp/cam_out.png"
            with open(filename, "r+") as f:

                fcntl.flock(f, fcntl.LOCK_EX|fcntl.LOCK_NB)#锁定文件，如果不锁定的话，yolo可能会读到残缺的文件！
                cv2.imwrite(filename, cv_image)#写入临时路径
                #print("wwwwww")
                saved=1
                fcntl.flock(f, fcntl.LOCK_UN)#解锁文件

            count=count+1
    except Exception as e:
        pass

def yolo_start_callback(msg):
    global start_flag
    start_flag=msg.data
    # print(start_flag)
def yolo_start_callback_early(msg):
    global start_flag
    start_flag=msg.data

rospy.init_node('image_listener')
image_sub = rospy.Subscriber("/usb_cam/image_raw", Image,image_callback)
cmd = rospy.Subscriber("/yolo_start_flag", Int8,yolo_start_callback)
cmd2 = rospy.Subscriber("/yolo_start_flag_early", Int8,yolo_start_callback_early)
out = rospy.Publisher("/yolo_result", Int32,queue_size=2)
out_ex = rospy.Publisher("/yolo_result_ex", ROSString,queue_size=2)
import time
import os
import subprocess


while start_flag==0:#没有启动之前 不要加载进程
    time.sleep(0.5)
process=subprocess.Popen(["/home/ucar/yolov3/darknet_gpu/darknet","detector","test","/home/ucar/yolov3/darknet_gpu/data/obj.data","/home/ucar/yolov3/darknet_gpu/cfg/yolov3-tiny.cfg","/home/ucar/yolov3/darknet_gpu/803.weights","-thresh","0.55"],stdout=subprocess.PIPE,stdin=subprocess.PIPE,cwd="/home/ucar/yolov3/darknet_gpu")
#process=subprocess.Popen(["/home/ucar/yolov3/darknet_gpu/darknet","detector","test","/home/ucar/yolov3/darknet_gpu/data/obj.data","/home/ucar/yolov3/darknet_gpu/cfg/yolov3-tiny.cfg","/home/ucar/yolov3/darknet_gpu/713.weights","-thresh","0.45"],stdout=subprocess.PIPE,stdin=subprocess.PIPE,cwd="/home/ucar/yolov3/darknet_gpu")




#process=subprocess.Popen(["/home/ucar/yolov3/darknet_gpu/darknet","detector","test","/home/ucar/yolov3/darknet_gpu/data/obj.data","/home/ucar/yolov3/darknet_gpu/cfg/yolov3-tiny.cfg","/home/ucar/yolov3/darknet_gpu/707-4.weights"],stdout=subprocess.PIPE,stdin=subprocess.PIPE,cwd="/home/ucar/yolov3/darknet_gpu")
while not rospy.is_shutdown():
    if saved==1:
        saved=2
    else:
        time.sleep(0.1)#图片尚未就绪或者 已经识别过了！不需要再次识别
        continue
    if start_flag==0:
        exit()#已经不需要了 释放内存!
        time.sleep(0.1)
        continue
    if count==0:
        time.sleep(0.1)
        continue
    output = process.stdout.readline()#读取>
    process.stdin.write(b"/home/ucar/temp/cam_out.png\n")
    process.stdin.flush()
    first=False
    key=-1
    value=-1
    ex=""
    while True:
        output = (process.stdout.readline()).decode('utf-8')[:-1]#读取输出
        if len(output)<1:
            continue
        if str(output)!="**end**":

            name,key,value,pos=output.split(":")
            if (not first):
                first=True
                value=int(value)
            ex=ex+output+"|"
            
        else:
            break
    ex=ex.rstrip('|')
    # if value>0:
    #     print(key)
    #     #out.publish(int(key))
    # else:

    #     print("nothing")
    #print(str(key))
    
    print(str(int(key))+">"+ex)
    out_ex.publish(str(int(key))+">"+ex)
    out.publish(int(key))
    time.sleep(0.3)   
            

rospy.spin()