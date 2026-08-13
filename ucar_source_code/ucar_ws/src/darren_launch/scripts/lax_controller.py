#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import numpy as np
# from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int8

class lax_controller:
    def __init__(self):    
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.cmd_vel_sub = rospy.Subscriber("/teb_cmd_vel", Twist, self.callback)
        self.tts_pub = rospy.Publisher("/voice/xf_tts_topic", String, queue_size=1)
        # Subscribe to darknet_ros to get BoundingBoxes from YOLOv3
        self.detection_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes , self.callback_det)
        self.arrive_A1_sub = rospy.Subscriber("/arrive_A1_flag", Int8 , self.callback_arrive_A1)
        self.arrive_A2_sub = rospy.Subscriber("/arrive_A2_flag", Int8 , self.callback_arrive_A2)
        self.arrive_C_sub = rospy.Subscriber("/arrive_C_flag", Int8 , self.callback_arrive_C)
        self.arrive_D_pub = rospy.Subscriber("/arrive_D_flag", Int8, self.callback_arrive_D)
        self.turnover_sub = rospy.Subscriber("/turnover_flag", Int8 , self.callback_turnover)
        self.stopover1_sub = rospy.Subscriber("/stopover1_flag", Int8, self.callback_stopover1)
        self.stopover2_sub = rospy.Subscriber("/stopover2_flag", Int8, self.callback_stopover2)
        self.stopover3_sub = rospy.Subscriber("/stopover3_flag", Int8, self.callback_stopover3)

        self.turnover_pub = rospy.Publisher("/turnover_flag", Int8 , queue_size=1)
        self.detover_flag_pub = rospy.Publisher("detover_flag",Int8,queue_size=1)
        self.detect_pub = rospy.Publisher("detect",String,queue_size=1)

        self.classes = []
        self.A1_classes = []
        self.A1_classes_tmp = []
        self.A2_classes = []
        self.A2_classes_tmp = []
        self.C_classes = []
        self.C_classes_tmp = []
        self.D_classes = []
        self.D_classes_tmp = []
        self.Total_classes = []
        self.double_classes = []
        self.A1_ref_classes = ["short", "long", "short_glasses", "long_glasses"]
        self.A2_ref_classes = ["short", "long", "short_glasses", "long_glasses"]
        self.C_ref_classes = ["short", "long", "short_glasses", "long_glasses"]
        self.D_ref_classes = ["short", "long", "short_glasses", "long_glasses"]
        self.A1_array = [0, 0, 0, 0]
        self.A2_array = [0, 0, 0, 0]
        self.C_array = [0, 0, 0, 0]
        self.D_array = [0, 0, 0, 0]
        self.tts_flag = True
        self.flag = True
        self.detover_flag = False
        self.det_start = 0
        self.det_A1_start = 0
        self.det_A2_start = 0
        self.det_C_start = 0
        self.det_D_start = 0
        self.C_flag = 1
        self.double_flag = 0
        
    def callback(self,msg):
        self.cmd_vel_pub.publish(msg)

    def callback_arrive_A1(self,msg):
        self.det_start = msg.data
        self.det_A1_start = msg.data

    def callback_arrive_A2(self,msg):
        self.det_A1_start = 0
        self.det_start = msg.data
        self.det_A2_start = msg.data

    def callback_arrive_C(self,msg):
        self.det_A2_start = 0
        self.det_start = msg.data
        self.det_C_start = msg.data

    def callback_arrive_D(self,msg):
        self.det_C_start = 0
        self.det_start = msg.data
        self.det_D_start = msg.data

    def callback_stopover1(self,msg):
        self.det_start = 0

    def callback_stopover2(self,msg):
        self.det_start = 0   

    def callback_turnover(self,msg):
        self.det_start = 0

    def callback_stopover3(self,msg):
        self.det_start = 0
        self.character_tts()

    def callback_det(self,data):
        global classes
        classes = []

        if self.det_start == 1:
            for box in data.bounding_boxes:
                classes.append(box.Class)
            print(classes)

            if self.det_A1_start == 1:
                self.A1_classes_tmp += classes
  
            if self.det_A2_start == 1:
                self.A2_classes_tmp += classes           

            if self.det_C_start == 1:
                if len(classes) >= 2:
                    if self.C_flag == 1:
                        if classes[0] == classes[1]:
                            self.double_flag = 1
                            self.C_flag = 0
                            self.double_classes = classes
                else:
                    self.C_classes_tmp += classes

            if self.det_D_start == 1:
                self.D_classes_tmp += classes

    def character_tts(self):
        if(len(self.A1_classes_tmp)>0):
            print("a1", len(self.A1_classes_tmp))
            for i in range(len(self.A1_classes_tmp)):    
                for j in range(4):
                    if self.A1_classes_tmp[i] == self.A1_ref_classes[j]:
                        self.A1_array[j] += 1
                        break
            for i in range(4):
                for j in range(0, 4-i-1):
                    if self.A1_array[j] < self.A1_array[j+1]:
                        self.A1_array[j], self.A1_array[j+1] = self.A1_array[j+1], self.A1_array[j]
                        self.A1_ref_classes[j], self.A1_ref_classes[j+1] = self.A1_ref_classes[j+1], self.A1_ref_classes[j]
            self.A1_classes = self.A1_ref_classes[0]
        else:
            self.A1_classes = []
        print("A1点识别到的是：")
        print(self.A1_classes)

        if(len(self.A2_classes_tmp)>0):
            print("a2", len(self.A2_classes_tmp))
            for i in range(len(self.A2_classes_tmp)):  
                for j in range(4):
                    if self.A2_classes_tmp[i] == self.A2_ref_classes[j]:
                        self.A2_array[j] += 1
                        # print "A2_array",j," = ",self.A2_array[j]
                        break
            for i in range(4):
                for j in range(0, 4-i-1):
                    if self.A2_array[j] < self.A2_array[j+1]:
                        self.A2_array[j], self.A2_array[j+1] = self.A2_array[j+1], self.A2_array[j]
                        self.A2_ref_classes[j], self.A2_ref_classes[j+1] = self.A2_ref_classes[j+1], self.A2_ref_classes[j]
            # for i in range(4):
            #     print self.A2_ref_classes[i]
            # for i in range(4):
            #     print self.A2_array[i]
            self.A2_classes = self.A2_ref_classes[0]
        else:
            self.A2_classes = []
        print("A2点识别到的是：")
        print(self.A2_classes)

        if self.double_flag == 1:
            self.C_classes = self.double_classes
        else:    
            if(len(self.C_classes_tmp)>0):
                print("--------------")
                print(self.C_classes_tmp)
                print(len(self.C_classes_tmp))
                for i in range(len(self.C_classes_tmp)):    
                    for j in range(4):
                        if self.C_classes_tmp[i] == self.C_ref_classes[j]:
                            self.C_array[j] += 1
                            # print "C_array",j," = ",self.C_array[j]
                            break
                for i in range(4):
                    for j in range(0, 4-i-1):
                        if self.C_array[j] < self.C_array[j+1]:
                            self.C_array[j], self.C_array[j+1] = self.C_array[j+1], self.C_array[j]
                            self.C_ref_classes[j], self.C_ref_classes[j+1] = self.C_ref_classes[j+1], self.C_ref_classes[j]
                # for i in range(4):
                #     print self.C_ref_classes[i]
                # for i in range(4):
                #     print self.C_array[i]
                self.C_classes = self.C_ref_classes[0]
            else:
                self.C_classes = []
        print("C点识别到的是：")
        print(self.C_classes)

        if(len(self.D_classes_tmp)>0):
            for i in range(len(self.D_classes_tmp)):  
                for j in range(4):
                    if self.D_classes_tmp[i] == self.D_ref_classes[j]:
                        self.D_array[j] += 1
                        break
            for i in range(4):
                for j in range(0, 4-i-1):
                    if self.D_array[j] < self.D_array[j+1]:
                        self.D_array[j], self.D_array[j+1] = self.D_array[j+1], self.D_array[j]
                        self.D_ref_classes[j], self.D_ref_classes[j+1] = self.D_ref_classes[j+1], self.D_ref_classes[j]
            # for i in range(4):
            #     print self.D_ref_classes[i]
            # for i in range(4):
            #     print self.D_array[i]
            if (len(self.A1_classes)+len(self.A2_classes)+len(self.C_classes))==2:
                self.D_classes = []
            else:
                self.D_classes = self.D_ref_classes[0]
        else:
            self.D_classes = []
        print("D点识别到的是：")
        print(self.D_classes)

        self.Total_classes = np.append(self.Total_classes, self.A1_classes)
        self.Total_classes = np.append(self.Total_classes, self.A2_classes)
        self.Total_classes = np.append(self.Total_classes, self.C_classes)
        self.Total_classes = np.append(self.Total_classes, self.D_classes)
        print("最终识别结果为：")
        print(self.Total_classes)


        if len(self.Total_classes) == 0 and self.tts_flag:
            self.detover_flag = True
            msg = String()
            msg.data = "本次送餐任务途中遇到0个人，其中长头发0人，戴眼镜0人"
            self.detect_pub.publish(msg)
            self.tts_flag = False
        # 一个人物模型
        if len(self.Total_classes) == 1:
            if self.Total_classes[0] == "short" and self.tts_flag:
                self.detover_flag = True
                msg = String()
                msg.data = "本次送餐任务途中遇到1个人，其中长头发0人，戴眼镜0人"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print(self.Total_classes)
            if self.Total_classes[0] == "long" and self.tts_flag:
                self.detover_flag = True             
                msg = String()
                msg.data = "本次送餐任务途中遇到1个人，其中长头发1人，戴眼镜0人"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print(self.Total_classes)
            if self.Total_classes[0] == "short_glasses" and self.tts_flag:
                self.detover_flag = True
                msg = String()
                msg.data = "本次送餐任务途中遇到1个人，其中长头发0人，戴眼镜1人"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print(self.Total_classes)
            if self.Total_classes[0] == "long_glasses" and self.tts_flag:
                self.detover_flag = True
                msg = String()
                msg.data = "本次送餐任务途中遇到1个人，其中长头发1人，戴眼镜1人"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print(self.Total_classes)

        # 两个人物模型
        if len(self.Total_classes) == 2:
            if self.Total_classes[0] == "short" and self.Total_classes[1] == "short" and self.tts_flag:
                self.detover_flag = True     
                msg = String()
                msg.data = "本次送餐任务途中遇到2个人，其中长头发0人，戴眼镜0人"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print(self.Total_classes)
            if (self.Total_classes[0] == "short" and self.Total_classes[1] == "long") or (self.Total_classes[0] == "long" and self.Total_classes[1] == "short") and self.tts_flag:
                self.detover_flag = True
                msg = String()
                msg.data = "本次送餐任务途中遇到2个人，其中长头发1人，戴眼镜0人"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print(self.Total_classes)
            if (self.Total_classes[0] == "short" and self.Total_classes[1] == "short_glasses") or (self.Total_classes[0] == "short_glasses" and self.Total_classes[1] == "short") and self.tts_flag:
                self.detover_flag = True  
                msg = String()
                msg.data = "本次送餐任务途中遇到2个人，其中长头发0人，戴眼镜1人"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print(self.Total_classes)
            if (self.Total_classes[0] == "short" and self.Total_classes[1] == "long_glasses") or (self.Total_classes[0] == "long_glasses" and self.Total_classes[1] == "short") and self.tts_flag:
                self.detover_flag = True  
                msg = String()
                msg.data = "本次送餐任务途中遇到2个人，其中长头发1人，戴眼镜1人"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print(self.Total_classes)
            if self.Total_classes[0] == "long" and self.Total_classes[1] == "long" and self.tts_flag:
                self.detover_flag = True      
                msg = String()
                msg.data = "本次送餐任务途中遇到2个人，其中长头发2人，戴眼镜0人"
                self.detect_pub.publish(msg)
                self.tts_flag = False 
                print(self.Total_classes)      
            if (self.Total_classes[0] == "long" and self.Total_classes[1] == "short_glasses") or (self.Total_classes[0] == "short_glasses" and self.Total_classes[1] == "long") and self.tts_flag:
                self.detover_flag = True     
                msg = String()
                msg.data = "本次送餐任务途中遇到2个人，其中长头发1人，戴眼镜1人"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print(self.Total_classes)
            if (self.Total_classes[0] == "long" and self.Total_classes[1] == "long_glasses") or (self.Total_classes[0] == "long_glasses" and self.Total_classes[1] == "long") and self.tts_flag:
                self.detover_flag = True         
                msg = String()
                msg.data = "本次送餐任务途中遇到2个人，其中长头发2人，戴眼镜1人"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print(self.Total_classes)
            if self.Total_classes[0] == "short_glasses" and self.Total_classes[1] == "short_glasses" and self.tts_flag:
                self.detover_flag = True       
                msg = String()
                msg.data = "本次送餐任务途中遇到2个人，其中长头发0人，戴眼镜2人"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print(self.Total_classes)
            if (self.Total_classes[0] == "short_glasses" and self.Total_classes[1] == "long_glasses") or (self.Total_classes[0] == "long_glasees" and self.Total_classes[1] == "short_glasses") and self.tts_flag:
                self.detover_flag = True      
                msg = String()
                msg.data = "本次送餐任务途中遇到2个人，其中长头发1人，戴眼镜2人"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print(self.Total_classes)
            if self.Total_classes[0] == "long_glasses" and self.Total_classes[1] == "long_glasses" and self.tts_flag:
                self.detover_flag = True           
                msg = String()
                msg.data = "本次送餐任务途中遇到2个人，其中长头发2人，戴眼镜2人"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print(self.Total_classes)

        if (self.detover_flag == 1) or len(self.Total_classes) >= 3:
            flag = Int8()
            flag.data = 1
            self.detover_flag_pub.publish(flag)
            self.detover_flag = False    

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("lax_controller")
        rospy.loginfo("Starting lax_controller node")
        lax_controller()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down lax_controller node.")