#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes
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
        self.arrive_c_sub = rospy.Subscriber("/arrive_C_flag", Int8 , self.callback_flag)
        self.detover_flag_pub = rospy.Publisher("detover_flag",Int8,queue_size=1)
        self.detect_pub = rospy.Publisher("detect",String,queue_size=1)

        self.Classes = []
        self.tts_flag = True
        self.flag = True
        self.detover_flag = False
        self.module_two = False
        self.det_start = 0
        # 此处设置人物模型数量
        self.module_num = 2

    def callback(self,msg):
        self.cmd_vel_pub.publish(msg)

    def callback_flag(self,msg):
        self.det_start = msg.data

    def callback_det(self,data):
        global classes
        classes = []
        for box in data.bounding_boxes:
		    classes.append(box.Class)
        if not self.det_start == 1:
                classes = []

        # 一个人物模型
        if self.module_num == 1 and len(classes) == 1:
            if classes[0] == "short" and self.tts_flag:
                self.detover_flag = True
                msg = String()
                msg.data = "识别出0个长发，0个戴眼镜"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print classes
            if classes[0] == "long" and self.tts_flag:
                self.detover_flag = True             
                msg = String()
                msg.data = "识别出1个长发，0个戴眼镜"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print classes
            if classes[0] == "short_glasses" and self.tts_flag:
                self.detover_flag = True
                msg = String()
                msg.data = "识别出0个长发，1个戴眼镜"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print classes
            if classes[0] == "long_glasses" and self.tts_flag:
                self.detover_flag = True
                msg = String()
                msg.data = "识别出1个长发，1个戴眼镜"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print classes

        # 在有两个人物在识别区，但是一次只能拍一个
        if self.module_num == 2 and len(classes) == 1 and self.flag:
            self.Classes = np.append(self.Classes,classes)
            self.Classes = np.unique(self.Classes)
            if len(self.Classes) == 2:
                classes = []
                classes = self.Classes
                self.flag = False
                self.module_two = True
        
        # 两个人物模型同时拍到
        if (self.module_num == 2 and len(classes) == 2) or self.module_two:
            self.module_two = False
            if classes[0] == "short" and classes[1] == "short" and self.tts_flag:
                self.detover_flag = True     
                msg = String()
                msg.data = "识别出0个长发，0个戴眼镜"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print classes
            if (classes[0] == "short" and classes[1] == "long") or (classes[0] == "long" and classes[1] == "short") and self.tts_flag:
                self.detover_flag = True
                msg = String()
                msg.data = "识别出1个长发，0个戴眼镜"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print classes
            if (classes[0] == "short" and classes[1] == "short_glasses") or (classes[0] == "short_glasses" and classes[1] == "short") and self.tts_flag:
                self.detover_flag = True  
                msg = String()
                msg.data = "识别出0个长发，1个戴眼镜"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print classes
            if (classes[0] == "short" and classes[1] == "long_glasses") or (classes[0] == "long_glasses" and classes[1] == "short") and self.tts_flag:
                self.detover_flag = True  
                msg = String()
                msg.data = "识别出1个长发，1个戴眼镜"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print classes
            if classes[0] == "long" and classes[1] == "long" and self.tts_flag:
                self.detover_flag = True      
                msg = String()
                msg.data = "识别出2个长发，0个戴眼镜"
                self.detect_pub.publish(msg)
                self.tts_flag = False 
                print classes         
            if (classes[0] == "long" and classes[1] == "short_glasses") or (classes[0] == "short_glasses" and classes[1] == "long") and self.tts_flag:
                self.detover_flag = True     
                msg = String()
                msg.data = "识别出1个长发，1个戴眼镜"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print classes
            if (classes[0] == "long" and classes[1] == "long_glasses") or (classes[0] == "long_glasses" and classes[1] == "long") and self.tts_flag:
                self.detover_flag = True         
                msg = String()
                msg.data = "识别出2个长发，1个戴眼镜"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print classes
            if classes[0] == "short_glasses" and classes[1] == "short_glasses" and self.tts_flag:
                self.detover_flag = True       
                msg = String()
                msg.data = "识别出0个长发，2个戴眼镜"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print classes
            if (classes[0] == "short_glasses" and classes[1] == "long_glasses") or (classes[0] == "long_glasses" and classes[1] == "short_glasses") and self.tts_flag:
                self.detover_flag = True      
                msg = String()
                msg.data = "识别出1个长发，2个戴眼镜"
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print classes
            if classes[0] == "long_glasses" and classes[1] == "long_glasses" and self.tts_flag:
                self.detover_flag = True           
                msg = String()
                msg.data = "识别出2个长发，2个戴眼镜"
                self.detect_pub.publish(msg)
                self.detect_pub.publish(msg)
                self.tts_flag = False
                print classes
        
        if self.detover_flag:
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
        print "Shutting down lax_controller node."