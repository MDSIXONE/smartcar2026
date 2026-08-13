#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys

class Client:
    def __init__(self):
        self.client_sub = rospy.Subscriber("/change_inflation_flag", Int8 , self.callback_change_inflation)
        self.change_inflation_flag = 0

    def callback_change_inflation(self, msg):
        self.change_inflation_flag = msg.data
    
    def get_change_inflation_flag(self):
        return self.change_inflation_flag

    def reset_change_inflation_flag(self):
        self.change_inflation_flag = 0

if __name__ == '__main__':
    rospy.init_node("dynamic_reconfigure_client")
    drClient = Client.init()
    local_client = dynamic_reconfigure.client.Client("/move_base/local_costmap/inflation_layer/")
    global_client = dynamic_reconfigure.client.Client("/move_base/local_costmap/inflation_layer/")
    while not rospy.is_shutdown():
        pass    

    rospy.spin()