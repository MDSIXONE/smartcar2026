#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
    参数服务器操作之查询_Python实现:    
        get_param(键,默认值)
            当键存在时，返回对应的值，如果不存在返回默认值
        get_param_cached
        get_param_names
        has_param
        search_param
"""

import rospy

if __name__ == "__main__":

    rospy.init_node("param_update")

    #获取参数
    global_cost_scaling_factor = rospy.get_param("/move_base/global_costmap/cost_scaling_factor")
    global_inflation_radius = rospy.get_param("/move_base/global_costmap/inflation_radius")
    local_cost_scaling_factor = rospy.get_param("/move_base/local_costmap/inflation_layer/cost_scaling_factor")
    local_inflation_radius = rospy.get_param("/move_base/local_costmap/inflation_layer/inflation_radius")
    rospy.loginfo("获取的数据:%.2f,%.2f,%.2f,%.2f",
                global_cost_scaling_factor,
                global_inflation_radius,
                local_cost_scaling_factor,
                local_inflation_radius)

    #设置参数
    rospy.set_param("/move_base/global_costmap/cost_scaling_factor",0)
    rospy.set_param("/move_base/global_costmap/inflation_radius",0)
    rospy.set_param("/move_base/local_costmap/inflation_layer/cost_scaling_factor",0)
    rospy.set_param("/move_base/local_costmap/inflation_layer/inflation_radius",0)

    #再次获取参数
    global_cost_scaling_factor = rospy.get_param("/move_base/global_costmap/cost_scaling_factor")
    global_inflation_radius = rospy.get_param("/move_base/global_costmap/inflation_radius")
    local_cost_scaling_factor = rospy.get_param("/move_base/local_costmap/inflation_layer/cost_scaling_factor")
    local_inflation_radius = rospy.get_param("/move_base/local_costmap/inflation_layer/inflation_radius")
    rospy.loginfo("再次获取的数据:%.2f,%.2f,%.2f,%.2f",
                global_cost_scaling_factor,
                global_inflation_radius,
                local_cost_scaling_factor,
                local_inflation_radius)