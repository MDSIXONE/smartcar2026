#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

pos_D3 = [0.055, -1.000, -0.710, 0.705]

# def PoseCallback(msg):
#     rospy.loginfo("Received a /robot_pose message!")
#     rospy.loginfo("Orientation Components: [%f, %f, %f, %f]"
#     %(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
#     lax_pose_pub.publish(msg)

# def ucar_get_pose():
#     rospy.init_node("lax_get_posee")
#     ucar_pose_sub = rospy.Subscriber("/robot_pose", PoseStamped, PoseCallback)
    
# def ucar_pub_pose():
#     rospy.init_node("lax_pub_posee")
#     ucar_pose_pub = rospy.Publisher("/lax_pose", Pose, queue_size=10)

# if __name__ == "__main__":
    
#     ucar_get_pose()
#     ucar_pub_pose()
#     rospy.spin()


class lax_pose:
    def __init__(self):    
        self.lax_pose_pub = rospy.Publisher("/lax_ucar_pose", Pose, queue_size=100)
        self.lax_pose_sub = rospy.Subscriber("/robot_pose", Pose, self.callback)

    def callback(self,msg):
        rospy.loginfo("Received a /robot_pose message!")
        rospy.loginfo("Orientation Components: [%f, %f, %f, %f]"
        %(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w))
        self.lax_pose_pub.publish(msg)


if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("lax_pose")
        lax_pose()
        rospy.loginfo("Starting lax_pose node")
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down lax_pose node."


