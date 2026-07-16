#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Forward the UCar driver's fixed /scan_raw topic for gmapping only."""

from __future__ import print_function

import rospy
from sensor_msgs.msg import LaserScan


class MappingScanRelay(object):
    def __init__(self):
        rospy.init_node('mapping_scan_relay')
        self.publisher = rospy.Publisher('/scan', LaserScan, queue_size=5)
        self.subscriber = rospy.Subscriber('/scan_raw', LaserScan, self.relay, queue_size=5)
        rospy.loginfo('mapping_scan_relay: /scan_raw -> /scan is ready')

    def relay(self, scan):
        self.publisher.publish(scan)


if __name__ == '__main__':
    try:
        MappingScanRelay()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
