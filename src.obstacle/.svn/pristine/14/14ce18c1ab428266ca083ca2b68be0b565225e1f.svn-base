#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import time
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler
from math import pi 


def initrobotpose(x, y, th):
    #rospy.init_node("initialpose_pub")
    
    pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=5)
    init_pose = PoseWithCovarianceStamped()
    init_pose.header.frame_id = "map"
    init_pose.pose.pose.position.x = x
    init_pose.pose.pose.position.y = y
    init_pose.pose.pose.position.z = 0.0
    init_pose.pose.pose.orientation = Quaternion(*quaternion_from_euler(0,0,th,axes='sxyz'))

    #time.sleep(0.1)
    pub.publish(init_pose)
#if __name__ == "__main__":
#    initrobotpose(0.0, 0.0, -1.57)
