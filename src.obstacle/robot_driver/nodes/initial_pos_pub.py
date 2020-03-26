#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler


def initial_pos_pub(x,y,th):
    publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size = 5)
    
    #Creating the message with the type PoseWithCovarianceStamped
    rospy.loginfo("This node sets the turtlebot's position to the red cross on the floor. It will shudown after publishing to the topic /initialpose")

    initial_pos = PoseWithCovarianceStamped()
    #filling header with relevant information
    initial_pos.header.frame_id = "map"
    initial_pos.header.stamp = rospy.Time.now()
    #filling payload with relevant information gathered from subscribing
    # to initialpose topic published by RVIZ via rostopic echo initialpose
    initial_pos.pose.pose.position.x = 0
    initial_pos.pose.pose.position.y = 0
    initial_pos.pose.pose.position.z = 0.0

    initial_pos.pose.pose.orientation = Quaternion(*quaternion_from_euler(0,0,th,axes='sxyz'))

    initial_pos.pose.covariance[0] = 0.25
    initial_pos.pose.covariance[7] = 0.25
    initial_pos.pose.covariance[1:7] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    initial_pos.pose.covariance[8:34] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] 
    initial_pos.pose.covariance[35] = 0.06853891945200942
    
    publisher.publish(initial_pos)
    

