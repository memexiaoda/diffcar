#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import time
import tf
from transform_utils import quat_to_angle
from geometry_msgs.msg import Point, Quaternion


class TfRelation():
    def __init__(self):
        self.base_frame = '/base_link'
        self.odom_frame = '/map'
        self.tf_flag = False
        self.tf_listener = tf.TransformListener()
        rospy.sleep(1)
        self.TfStart()      
    
    def TfStart(self):
        while not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_footprint'
                self.tf_flag = True
                rospy.loginfo("Success find transform between /odom and /base_link or /base_footprint")
                break     
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                try:
                    self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                    self.base_frame = '/base_link'
                    self.tf_flag = True
                    rospy.loginfo("Success find transform between /odom and /base_link or /base_footprint")
                    break
                except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                    self.tf_flag = False
                    time.sleep(3)
                    rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                    rospy.signal_shutdown("tf Exception")
                    continue
                
#    def PubPos(self):   
        #PosPub = rospy.Publisher("RobotPose", Point, queue_size=5)
        #print"tf_pose::PubPos tf_flag: " ,self.tf_flag  
           
        #while (self.tf_flag == True) and not rospy.is_shutdown(): # Get the current transform between the odom and base frames
#        if (self.tf_flag == True):
#            try:
#                (trans, rot)  = self.tf_listener.lookupTransform('/map', self.base_frame, rospy.Time(0))   
 #               pos = Point(*trans)
#                pos.z = quat_to_angle(Quaternion(*rot))
                #print"tf_pose::PubPos  pos: ",pos
 #               #PosPub.publish(pos)
 #           except (tf.Exception, tf.ConnectivityException, tf.LookupException):
 #               rospy.loginfo("TF Exception")
 #               print"tf_pose::PubPos  Failed to get the Robotpose"
 #               return None
                #continue
            #time.sleep(5)
  #          return pos
    
    
    def get_odom_map(self):
        # Get the current transform between the odom and base frames
        #print"tf_pose::get_odom_map tf_flag: ",self.tf_flag
        if (self.tf_flag == True):
            try:
                #(trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
                (trans, rot)  = self.tf_listener.lookupTransform('/map',self.base_frame, rospy.Time(0))         
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("TF Exception")
                print"tf_pose::get_odom_map  Failed to get the Robotpose"
                return None
            return (Point(*trans), quat_to_angle(Quaternion(*rot))) 
    

