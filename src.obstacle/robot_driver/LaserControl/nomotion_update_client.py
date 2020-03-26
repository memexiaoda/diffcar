#!/usr/bin/env python
# -*- coding:utf-8 -*-

import sys,os
import time
import math
from math import fmod, pi
#import ConfigParser
#from ConfigIni import Db_Connector
import MySQLdb
#import unittest
import rospy
#import rostest

import tf
from transform_utils import quat_to_angle, normalize_angle
from tf.msg import tfMessage
from tf import transformations
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from std_srvs.srv import*
from geometry_msgs.msg import Twist, Point, Quaternion,Pose, Vector3,PoseWithCovarianceStamped
from rospy.timer import sleep
#from math import radians, copysign, sqrt, pow, pi, atan, degrees,sin,cos,asin
#class TestBasicLocalization(unittest.TestCase):        
class TestBasicLocalization():
    def __init__(self):
        self.speed = Vector3()
        print self.speed
        global fdb
        #fdb = Db_Connector("initpose.conf")
        
        self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        rospy.Subscriber('/dxdrpub', Vector3, self.speed_fun)
        print "Subscriber"
        # Initialize the tf listener
        rospy.init_node('nomotion_update', anonymous=True)    
        #self.pose = pose
        self.tf_listener = tf.TransformListener()
        self.odom_frame = '/map'
        self.base_frame = '/base_footprint'
        print "888888888800000000000"
        rospy.sleep(2)
        
        while True:
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_footprint'
                break     
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                try:
                    self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                    self.base_frame = '/base_link'
                    break
                except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                    time.sleep(5)
                    rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                    rospy.signal_shutdown("tf Exception")
                    continue
        print "come right"
        rospy.wait_for_service('request_nomotion_update')
        self.request_nomotion = rospy.ServiceProxy('request_nomotion_update', Empty)   

        self.test_basic_localization()
        rospy.spin()
        
    def setUp(self):
        self.tf = None
        self.target_x = None
        self.target_y = None
        self.target_a = None
        print "setUp"
        
    def speed_fun(self, data):
        #linespeed = data.
        self.speed.x = data.x
        self.speed.y = data.y
        self.speed.z = data.z
        #print "self.speed.x=%s,self.speed.y=%s,self.speed.z=%s"%(self.speed.x,self.speed.y,self.speed.z)
#         print "%s"%(data)
#         print "------"
#         print "%s"%(self.speed)
#         print "++++++"
#     def tf_cb(self, msg):
#         for t in msg.transforms:
#             if t.header.frame_id == 'map':
#                 self.tf = t.transform
#                 (a_curr, a_diff) = self.compute_angle_diff()
#                 print 'Curr:\t %16.6f %16.6f %16.6f' % (self.tf.translation.x, self.tf.translation.y, a_curr)
#                 print 'Target:\t %16.6f %16.6f %16.6f' % (self.target_x, self.target_y, self.target_a)
#                 print 'Diff:\t %16.6f %16.6f %16.6f' % (
#                     abs(self.tf.translation.x - self.target_x), abs(self.tf.translation.y - self.target_y), a_diff)

#     def compute_angle_diff(self):
#         rot = self.tf.rotation
#         a = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2]
#         d_a = self.target_a
# 
#         return (a, abs(fmod(a - d_a + 5*pi, 2*pi) - pi))
    def get_odom_map(self):
        print "get_odom_map"
        # Get the current transform between the odom and base frames
        try:
            #(trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            (trans, rot)  = self.tf_listener.lookupTransform('/map',self.base_frame, rospy.Time(0))
            #self.tf_listener.transformPose("/odom", ps)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return (Point(*trans), quat_to_angle(Quaternion(*rot))) 
    
    def test_basic_localization(self):
#         global_localization = int(sys.argv[1])
#         self.target_x = float(sys.argv[2])
#         self.target_y = float(sys.argv[3])
#         self.target_a = float(sys.argv[4])
#         tolerance_d = float(sys.argv[5])
#         tolerance_a = float(sys.argv[6])
#         target_time = float(sys.argv[7])
# 
#         if global_localization == 1:
            #print 'Waiting for service global_localization'
            
        
        
        
            
        while True:
#             while(rospy.rostime.get_time() == 0.0):
#                 #print 'Waiting for initial time publication'
#                 time.sleep(0.1)
#                 print "time.sleep(0.1)"
            start_time = rospy.rostime.get_time()
            # TODO: This should be replace by a pytf listener
            init_pose = PoseWithCovarianceStamped()
            
            posmap = Point()
            
            if abs(self.speed.x) == 0.0 and abs(self.speed.z) == 0.0: #and math.fabs(self.speed.z) == 0.0:
            #if rospy.is_shutdown():
                print "get_odom_map"
                (posmap, rotmap) = self.get_odom_map()
                print posmap.x,posmap.y,rotmap
                init_pose.header.frame_id = "map"
                init_pose.pose.pose.position.x = posmap.x
                init_pose.pose.pose.position.y = posmap.y
#                 (init_pose.pose.pose.orientation.x,
#                  init_pose.pose.pose.orientation.y,
#                  init_pose.pose.pose.orientation.z,
#                  init_pose.pose.pose.orientation.w) = transformations.quaternion_from_euler(0, 0, rotmap)
#                 init_pose.pose.covariance[6*0+0] = 0.5 * 0.5
#                 init_pose.pose.covariance[6*1+1] = 0.5 * 0.5
#                 init_pose.pose.covariance[6*3+3] = math.pi/12.0 * math.pi/12.0
                init_pose.pose.pose.orientation = Quaternion(*quaternion_from_euler(0,0,rotmap,axes='sxyz'))
                try:
                    Sqlcon = MySQLdb.connect(host = '127.0.0.1', user = 'root', passwd = '1', db = 'xrs2015', port = 3306, charset = 'utf8' )
                    cur = Sqlcon.cursor()
                    cur.execute("update rb_point set posex = %.3f,posey = %.3f,posez = %.3f where id = 1;"%(posmap.x,posmap.y,rotmap) )
                    Sqlcon.commit()
                    cur.close()
                    Sqlcon.close()
                    print "curpose:",posmap.x,posmap.y,rotmap
                except MySQLdb.Error,e:
                    print "MySQL Error %d: %s" %(e.args[0], e.args[1])
                #fdb.set_db('initialpose', 'pose_x',posmap.x)
                #fdb.set_db('initialpose', 'pose_y', posmap.y)
                #fdb.set_db('initialpose', 'pose_z',rotmap)
                #print posmap.x,posmap.y,rotmap
                #self.initialpose_pub.publish(init_pose)
                #print init_pose
                #time.sleep(0.3)
                
                resp1 = self.request_nomotion()
                print resp1
                print '0000000'
    #             resp2 = request_nomotion.call(EmptyRequest())
    #             print resp2
    #             print '111111'
            #EmptyRequest()
                target_time = 5.0
                while (rospy.rostime.get_time() - start_time) < target_time:
                #print 'Waiting for end time %.6f (current: %.6f)'%(target_time,(rospy.rostime.get_time() - start_time))
                    time.sleep(0.1)
        
#         (a_curr, a_diff) = self.compute_angle_diff()
#         print 'Curr:\t %16.6f %16.6f %16.6f' % (self.tf.translation.x, self.tf.translation.y, a_curr)
#         print 'Target:\t %16.6f %16.6f %16.6f' % (self.target_x, self.target_y, self.target_a)
#         print 'Diff:\t %16.6f %16.6f %16.6f' % (
#             abs(self.tf.translation.x - self.target_x), abs(self.tf.translation.y - self.target_y), a_diff)
#         self.assertNotEquals(self.tf, None)
#         self.assertTrue(abs(self.tf.translation.x - self.target_x) <= tolerance_d)
#         self.assertTrue(abs(self.tf.translation.y - self.target_y) <= tolerance_d)
#         self.assertTrue(a_diff <= tolerance_a)
        
if __name__ == '__main__':
    time.sleep(50)
    try:
        TestBasicLocalization()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
    
#     rostest.run('amcl', 'amcl_localization',
#                 TestBasicLocalization)
#                 TestBasicLocalization, sys.argv)
