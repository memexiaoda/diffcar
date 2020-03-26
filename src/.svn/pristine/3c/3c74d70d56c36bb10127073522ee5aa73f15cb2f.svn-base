#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import MySQLdb
from geometry_msgs.msg import Pose,Quaternion,Point, PoseStamped, PoseWithCovarianceStamped, TwistWithCovariance, Twist, Vector3
from transform_utils import quat_to_angle, normalize_angle
from visualization_msgs.msg import Marker
from actionlib_msgs.msg import *  
from math import radians, pi 
import time

class marker():
    def __init__(self):
        rospy.init_node('mysqlpose', anonymous=True)
        waypoints = list()
        conn=MySQLdb.connect(host='192.168.1.171', user='root', passwd='1', db='xrs2015', port=3306, charset='utf8')
        cur = conn.cursor()
        cur.execute('select * from rb_topologypoint' )
        self.result=cur.fetchall()
        cur.close()
        conn.close()
        le1 = len(self.result)
        for i in range(le1):
            waypoints.append(Pose(Point(self.result[i][2], self.result[i][3], 0), self.result[i][0]))
        self.init_markers()
        for waypoint in waypoints:
            p = Point()
            p = waypoint.position
            print p
            self.markers.points.append(p)
            self.marker_pub.publish(self.markers)   
            time.sleep(0.1)             
    def init_markers(self):
        # Set up our waypoint markers
        # 设置标记的尺寸
        marker_scale = 0.4
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0, 'b': 0, 'a': 1.0}
        
        # Define a marker publisher.
        # 定义一个标记的发布者
        self.marker_pub = rospy.Publisher('visualization_marker', Marker,queue_size=0)
        
        # Initialize the marker points list.
        # 初始化标记点的列表
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        self.markers.type = Marker.SPHERE_LIST
        self.markers.action = Marker.ADD
        self.markers.lifetime = rospy.Duration(marker_lifetime)
        self.markers.scale.x = marker_scale
        self.markers.scale.y = marker_scale
        self.markers.color.r = marker_color['r']
        self.markers.color.g = marker_color['g']
        self.markers.color.b = marker_color['b']
        self.markers.color.a = marker_color['a']
        
        self.markers.header.frame_id = 'odom'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()       
if __name__ == '__main__':
    marker()
    time.sleep(2000)
