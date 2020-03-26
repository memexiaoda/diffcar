#!/usr/bin/env python
# -*- coding:utf-8 -*-
""" nav_test.py - Version 1.1 2013-12-20

    Command a robot to move autonomously among a number of goal locations defined in the map frame.
    On each round, select a new random sequence of locations, then attempt to move to each location
    in succession.  Keep track of success rate, time elapsed, and total distance traveled.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseArray,PoseWithCovarianceStamped, Point, Quaternion, Twist
from carbot_msgs.msg import IDPoseArray,IDPose,InspectInfo,FeedbackMsg,Taskarrived
from carbot_msgs.srv import Delayaction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String, Int32
from random import sample
from math import pow, sqrt
from visualization_msgs.msg import Marker
import PyKDL
from setuptools.dist import sequence
from symbol import except_clause
from initial_pos_pub import initial_pos_pub
from tf_pose import TfRelation
import threading
#import MySQLdb
from std_msgs.msg import String,Float32,Int32
from math import  sqrt, pow, pi,sin,cos,asin
from transform_utils import normalize_angle,quat_to_angle
from tf.transformations import quaternion_from_euler

class NavTest():
    def __init__(self):
        rospy.init_node('nav_rail', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        # How long in seconds should the robot pause at each location?
        self.rest_time = rospy.get_param("~rest_time", 1)
        # Are we running in the fake simulator?
        self.fake_test = rospy.get_param("~fake_test", False)
        self.DockPile = rospy.get_param("/move_base/DockPile", "cz")
        self.DockPoint = rospy.get_param("/move_base/DockPoint", "cd")
        self.Carport = rospy.get_param("/move_base/Carport", "ku")
        # Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']
        # Set up the goal locations. Poses are defined in the map frame.  
        # An easy way to find the pose coordinates is to point-and-click
        # Nav Goals in RViz when running in the simulator.
        # Pose coordinates are then displayed in the terminal
        # that was used to launch RViz.
        self.locations = dict()
        self.locations_start = dict()
        self.locations_start_point_ID = dict()
        self.locations_line_ID = dict()
        self.locations_name = list()
        self.locations_pre_name = list()
        self.locations_curvity = dict()
        self.locations_passinground_task = dict()
        self.Delayaction_srv = Delayaction()
        # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.arrived_point_pub = rospy.Publisher('/task_arrived', Taskarrived, queue_size=1)
        self.move_info_pub = rospy.Publisher('/move_info', InspectInfo, queue_size=1)
        self.control_style = 'manual'
        self.current_task_pub = rospy.Publisher('/current_task', IDPose, queue_size=1)
        self.control_style_tl = rospy.Publisher('/cancel_done', String, queue_size=1)
        
        rospy.loginfo("Waiting for move_base action server...")
        
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(600))
        
        rospy.loginfo("Connected to move base server")
        
        # A variable to hold the initial pose of the robot to be set by 
        # the user in RViz
        initial_pose = PoseWithCovarianceStamped()
        # Variables to keep track of success rate, running time,
        # and distance traveled
        n_locations = len(self.locations)
        n_goals = 0
        n_successes = 0
        i = 0 #n_locations
        
        self.distance_traveled = 0
        distance = 0.0
        
        start_time = rospy.Time.now()
        
        self.running_time = 0
        self.TaskProcessQ = Int32()
        self.location = ""
        self.location_pre = ""
        self.last_location = ""
        self.stoppoint = ""
        self.currentgoal = ""
        self.soft_key = ""
        self.BC_Status = 0
        self.twicereact = 0
        self.stop_state = -1
        self.failtime = 0
        
        waypoints = list()
        rate = 1
        self.r = rospy.Rate(rate)
        
        self.InspectInfo = InspectInfo()
        self.rows = None
		
        self.StatuAreaInfo = {}
        self.idposearray = None #IDPoseArray()
        self.allposearray = None #IDPoseArray()
        self.calcposearray = IDPoseArray()
        
        goal_arrived = Taskarrived()
        # Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        #rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)

        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        rospy.Subscriber("/task_points", IDPoseArray, self.get_task_points)
        rospy.Subscriber("/all_points", IDPoseArray, self.get_all_points)
        rospy.Subscriber("/control_style", String, self.controlstyleCallback)
        rospy.Subscriber("/FeedbackMsg", FeedbackMsg, self.FeedbackMsgCallback)
        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
            rospy.sleep(1)         
        rospy.loginfo("Starting navigation test")
        
        self.tf_relation = TfRelation()
        rospy.sleep(5)
        StatuThread = threading.Thread(target=self.SubStatuCallBack, args=())
        StatuThread.setDaemon(True)
        StatuThread.start()
        self.r.sleep()
        #Initialize the visualization markers for RViz
        self.init_markers()
        Delay_motion_call = False
        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():
            try:
                #检测剩余任务
                if len(self.locations) == 0:
                    i = 0
                    rospy.sleep(1)
                    print "wait for inspect task"
                    continue
                #计算任务序号
                if i < len(self.locations_name):
                    self.location = self.locations_name[i]
                    self.location_pre = self.locations_pre_name[i]
                    print  "if i < len(self.locations_name):  i= " , i
                # Keep track of the distance traveled.
                # Use updated initial pose if available.
                # initial_pose.header.stamp = ""
                if initial_pose.header.stamp == "":
                    try:
                        if self.last_location == "":
                            self.last_location = self.locations_pre_name[i]
                            distance = sqrt(pow(self.locations[self.location].position.x - 
                                            self.locations_start[self.last_location].position.x, 2) +
                                        pow(self.locations[self.location].position.y - 
                                            self.locations_start[self.last_location].position.y, 2))
                            print "distance",distance
                    except Exception as err:
                        print "distance exception initial_pose.header.stamp == ''",err
                else:
                    try:
                        rospy.loginfo("Updating current pose.")
                        distance = sqrt(pow(self.locations[self.location].position.x - 
                                        initial_pose.pose.pose.position.x, 2) +
                                    pow(self.locations[self.location].position.y - 
                                        initial_pose.pose.pose.position.y, 2))                
                    except Exception as err:
                        print "distance exception "
                    initial_pose.header.stamp = ""
                # 跃过在同一直线上的停车点
                if self.locations_corner[self.location] != 'corner' and self.location != self.stoppoint:
                     self.distance_traveled += distance
                     # How long have we been running?
                     self.running_time = rospy.Time.now() - start_time
                     self.running_time = self.running_time.secs / 60.0  
                     n_successes += 1
                     rospy.loginfo("任务停车点：" + str(self.stoppoint))
                     rospy.loginfo("跳跃中间点：" + str(self.location))
                     i += 1
                     continue
                # 停止任务，切换到手动
                if self.control_style == 'manual':
                    self.locations_name = list()
                    self.locations = dict()
                    self.idposearray = None
                    i = 0
                    continue
                # Set up the next goal location
                self.goal = MoveBaseGoal()
                self.goal.target_pose.pose = self.locations[self.location]
                print "目标点",self.location
                print "坐标值",self.locations[self.location]
                self.goal.target_pose.header.frame_id = 'map'
                self.goal.target_pose.header.stamp = rospy.Time.now()
                # Let the user know where the robot is going next
                rospy.loginfo("Going to: " + str(self.location))
                waypoints.append(Pose(Point(self.locations[self.location].position.x, self.locations[self.location].position.y, 0),self.locations[self.location].orientation))
                # Update the marker display
                self.markers.points.append(Point(self.locations[self.location].position.x, self.locations[self.location].position.y, 0))
                self.marker_pub.publish(self.markers)
                
                self.move_base.send_goal(self.goal)
                self.twicereact += 1
                cur_task = IDPose()
                cur_task.poseid_start = self.locations_start_point_ID[self.location]
                cur_task.poseid_end = self.location
                cur_task.curvity = self.locations_curvity[self.location]
                self.current_task_pub.publish(cur_task)
                error_distance = 0.0
                self.currentgoal = self.location
                
                # Allow 5 minutes to get there
                finished_within_time = self.move_base.wait_for_result(rospy.Duration(60000)) 
                # Check for success or failure
                if not finished_within_time:
                    self.move_base.cancel_goal()
                    rospy.loginfo("Timed out achieving goal")
                    continue
                else:
                    state = self.move_base.get_state()
                    if state == GoalStatus.SUCCEEDED:
                        rospy.loginfo("Goal succeeded!")
                        rospy.loginfo("State:" + str(state))
                        (position_l, rotation) = self.tf_relation.get_odom_map()
                        error_distance_x = abs(self.locations[self.currentgoal].position.x - self.InspectInfo.PosX)
                        error_distance_y = abs(self.locations[self.currentgoal].position.y - self.InspectInfo.PosY)
                        rospy.loginfo("location: " + str(self.currentgoal))
                        rospy.loginfo("location_pre: " + str(self.location_pre))
                        rospy.loginfo("error_distance_x: " + str(error_distance_x))
                        rospy.loginfo("error_distance_y: " + str(error_distance_y))
          
                        if self.twicereact < 2 and (error_distance_x > 0.1 and error_distance_y > 0.1) and self.currentgoal != self.DockPile:
                            #self.move_base.cancel_goal()
                            goal_arrived.Taskarrived = self.currentgoal
                            goal_arrived.Request = False
                            goal_arrived.Bepassinground = self.locations_passinground_task[self.location]
                            self.arrived_point_pub.publish(goal_arrived) 
                            rospy.sleep(self.rest_time)
                            self.cmd_vel_pub.publish(Twist())
                            continue
                        else:
                            self.twicereact = 0
                            n_successes += 1
                            self.distance_traveled += distance
                            if self.currentgoal == self.DockPile:
                                goal_arrived.Bepassinground = False
                            else:
                                goal_arrived.Bepassinground = self.locations_passinground_task[self.location]
                            goal_arrived.Taskarrived = self.currentgoal
                            goal_arrived.Request = True
                            self.arrived_point_pub.publish(goal_arrived)
                            rospy.loginfo("current goal ID: " + str(self.currentgoal))                                                                                                                              
#                         if self.failtime > countMax and not Delay_motion_call:
#                             srv = Delayaction()
#                             srv.request.Overtime = true;
#                             srv.request.Arrived_goal = _InspectInfo.CurrentPoint;
#                             srv.request.Task_goal = _InspectInfo.EndPoint;
#                             srv.request.Current_lineID = _InspectInfo.LineID;
#                             srv.request.Limit_time = event_limit_time;
#                             Delay_motion_call = self.Delay_motion_client(srv)
#                             if not Delay_motion_call:
#                                 rospy.sleep(self.rest_time*5)
#                                 continue
#                             else:
#                                 self.locations_name = list()
#                                 self.locations = dict()
#                                 self.idposearray = None
#                                 i = 0
#                                 self.failtime = 0
#                                 call_succed = False
#                                 rospy.loginfo("calc out ABORTED goal")
#                                 continue
#                         else:
#                             rospy.sleep(self.rest_time)
#                             continue
                    else:
                        rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))
                        #self.move_base.cancel_goal()
                        rospy.sleep(self.rest_time)
                        self.cmd_vel_pub.publish(Twist())
                        continue
                rospy.sleep(self.rest_time)        
                # Increment the counters
                i += 1
                n_goals += 1
                # Store the last location for distance calculations
                self.last_location = self.location
                
                if i == len(self.locations):
                    self.locations_name = list()
                    self.locations = dict()
                    self.idposearray = None
                    i = 0
                # How long have we been running?
                self.running_time = rospy.Time.now() - start_time
                self.running_time = self.running_time.secs / 60.0  
                # Print a summary success/failure, distance traveled and time elapsed
                rospy.loginfo("Success so far: " + str(n_successes) + "/" + 
                            str(n_goals) + " = " + 
                            str(100 * n_successes/n_goals) + "%")
                rospy.loginfo("Running time: " + str(trunc(self.running_time, 1)) + 
                            " min Distance: " + str(trunc(self.distance_traveled, 1)) + " m")
                rospy.sleep(self.rest_time)
            except Exception as err:
                print"nav error :",err
                rospy.sleep(self.rest_time)

    def SubStatuCallBack(self):
        #self.tf_relation = TfRelation()  #TF relation
        while not rospy.is_shutdown():
            try:
                (position_l, rotation) = self.tf_relation.get_odom_map()
                self.InspectInfo.PosX =  position_l.x
                self.InspectInfo.PosY =  position_l.y
                self.InspectInfo.Angle = rotation
                self.r.sleep()
            except Exception as err:
                rospy.loginfo("nav_demo get_odom_map: err" + str(err))
                self.r.sleep()
                continue
            try:
                self.StatuAreaInfo = self.Area_line()
                print "The returned value: self.StatuAreaInfo is ",self.StatuAreaInfo.get('Fname') ,self.StatuAreaInfo.get('Fnameend')
                if (not self.StatuAreaInfo):
                    print"SubStatuCallBack:: area and pose are None "
                    
            except Exception as err:
                print"SubStatuCallBack:: get area and pose value error :",err

            if self.StatuAreaInfo.get('Fname') == None:
                self.InspectInfo.InCtrlArea = 1
                self.r.sleep()
                print "---------------------------------------------------------------------------"
                # try:
                #     (position_l, rotation) = self.tf_relation.get_odom_map()
                #     self.InspectInfo.PosX =  position_l.x
                #     self.InspectInfo.PosY =  position_l.y
                #     self.InspectInfo.Angle = rotation
                # except Exception as err:
                #     rospy.loginfo("nav_demo get_odom_map: err" + str(err))
                #     self.r.sleep() 
                #     continue
           
                # try:
                #     if len(self.locations) > 0 and self.location != '' and self.location != None and len(self.locations_start_point_ID) > 0 :
                #         start_point_ID = self.locations_start_point_ID[self.location]
                #         end_point_ID = self.location
                #         self.InspectInfo.LineID = self.locations_line_ID[self.location]
                #         distance_line = sqrt(pow(self.locations[self.location].position.x - 
                #                 self.locations_start[self.last_location].position.x, 2) +
                #             pow(self.locations[self.location].position.y - 
                #                 self.locations_start[self.last_location].position.y, 2))
                #         distance_to_goal =  sqrt(pow( position_l.x - self.locations[end_point_ID].position.x, 2) +
                #                                     pow( position_l.y - self.locations[end_point_ID].position.y, 2))
                #         self.InspectInfo.LinePercent = float("%.2f"%(100.0-(distance_to_goal/distance_line*100)))
                #         self.InspectInfo.MoveState = "moving to " + str(self.location)
                #         self.InspectInfo.StartPoint = self.locations_start_point_ID[self.location]
                #         self.InspectInfo.EndPoint = self.location
                #         self.InspectInfo.CurrentPoint = self.InspectInfo.StartPoint
                #         if self.stoppoint == 'cz':
                #             self.InspectInfo.MoveState = "gohome"
                #     else:
                #         if self.InspectInfo.LineID == "":
                #             self.InspectInfo.StartPoint = 'cz'
                #             self.InspectInfo.EndPoint = 'cd'
                #             self.InspectInfo.CurrentPoint = 'cz'
                #             self.InspectInfo.LineID = 'cz-cd'
                #             self.InspectInfo.LinePercent = 0.0
                #             #print "self.InspectInfo.LineID = 'cd-ku'"
                #         else:
                #             self.InspectInfo.LinePercent = 100.00
                #             self.InspectInfo.CurrentPoint = self.location
                #             self.InspectInfo.MoveState = "stop at " + str(self.location)
                #         #self.InspectInfo.CurrentPoint = 'B4'
                            

                #     self.InspectInfo.OperationMileage = self.distance_traveled
                #     self.InspectInfo.RuntimeLength = self.running_time
                #     #print "self.InspectInfo.LineID", self.InspectInfo.LineID
                #     self.move_info_pub.publish(self.InspectInfo)
                #     self.r.sleep()    
                # except Exception as err:
                #     rospy.loginfo("nav_demo start_point_ID err:" + str(err))
                #     print "nav_demo start_point_ID err:", str(err)
                #     self.r.sleep()
                #     continue
            else:
                self.InspectInfo.InCtrlArea = 0
                self.InspectInfo.PosX = self.StatuAreaInfo.get('Curx')
                self.InspectInfo.PosY = self.StatuAreaInfo.get('Cury')
                self.InspectInfo.Angle = self.StatuAreaInfo.get('CurTh')        
                self.InspectInfo.LineID = self.StatuAreaInfo.get('CurLine')
                self.InspectInfo.LinePercent = self.StatuAreaInfo.get('CurLineDone')*100
                self.InspectInfo.StartPoint = self.StatuAreaInfo.get('Fname')
                self.InspectInfo.EndPoint = self.StatuAreaInfo.get('Fnameend')
                self.InspectInfo.OperationMileage = self.distance_traveled
                self.InspectInfo.RuntimeLength = self.running_time
                
                if len(self.locations) > 0:
                    self.InspectInfo.MoveState = "moving to " + str(self.location)
                    self.InspectInfo.MoveStateAction = "moving"
                    self.InspectInfo.CurrentPoint = self.InspectInfo.StartPoint
                    if self.stoppoint == self.DockPile:
                        self.InspectInfo.MoveState = "gohome"
                        self.InspectInfo.MoveStateAction = "gohome"
                else:
                    if self.location == '':
                        if self.InspectInfo.LinePercent > 99.5:
                            self.InspectInfo.CurrentPoint = self.InspectInfo.EndPoint
                            self.InspectInfo.MoveState = "stop at " + str(self.InspectInfo.CurrentPoint)
                            self.InspectInfo.MoveStateAction = "stop_at"
                        elif self.InspectInfo.LinePercent < 0.5:
                            self.InspectInfo.CurrentPoint = self.InspectInfo.StartPoint
                            self.InspectInfo.MoveState = "stop at " + str(self.InspectInfo.CurrentPoint)
                            self.InspectInfo.MoveStateAction = "stop_at"
                        else:
                            self.InspectInfo.CurrentPoint = self.InspectInfo.StartPoint
                            self.InspectInfo.MoveState = "stop between " + str(self.InspectInfo.StartPoint) + " and " + str(self.InspectInfo.EndPoint)
                            self.InspectInfo.MoveStateAction = "stop_between"
                    else:
                        if self.InspectInfo.LinePercent > 99.5: #and 
                            if self.location == self.InspectInfo.EndPoint:
                                self.InspectInfo.CurrentPoint = self.InspectInfo.EndPoint
                                self.InspectInfo.MoveState = "stop at " + str(self.InspectInfo.CurrentPoint)
                                self.InspectInfo.MoveStateAction = "stop_at"
                        elif self.InspectInfo.LinePercent < 0.5:
                            if self.location == self.InspectInfo.StartPoint:
                                self.InspectInfo.CurrentPoint = self.InspectInfo.StartPoint
                                self.InspectInfo.MoveState = "stop at " + str(self.InspectInfo.CurrentPoint)
                                self.InspectInfo.MoveStateAction = "stop_at"
                        else:
                            self.InspectInfo.CurrentPoint = self.InspectInfo.StartPoint
                            self.InspectInfo.MoveState = "stop between " + str(self.InspectInfo.StartPoint) + " and " + str(self.InspectInfo.EndPoint)
                            self.InspectInfo.MoveStateAction = "stop_between"
            try:
                self.move_info_pub.publish(self.InspectInfo)
                self.r.sleep()
            except Exception as err:
                rospy.loginfo("nav_demo get_odom_map : err" + str(err))
                self.r.sleep()

    def controlstyleCallback(self,data):
        self.control_style =  data.data
        if self.control_style == 'manual':
            #if self.move_base.get_state == GoalStatus.ACTIVE:
            self.move_base.cancel_goal()
            rospy.sleep(2)
            self.cmd_vel_pub.publish(Twist())
            
            cancel_done = String()
            cancel_done.data = "cancel_done"
            self.control_style_tl.publish(cancel_done)
            rospy.loginfo("取消任务，切换到手动模式")
            
    def Delay_motion_client(self,srv):
        rospy.wait_for_service('/move_base/delay_motion',Delayaction)
        try:
            delay_motion = rospy.ServiceProxy('/move_base/delay_motion', Delayaction)
            result = delay_motion(self.Delayaction_srv)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return  result  
        
    def FeedbackMsgCallback(self,data):
        self.BC_Status = data.BC_Status
        #pass
            
    def get_all_points(self,data):
        self.allposearray = data.idposearray

    def get_task_points(self,IDPoseArrayList):
        #print "4"
        print "IDPoseArrayList",IDPoseArrayList
        count = 0
        self.locations_name = list()
        self.locations_pre_name = list()
        self.locations_start_point_ID = dict()
        self.locations_line_ID = dict()
        self.locations_corner = dict()
        self.locations = dict()
        self.locations_start = dict()
        self.locations_curverty = dict()
        self.locations_passinground_task = dict()
        pose_node = IDPose()
        self.idposearray = IDPoseArrayList.idposearray

        for pose_node in IDPoseArrayList.idposearray:
            self.locations_name.append(pose_node.poseid_end)
            self.locations_pre_name.append(pose_node.poseid_start)

            self.locations[pose_node.poseid_end] = Pose(Point(pose_node.pose.position.x,pose_node.pose.position.y,pose_node.pose.position.z), Quaternion(pose_node.pose.orientation.x,pose_node.pose.orientation.y,pose_node.pose.orientation.z,pose_node.pose.orientation.w))
            self.locations_start[pose_node.poseid_start] = Pose(Point(pose_node.pose_start.position.x,pose_node.pose_start.position.y,pose_node.pose_start.position.z), Quaternion(pose_node.pose_start.orientation.x,pose_node.pose_start.orientation.y,pose_node.pose_start.orientation.z,pose_node.pose_start.orientation.w))

            self.locations_start_point_ID[pose_node.poseid_end] = pose_node.poseid_start
            self.locations_line_ID[pose_node.poseid_end] = pose_node.line_id
            self.locations_corner[pose_node.poseid_end] = pose_node.pose_corner
            
            self.locations_curvity[pose_node.poseid_end] = pose_node.curvity
            self.locations_passinground_task[pose_node.poseid_end] = pose_node.passinground_task
            count+=1
        self.n_locations = len(self.locations)
        self.stoppoint = self.locations_name[len(self.locations_name)-1]
                
    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def init_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.4
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 0, 'g': 0, 'b': 1.0, 'a': 1.0}
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=5)
        
        # Initialize the marker points list.
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
        
        self.markers.header.frame_id = 'map'
        self.markers.header.stamp = rospy.Time.now()
        self.markers.points = list()



    #计算当前点与路径直线间的距离            
    def cal_distance(self,xcur, ycur,startpose_x,startpose_y,nextpose_x,nextpose_y):
        try:
            a_x = nextpose_x- startpose_x
            a_y = nextpose_y- startpose_y
            if a_x == 0:
                if a_y >= 0:
                    ptl = xcur - nextpose_x
                else:
                    ptl = nextpose_x- xcur
            else:
                K = (nextpose_y- startpose_y)/(nextpose_x- startpose_x)
                B = (nextpose_x*startpose_y - startpose_x*nextpose_y)/(nextpose_x- startpose_x)
                if a_x>0:
                    if (-K*xcur + ycur - B) > 0:
                        ptl = -abs(-K*xcur + ycur - B)/sqrt(pow(-K,2) + 1)    
                    else:
                        ptl = abs(-K*xcur + ycur - B)/sqrt(pow(-K,2) + 1)    
                else:
                    if (-K*xcur + ycur - B) > 0:
                        ptl = abs(-K*xcur + ycur - B)/sqrt(pow(-K,2) + 1)    
                    else:
                        ptl = -abs(-K*xcur + ycur - B)/sqrt(pow(-K,2) + 1)    
            #print "AreaInformation::cal_distance  ptl ==%f" %(ptl)
            return ptl
        except Exception as err:    
            print "AreaInformation::AreaInfor::cal_distance  Error: ",err
        #return

    #计算路径与地图正方向间夹角
    def cal_path_rotation(self,xcur,ycur,nextpose_x,nextpose_y):
    #计算下一条路径直线和全局正方间的夹角 (规定正方向向量（xcur+1,ycur）)
        prodis = sqrt(pow(nextpose_x-xcur,2)+pow(nextpose_y-ycur,2))
        curdis = 1
        dot_product = (nextpose_x-xcur)
        dif_product = (nextpose_y-ycur)
        try:
            if prodis == 0:
                return 0
            cosm = dot_product / (prodis*curdis)
            sinm = dif_product / (prodis*curdis)
                    
            if cosm >= 0:
                path_th = asin(sinm)
            else:
                if sinm >=0:
                    path_th = pi - asin(sinm)
                else:
                    path_th = -pi - asin(sinm)
            return path_th
        except Exception as err:
            print "AreaInformation::AreaInfor::cal_path_rotation  Error:",err
            #return

    def Area_line(self):
        ReAreaInfo = {}
        start_point = Point()
        goal_point = Point()
        position = Point()
        try:
            (position, rotation) = self.tf_relation.get_odom_map()
            #print "AreaInformation::Area_line the position,rotation" ,position,rotation
        except Exception as err:
            #self.InspectInfo.InCtrlArea = 2
            print "AreaInformation::AreaInfor::Area_line  get odom Error"
            return       
                        
        try:
            if self.allposearray != None or self.idposearray != None:
                if self.idposearray != None:
                    #print "Area_line len(self.idposearray)++++++" ,self.idposearray
                    calcposearray = self.allposearray
                else:
                    #print "Area_line len(self.allposearray)++++++" ,self.allposearray
                    calcposearray = self.allposearray
                for row in calcposearray:
                    if rospy.is_shutdown():
                        break                   
                    line_id = row.line_id
                    fname_start = row.poseid_start
                    start_point.x = row.pose_start.position.x
                    start_point.y = row.pose_start.position.y
                    fname_end = row.poseid_end
                    goal_point.x = row.pose.position.x
                    goal_point.y = row.pose.position.y
                    line_distance = sqrt(pow((goal_point.x - start_point.x), 2) + pow((goal_point.y - start_point.y), 2))
                    
                    if (line_distance > 1.5):
                        distance_Th = 1.0 #0.5
                    else:
                        distance_Th = 0.6 #0.35				
                    try:			
                        distances = self.cal_distance(position.x, position.y, start_point.x, start_point.y, goal_point.x, goal_point.y)
                        #print "AreaInformation::Area_line  distances %.2f" %distances				
                        distance_done = sqrt(pow((position.x - start_point.x), 2) + pow((position.y - start_point.y), 2))
                        distance_overplus = sqrt(pow(goal_point.x-position.x,2) + pow(goal_point.y-position.y,2))
                        path_rot = self.cal_path_rotation(start_point.x,start_point.y,goal_point.x,goal_point.y)
                    except Exception as err:
                        print "AreaInformation::AreaInfor::Area_line The distance calculate exception :",err
                        continue
                                        
                    th_angle = rotation - path_rot
                    #th_angle = normalize_angle(th_angle)
                    if th_angle >= pi:
                        th_angle -= pi*2
                    elif th_angle < -pi:
                        th_angle += pi*2
                    else:
                        th_angle = th_angle
                    
                    #print "AreaInformation::Area_line the distances and angle:%.2f,%.2f" ,distances,th_angle		
                    if (abs(distances) <= distance_Th) and (-pi/3 < th_angle <= pi/3):
                        if (start_point.x <= goal_point.x):
                            s_x = start_point.x
                            g_x = goal_point.x
                            x_range = goal_point.x - start_point.x
                        else:
                            s_x = goal_point.x
                            g_x = start_point.x
                            x_range = start_point.x - goal_point.x
                            
                        if (start_point.y <= goal_point.y):
                            s_y = start_point.y
                            g_y = goal_point.y
                            y_range = goal_point.y - start_point.y
                        else:
                            s_y = goal_point.y
                            g_y = start_point.y
                            y_range = start_point.y - goal_point.y
                        
                        try:	
                            if (y_range <= x_range):
                                if ((s_x - distance_Th) <= position.x <= (g_x + distance_Th)):
                                    try:
                                        if (distance_done >= line_distance ) and (distance_overplus <= 0.5):
                                            distance_done = line_distance
                                            #print"AreaInformation::Area_line the distance_done and distance_overplus11 :%.2f,%.2f",distance_done ,distance_overplus
                                        elif (distance_overplus >= line_distance) and (distance_done <= 0.5):
                                            distance_done = 0
                                            #print"AreaInformation::Area_line the distance_done and distance_overplus12 :%.2f,%.2f",distance_done ,distance_overplus
                                        elif (distance_done >= line_distance) and (distance_overplus > 0.5):
                                            #print"AreaInformation::Area_line continue the distance_overplus11:%.2f",distance_overplus
                                            continue
                                        elif (distance_overplus >= line_distance) and (distance_done > 0.5):
                                            #print"AreaInformation::Area_line continue the distance_overplus12:%.2f",distance_overplus
                                            continue
                                        else:
                                            pass
                                                                    
                                        ReAreaInfo['CurLine'] = line_id
                                        print "Area_line cur_line 1111",line_id
                                        ReAreaInfo['CurLinePass'] = distance_done
                                        ReAreaInfo['CurLineDone'] = distance_done/line_distance
                                        ReAreaInfo['Curx'] = position.x
                                        ReAreaInfo['Cury'] = position.y
                                        ReAreaInfo['CurTh'] = rotation
                                        ReAreaInfo['Fname'] = fname_start
                                        ReAreaInfo['Fnameend'] = fname_end
                                        print"AreaInformation::Area_line ReAreaInfo1: ",ReAreaInfo
                                        break
                                    except Exception as err:
                                        print "AreaInformation::AreaInfor::Area_line exception if-else judge1 the Error: ",err
                                        continue							
                            else:
                                if ((s_y - distance_Th) <= position.y <= (g_y + distance_Th)):
                                    try:
                                        if (distance_done >= line_distance ) and (distance_overplus <= 0.5):
                                            distance_done = line_distance
                                            #print"AreaInformation::Area_line the distance_done and distance_overplus21 :%.2f,%.2f",distance_done ,distance_overplus
                                        elif (distance_overplus >= line_distance) and (distance_done <= 0.5):
                                            distance_done = 0
                                            #print"AreaInformation::Area_line the distance_done and distance_overplus22 :%.2f,%.2f",distance_done ,distance_overplus
                                        elif (distance_done >= line_distance) and (distance_overplus > 0.5):
                                            #print"AreaInformation::Area_line continue the distance_overplus21:%.2f",distance_overplus
                                            continue
                                        elif (distance_overplus >= line_distance) and (distance_done > 0.5):
                                            #print"AreaInformation::Area_line continue the distance_overplus22:%.2f",distance_overplus
                                            continue
                                        else:
                                            pass
                                        
                                        ReAreaInfo['CurLine'] = line_id
                                        print "Area_line cur_line 2222",line_id
                                        ReAreaInfo['CurLinePass'] = distance_done
                                        ReAreaInfo['CurLineDone'] = distance_done/line_distance
                                        ReAreaInfo['Curx'] = position.x
                                        ReAreaInfo['Cury'] = position.y
                                        ReAreaInfo['CurTh'] = rotation
                                        ReAreaInfo['Fname'] = fname_start
                                        ReAreaInfo['Fnameend'] = fname_end
                                        #ReAreaInfo['PosID'] = posid
                                        #print"AreaInformation::Area_line ReAreaInfo2: ",ReAreaInfo
                                        break
                                    except Exception as err:
                                        print "Area_line exception if-else judge2 the Error: ",err
                                        continue
                        except Exception as err:
                            print "Area_line exception  The if-else the Error: ",err
                            continue
                    else:
                        continue			
            else:
                print "Area_line the length of self.rows is 0"
        except Exception as err:
            print "Area_line The calculate exception error:",err
            #return None
        print "Area_line The Calculate Done "
        return ReAreaInfo		

def trunc(f, n):
    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])

if __name__ == '__main__':
    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
