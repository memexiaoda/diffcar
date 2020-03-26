#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion,Vector3
from transform_utils import quat_to_angle, normalize_angle
from numpy import sign
from math import radians, sqrt, pow, pi,sin,cos,asin
from visualization_msgs.msg import Marker
import zmq
from PackageMessage_pb2 import PackageMessage
import uuid
import time
from std_msgs.msg import String
from AllParameter import SERVICEPUBIP, SERVICESUBIP, AllTopic, ORDER, SENDER, RECVER
from tf_pose import TfRelation

class PIDCONTRL():
    def __init__(self):
        self.Perror = 0.0
        self.PrevErr = 0.0
        self.Ierror = 0.0
        self.Poutput = 0.0
    
class ControlCar():
    def __init__(self):
        # Give the node a name
        #rospy.init_node('ControlCar', anonymous=False)         
        # Set rospy to execute a shutdown function when terminating the script
        #rospy.on_shutdown(self.shutdown)        
        # How fast will we check the odometry values?
        rate = 10
        self.r = rospy.Rate(rate)
        
        self.StLineSpeed = 0.0
        self.StAngleSpeed = 0.0
        self.obstacle_flag = 0 
        
        self.max_speed = rospy.get_param('LaserControl/max_speed', 0.8)          #最大线速度
        self.min_speed = rospy.get_param('Lasercontrol/min_speed', 0.1)           #最小线速度
        self.angular_speed = rospy.get_param('Lasercontrol/ang_speed', 0.1)    # radians per second
        self.distance_slow = rospy.get_param('Lasercontrol/distance_slow', 1.4) #减速距离
        
        self.d_max_speed = rospy.get_param('LaserControl/d_max_speed', -0.3)
        self.d_min_speed = rospy.get_param('LaserControl/d_max_speed', -0.1)
        self.d_angular_speed = rospy.get_param('Lasercontrol/d_ang_speed', -0.1)    # radians per second
        
        self.Kp = rospy.get_param('arbotix/controllers/base_controller/Kp', 5)
        self.Kd = rospy.get_param('arbotix/controllers/base_controller/Kd', 1)
        self.Ki = rospy.get_param('arbotix/controllers/base_controller/Ki', 0)
        self.Ko = rospy.get_param('arbotix/controllers/base_controller/Ko', 50)
     
        #self.angular_tolerance = rospy.get_param("~angular_tolerance", radians(0.1)) # degrees to radians
        self.angular_tolerance = radians(0.1) # degrees to radians
        self.MaxOutput = 0.3 #0.07           #PID最大误差值
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)  
     
        self.tf_relation = TfRelation()   #TF relation
        
        self.con = zmq.Context()
        self.PublishMsg = PackageMessage()
        self.Token = "Laser"
        self.ErrorDisPub = rospy.Publisher("ErrDistance", String,queue_size=5)
  
            
    def zmqpubcon(self):
        self.sock = self.con.socket(zmq.PUB)
        self.sock.connect(SERVICEPUBIP)
        
    def Pubmsg(self):
        self.PublishMsg.Clear()
        self.PublishMsg.SessionId = "%s" %uuid.uuid1()
        self.PublishMsg.Time = int(time.time() * 100000000)
        self.PublishMsg.From = SENDER
        self.PublishMsg.To = RECVER[0]
        self.PublishMsg.CallMessage.Function = ORDER[0]
        
    def zmqsubcon(self):
        self.subsock = self.con.socket(zmq.SUB)
        self.subsock.connect(SERVICESUBIP)
        self.subsock.setsockopt(zmq.SUBSCRIBE,AllTopic[0])
        
    #The receiving function has not been called   
    def recvmsg(self,sendsessionid):
        recvmsg = PackageMessage()
        for k in range(3):
            try:
                topic,re,msg = self.subsock.recv_multipart()
                if (msg != None) and (msg != ""):
                    recvmsg.ParseFromString(msg)
                    if recvmsg.To == SENDER and recvmsg.SessionId == sendsessionid: 
                        if recvmsg.ResultMessage.ErrorCode != 0:
                            print recvmsg.ResultMessage.ErrorCode
                            return recvmsg.ResultMessage.ErrorCode
                        return 0
                        #break
                    else:
                        continue
                else:
                    print "ControlThread::recvmsg  Recvmessage is NULL!" 
                    return 2
            except Exception as err:
                print err
                return 3          

    def Start_Speed(self,data):
        self.StLineSpeed = data.x
        self.StAngleSpeed = data.z
        
    def TokenCallBack(self,data):
        if data.data != "":
            self.Token = data.data
                           
    def d_keep_line(self,lastgoal,goal,work,stop,rfid,curfname):
        
        self.distance_slow =  rospy.get_param('LaserControl/distance_slow', 1.4)
        
        move_cmd = Twist()
        move_cmd.linear.x = self.StLineSpeed
        move_cmd.angular.z = self.StAngleSpeed
        
        position = Point()
        th_pid = PIDCONTRL()
        dis_pid = PIDCONTRL() 
        
        try:            
            start = Point()       
            start = lastgoal
            (position, rotation) = self.tf_relation.get_odom_map()
            #th_start = rotation
        except Exception as err:    
            print "ControlThread::d_keep_line  get odom Error"
            
        distance = 0.0
        distance_overplus = 0.0
        goal_distance = sqrt(pow(goal.x - start.x,2) + pow(goal.y - start.y,2))
        #计算路径直线与全局地图正方向间夹角
        #path_rot = self.cal_path_rotation(start.x,start.y,th_start,goal.x,goal.y)
        #当前点距离目标的距离
        distance_overplus = sqrt(pow(goal.x-position.x,2) + pow(goal.y-position.y,2))
        
        #等待settoken信号
        self.zmqpubcon()
        self.zmqsubcon()
        rospy.Subscriber("TokenPose", String,self.TokenCallBack)
                         
        #while distance < goal_distance  and not rospy.is_shutdown():
        while not rospy.is_shutdown()and (goal_distance > 0):        
            print "ControlThread::d_keep_line curfname:", curfname
            if self.Token == "DriverControl" or self.Token=="Laser_manual"  or self.Token=="Laser_stop":
                print "ControlThread::d_keep_line Break Token: ", self.Token
                break
            try:    
                if distance_overplus <= self.distance_slow and stop == 1:
                    if self.d_min_speed > move_cmd.linear.x:
                        move_cmd.linear.x = min(self.d_min_speed,move_cmd.linear.x + 0.02)#0.02
                        #print "ControlThread::d_keep_line dkspeed 1-1 "
                    else:
                        if distance_overplus > 0.018:
                            if distance > (goal_distance):
                                move_cmd.linear.x = 0.0
                                print "ControlThread::d_keep_line dkspeed 1-2-1 0.0"
                                #if abs(th_pid.Perror) < 0.01:
                                    #print "ControlThread::d_keep_line distance_overplus 4=%f" %distance_overplus 
                                break
                            else:
                                move_cmd.linear.x = min(self.d_min_speed,move_cmd.linear.x)
                                #print "ControlThread::d_keep_line dkspeed 1-2-1-2 "
                        else:
                            move_cmd.linear.x = 0.0
                            print "ControlThread::d_keep_line dkspeed 1-2-2 0.0"
                            #if abs(th_pid.Perror) < 0.01:
                                #print "ControlThread::d_keep_line distance_overplus 5=%f" %distance_overplus 
                            break
                else:
                    if self.d_max_speed > move_cmd.linear.x:
                        move_cmd.linear.x = min(self.d_max_speed,move_cmd.linear.x + 0.02)#0.02
                        #print "ControlThread::d_keep_line dkspeed 2-1"
                    else:                   
                        if distance > (goal_distance):
                            print "ControlThread::d_keep_line break distance_overplus 3=%f" %distance_overplus
                            break                           
                        else:
                            move_cmd.linear.x = min(self.max_speed,move_cmd.linear.x - 0.02)
                            #print "ControlThread::d_keep_line dkspeed 2-2 "          
            except Exception as err:    
                print "ControlThread::d_keep_line  exception Error: ",err
                
            print "ControlThread::d_keep_line move_cmd.linear.x: ", move_cmd.linear.x
            ###################################################
            
            #th_pid.Perror = -th_pid.Perror
            #if move_cmd.linear.x == 0.0:
                #error_th = 1.5 * self.CacPid(th_pid)
            #else:
                #error_th = self.CacPid(dis_pid) + 1.5*self.CacPid(th_pid) 
            #print"ControlThread::d_keep_line CacPid error_th: ",error_th           
            #move_cmd.angular.z = error_th
            ####################################################
            move_cmd.angular.z = 0.0
            
            self.cmd_vel.publish(move_cmd)
            self.Pubmsg()
            self.PublishMsg.CallMessage.Parameters.append("%d" %(move_cmd.linear.x *100))
            self.PublishMsg.CallMessage.Parameters.append("%d" %(-move_cmd.angular.z *100))
            try:
                self.sock.send_multipart([AllTopic[3], "\0", self.PublishMsg.SerializeToString()])
            except Exception as err:
                print "ControlThread::d_keep_line sock.send_multipart() Error"
            print "ControlThread::d_keep_line send speed_Msg: ", self.PublishMsg
            #===================================================================================
            self.r.sleep()            
            try:
                (position, rotation) = self.tf_relation.get_odom_map()
            except Exception as err:    
                print "ControlThread::d_keep_line  get_odom Error"
                
            if position == None or rotation == None:
                self.carstop()
                self.Pubmsg()
                self.PublishMsg.To = RECVER[1]
                self.PublishMsg.CallMessage.Function = ORDER[13]
                self.PublishMsg.CallMessage.Parameters.append(str(rfid))
                try:
                    self.sock.send_multipart([AllTopic[0], "\0", self.PublishMsg.SerializeToString()])
                except Exception as err:
                    print "ControlThread::d_keep_line sock.send_multipart() exception "
                SendError = String()
                SendError = "0.0:stop"
                self.ErrorDisPub.publish(SendError)
                return
            #####################################################3 
            #th_pid.Perror = rotation - path_rot
            #if th_pid.Perror >= pi:
                #th_pid.Perror -= pi*2
            #elif th_pid.Perror < -pi:
                #th_pid.Perror += pi*2
            #else:
                #th_pid.Perror = th_pid.Perror
                
            #dis_pid.Perror = self.cal_distance(position.x, position.y, start.x, start.y, goal.x, goal.y)
            #print "ControlThread::d_keep_line cal_distance dis_pid.Perror==%f" %(dis_pid.Perror)
            #########################################################################
            distance_overplus = sqrt(pow(goal.x-position.x,2) + pow(goal.y-position.y,2))
            # Compute the Euclidean distance from the start
            distance = sqrt(pow((position.x - start.x), 2) + pow((position.y - start.y), 2))
            SendError = String()
            SendError = "0.0:%s" %(work)
            #print "++++++++++++++++++++++++++++++++++++++++++++++++   %.2f" %dis_pid.Perror
            self.ErrorDisPub.publish(SendError)
            
        if stop == 1:
            self.carstop()
            
        self.Pubmsg()
        self.PublishMsg.To = RECVER[1]
        self.PublishMsg.CallMessage.Function = ORDER[1]
        self.PublishMsg.CallMessage.Parameters.append(str(rfid))
        try:
            self.sock.send_multipart([AllTopic[0], "\0", self.PublishMsg.SerializeToString()])
        except Exception as err:
            print"ControlThread::d_keep_line sock.send_multipart() exception"
        print "ControlThread::d_keep_line Lasermovedone PublishMsg: ", self.PublishMsg
        
        #resultA = self.recvmsg(self.PublishMsg.SessionId)
        #time.sleep(1)
        #while resultA != 0:
        #   self.sock.send_multipart([AllTopic[0], "\0", self.PublishMsg.SerializeToString()])
        #   time.sleep(1)
        #   resultA = self.recvmsg(self.PublishMsg.SessionId)

        SendError = String()
        SendError = "0.0:stop" 
        self.ErrorDisPub.publish(SendError)
        return 
        
    def keep_line(self,lastgoal,goal,work,stop,rfid,curfname):
        self.max_speed = rospy.get_param('LaserControl/max_speed', 0.8)                  #最大线速度
        self.min_speed = rospy.get_param('LaserControl/min_speed', 0.1)                  #最小线速度
        self.distance_slow =  rospy.get_param('LaserControl/distance_slow', 1.4)  
        #print "ControlThread::keep_line speed:self.max_speed",self.max_speed,"self.min_speed",self.min_speed
        if rfid == "cda":
            #rospy.set_param('LaserControl/max_speed',0.3)
            self.max_speed = 0.3
                        
        move_cmd = Twist()
        position = Point()
        start = Point()
        th_pid = PIDCONTRL()
        dis_pid = PIDCONTRL() 
        try:                  
            start = lastgoal
            (position, rotation) = self.tf_relation.get_odom_map()
            th_start = rotation 
        except Exception as err:    
            print "ControlThread::keep_line  get odom Error"
            
        move_cmd.linear.x = self.StLineSpeed
        move_cmd.angular.z = self.StAngleSpeed
        distance = 0.0
        distance_overplus = 0.0
        goal_distance = sqrt(pow(goal.x - start.x,2) + pow(goal.y - start.y,2))
        #计算路径直线与全局地图正方向间夹角
        path_rot = self.cal_path_rotation(start.x,start.y,th_start,goal.x,goal.y) 
        distance_overplus = sqrt(pow(goal.x-position.x,2) + pow(goal.y-position.y,2))
        
        # Enter the loop to move along a side
        self.zmqpubcon()
        self.zmqsubcon()
        rospy.Subscriber("TokenPose", String,self.TokenCallBack)
                  
        #while distance < goal_distance  and not rospy.is_shutdown():    
        while not rospy.is_shutdown() and (goal_distance > 0):
            if self.Token == "DriverControl" or self.Token=="Laser_manual" or self.Token=="Laser_stop":
                #print distance_overplus
                print "ControlThread::keep_line Break Token: ", self.Token
                break    
            #====================================================================================
            try:
                if distance_overplus <= self.distance_slow and stop == 1:
                    if self.min_speed < move_cmd.linear.x:
                        move_cmd.linear.x = max(self.min_speed,move_cmd.linear.x - 0.05)#0.02
                    else:
                        if distance_overplus > 0.018:
                            if distance > (goal_distance):
                                move_cmd.linear.x = 0.0
                                print "ControlThresd::keep_line linear_speed:00 1 and th_pid.Perror: ",abs(th_pid.Perror)
                                if abs(th_pid.Perror) < 0.01:
                                    print "ControlThresd::keep_line distance_overplus 1=%f" %distance_overplus 
                                break
                            else:
                                move_cmd.linear.x = max(self.min_speed,move_cmd.linear.x)
                        else:
                            move_cmd.linear.x = 0.0
                            print "ControlThresd::keep_line linear_speed:00 2 and th_pid.Perror: ",abs(th_pid.Perror)
                            if abs(th_pid.Perror) < 0.01:
                                print "ControlThresd::keep_line distance_overplus 2=%f" %distance_overplus 
                            break
                else:
                    if self.max_speed < move_cmd.linear.x:
                        move_cmd.linear.x = max(self.max_speed,move_cmd.linear.x - 0.05)#0.02
                    else:
                        if distance > (goal_distance):
                            print "ControlThread::keep_line Break distance_overplus 3=%f" %distance_overplus 
                            break
                        else:
                            move_cmd.linear.x = min(self.max_speed,move_cmd.linear.x + 0.05)
            except Exception as err:    
                print "ControlThread::keep_line distance_slow exception Error: ",err
                
            print "ControlThread::keep_line move_cmd.linear.x: ", move_cmd.linear.x   
            th_pid.Perror = -th_pid.Perror
            if move_cmd.linear.x == 0.0:
                error_th = 1.5 * self.CacPid(th_pid)
            else:
                error_th = self.CacPid(dis_pid) + 1.5*self.CacPid(th_pid)             
            #print"ControlThread::keep_line CacPid error_th: ",error_th 
            move_cmd.angular.z = error_th
            try:
                print"ControlThread::keep_line self.obstacle_flag: ",self.obstacle_flag
                if self.obstacle_flag == 0:
                    self.cmd_vel.publish(move_cmd)
                    self.Pubmsg()
                    self.PublishMsg.CallMessage.Parameters.append("%d" %(move_cmd.linear.x *100))
                    self.PublishMsg.CallMessage.Parameters.append("%d" %(-move_cmd.angular.z *100))
                    self.sock.send_multipart([AllTopic[3], "\0", self.PublishMsg.SerializeToString()])
                    print "ControlThread::keep_line cmd_vel PublishMsg: ",self.PublishMsg
                elif self.obstacle_flag == 1:
                    self.carstop()
                    print "ControlThread::keep_line obstacle-carstop"
                else:
                    self.slowdown()
                    print "ControlThread::keep_line obstacle-slowdown"
            except Exception as err: 
                print"ControlThread::keep_line obstacle controlrun excepttion: ",err
            #===================================================================================
            self.r.sleep()
            try:
                (position, rotation) = self.tf_relation.get_odom_map()
            except Exception as err:    
                print "ControlThread::keep_line  get_odom Error: ",err
            if position == None or rotation == None:
                self.carstop()
                print "ControlThread::keepline postion rotation is None"
                self.Pubmsg()
                self.PublishMsg.To = RECVER[1]
                self.PublishMsg.CallMessage.Function = ORDER[13]
                self.PublishMsg.CallMessage.Parameters.append(str(rfid))
                try:
                    self.sock.send_multipart([AllTopic[0], "\0", self.PublishMsg.SerializeToString()])
                except Exception as err:
                    print "ControlThread::keep_line  sock.send_multipart() Error: ",err
                SendError = String()
                SendError = "0.0:stop"
                self.ErrorDisPub.publish(SendError)
                return
            
            th_pid.Perror = rotation - path_rot
            if th_pid.Perror >= pi:
                th_pid.Perror -= pi*2
            elif th_pid.Perror < -pi:
                th_pid.Perror += pi*2
            else:
                th_pid.Perror = th_pid.Perror
            dis_pid.Perror = self.cal_distance(position.x, position.y, start.x, start.y, goal.x, goal.y)
            #print "ControlThread::keepline dis_pid.Perror %.2f" %dis_pid.Perror
            distance_overplus = sqrt(pow(goal.x-position.x,2) + pow(goal.y-position.y,2))
            # Compute the Euclidean distance from the start
            distance = sqrt(pow((position.x - start.x), 2) + pow((position.y - start.y), 2))
            SendError = String()
            SendError = "%.2f:%s" %(error_th,work)
            self.ErrorDisPub.publish(SendError)
            
        if stop == 1:
            self.carstop()
        #if self.Token != "Laser_manual":
        self.Pubmsg()
        self.PublishMsg.To = RECVER[1]
        self.PublishMsg.CallMessage.Function = ORDER[1]
        self.PublishMsg.CallMessage.Parameters.append(str(rfid))
        try:
            self.sock.send_multipart([AllTopic[0], "\0", self.PublishMsg.SerializeToString()])
        except Exception as err:
            print "ControlThread::keep_line  sock.send_multipart() exception: ",err 
        
        print"ControlThread::keep_line Lasermove done "        
        #resultA = self.recvmsg(self.PublishMsg.SessionId)
	    #time.sleep(1)
        #while resultA != 0:
            #self.sock.send_multipart([AllTopic[0], "\0", self.PublishMsg.SerializeToString()])
	        #time.sleep(1)
            #resultA = self.recvmsg(self.PublishMsg.SessionId)

        SendError = String()
        SendError = "0.0:stop" 
        self.ErrorDisPub.publish(SendError)
        return 
            
    def turn_angle(self,lastgoal,goal,curfname):
        move_cmd = Twist()
        position = Point()
        turn_angle = 0
        try:
            (position,rotation) = self.tf_relation.get_odom_map()
        except Exception as err:    
            print "ControlThread::turn_angle get odom Error"
        last_angle = rotation
        path_rot = self.cal_path_rotation(lastgoal.x,lastgoal.y,rotation,goal.x,goal.y)  
        
        goal_angle = path_rot - rotation                   
        if goal_angle >= pi:
            goal_angle -= (2*pi)
            move_cmd.angular.z = -self.angular_speed
        elif goal_angle < -pi:
            goal_angle += (2*pi)
            move_cmd.angular.z = self.angular_speed
        else:
            move_cmd.angular.z = sign(goal_angle)*self.angular_speed 
        move_cmd.linear.x = 0.0
            
        self.zmqpubcon()
        self.zmqsubcon()
        self.carstop() 
        rospy.Subscriber("TokenPose", String,self.TokenCallBack)
        
        while abs(turn_angle + self.angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle
            if self.Token == "Laser_stop":
                print "ControlThread::turn_angle  self.Token == Laser_stop"
                break
            print "ControlThread::turn_angle move_cmd: ", move_cmd
            self.cmd_vel.publish(move_cmd)
            self.Pubmsg()
            self.PublishMsg.CallMessage.Parameters.append("%d" %(move_cmd.linear.x *100))
            self.PublishMsg.CallMessage.Parameters.append("%d" %(-move_cmd.angular.z *100))
            self.sock.send_multipart([AllTopic[3], "\0", self.PublishMsg.SerializeToString()])     
            self.r.sleep()
            # Get the current rotation
            try:
                (position, rotation) = self.tf_relation.get_odom_map()
                if position == None or rotation == None:
                    print "ControlThread::turn_angle the position and rotation is None"
                    break 
            except Exception as err:    
                print "ControlThread::turn_angle get_odom Error"                              
            # Compute the amount of rotation since the last lopp
            delta_angle = normalize_angle(rotation - last_angle)       
            turn_angle += delta_angle
            last_angle = rotation
        self.carstop()
        print "ControlThread::turn_angle done" 
#       self.Pubmsg()
#       self.PublishMsg.To = RECVER[1]
#       self.PublishMsg.CallMessage.Function = ORDER[1]
#       self.PublishMsg.CallMessage.Parameters.append("laserdone")
#       self.sock.send_multipart([AllTopic[1], "\0", self.PublishMsg.SerializeToString()])

    def carstop(self):
        print "ControlThread carstop"
        self.cmd_vel.publish(Twist())
        self.Pubmsg()
        self.PublishMsg.CallMessage.Parameters.append("0")
        self.PublishMsg.CallMessage.Parameters.append("0") 
        try:
            self.sock.send_multipart([AllTopic[3], "\0", self.PublishMsg.SerializeToString()])
        except Exception as err:    
            print "ControlThread::carstop sock.send_multipart exception" 
        #self.recvmsg(self.PublishMsg.SessionId)
    def slowdown(self):
        slowspeed = Twist()
        #slowspeed.linear.x = self.min_speed
        slowspeed.linear.x = 0.2
        slowspeed.angular.z = 0.0
        self.cmd_vel.publish(slowspeed)
        self.Pubmsg()
        self.PublishMsg.CallMessage.Parameters.append("%d" %(slowspeed.linear.x *100))
        self.PublishMsg.CallMessage.Parameters.append("%d" %(-slowspeed.angular.z *100))
        try:
            self.sock.send_multipart([AllTopic[3], "\0", self.PublishMsg.SerializeToString()])
            #print "ControlThread::slowdown slowspeed: ",slowspeed
        except Exception as err:
            print "ControlThread::slowdown sock.send_multipart exception"
        
    def shutdown(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("ControlThread::shutdown Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)   
    #计算当前点与路径直线间的距离            
    def cal_distance(self, xcur, ycur,startpose_x,startpose_y,nextpose_x,nextpose_y):
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
            #print "ControlThread::cal_distance ptl ==%f" %(ptl)
            return ptl
        except Exception as err:
            print "ControlThread::cal_distance  Error" 
    
    #计算路径与地图正方向间夹角
    def cal_path_rotation(self,xcur,ycur,thcur,nextpose_x,nextpose_y):
        #计算下一条路径直线和全局正方间的夹角 (规定正方向向量（xcur+1,ycur）)
        prodis = sqrt(pow(nextpose_x-xcur,2)+pow(nextpose_y-ycur,2))
        curdis = 1
        dot_product = (nextpose_x-xcur)
        dif_product = (nextpose_y-ycur)
        try:
            if prodis==0:
                cosm = cos(thcur)
                sinm = sin(thcur)
            else:
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
            print "ControlThread::cal_path_rotation  Error" 
        
    def CacPid(self, pt):
        
        Perror = pt.Perror
        output = (self.Kp*Perror + self.Kd*(Perror - pt.PrevErr) + self.Ki*pt.Ierror)/self.Ko
        pt.PrevErr = Perror    
        
        if output >= self.MaxOutput:
            output = self.MaxOutput
        elif output < -self.MaxOutput:
            output = -self.MaxOutput
        else:
            pt.Ierror = pt.Ierror + Perror            
        pt.Poutput = output
        #print"ControlThread::CacPid pt.output: ",pt.Poutput
        return output
    
    def init_markers(self):
        # Set up our waypoint markers
        marker_scale = 0.2
        marker_lifetime = 0 # 0 is forever
        marker_ns = 'waypoints'
        marker_id = 0
        marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
        
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=5)
        
        # Initialize the marker points list.
        self.markers = Marker()
        self.markers.ns = marker_ns
        self.markers.id = marker_id
        self.markers.type = Marker.CUBE_LIST
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
        print self.markers.points
#if __name__ == '__main__':
    #ControlCar()
