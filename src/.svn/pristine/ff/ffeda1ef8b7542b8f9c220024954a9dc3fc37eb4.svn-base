#!/usr/bin/env python 
# coding:utf8

import rospy
import Tkinter
from Tkinter import *
from em import expand
from math import *
import tf
from geometry_msgs.msg import *
from transform_utils import quat_to_angle, normalize_angle
import thread
import zmq
from AllParameter import *
import uuid
from PackageMessage_pb2 import PackageMessage
import time
from Tkinter import Button

class PIDCONTRL(): 
    Perror = 0.0
    PrevErr = 0.0
    Ierror = 0.0
    Poutput = 0.0

class TkCtrl():
    def __init__(self):
        rospy.init_node("CarCtrlU", anonymous=False)
        self.max_speed = rospy.get_param('LaserControl/max_speed', 0.5)                                    #最大线速度
        self.min_speed = rospy.get_param('Lasercontrol/min_speed', 0.05)                     #最小线速度
        self.angular_speed = rospy.get_param('Lasercontrol/ang_speed', 0.1)    # radians per second
        
        self.d_max_speed = rospy.get_param('LaserControl/d_max_speed', -0.3)
        self.d_min_speed = rospy.get_param('LaserControl/d_max_speed', -0.1)
        self.d_angular_speed = rospy.get_param('Lasercontrol/d_ang_speed', -0.1)  
        
        rate = 10
        self.r = rospy.Rate(rate)
        self.step = 10
        self.stopflag = False
        self.slowflag = False
        self.Kp = rospy.get_param('arbotix/controllers/base_controller/Kp', 5)
        self.Kd = rospy.get_param('arbotix/controllers/base_controller/Kd', 1)
        self.Ki = rospy.get_param('arbotix/controllers/base_controller/Ki', 0)
        self.Ko = rospy.get_param('arbotix/controllers/base_controller/Ko', 50)   
        self.MaxOutput = 1.0 #0.07                    #PID最大误差值
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 5)
        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform('/map', '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            #self.tf_listener.waitForTransform('/map', '/odom', rospy.Time(), rospy.Duration(1.0)) 
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform('/map', '/base_link', rospy.Time(), rospy.Duration(1.0))
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")
        self.con = zmq.Context()
        self.pubconnect()
        self.PublishMsg = PackageMessage()
        
        CtrlGui = Tk(className="智能车控制")
        CtrlGui.geometry(("%dx%d")%(300,250))
        CtrlGui.resizable(False, False)
        btfont = ("times", 12, "italic")
        Fbt = Button(CtrlGui, text="直行", command=self.forward)
        Fbt.config(font=btfont, bg="green", fg="black")
        Fbt.pack(side=TOP)
        Lbt = Button(CtrlGui, text="左转", command=self.turnleft)
        Lbt.config(font=btfont, bg="green", fg="black")
        Lbt.pack(side=LEFT)
        Rbt = Button(CtrlGui, text="右转", command=self.turnright)
        Rbt.config(font=btfont, bg="green", fg="black")
        Rbt.pack(side=RIGHT)
        Sbt = Button(CtrlGui, text="减速", command=self.speedslow)
        Sbt.config(font=btfont, bg="green", fg="black")
        Sbt.pack(expand=YES)
        Stbt = Button(CtrlGui, text="停车", command=self.carstop)
        Stbt.config(font = btfont, bg="green", fg="black")
        Stbt.pack(expand=YES)
        Bakt =  Button(CtrlGui, text="倒车", command=self.backward)
        Bakt.config(font = btfont, bg="green", fg="black")
        Bakt.pack(side=BOTTOM)
        CtrlGui.mainloop()
        
    def pubconnect(self):
        self.sock = self.con.socket(zmq.PUB)
        self.sock.connect(SERVICEPUBIP)
    def Pubmsg(self):
        self.PublishMsg.Clear()
        self.PublishMsg.SessionId = "%s" %uuid.uuid1()
        self.PublishMsg.Time = int(time.time() * 100000000)
        self.PublishMsg.From = SENDER
        self.PublishMsg.To = RECVER[0]
        self.PublishMsg.CallMessage.Function = ORDER[0]

    def backward(self):
        self.bwdthread = thread.start_new_thread(self.backwardbackcall, ())
    def backwardbackcall(self):
        self.stopflag = False
        self.slowflag = False
        startpose = Point()
        goalpose = Point()
        move_cmd = Twist()
        th_pid = PIDCONTRL()
        dis_pid = PIDCONTRL()
        d_startangle = 0.0
        (startpose,  startangle) = self.get_odom_map()
        d_startangle = normalize_angle(startangle - pi/2)
        goalpose.x = startpose.x + (self.step*cos(d_startangle))
        goalpose.y = startpose.y + (self.step*sin(d_startangle))
        while self.getstopflag() == False:
            if self.slowflag == True:
                if self.d_min_speed < move_cmd.linear.x:
                    move_cmd.linear.x = max(self.d_min_speed,move_cmd.linear.x - 0.02)
                else:
                    move_cmd.linear.x = self.d_min_speed
            else:
                if self.d_max_speed < move_cmd.linear.x:
                    move_cmd.linear.x = max(self.d_max_speed,move_cmd.linear.x - 0.02)
                else:
                    move_cmd.linear.x = min(self.max_speed,move_cmd.linear.x + 0.02)
            th_pid.Perror = -th_pid.Perror
            error_th = self.CacPid(dis_pid) + 2*self.CacPid(th_pid)             
            #move_cmd.angular.z = error_th    
            move_cmd.angular.z = 0.0  
            self.cmd_vel.publish(move_cmd)
            self.Pubmsg()
            self.PublishMsg.CallMessage.Parameters.append("%d" %(move_cmd.linear.x *100))
            self.PublishMsg.CallMessage.Parameters.append("%d" %(-move_cmd.angular.z *100))
            self.sock.send_multipart([AllTopic[3], "\0", self.PublishMsg.SerializeToString()])
            self. r.sleep()
            (position,rotation) = self.get_odom_map()
            th_pid.Perror =  rotation-d_startangle
            th_pid.Perror = normalize_angle(th_pid.Perror)
            dis_pid.Perror = self.cal_distance(startpose,goalpose,position.x, position.y)
    def forward(self):
        self.fwdthread = thread.start_new_thread(self.forwardbackcall, ())
            #return 0
    def forwardbackcall(self):
            self.stopflag = False
            self.slowflag = False
            startpose = Point()
            goalpose = Point()
            move_cmd = Twist()
            th_pid = PIDCONTRL()
            dis_pid = PIDCONTRL()
            (startpose,  startangle) = self.get_odom_map()
            goalpose.x = startpose.x + (self.step*cos(startangle))
            goalpose.y = startpose.y + (self.step*sin(startangle))
            while self.getstopflag() == False:
                        if self.slowflag == True:
                            if self.min_speed < move_cmd.linear.x:
                                move_cmd.linear.x = max(self.min_speed,move_cmd.linear.x - 0.02)
                            else:
                                move_cmd.linear.x = self.min_speed
                        else:
                            if self.max_speed < move_cmd.linear.x:
                                move_cmd.linear.x = max(self.max_speed,move_cmd.linear.x - 0.02)
                            else:
                                move_cmd.linear.x = min(self.max_speed,move_cmd.linear.x + 0.02)
                        th_pid.Perror = -th_pid.Perror
                        error_th = self.CacPid(dis_pid) + 2*self.CacPid(th_pid)             
                        move_cmd.angular.z = error_th
                        move_cmd.angular.z = 0.0   
                        self.cmd_vel.publish(move_cmd)
                        self.Pubmsg()
                        self.PublishMsg.CallMessage.Parameters.append("%d" %(move_cmd.linear.x *100))
                        self.PublishMsg.CallMessage.Parameters.append("%d" %(-move_cmd.angular.z *100))
                        self.sock.send_multipart([AllTopic[3], "\0", self.PublishMsg.SerializeToString()])
                        self. r.sleep()
                        (position,rotation) = self.get_odom_map()
                        th_pid.Perror =  rotation-startangle
                        th_pid.Perror = normalize_angle(th_pid.Perror)
                        dis_pid.Perror = self.cal_distance(startpose,goalpose,position.x, position.y)
    def getstopflag(self):
            return self.stopflag
    def turnleft(self):
            tlthread = thread.start_new_thread(self.turnleftcallback, ())
    def turnleftcallback(self):
            self.stopflag = False
            move_cmd = Twist()
            while self.getstopflag() == False:
                        move_cmd.angular.z = self.angular_speed
                        self.cmd_vel.publish(move_cmd)
                        self.Pubmsg()
                        self.PublishMsg.CallMessage.Parameters.append("%d" %(0))
                        self.PublishMsg.CallMessage.Parameters.append("%d" %(-move_cmd.angular.z *100))
                        self.sock.send_multipart([AllTopic[3], "\0", self.PublishMsg.SerializeToString()])
                        self.r.sleep()
                    
    def turnright(self):
        trthread = thread.start_new_thread(self.turnrightcallback, ())
    def turnrightcallback(self):
            self.stopflag = False
            move_cmd = Twist()
            while self.getstopflag() == False:
                        move_cmd.angular.z = -self.angular_speed
                        self.cmd_vel.publish(move_cmd)
                        self.Pubmsg()
                        self.PublishMsg.CallMessage.Parameters.append("%d" %(0))
                        self.PublishMsg.CallMessage.Parameters.append("%d" %(-move_cmd.angular.z *100))
                        self.sock.send_multipart([AllTopic[3], "\0", self.PublishMsg.SerializeToString()])
                        self.r.sleep()
    def speedslow(self):
            self.slowflag = True
    def carstop(self):
            self.stopflag = True
            self.cmd_vel.publish(Twist())
            self.Pubmsg()
            self.PublishMsg.CallMessage.Parameters.append("%d" %(0))
            self.PublishMsg.CallMessage.Parameters.append("%d" %(0))
            self.sock.send_multipart([AllTopic[3], "\0", self.PublishMsg.SerializeToString()])
            rospy.sleep(1)
       
    def get_odom_map(self):
        # Get the current transform between the odom and base frames
        try:
            #(trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            (trans, rot)  = self.tf_listener.lookupTransform('/map','/base_footprint', rospy.Time(0))         
            #self.tf_listener.transformPose("/odom", ps)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return None
        return (Point(*trans), quat_to_angle(Quaternion(*rot)))    
    def cal_distance(self, start, goal, xcur, ycur):
        x1 = goal.x
        y1 = goal.y
        x2 = start.x
        y2 = start.y
        a_x = x1 - x2
        a_y = y1 - y2
        if x2 == x1:
            if a_x == 0 and a_y >= 0:
                ptl = xcur - x1
            else:
                ptl = x1 - xcur
        else:
            K = (y1 - y2)/(x1 - x2)
            B = (x1*y2 - x2*y1)/(x1 - x2)
            if (a_x>=0 and a_y>=0) or (a_x>=0 and a_y<=0):
                if (-K*xcur + ycur - B) > 0:
                    ptl = -abs(-K*xcur + ycur - B)/sqrt(pow(-K,2) + 1)
                else:
                    ptl = abs(-K*xcur + ycur - B)/sqrt(pow(-K,2) + 1)
            else:
                if (-K*xcur + ycur - B) > 0:
                    ptl = abs(-K*xcur + ycur - B)/sqrt(pow(-K,2) + 1)
                else:
                    ptl = -abs(-K*xcur + ycur - B)/sqrt(pow(-K,2) + 1)
        #print "====%f" %(ptl)
        return ptl
    def CacPid(self, pt):
        Perror = pt.Perror
        output = (self.Kp*Perror + self.Kd*(Perror - pt.PrevErr) + self.Ki*pt.Ierror)/self.Ko
        pt.PrevErr = Perror    
        #output = output + pt.Poutput
        if output >= self.MaxOutput:
            output = self.MaxOutput
        elif output < -self.MaxOutput:
            output = -self.MaxOutput
        else:
            pt.Ierror = pt.Ierror + Perror
            
        pt.Poutput = output
        return pt.Poutput
if __name__ == "__main__":
        TkCtrl()
    	rospy.spin()
