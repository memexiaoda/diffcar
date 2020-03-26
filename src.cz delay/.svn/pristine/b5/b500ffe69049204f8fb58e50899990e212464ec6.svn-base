#!/usr/bin/env python 
#-*- coding:utf-8 -*-

import rospy
from em import expand
from math import *
from geometry_msgs.msg import *
from transform_utils import quat_to_angle, normalize_angle
import threading
import zmq
from AllParameter import *
import uuid
from PackageMessage_pb2 import PackageMessage
import time
from tf_pose import TfRelation
from ControlThread  import ControlCar

class PIDCONTRL(): 
    Perror = 0.0
    PrevErr = 0.0
    Ierror = 0.0
    Poutput = 0.0

class TkCtrl():
    def __init__(self):
        #rospy.init_node("CarCtrl", anonymous=False)
        self.max_speed = rospy.get_param('LaserControl/max_speed', 0.5)                                    #最大线速度
        self.min_speed = rospy.get_param('Lasercontrol/min_speed', 0.05)                     #最小线速度
        self.angular_speed = rospy.get_param('Lasercontrol/ang_speed', 0.1)    # radians per second
        self.Kp = rospy.get_param('arbotix/controllers/base_controller/Kp', 5)
        self.Kd = rospy.get_param('arbotix/controllers/base_controller/Kd', 1)
        self.Ki = rospy.get_param('arbotix/controllers/base_controller/Ki', 0)
        self.Ko = rospy.get_param('arbotix/controllers/base_controller/Ko', 50)   
        self.MaxOutput = 1.0 #0.07                    #PID最大误差值
        
        rate = 10
        self.r = rospy.Rate(rate)
        self.step = 10
        self.stopflag = False
        self.slowflag = False
        
        # Publisher to control the robot's speed
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 5)
    
        self.tf_relation = TfRelation()  #TF relation
        
        self.con = zmq.Context()
        self.pubconnect()
        self.PublishMsg = PackageMessage()
        self.move_cmd = Twist()
        self.controlcar = ControlCar()

    def CarStatu(self,carstate):
        self.controlcar =  carstate
        
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
    
    def getstopflag(self):
        return self.stopflag  
          
    def forward(self):
        try:
            #self.fwdthread = thread.start_new_thread(self.forwardbackcall, ())
            forward_Thread = threading.Thread(target=self.forwardbackcall, args=())
            forward_Thread.setDaemon(True)
            forward_Thread.start()
            #return 
        except Exception as err:    
            print "CtrlCar::forward forward_Thread exception Error:",err
            #return
    def forwardbackcall(self):
        self.max_speed = rospy.get_param('LaserControl/max_speed', 0.5)                                    #最大线速度
        self.min_speed = rospy.get_param('Lasercontrol/min_speed', 0.05)                     #最小线速度
        print "menul TkCtrl speed:max_speed",self.max_speed,"min_speed",self.min_speed
        self.stopflag = False
        self.slowflag = False
        startpose = Point()
        goalpose = Point()
        move_cmd = Twist()
        th_pid = PIDCONTRL()
        dis_pid = PIDCONTRL()
        try:   
            (startpose,  startangle) = self.tf_relation.get_odom_map()
            goalpose.x = startpose.x + (self.step*cos(startangle))
            goalpose.y = startpose.y + (self.step*sin(startangle))
        except Exception as err:    
            print "CtrlCar::forwardbackcall  get_odom_map Error"
                
        while self.getstopflag() == False and not rospy.is_shutdown():
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
            try:
                th_pid.Perror = -th_pid.Perror
                error_th = self.CacPid(dis_pid) + 2*self.CacPid(th_pid)             
                move_cmd.angular.z = error_th
                self.move_cmd = move_cmd    
                self.cmd_vel.publish(move_cmd)
                self.Pubmsg()
                self.PublishMsg.CallMessage.Parameters.append("%d" %(move_cmd.linear.x *100))
                self.PublishMsg.CallMessage.Parameters.append("%d" %(-move_cmd.angular.z *100))
                self.sock.send_multipart([AllTopic[3], "\0", self.PublishMsg.SerializeToString()])
                self.r.sleep()
                try:
                    (position,rotation) = self.tf_relation.get_odom_map()
                except Exception as err:    
                    print "CtrlCar::forwardbackcall  get_odom_map Error Error"
                th_pid.Perror =  rotation-startangle
                th_pid.Perror = normalize_angle(th_pid.Perror)
                dis_pid.Perror = self.cal_distance(startpose,goalpose,position.x, position.y)
            except Exception as err:    
                print "CtrlCar::forwardbackcall Error"

    def turnleft(self):
        try:
            #tlthread = thread.start_new_thread(self.turnleftcallback, ())
            turnleft_Thread = threading.Thread(target=self.turnleftcallback, args=())
            turnleft_Thread.setDaemon(True)
            turnleft_Thread.start()
        except Exception as err:    
            print "CtrlCar::turnleft_Thread exception Error: ",err
            #return          
    def turnleftcallback(self):
            self.stopflag = False
            move_cmd = Twist()
            while self.getstopflag() == False and not rospy.is_shutdown():
                try:
                    move_cmd.angular.z = self.angular_speed
                    self.cmd_vel.publish(move_cmd)
                    self.Pubmsg()
                    self.PublishMsg.CallMessage.Parameters.append("%d" %(0))
                    self.PublishMsg.CallMessage.Parameters.append("%d" %(-move_cmd.angular.z *100))
                    self.sock.send_multipart([AllTopic[3], "\0", self.PublishMsg.SerializeToString()])
                    self.r.sleep()
                except Exception as err:    
                    print "CtrlCar::turnleft_callback Error"
                    
    def turnright(self):
        try:
            #trthread = thread.start_new_thread(self.turnrightcallback, ())
            turnright_Thread = threading.Thread(target=self.turnrightcallback, args=())
            turnright_Thread.setDaemon(True)
            turnright_Thread.start()
        except Exception as err:    
            print "CtrlCar::turnright_Thread exception Error: ",err
            #return            
    def turnrightcallback(self):
            self.stopflag = False
            move_cmd = Twist()
            while self.getstopflag() == False and not rospy.is_shutdown():
                try:
                    move_cmd.angular.z = -self.angular_speed
                    self.cmd_vel.publish(move_cmd)
                    self.Pubmsg()
                    self.PublishMsg.CallMessage.Parameters.append("%d" %(0))
                    self.PublishMsg.CallMessage.Parameters.append("%d" %(-move_cmd.angular.z *100))
                    self.sock.send_multipart([AllTopic[3], "\0", self.PublishMsg.SerializeToString()])
                    self.r.sleep()
                except Exception as err:    
                    print "CtrlCar::turnrightcallback exception Error"
    def speedslow_manual(self):
        self.slowflag = True
            
    def carstop_manual(self):
        print "CtrlCar::carstop"
        #while self.controlcar.ControlStatu != "end":
            #self.controlcar.Token = "Laser_manual"
            #self.r.sleep()
        if self.move_cmd.linear.x > self.min_speed:
            #move_cmd.linear.x = max(self.min_speed,move_cmd.linear.x - 0.02)
            self.slowflag = True
        else:
            try:
                self.stopflag = True
                self.cmd_vel.publish(Twist())
                self.Pubmsg()
                self.PublishMsg.CallMessage.Parameters.append("%d" %(0))
                self.PublishMsg.CallMessage.Parameters.append("%d" %(0))
                self.sock.send_multipart([AllTopic[3], "\0", self.PublishMsg.SerializeToString()])
                rospy.sleep(1)
            except Exception as err:    
                print "CtrlCar::carstop  carstop error"
     
    def cal_distance(self, start, goal, xcur, ycur):
        x1 = goal.x
        y1 = goal.y
        x2 = start.x
        y2 = start.y
        a_x = x1 - x2
        a_y = y1 - y2
        if a_x == 0:
            if a_y >= 0:
                ptl = xcur - x1
            else:
                ptl = x1 - xcur
        else:
            K = (y1 - y2)/(x1 - x2)
            B = (x1*y2 - x2*y1)/(x1 - x2)
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
# if __name__ == "__main__":
#         TkCtrl()
    
