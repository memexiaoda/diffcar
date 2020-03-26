#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Vector3,Twist
from std_msgs.msg import Bool
import zmq
#from StatMessage_pb2 import StatMessage
from PackageMessage_pb2 import PackageMessage
#import uuid
import time
#import threading
from AllParameter import SERVICEPUBIP, SERVICESUBIP, AllTopic, ORDER , SENDER,\
    RECVER
#from ConfigIni import Db_Connector

class ENCODECANCUL:
    def __init__(self):
        rospy.init_node('EncodeCancul', anonymous=False)
        self.con = zmq.Context()
        self.left_encode = (4)
        self.right_encode = (4)
        self.SpeedPub = rospy.Publisher('dxdrpub', Twist, queue_size=20)
        self.r = rospy.Rate(10)
        self.dr = 0.0
        self.dx = 0.0
        self.ddx = 0.0
        self.ddr = 0.0
        self.t_delta = rospy.Duration(1.0/10.0)
        self.t_next = rospy.Time.now() + self.t_delta
        self.then = rospy.Time.now()
        self.enc_left = None
        self.enc_right = None
        self.ticks_meter = float(rospy.get_param('/arbotix/controllers/base_controller/ticks_meter'))
        self.base_width = float(rospy.get_param('/arbotix/controllers/base_controller/base_width'))
        #self.Sessionid = ""
        
    def update(self):
        self.ZmqSubConnect()
        SubMsg = PackageMessage()
        while not rospy.is_shutdown():
            try:
                topic, re, msg = self.sub.recv_multipart()
                if (msg != None) and (msg != ""):
                    SubMsg.ParseFromString(msg)
                    if (SubMsg.To == SENDER or SubMsg.To == "*") and SubMsg.CallMessage.Function == ORDER[12]:
                        if len(SubMsg.CallMessage.Parameters) >= 2:
                            self.left_encode = long(SubMsg.CallMessage.Parameters[0])
                            self.right_encode = long(SubMsg.CallMessage.Parameters[1])
                            #print self.left_encode, "+++", SubMsg.CallMessage.Parameters[1]
                            self.EncodeCul()
                            #self.r.sleep()
                            #print rospy.Time.now()
                    else:
                        SubMsg.Clear()
                        continue
                else:
                    #print "EncodeCancul::update msg is None"
                    continue
            except Exception as err:
                print "EncodeCancul::update error: ",err
                SubMsg.Clear()
                continue
            
    def EncodeCul(self):
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()
            try:
                (left, right) = self.status()
            except Exception as e:
                print "Could not update encoders: " + str(e)
                return
            rospy.logdebug("Encoders: " + str(left) +"," + str(right))
            if self.enc_left == None:
                d_left = 0
                d_right = 0
            else:
                d_left = (left - self.enc_left)/self.ticks_meter
                d_right = (right - self.enc_right)/self.ticks_meter
            self.enc_left = left
            self.enc_right = right

            d = (d_left+d_right)/2
            th = (d_right-d_left)/self.base_width

            self.dx = d / elapsed
            self.dr = th / elapsed
            if abs(self.dx) > (abs(self.ddx)*1.5) and self.ddx != 0.0:
                #self.dx = self.dx * 0.73
                self.dx = self.dx
            self.ddx = self.dx
     
        #dxdr = Vector3()
        
        #dxdr.x = float("%.2f" %self.dx)
        #dxdr.y = 0.0
        #dxdr.z = float("%.2f" %(self.dr * 0.95))
        #dxdr.z = float("%.2f" %(self.dr * 0.785))
        dxdr = Twist()
        dxdr.linear.x = float("%.2f" %(self.dx * 0.92))
        dxdr.angular.z = float("%.2f" %(self.dr * 0.855))
        self.SpeedPub.publish(dxdr)
	#time.sleep(0.02)
    def status(self):
        try:
            left = self.left_encode
            right = self.right_encode
            #left = ord(self.left_encode[0])*255*255*255 + ord(self.left_encode[1])*255*255 + ord(self.left_encode[2])*255 + ord(self.left_encode[3])
            #right = ord(self.right_encode[0])*255*255*255 + ord(self.right_encode[1])*255*255 + ord(self.right_encode[2])*255 + ord(self.right_encode[3])
            return (left,right)
        except:
            return None
    def ZmqPubConnect(self):
        self.pub = self.con.socket(zmq.PUB)
        self.pub.connect(SERVICEPUBIP)
    def ZmqSubConnect(self):
        self.sub = self.con.socket(zmq.SUB)
        self.sub.connect(SERVICESUBIP)
        self.sub.setsockopt(zmq.SUBSCRIBE,AllTopic[4])
if __name__  ==  "__main__":
    try:
        encodecancul = ENCODECANCUL()
        encodecancul.update()
    except Exception as err:
        print "EncodeCancul exception",err 

        
