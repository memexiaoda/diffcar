#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import zmq
from PackageMessage_pb2 import PackageMessage
from StatMessage_pb2 import StatMessage
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Point
import time
from AllParameter import SERVICEPUBIP, SENDER, RECVER, ORDER, AllTopic
import uuid
from AreaInformation import AreaInfor
#import threading

class LASERSTATUSPUB():
    
    def __init__(self):
        rospy.init_node("LaserStatus", anonymous=False) # LaserStatus
        self.AmclStatus = 0
        rospy.Subscriber("AmclState", Bool, self.AmclState)
        while self.AmclStatus == 0:
            time.sleep(1)
        print"LaserStatusPub:: main start working...."
        
        self.ErrStatus = 0.0
        self.RunStatus = 0
        self.Token = ""
        self.Battery_state = 0
        
        self.StatuAreaInfo = {}
        #self.position = Point()
        self.con = zmq.Context()
        StatusMsg = PackageMessage()
        StatusParams = StatMessage()
        self.Area_Information = AreaInfor()   
        
     #   rospy.Subscriber("RobotPose", Point, self.CurPose)
        rospy.Subscriber("Battery_state", String, self.CurBattery)
        rospy.Subscriber("ErrDistance", String, self.ErrorState)
     #   rospy.Subscriber("AmclState", Bool, self.AmclState)
        rospy.Subscriber("NodeState", String, self.NodeState)
        rospy.Subscriber("TokenPose", String, self.TokenPose)
        self.pub = self.con.socket(zmq.PUB)
        self.pub.connect(SERVICEPUBIP)

#        AreaThread = threading.Thread(target=self.Area_Information.Plan_line, args=())
#        AreaThread.setDaemon(True)
#        AreaThread.start()
#        print "LaserStatusPub:: The Area_Information.Plan_line AreaThread Done"
        
        while not rospy.is_shutdown():
            #time.sleep(5)
            time.sleep(1)
            StatusMsg.Clear()
            StatusParams.Clear()
            StatusMsg.SessionId = "%s" %uuid.uuid1()
	        #StatusMsg.Token = self.Token
            StatusMsg.Time = int(time.time()*100000000)
            StatusMsg.From = SENDER
            StatusMsg.To = RECVER[1]
            StatusMsg.CallMessage.Function = ORDER[7]
            
            try:
                self.StatuAreaInfo = self.Area_Information.Area_line()
                print "LaserStatusPub:: The returned value: self.StatuAreaInfo is ",self.StatuAreaInfo.get('Fname') ,self.StatuAreaInfo.get('Fnameend')
                #self.position = self.Area_Information.PubPos()
                #print "LaserStatusPub:: The returned value: self.position",self.position
                #if (not self.StatuAreaInfo) or (not self.position):
                if (not self.StatuAreaInfo):
                    print"LaserStatusPub:: area and pose are None "                
                    continue
                    
            except Exception as err:
                print"LaserStatusPub:: get area and pose value error :",err
                continue
            
            if self.AmclStatus == 1:
                
                try:
                    StatusParams.Stat = 0
                    StatusParams.LaserMessage.LocateStat = self.AmclStatus
                    StatusParams.LaserMessage.Token = self.Token
                    StatusParams.LaserMessage.MobileStat = self.RunStatus
                    StatusParams.LaserMessage.CrosswiseError = self.ErrStatus
                    #print"LaserStatusPub::while the self.Battery_state: ",self.Battery_state
                    StatusParams.LaserMessage.Charging = self.Battery_state                    
#                    StatusParams.LaserMessage.CurPose_x = self.position.x
#                    StatusParams.LaserMessage.CurPose_y = self.position.y                    
#                    StatusParams.LaserMessage.CurPose_z = self.position.z
                    
                    StatusParams.LaserMessage.CurPose_x = self.StatuAreaInfo.get('Curx')
                    StatusParams.LaserMessage.CurPose_y = self.StatuAreaInfo.get('Cury')
                    StatusParams.LaserMessage.CurPose_th = self.StatuAreaInfo.get('CurTh')        
                    StatusParams.LaserMessage.CurLine = self.StatuAreaInfo.get('CurLine')
                    StatusParams.LaserMessage.CurLinePass = self.StatuAreaInfo.get('CurLinePass')
                    StatusParams.LaserMessage.CurLineDone = self.StatuAreaInfo.get('CurLineDone')
                    StatusParams.LaserMessage.Curx = self.StatuAreaInfo.get('Curx')
                    StatusParams.LaserMessage.Cury = self.StatuAreaInfo.get('Cury')
                    StatusParams.LaserMessage.Fname = self.StatuAreaInfo.get('Fname')
                    StatusParams.LaserMessage.Fnameend = self.StatuAreaInfo.get('Fnameend')
                    StatusParams.LaserMessage.PosID = self.StatuAreaInfo.get('PosID')
                
                except Exception as err:
                    print "LaserStatusPub exception the evaluation error and continue ... "
                    continue
                
                StatusMsg.CallMessage.Parameters.append(StatusParams.SerializeToString())
                
                try:
                    self.pub.send_multipart([AllTopic[0], "\0", StatusMsg.SerializeToString()])
                    #print "LaserStatusPub::StatusParams pub_status: ", StatusParams
                except Exception as err:
                    print "LaserStatusPub exception the pub.send_multipart Error: ",err
                    continue
            time.sleep(4)
        #rospy.spin()
             
    def ErrorState(self, data):
        if (data.data != None) and (data.data != ""):
            Index = data.data.find(":")
            self.ErrStatus = float(data.data[:Index])
            if data.data[Index+1:] == "fward":
                self.RunStatus = 1
            else:
                self.RunStatus = 0
                
    def AmclState(self, data):
        if data.data:
            self.AmclStatus = 1
        else:
            self.AmclStatus = 0
        print"LaserStatusPub::AmclState AmclStatus: ",self.AmclStatus 
        
    def NodeState(self):
        return
    
    def TokenPose(self,data):
        if (data.data != None) and (data.data != ""):
            self.Token = data.data       

    def CurBattery(self,data):
        #print"LaserStatusPub::CurBattery the data.data:",data.data
        if (data.data == "charging"):
            self.Battery_state = 1 
        else:
            self.Battery_state = 0
        print"LaserStatusPub::CurBattery the self.Battery_state: ",self.Battery_state
 
if __name__ == "__main__":
    try:
        LASERSTATUSPUB()  
    except Exception as err:
        print "LaserStatusPub __main__ exception",err           
            
