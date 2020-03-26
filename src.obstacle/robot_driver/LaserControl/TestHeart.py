#!/usr/bin/env python 


import rospy
import zmq
from PackageMessage_pb2 import PackageMessage
from AllParameter import SERVICEPUBIP, SERVICESUBIP, SENDER
from std_msgs.msg import String
import threading
import time
import thread
import uuid

Con = zmq.Context()
HEART = "Lived"

def PubFun():
    Pub=Con.socket(zmq.PUB)
    Pub.connect(SERVICEPUBIP)
    PubMsg = PackageMessage()
    while True:
        PubMsg.SessionId = "%s" %uuid.uuid1()
        PubMsg.Time = int(time.time()*100000000)
        PubMsg.From = SENDER
        PubMsg.To = SENDER
        PubMsg.CallMessage.Function = "TestHeart"
        Pub.send_multipart(["LaserHeart", "\0", PubMsg.SerializeToString()])
        print HEART
        if HEART == "Lived":
            time.sleep(5)
        else:
            #Pub.connect(SERVICEPUBIP)
            time.sleep(1)
def SubFun():
    time.sleep(1)
    Sub = Con.socket(zmq.SUB)
    Sub.connect(SERVICESUBIP)
    Sub.setsockopt(zmq.SUBSCRIBE,"LaserHeart")
    HeartPub = rospy.Publisher("laserheart", String, queue_size=5)
    RecvMsg = PackageMessage()
    LastTime = int(time.time())
    NowTime = int(time.time())
    global HEART
    while True:
        try:
            topic,re,msg = Sub.recv_multipart(flags=zmq.NOBLOCK)
            if msg != None:
                RecvMsg.ParseFromString(msg)
                LastTime = int(time.time())
                if RecvMsg.To == SENDER and RecvMsg.CallMessage.Function == "TestHeart":
#                     print RecvMsg
                    HEART = "Lived"
                    HeartPub.publish(HEART)
        except:
            NowTime = int(time.time())
            DelTime = NowTime - LastTime
#             print DelTime
            time.sleep(0.01)
            if DelTime > 5:
                #global HEART
                HEART = "Dead"
                HeartPub.publish(HEART)
                Sub.connect(SERVICESUBIP)
                Sub.setsockopt(zmq.SUBSCRIBE,"LaserHeart")
                time.sleep(1)
if __name__ == "__main__":
    rospy.init_node("HeartPub")
    PubThread = threading.Thread(target=PubFun, args=())
    SubThread = threading.Thread(target=SubFun, args=())
    PubThread.setDaemon(True)
    SubThread.setDaemon(True)
    PubThread.start()
    SubThread.start()
    PubThread.join()
    SubThread.join()
