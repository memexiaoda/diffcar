#!/usr/bin/env python
# -*- coding: utf-8 -*-
import zmq
from PackageMessage_pb2 import PackageMessage
from RosStatus_pb2 import RosStatus
import time
import thread
import rospy
from rosgraph_msgs.msg import Log
#byte DEBUG=1 #debug level
#byte INFO=2  #general level
#byte WARN=4  #warning level
#byte ERROR=8 #error level
#byte FATAL=16 #fatal/critical level
class status():
    def __init__(self):
        rospy.init_node('up_status', anonymous=True)
        self._message_queue = []
        self._subscriber = None
        StatusMsg = PackageMessage()
        StatusPrs = RosStatus()
        self._topic = '/rosout_agg'
        self._subscribe(self._topic)
        self.pub =self.context.socket(zmq.PUB)
        self.pub.connect("tcp://192.168.1.128")
        while True:
            StatusMsg.Clear()
            StatusPrs.Clear()
            StatusMsg.SessionId = "324354-SD-222d"
            StatusMsg.Time = int(time.time()*100000000)
            StatusMsg.From = "RosStatus"
            StatusMsg.To = "RoutePlanning"
            StatusMsg.CallMessage.function = "RosStatus"
            StatusPrs.arbotix_status = self.arbotix_status
            StatusPrs.amcl_status = self.amcl_status
            StatusPrs.laser_status = self.laser_status
            StatusPrs.ros_info = self.info
            StatusPrs.ros_warn = self.warn
            StatusPrs.ros_error = self.error
            StatusMsg.CallMessage.parameters.append(StatusPrs.SerializeToString())
            self.pub.send_multipart("ControlRpc", "\0", StatusMsg.SerializeToString()])
        rospy.spin()
    def queue_message(self, log_msg):
        """
        Callback for adding an incomming message to the queue.
        """
        msg = self.convert_rosgraph_log_message(log_msg)
        #self._message_queue.append(msg)
        
    def _subscribe(self, topic): 
        if self._subscriber:
            self._subscriber.unregister()
        self._subscriber = rospy.Subscriber(topic, Log, self.queue_message)
        self._currenttopic = topic
    def convert_rosgraph_log_message(self,log_msg):
        msg = Message()
        #msg.set_stamp_format('hh:mm:ss.ZZZ(yyyy-MM-dd)')
        msg.message = log_msg.msg
        msg.severity = log_msg.level
        msg.node = log_msg.name
        #msg.stamp = (log_msg.header.stamp.secs, log_msg.header.stamp.nsecs)
        msg.topics = sorted(log_msg.topics)
        msg.location = log_msg.file+':'+log_msg.function+':'+str(log_msg.line)
        a= 'Done initializing likelihood field model.'
        b= 'Started DiffController (base_controller). Geometry: 0.611m wide, 1000.0 ticks/m.'
        if msg.message == a :
            #print 'amcl have fireup----------------------------'
            #print 'laser have fireup---------------------------'
            self.amcl_status = 1
            self.laser_status =1
        if msg.message == b:
            #print 'arbotix have fireup-------------------------'
            self.arbotix_status = 1
        if msg.severity == 4:
            self.info = msg.message
        if msg.severity == 8:
            self.warn = msg.message
        if msg.severity == 16:
            self.error = msg.message
        return msg
class Message():
    def __init__(self):
        self.message = None
        self.severity = None
        self.node = None
        self.topics = []
        self.location = None
        
if __name__ =='__main__':
    status()

