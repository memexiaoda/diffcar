#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import re
import time
import thread
import rospy
from rosgraph_msgs.msg import Log
class status():
    def __init__(self):
        rospy.init_node('up_status', anonymous=True)
        self._message_queue = []
        self._subscriber = None
        self._topic = '/rosout_agg'
        self._subscribe(self._topic)
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
            print 'amcl have fireup----------------------------'
            print 'laser have fireup---------------------------'
        if msg.message == b:
            print 'arbotix have fireup-------------------------'
        #if msg.severity==4 or msg.severity==8 or msg.severity==16:
        if msg.severity==8:
            print msg.message
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

