#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from std_msgs.msg import Bool,String,Float32,Int32
from sensor_msgs.msg import LaserScan
import time

LObstacleSector = 36
LObstacleSlow = 1.8
Amcl_Status = False

def ParamGetCallObstacleSector(data):
    global LObstacleSector
    if data.data != None:
        LObstacleSector = data.data
    #print "LObstacleSector",LObstacleSector
        
def ParamGetCallObstacleSlow(data):
    global LObstacleSlow
    if data.data != None:
        LObstacleSlow = data.data
        #print "LaserObstacle::ParamGetCallObstacleSlow LObstacleSlow",LObstacleSlow

def AmclState(data):
    global Amcl_Status
    Amcl_Status = data.data
    print"LaserObstacle::AmclState Amcl_Status： ",Amcl_Status

def scan_back(data):
    obstacle_pub = rospy.Publisher("obstacle", String, queue_size=5)
    #i = len(data.ranges)/33*16 + 5
    #j = len(data.ranges)/33 - 3
    i = int(len(data.ranges)/380.0*(190-LObstacleSector))
    j = int(len(data.ranges)/190.0*LObstacleSector)
    #print "LObstacleSector",LObstacleSector,"LObstacleSlow",LObstacleSlow
    
    dis_average = 0.0
    distance = 0.0
    count = 0
    obstacle_dis = String()
    for k in range(j):
        if data.ranges[k+i] < LObstacleSlow:
            count += 1
            distance += data.ranges[k+i]
    if count > 5:
        dis_average = distance/count
    else:
        dis_average = -0.01
    obstacle_dis = "%.2f" %dis_average
    obstacle_pub.publish(obstacle_dis)
    
 
def main():        
    rospy.init_node("Laserobstace", anonymous=False)   
    global Amcl_Status
    rospy.Subscriber("AmclState", Bool, AmclState)
    while Amcl_Status == False:
        time.sleep(1)
    print"LaserObstacle::main start working...."
    scan_sub = rospy.Subscriber("scan", LaserScan, scan_back)
    slow_sub = rospy.Subscriber("ParamPubSetLaserThresholdSlow", Float32, ParamGetCallObstacleSlow)
    sector_sub = rospy.Subscriber("ParamPubSetLaserThresholdSector", Int32, ParamGetCallObstacleSector)
    rospy.spin()
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("LaserObstacle terminated.")
