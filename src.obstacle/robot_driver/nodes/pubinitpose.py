#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Quaternion,Point, PoseStamped, PoseWithCovarianceStamped

def talker():
 #   sub = rospy.Subscribe('odom', Odometry, callback)
    pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    rospy.init_node('pubpose', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        p = PoseWithCovarianceStamped()
        p.header.stamp = rospy.Time.now()
        p.header.frame_id = 'map'
        
        p.pose.pose.position.x = 1
        p.pose.pose.position.y = 1
        p.pose.pose.position.z = 0           
        rospy.loginfo(p)
        #p.pose
        pub.publish(p)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
