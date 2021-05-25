#!/usr/bin/env python3
import rospy 
from spot_ros_msgs.msg import HmiReturn

angle_f = 0.0
angle_b = False

rospy.init_node('angle_publisher', anonymous = True)
pub = rospy.Publisher('/angle_topic', HmiReturn, queue_size=5)
rate = rospy.Rate(60)	

while not rospy.is_shutdown():
    angle_f = 45.0
    angle_b = True
    pub.publish(angle_f, angle_b)
    rate.sleep()