#!/usr/bin/env python

import rospy
import spot_ros_msgs.msg
from nav_msgs.msg import Odometry
from spot_ros_msgs.msg import broadmsg
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import sin, cos, atan2

x_bot1 = 0.0
y_bot1 = 0.0
theta = 0.0

def newOdom (msg):
    global x_bot1
    global y_bot1
    global theta
    x_bot1 = msg.pose.pose.position.x
    y_bot1 = msg.pose.pose.position.y
#    rot_q = msg.pose.pose.orientation
#   (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node('speed_controller_broadcaster')
sub = rospy.Subscriber('/robot_broadcaster/odom', Odometry, newOdom)
pub1 = rospy.Publisher('/robot_broadcaster/cmd_vel', Twist, queue_size=1)
pub2 = rospy.Publisher('/broadcaster', broadmsg, queue_size=10)
speed = Twist()
r = rospy.Rate(4)

while not rospy.is_shutdown():
    #angle = atan2(y_bot,x_bot)
    x_broad = x_bot1
    y_broad = y_bot1 
    #speed.linear.x = 1.0
    #speed.angular.z = -0.5 
    pub1.publish(speed)
    pub2.publish(x_broad, y_broad)
    rospy.loginfo(x_broad)
    rospy.loginfo(y_broad)

    r.sleep()
