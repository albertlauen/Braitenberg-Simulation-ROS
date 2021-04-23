#!/usr/bin/env python

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import sin, cos, atan2

x_bot = 0.1
y_bot = 0.0
theta = 0.0
m = 30
def newOdom (msg):
    global x_bot
    global y_bot
    global theta
    global m

    x_bot = msg.pose.pose.position.x
    y_bot = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node('speed_controller2')
sub = rospy.Subscriber('/robot2/odom/local', Odometry, newOdom)
pub = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=1)
speed = Twist()
r = rospy.Rate(10)

while not rospy.is_shutdown():
    gradient = m*(-x_bot)/100
    if -3.14 <= theta <= 3.14:
        speed.linear.x = np.clip(abs((gradient)/(theta+0.01)),-1,1)
        speed.angular.z = np.clip(((gradient)/(theta+0.01)),-1,1)
    pub.publish(speed)
    r.sleep()
    