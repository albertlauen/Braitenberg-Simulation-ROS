#!/usr/bin/env python

from numpy.core.numeric import _move_axis_to_0
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from spot_ros_msgs.msg import BraitMsg
from math import sin, cos, atan2

angular_velocity = 0.0
braitenberg_activated = False

def newOdom (msg):
    global angular_velocity, braitenberg_activated
    angular_velocity = msg.brait_angular_velocity
    braitenberg_activated = msg.brait_activated

rospy.init_node('speed_controller1')
sub = rospy.Subscriber('/braitenberg_topic', BraitMsg, newOdom)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
speed = Twist()
r = rospy.Rate(10)

while not rospy.is_shutdown():
    if braitenberg_activated == True:
        speed.angular.z = angular_velocity
        speed.linear.x = 0.5
    else:
        speed.angular.z = 0.0
        speed.linear.x = 0.0

    pub.publish(speed)
    r.sleep()