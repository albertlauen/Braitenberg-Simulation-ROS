#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from math import sin, cos, atan2, pi

angle = 0.0

speed = Twist()

pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
r = rospy.Rate(10)

while rospy.is_shutdown:

    if angle ==
        speed.angular.z = 
        speed.linear.x = 0.3

