#!/usr/bin/env python

import rospy
import numpy as np
from spot_ros_msgs.msg import BraitenMsg, KinematicState
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Transform
from math import sin, cos, atan2

x_bot = 0.0
y_bot = 0.0
theta = 0.0
m = 30
def newOdom (msg):
    global x_bot
    global y_bot
    global theta
    global m
    x_bot = msg.vision_tform_body.translation.x
    y_bot = msg.vision_tform_body.translation.y
    rot_q = msg.vision_tform_body.rotation
    (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

rospy.init_node('speed_controller')
sub = rospy.Subscriber("kinematic_state", KinematicState, newOdom)
pub = rospy.Publisher("braitenberg_topic", BraitenMsg, queue_size=20)
speed = Twist()
r = rospy.Rate(10)

while not rospy.is_shutdown():
    gradient = m*(-x_bot)/100
    if -3.14 <= theta <= 3.14:
        speed.linear.x = np.clip(abs((gradient)/(theta+0.01)),-1,1)
        speed.angular.z = np.clip(((gradient)/(theta+0.01)),-1,1)
    velocity = speed.linear.x
    angle = speed.angular.z
    pub.publish(velocity, angle)
    r.sleep()
    