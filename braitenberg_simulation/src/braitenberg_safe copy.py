#!/usr/bin/env python
import rospy
import numpy as np
#from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from math import sin, cos, atan2, pi

x_bot = 0.1
y_bot = 0.0
theta = 0.0
m = 30
angle = 270.0
diff = 0.0
angle_rad = 0.0
theta_converted = 0.0


def newOdom (msg):
    global x_bot, y_bot, angle, diff, angle_rad, theta, theta_converted, m
    x_bot = msg.pose.pose.orientation.x
    y_bot = msg.pose.pose.orientation.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    angle_rad = angle*pi/180
    diff = np.clip((angle_rad - theta), -pi,pi)
    #print(theta)
    
rospy.init_node('speed_controller1')
sub1 = rospy.Subscriber('/odom', Odometry, newOdom)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
speed = Twist()
r = rospy.Rate(10)

while not rospy.is_shutdown():
    gradient = 1
    print(diff)
    if diff > theta:
        speed.angular.z = 0.5
        speed.linear.x = 0.1
    elif diff <= theta:
        speed.angular.z = -0.5
        speed.linear.x = 0.1
    else:
        speed.linear.x = 0.0
        speed.angular.z = 0.0
    #gradient = m*(x_bot)/100    
    #if diff >= pi:
    #    speed.linear.x = np.clip(abs((gradient)/(diff+0.01)),-1,1)
    #    speed.angular.z = -np.clip(((gradient)/(diff+0.01)),-1,1)
        #print('Left')
    #else:
    #    speed.linear.x = np.clip(abs((gradient)/(diff+0.01)),-1,1)
    #    speed.angular.z = np.clip(((gradient)/(diff+0.01)),-1,1)    
        #print('Right')
    #print(theta)
    #print(diff)
    pub.publish(speed)
    r.sleep()