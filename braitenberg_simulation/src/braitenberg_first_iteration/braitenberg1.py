#!/usr/bin/env python
import rospy
import numpy as np
from rospy import Time
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, TransformStamped
from math import sin, cos, atan2, pi

#from tf import TransformBroadcaster


x_bot = 0.1
y_bot = 0.0
theta = 0.0
m = 30
angle_set = 0.0
angle = 0.0
diff = 0.0
angle_rad = 0.0
theta_converted = 0.0
translation = (0.0, 0.0, 0.0)
rotation = (0.0, 0.0, 0.0, 0.0)

def newAngle (msg):
    global angle
    angle = msg.data
    #print(angle)

def newOdom (msg):
    global x_bot, y_bot, angle, diff, angle_rad, theta, theta_converted, m

    rot_q = msg.transform.rotation
    (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #print(theta)
    angle_set = atan2(y_bot,x_bot)
    print(rot_q)
    
rospy.init_node('speed_controller1')
sub1 = rospy.Subscriber('/tf/transforms', tfMessage, newOdom)
sub2 = rospy.Subscriber('/broadcaster_angle', Float32, newAngle)
pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
speed = Twist()
t = TransformStamped()
r = rospy.Rate(10)

while not rospy.is_shutdown():
    #angle_rad = angle_set*pi/180
    gradient = 1
    diff = np.clip((angle + angle_set + theta), -pi,pi)
    #print(theta)
    #b.sendTransform(translation, rotation, Time.now(), '/robot1', '/world')
    #gradient = m*(x_bot)/100    
    if diff >= pi:
        speed.linear.x = np.clip(abs((gradient)/(diff+0.01)),-1,1)
        speed.angular.z = -np.clip(((gradient)/(diff+0.01)),-1,1)
        #print('Left')
    else:
        speed.linear.x = np.clip(abs((gradient)/(diff+0.01)),-1,1)
        speed.angular.z = np.clip(((gradient)/(diff+0.01)),-1,1)    
        #print('Right')
    #print(theta)
    #print(diff)
    pub.publish(speed)
    r.sleep()