#!/usr/bin/env python
import rospy
import numpy as np
import time
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

illuminance_left = 0
illuminance_right = 0

def camera_left(msg):
    global illuminance_left
    illuminance_left = int(msg.data)
def camera_right(msg):
    global illuminance_right
    illuminance_right = int(msg.data)

rospy.init_node('Braitenberg_robot', anonymous = True)
sub1 = rospy.Subscriber('/left', Float64, camera_left, queue_size = 1)
sub2 = rospy.Subscriber('/right', Float32, camera_right, queue_size = 1)
pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
#pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(10)
speed = Twist()

while not rospy.is_shutdown():
    #Love
    diff = np.clip(illuminance_left - illuminance_right, -0.5, 0.5)
    #Fear
    #diff = np.clip(-illuminance_left + illuminance_right, -1.5, 1.5)
    speed.linear.x = 0.3
    speed.angular.z = (diff)
    print(diff)
    pub.publish(speed)
    rate.sleep()
