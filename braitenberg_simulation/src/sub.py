#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs import Illuminance

light = 0.0

def newIlluminance(msg):
    global light
    light = msg.illiuminance

sub = rospy.Subscriber('/sub_illuminance', Illuminance, newIlluminance)

while  not rospy.is_shutdown():
    rospy.loginfo(light)
    r.sleep()