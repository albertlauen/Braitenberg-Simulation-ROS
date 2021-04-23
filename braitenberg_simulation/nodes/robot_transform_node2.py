#! /usr/bin/env python

from tf import TransformBroadcaster
import rospy
from rospy import Time 

def main():
    rospy.init_node('robot2_transform')
    
    b = TransformBroadcaster()
    
    translation = (0.0, 0.0, 0.0)
    rotation = (0.0, 0.0, 0.0, 1.0)
    rate = rospy.Rate(5)  # 5hz
    
    x, y = 0.0, 0.0
    
    while not rospy.is_shutdown():

        b.sendTransform(translation, rotation, Time.now(), 'robot2_tf/odom_link', '/world')
        rate.sleep()
    


if __name__ == '__main__':
    main()