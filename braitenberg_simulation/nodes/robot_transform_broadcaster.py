#! /usr/bin/env python
import rospy
from rospy import Time 
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sin, cos, atan2
from tf import TransformBroadcaster

x_bot = 3
y_bot = 2
theta = 0.0

def main():
    global x_bot
    global y_bot
    global theta

    rospy.init_node('robot10_transform')
    b = TransformBroadcaster()
    translation = (0.0, 0.0, 0.0)
    rotation = (0.0, 0.0, 0.0, 0.0)
    rate = rospy.Rate(5)  # 5hz
    #pub = rospy.Publisher('/broadcaster_angle', Float32, queue_size=1)

    while not rospy.is_shutdown():
    #    angle = atan2(y_bot,x_bot)
    
    #    data = angle
        b.sendTransform(translation, rotation, Time.now(), '/robot10_tf/odom_link', '/world')
    #    pub.publish(data)
    #    rospy.loginfo(data)
        rate.sleep()
    
if __name__ == '__main__':
    main()



