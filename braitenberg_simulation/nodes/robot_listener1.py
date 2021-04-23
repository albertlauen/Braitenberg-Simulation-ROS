#! /usr/bin/env python
import sys
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    turtle_vel = rospy.Publisher('robot1/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
        
    while not rospy.is_shutdown():
            
        (trans,rot) = listener.lookupTransform('robot1_tf/odom_link', 'robot10_tf/odom_link', rospy.Time(0))
        angular = 4 * math.atan2(trans[1], trans[0])
        linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)
    
        rate.sleep()