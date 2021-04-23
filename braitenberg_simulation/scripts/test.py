#! /usr/bin/env python

from tf import TransformBroadcaster
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Vector3, Quaternion
from rospy import Time 

rospy.init_node('robot1_transform')
    
b = TransformBroadcaster()
translation = [0.0,0.0,0.0]
rotation = [0.0,0.0,0.0,0.0]
def newOdom (msg):
#Odometry recieved from robots position, based on the world frame
    global translation, rotation
    translation.x = msg.pose.pose.position.x
    translation.y = msg.pose.pose.position.y
    translation.z = msg.pose.pose.position.z
    rotation = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion ([rotation.x, rotation.y, rotation.z, rotation.w])
    
sub1 = rospy.Subscriber('/robot1/odom', Odometry, newOdom)
pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
speed = Twist()
rate = rospy.Rate(5) 

while not rospy.is_shutdown():
    print(translation)
    print(rotation)
    
    speed.linear.x = 0.5
    speed.angular.z = 0.5
    pub.publish(speed)
    rate.sleep()
    

