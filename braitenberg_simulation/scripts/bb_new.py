#!/usr/bin/env python
import rospy
import numpy as np
import spot_ros_msgs.msg
from rospy import Time
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from spot_ros_msgs.msg import broadmsg
from geometry_msgs.msg import Twist
from math import sin, acos, atan2, pi, sqrt
from scipy.spatial.transform import Rotation as Rot

#Variables
x_bot = 1.0
y_bot = 0.0
x_broad = 1.0
y_broad = 0.0
theta = 0.0
diff = 0.0
angle_recieved = 0.0
angle_bot = 0.0
angle1_converted = 0.0
theta_converted = 0.0
angle1_2pi = 0.0
theta_2pi = 0.0
point1 = 0.0
point2 = 0.0
vector_x = 0.0
vector_y = 0.0
vector_1 = [0, 0]
vector_2 = [0, 0]
rotation = [0, 0, 0]
#Classes
speed = Twist()
rot = 0 
#Initiate nodes
rospy.init_node('robot_controller1')

def newOdom (msg):
    #Odometry recieved from robots position, based on the world frame
    global x_bot, y_bot, angle_bot, diff, theta, m, Rot, rotation, rot, theta_converted, angle1_converted
    x_bot = msg.pose.pose.position.x
    y_bot = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
def newAngle (msg):
    #Angle recieved from broadcasted robot
    global x_broad, y_broad
    x_broad = msg.x_broad
    y_broad = msg.y_broad
    #rospy.loginfo(x_broad)
    #rospy.loginfo(y_broad)

#Subscribers and Publishers
sub1 = rospy.Subscriber('/robot1/odom', Odometry, newOdom)
sub2 = rospy.Subscriber('/broadcaster', broadmsg, newAngle)
pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=5)
#Rate in Hz
r = rospy.Rate(10)
while not rospy.is_shutdown():
    vector_x = x_broad - x_bot
    vector_y = y_broad - y_bot
    #angle1 = np.arccos(dot_product)
    #angle1 = acos((y_broad*y_bot)/(sqrt(x_bot**2+y_bot**2) + sqrt(x_broad**2+y_broad**2)))
    angle1 = atan2(vector_y - 0, vector_x - 1)
    
    angle1_2pi = np.mod(angle1, 2*np.pi)
    theta_2pi = np.mod(theta, 2*np.pi)
    #angle2 = atan2(y_broad, x_broad)
    angle1_converted = (angle1_2pi/(2*pi))*360
    theta_converted = (theta_2pi/(2*pi))*360
    diff = abs(theta_converted - angle1_converted) - 180
    #if 20 >= diff:
    #    speed.angular.z = 0.5
    #    speed.linear.x = 0.2
    #elif 40 <= diff:
    #    speed.angular.z = -0.5 
    #    speed.linear.x = 0.2
    speed.angular.z = 0.5
    #speed.linear.x = 0.5
    #rospy.loginfo(angle1)
    #print('angle1')
    #rospy.loginfo(angle2)
    #print('angle2')
    #rospy.loginfo(rotation[2])
    #rospy.loginfo(diff)
    rospy.loginfo(angle1_converted)
    #rospy.loginfo('theta')
    #rospy.loginfo(theta_converted)
    #rospy.loginfo(diff)
    #Broadcast transforms (odom_link, base_link, chassis_link)
    #Code driving the robot
    pub.publish(speed)
    r.sleep()



