#!/usr/bin/env python
import rospy
import tf2_ros
import numpy as np
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from math import sin, cos, atan2, pi

x_bot = 0.1
y_bot = 0.0
theta = 0.0
m = 30
angle_set = 0.0
angle = 0.0
diff = 0.0
angle_rad = 0.0
theta_converted = 0.0

def newAngle (msg):
    global angle
    angle = msg.data
    #print(angle)

def newOdom (msg):
    global x_bot, y_bot, angle, diff, angle_rad, theta, theta_converted, m, br, t, q

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = turtlename
    t.transform.translation.x = msg.x
    t.transform.translation.y = msg.y
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

    x_bot = msg.pose.pose.position.x
    y_bot = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion ([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    #print(theta)
    angle_set = atan2(y_bot,x_bot)
    



if __name__ == '__main__':
    rospy.init_node('tf2_robot_broadcaster')
    turtlename = rospy.get_param('~turtle')
    rospy.Subscriber('/%s/pose' % turtlename, turtlesim.msg.Pose, handle_turtle_pose, turtlename)
    rospy.spin()
    rospy.init_node('speed_controller1')
    sub1 = rospy.Subscriber('/robot1/odom', Odometry, newOdom)
    sub2 = rospy.Subscriber('/broadcaster_angle', Float32, newAngle)
    pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
    speed = Twist()
    r = rospy.Rate(10)

while not rospy.is_shutdown():
    #print(angle)
    #angle_rad = angle_set*pi/180
    gradient = 1
    diff = np.clip((angle + angle_set + theta), -pi,pi)
    print(theta)
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