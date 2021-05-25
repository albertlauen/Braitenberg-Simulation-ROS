#!/usr/bin/env python
import rospy 
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
import numpy as np
import math
import time
from scipy.spatial.transform import Rotation
import datetime
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import threading
from datetime import datetime
color1 = [0,0,0]
color2 = [0,0,0]
color3 = [0,0,0]
color4 = [0,0,0]
color5 = [0,0,0]
color6 = [0,0,0]
average_color = [0,0,0]
average_color_value = 0.0
def camera2_callback(data):
    global color1, color2, color3, color4, color5, color6
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data)
    #color = image(height,width)
    color1 = (image[1, 1])
    color2 = (image[230, 1])
    color3 = (image[230, 310])
    color4 = (image[1, 310])
    color5 = (image[100, 160])
    color6 = (image[140, 160])
    cv2.imshow('camera', image)
    cv2.waitKey(1)
rospy.init_node('camera2', anonymous = True)
sub = rospy.Subscriber('/camera2/rgb/image_raw', Image, camera2_callback, queue_size = 1,  buff_size=2**24)
pub = rospy.Publisher('/left', Float64, queue_size=5)
rate = rospy.Rate(60)			
cv2.destroyAllWindows()
while not rospy.is_shutdown():
    average_color = (color1 + color2 + color3 + color4 + color5 + color6)
    average_color_value = (average_color[0] + average_color[1] + average_color[2])/3
    data = average_color_value
    print(data)
    pub.publish(data)
    rate.sleep()