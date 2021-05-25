#!/usr/bin/env python3
import rospy 
from sensor_msgs.msg import Image as Img
import PIL
from PIL import Image
import requests
from io import BytesIO
from PIL import ImageFilter
from PIL import ImageEnhance
from IPython.display import display
import numpy as np
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
from std_msgs.msg import Float32
import threading
from datetime import datetime
R = 0
G = 0
B = 0
rgb = [0,0,0]
pix = 0
average_color = [0,0,0]
average_color_value = 0.0
def camera1_callback(data):
    global rgb, pix, average_color_value, R, G, B
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data)
    for x in range(0,image.shape[0]):
        for y in range(0,image.shape[1]):
            rgb = image[x,y]
            R += rgb[0]
            G += rgb[1]
            B += rgb[2]
    pix = image.size
    average_color_value = (R + G + B)/pix
    rgb = [0,0,0]
    R = 0
    G = 0 
    B = 0
    rgb = [0,0,0]
    cv2.imshow('camera', image)
    cv2.waitKey(1)

rospy.init_node('camera1', anonymous = True)
sub = rospy.Subscriber('/camera1/rgb/image_raw', Img, camera1_callback, queue_size = 1,  buff_size=2**24)
pub = rospy.Publisher('/right', Float32, queue_size=5)
rate = rospy.Rate(60)			
cv2.destroyAllWindows()

while not rospy.is_shutdown():
    data = average_color_value
    print(data)
    pub.publish(data)
    rate.sleep()
