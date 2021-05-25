#!/usr/bin/env python3

import rospy 
import cv2 
import numpy as np
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError 
from spot_ros_msgs.msg import BraitMsg

personDetected, ThermalOutOfImage, size = False, False, False
hFieldOfView = 12 # divided by 2
hResolution = 168 #divided by 2
angle = 0.0
diff = 0.0
Amax = 0.0
Amin = 0.0
Vmax = 20.0
Vmin = 0.0
centerY = 0.0
Area = 0.0
brait_angular_velocity = 0.0
brait_activated = False

rospy.init_node('thermalNode', anonymous = True)
rate = rospy.Rate(60)

video = cv2.VideoCapture(0)
while not rospy.is_shutdown():
    #global brait_angular_velocity
    ret, frame = video.read()
    if ret == True:
        width ,height = 500 , 400
        img = cv2.resize(frame,(width,height)) 
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        contours, _ = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
        if len(contours) > 0:
            brait_activated = True
        else:
            brait_activated = False
        for cnt in contours:
            (x, y, w, h) = cv2.boundingRect(cnt)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 10) 
            centerX = ((x + (x+w))/2) 
            centerY = ((y + (y+h))/2)
            Size1 = w
            Size2 = h
            #print(Size1)
            #print(Size2)
            Amax = 672*376
            Area = abs(Size1*Size2)
            #print(Area)
            break
    if Area <= (Amax/15):
    	if centerY >= 188:
    	    diff = -Area
    	else:
    	    diff = Area
    else:
    	diff = 0.0

    #print(Area)
    diff_changed = (((diff - Vmin)*(Vmax - Vmin))/(Amax - Amin + 0.01)) + Vmin
    #print(diff)
    brait_angular_velocity = round(np.clip((diff_changed),-0.5,0.5), 2)
    print(brait_angular_velocity)
    #pub = rospy.Publisher('braitenberg_topic', BraitMsg, queue_size = 10)	
    #pub.publish(brait_angular_velocity, brait_activated)
    img_rotate_90 = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    cv2.imshow('camera', img)
    cv2.waitKey(1)
    rate.sleep()
        
cv2.destroyAllWindows()
