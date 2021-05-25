#!/usr/bin/env python
import rospy
import numpy as np
from audio_common_msgs.msg import AudioData
from geometry_msgs.msg import Twist
from usb_pixel_ring_v2 import PixelRing
from tuning import Tuning
import usb.core
import usb.util
import time
dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)

audio1 = 0.0
audio2 = 0.0
audio3 = 0.0
audio4 = 0.0
audio_right = 0.0
audio_left = 0.0

def Mic1(msg):
    audio1 = msg

def Mic2(msg):
    audio2 = 

def Mic3(msg):
    audio3 = 

def Mic4(msg):
    audio4 =

rospy.init_node('Microphone-channel')
sub1 = rospy.Subscriber('/audio/audio_channel2', AudioData, Mic1)
sub2 = rospy.Subscriber('/audio/audio_channel3', AudioData, Mic2)
sub3 = rospy.Subscriber('/audio/audio_channel4', AudioData, Mic2)
sub4 = rospy.Subscriber('/audio/audio_channel5', AudioData, Mic2)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
speed = Twist()
r = rospy.Rate(10)
while not rospy.is_shutdown():
    if dev:
        while True:
            try:
                audio_left = audio1 + audio2
                audio_right = audio3 + audio4
                if audio_left >= audio_right:
                speed.angular.z = 0.5
                speed.linear.x = 0.2
                else audio_right >= audio_left:
                speed.angular.z = -0.5
                speed.linear.x = 0.2
                time.sleep(1)
            except KeyboardInterrupt:
                break



