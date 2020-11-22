#!/usr/bin/env python
from sensor_msgs.msg import NavSatFix, Imu,Image
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Float64

from mavros_msgs.msg import OverrideRCIn
import rospy
import matplotlib.pyplot as plt
import math
import sys, random, math
from math import sqrt,cos,sin,atan2
import os
import roslib
import numpy as np
import csv
import time
import tf
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()
vx=0
vy=0
v=0

def processimage(in) :
    ##  cv_image is a cv2 image object. proceed forward with merging your code
    global bridge
    cv_image = bridge.imgmsg_to_cv2(in, "bgr8")
    return cv_bridge
    

def getVelocity(measurement):
	global vx,vy,v
	x=measurement.twist.linear.x
	y=measurement.twist.linear.y
	vx=math.sqrt(x ** 2)
	vy=math.sqrt(y ** 2)
	v = math.sqrt((vx**2) + (vy**2))

def dist(a,b):
	return math.sqrt((a[1]-b[1])**2+(a[0]-b[0])**2)
def letstrack() :
    ## add whatever you want to add
    while True:
        pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size=1000)
        control = OverrideRCIn()
        
def listener():
    rospy.init_node('listener', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	#r = rosrate(0.05)
	# rospy.Subscriber("/mavros/global_position/global", NavSatFix, plot_gps_measurements)
	#rospy.Subscriber("/mavros/global_position/global", NavSatFix, getGPS)
	rospy.Subscriber("/mavros/global_position/raw/gps_vel", TwistStamped, getVelocity)
	rospy.Subscriber("/webcam/image_raw",Image,getimage)
	letsdetect()
	rospy.spin()
	
	#rospy.Subscriber("/mavros/imu/data", Imu, getPitch)
	#rospy.Subscriber("/mavros/global_position/rel_alt", Float64, getAltitude
    
if __name__=='__main__' :
    listener()
    
