#!/usr/bin/env python

#Team Id : 3434
#Author List : Bhuvan Jhamb, Archit Chaudhary, Keshari Nath Tiwari, Parag Saroha
#File Name : task_1.2.py
#Theme : Pollinator Bee (PB)
#Functions : __init__(self),image_callback(self,msg)
#Global Variables : None


import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import	Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
import sys


import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
PY3 = sys.version_info[0] == 3

if PY3:
    xrange = range

import numpy as np
import cv2 as cv




import thread
import time


 
class WayPoint:
	
	def __init__(self):

		rospy.init_node('ros_bridge')

		self.command_r=rospy.Publisher('red', Int32, queue_size=1)
		self.command_b=rospy.Publisher('blue', Int32, queue_size=1)
		self.command_g=rospy.Publisher('green', Int32, queue_size=1)
		self.command_t=rospy.Publisher('total', Int32, queue_size=1)
		self.image_pub = rospy.Publisher('archit', Image )
		
		# Create a ROS Bridge
		self.ros_bridge = cv_bridge.CvBridge()
		self.r=0
		self.b=0
		self.g=0
		self.t=0
		self.bridge = CvBridge()
		
		# Subscribe to whycon image_out
		self.image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, self.image_callback)
		
			
		
		

	def image_callback(self,msg):
	   	# 'image' is now an opencv frame
		image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
	        img = image.copy()
		
                #blurred_frame = cv2.GaussianBlur(img, (5, 5), 0)
    		#hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		
		
 
	    	lower_red = np.array([170, 170,220])
    		upper_red = np.array([200, 200,255])
		#print "yfht"
    		mask = cv2.inRange(img, lower_red, upper_red)
 
		#ret,thresh = cv2.threshold(gray,0,255,0)
		#im2,contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)



    		_, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		
 
    		self.r=0
    		#cv2.imshow("Mask", mask)

		i=0
		while i<len(contours):		
			try:
				M = cv2.moments(contours[i])
				cxb = int(M['m10']/M['m00'])
				cyb = int(M['m01']/M['m00'])
			    	if cxb>75 and cyb> 75:
					# compute the center of the contour
					self.r=1
					cv2.rectangle(img, (cxb-15,cyb-15),(cxb+15,cyb+15) , (0,0,255),  thickness=3, lineType=8, shift=0 )

					#print self.r
					break
 
				
			except:
				ff=0
				#print "exception"
			i=i+1
		lower_blue = np.array([220, 100,100])
    		upper_blue = np.array([255, 200,200])
		#print "yfht"
    		mask = cv2.inRange(img, lower_blue, upper_blue)
 
		#ret,thresh = cv2.threshold(gray,0,255,0)
		#im2,contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)



    		_, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		
 
    		self.b=0
    		#cv2.imshow("Mask", mask)

		i=0
		while i<len(contours):		
			try:
				M = cv2.moments(contours[i])
				cxb = int(M['m10']/M['m00'])
				cyb = int(M['m01']/M['m00'])
			    	if cxb>75 and cyb> 75:
					# compute the center of the contour
					self.b=1
					cv2.rectangle(img, (cxb-15,cyb-15),(cxb+15,cyb+15) , (255,0,0),  thickness=3, lineType=8, shift=0 )

					#print self.b
					break
 
				
			except:
				ff=0
				#print "exception"
			i=i+1
		lower_green = np.array([120, 220,120])
    		upper_green = np.array([200, 255,200])
		#print "yfht"
    		mask = cv2.inRange(img, lower_green, upper_green)
 
		#ret,thresh = cv2.threshold(gray,0,255,0)
		#im2,contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)



    		_, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
		
 
    		self.g=0
    		#cv2.imshow("Mask", mask)

		i=0
		while i<len(contours):		
			try:
				M = cv2.moments(contours[i])
				cxb = int(M['m10']/M['m00'])
				cyb = int(M['m01']/M['m00'])
			    	if cxb>75 and cyb> 75:
					# compute the center of the contour
					self.g=1
					cv2.rectangle(img, (cxb-15,cyb-15),(cxb+15,cyb+15) , (0,255,0),  thickness=3, lineType=8, shift=0 )

					#	print self.g
					break
 
				
			except:
				ff=0
				#print "exception"
			i=i+1
			
		self.t=self.r+self.g+self.b
		self.command_r.publish(self.r)
		self.command_b.publish(self.b)
		self.command_g.publish(self.g)
		self.command_t.publish(self.t)
    		self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))  
		cv2.imshow("FRAME",img)		
		cv2.waitKey(1)
	   	#cv2.destroyAllWindows()
		
 
		
	        
	


		
if __name__ == '__main__':
    	
	test = WayPoint()
	rospy.spin()

		
