#!/usr/bin/env python

#The required packages are imported here
from plutodrone.msg import *
from pid_tune.msg import *
from std_msgs.msg import Int32
from std_msgs.msg import Float64
import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import	Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
import time
import sys
PY3 = sys.version_info[0] == 3

if PY3:
    xrange = range

import numpy as np
import cv2 as cv
iktala=0
def angle_cos(p0, p1, p2):
   		d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
   	 	return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )
	

class DroneFly():
	"""docstring for DroneFly"""
	
        def __init__(self):
		#rospy.init_node('ros_bridge')

		self.command_r=rospy.Publisher('red', Int32, queue_size=1)
		self.command_b=rospy.Publisher('blue', Int32, queue_size=1)
		self.command_g=rospy.Publisher('green', Int32, queue_size=1)
		# Create a ROS Bridge
		self.ros_bridge = cv_bridge.CvBridge()
		self.r=0
		self.b=0
		self.g=0
		

		# Subscribe to whycon image_out
		self.image_sub = rospy.Subscriber('visionSensor/image_rect', Image, self.image_callback)
		
		
		
		rospy.init_node('pluto_fly', disable_signals = True)

		self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=10)
		
		self.command_z=rospy.Publisher('alt_error', Float64, queue_size=1)
		self.command_x=rospy.Publisher('pitch_error', Float64, queue_size=1)
		self.command_yaw=rospy.Publisher('yaw_error', Float64, queue_size=1)
		self.command_y=rospy.Publisher('roll_error', Float64, queue_size=1)
		self.command_zero_line=rospy.Publisher('line', Float64, queue_size=1)
		self.command_one_line=rospy.Publisher('line1', Float64, queue_size=1)
		self.command_minus_line=rospy.Publisher('line_1', Float64, queue_size=1)
		self.command_iroll=rospy.Publisher('roll', Float64, queue_size=1)
		self.command_ipitch=rospy.Publisher('pitch', Float64, queue_size=1)
		self.command_ithrot=rospy.Publisher('alt', Float64, queue_size=1)

		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)
		rospy.Subscriber('drone_yaw', Float64, self.get_yaw)

		# To tune the drone during runtime
		rospy.Subscriber('/pid_tuning_altitude', PidTune, self.set_pid_alt)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.set_pid_roll)
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.set_pid_pitch)
		rospy.Subscriber('/pid_tuning_yaw', PidTune, self.set_pid_yaw)
		
		self.cmd = PlutoMsg()

		# Position to hold.
		self.wp_x = -5.63
		self.wp_y = -5.63
		self.wp_z = 30.0
		self.wp_yaw = -0.0403664580154


		self.zero_line=0.0
		
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1000
		self.cmd.plutoIndex = 0

		self.drone_x = 0.0
		self.drone_y = 0.0
		self.drone_z = 0.0
		self.yaw_z = 0.0

		#PID constants for Roll
		self.kp_roll = 6.0
		self.ki_roll = 0.10
		self.kd_roll = 5.0

		#PID constants for Pitch
		self.kp_pitch = 6.0
		self.ki_pitch = 0.0
		self.kd_pitch = 5.0
		
		#PID constants for Yaw
		self.kp_yaw = 4188.0
		self.ki_yaw = 85.0
		self.kd_yaw = 3665.0

		#PID constants for Throttle
		self.kp_throt = 1485.0
		self.ki_throt = 0.0
		self.kd_throt = 1058.0

		# Correction values after PID is computed
		self.correct_roll = 0.0
		self.correct_pitch = 0.0
		self.correct_yaw = 0.0
		self.correct_throt = 0.0

		# Loop time for PID computation. You are free to experiment with this
		self.seconds=time.time()
		self.last_time = self.seconds
		self.loop_time = 0.032
	
		self.iroll=0.0
		self.droll=0.0
		self.lastroll=0.0

		self.ipitch=0.0
		self.dpitch=0.0
		self.lastpitch=0.0

		self.ithrot=0.0
		self.dthrot=0.0
		self.lastthrot=0.0

		self.iyaw=0.0
		self.dyaw=0.0
		self.lastyaw=0.0



		self.z=0.0
		self.y=0.0
		self.x=0.0
		self.yawn=0.0
		
                print(self.ithrot)
		rospy.sleep(.1)
	
	def init(self):
		self.ithrot=0.0
		self.dthrot=0.0
		self.lastthrot=0.0
	


	def arm(self):
		self.cmd.rcAUX4 = 1500
		self.cmd.rcThrottle = 1000
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)

	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)


	def position_hold(self):

		rospy.sleep(2)

		print "disarm"
		self.disarm()
		rospy.sleep(.2)
		print "arm"
		self.arm()
		rospy.sleep(.1)
                self.iroll = 0.0
                self.ithrot = 0.0
                self.ipitch = 0.0
		j=0

		while True:
			self.command_r.publish(self.r)
			self.command_b.publish(self.b)
			self.command_g.publish(self.g)
		
			
			self.calc_pid()

			# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on


			if j==0 :
				if self.drone_x < -5.3 and self.drone_x > -5.9 and self.drone_y < -5.3 and self.drone_y > -5.9  and self.wp_x == -5.63 and self.wp_y == -5.63 and self.x<0.15 and self.x>-0.15 and self.y>-0.15 and self.y<0.15 :
					self.wp_x=5.57
					self.wp_y=-5.63
					j=1
			if j==1 :
				if self.drone_x > +5.3 and self.drone_x < +5.9 and self.drone_y < -5.3 and self.drone_y > -5.9 and self.wp_x == 5.57 and self.wp_y == -5.63 and self.x<0.15 and self.x>-0.15 and self.y>-0.15 and self.y<0.15 :
					self.wp_x=5.55
					self.wp_y=5.54
					j=2
		
			if j==2 :
				if self.drone_x > +5.3 and self.drone_x < +5.9 and self.drone_y > 5.3 and self.drone_y < 5.9 and self.wp_x == 5.55 and self.wp_y == 5.54 and self.x<0.15 and self.x>-0.15 and self.y>-0.15 and self.y<0.15 :
					self.wp_x=-5.6
					self.wp_y=5.54
					j=3
		
			if j==3 :
				if self.drone_x < -5.3 and self.drone_x > -5.9 and self.drone_y > 5.3 and self.drone_y < 5.9 and self.wp_x == -5.6 and self.wp_y == 5.54 and self.x<0.15 and self.x>-0.15 and self.y>-0.15 and self.y<0.15 :
					self.wp_x=0
					self.wp_y=0
					j=4
			if j==4 :
				if self.drone_x > -0.3 and self.drone_x < 0.3 and self.drone_y > -0.3 and self.drone_y < 0.3 and self.wp_x == 0.0 and self.wp_y == 0.0 and self.x<0.15 and self.x>-0.15 and self.y>-0.15 and self.y<0.15 :
					self.wp_x=0
					self.wp_y=0
					self.wp_z=41
					j=5
			if j==5 :
				if self.drone_x > -0.3 and self.drone_x < 0.3 and self.drone_y > -0.3 and self.drone_y < 0.3 and self.wp_x == 0.0 and self.wp_y == 0.0 and self.x<0.15 and self.x>-0.15 and self.y>-0.15 and self.y<0.15 and self.drone_z > 40 :
					
					self.wp_x=0
					self.wp_y=0
					self.wp_z=42
					self.cmd.rcThrottle = 1300
					time.sleep(3)
					self.disarm()
					j=6
					exit()




		 	
			pitch_value = int(1500 - self.correct_pitch)
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
															
			roll_value = int(1500 - self.correct_roll)
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
			
			yaw_value = int(1500 - self.correct_yaw)
			self.cmd.rcYaw = self.limit (yaw_value, 1530, 1470)
			
			throt_value = int(1500 - self.correct_throt)
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)
															
			self.pluto_cmd.publish(self.cmd)
			self.command_z.publish(self.z)
			self.command_yaw.publish(self.yawn)
			self.command_x.publish(self.x)
			self.command_y.publish(self.y)
			self.command_zero_line.publish(self.zero_line)
			self.command_one_line.publish(-2.5)
			self.command_minus_line.publish(-1.5)
						
			self.command_iroll.publish(self.iroll)
			self.command_ipitch.publish(self.ipitch)
			self.command_ithrot.publish(self.ithrot)
			
	def calc_pid(self):
		self.seconds = time.time()
		current_time = self.seconds - self.last_time
		if(current_time >= self.loop_time):
                        
			self.pid_roll()
			self.pid_pitch()
			self.pid_throt()
			self.pid_yaw()
			
			self.last_time = self.seconds


	def pid_roll(self):
		dt=0.0320298671722
		self.y=self.wp_y-self.drone_y
		#self.iroll=0.0
		self.iroll = self.iroll + self.y*dt
        	self.droll=(self.y-self.lastroll)
		self.lastroll=self.y
        	self.correct_roll=self.kp_roll*self.y+self.kd_roll*self.droll+(self.ki_roll*self.iroll/100)

	def pid_pitch(self):

		dt=0.0320298671722
		self.x=self.wp_x-self.drone_x
		#self.ipitch=0.0		
		self.ipitch = self.ipitch + self.x*dt
        	self.dpitch=(self.x-self.lastpitch)
		self.lastpitch=self.x
        	self.correct_pitch=self.kp_pitch*self.x+self.kd_pitch*self.dpitch+(self.ki_pitch*self.ipitch/100)

	def pid_throt(self):

		dt=0.0320298671722
		self.z=self.wp_z-self.drone_z		
		self.ithrot = (self.ithrot) + (self.z*dt)
         	self.dthrot=(self.z-self.lastthrot)/dt
		self.lastthrot=self.z
        	self.correct_throt=(self.kp_throt*self.z+self.kd_throt*self.dthrot+(self.ki_throt*self.ithrot/100))/100

	def pid_yaw(self):
		dt=0.0320298671722
		self.yawn=self.wp_yaw+self.yaw_z
		#self.iyaw=0.0
		self.iyaw = self.iyaw + self.yawn*dt
        	self.dyaw=(self.yawn-self.lastyaw)/dt
		self.lastyaw=self.yawn
        	self.correct_yaw=(self.kp_yaw*self.yawn+self.kd_yaw*self.dyaw+(self.ki_yaw*self.iyaw/100))/100
		print(self.correct_yaw)

		
	def limit(self, input_value, max_value, min_value):

		#Use this function to limit the maximum and minimum values you send to your drone
		if input_value > max_value:
			return max_value
		if input_value < min_value:
			return min_value
		else:
			return input_value
			

	#You can use this function to publish different information for your plots
	# def publish_plot_data(self):


	def set_pid_alt(self,pid_val):
		
		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Altitude
		
		self.kp_throt = pid_val.Kp
		self.ki_throt = pid_val.Ki
		self.kd_throt = pid_val.Kd

	def set_pid_roll(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Roll

		self.kp_roll = pid_val.Kp
		self.ki_roll = pid_val.Ki
		self.kd_roll = pid_val.Kd
		
	def set_pid_pitch(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Pitch

		self.kp_pitch = pid_val.Kp
		self.ki_pitch = pid_val.Ki
		self.kd_pitch = pid_val.Kd
		
	def set_pid_yaw(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Yaw

		self.kp_yaw = pid_val.Kp
		self.ki_yaw = pid_val.Ki
		self.kd_yaw = pid_val.Kd
		
	def get_pose(self,pose):

		#This is the subscriber function to get the whycon poses
		#The x, y and z values are stored within the drone_x, drone_y and the drone_z variables
		
		self.drone_x = pose.poses[0].position.x
		self.drone_y = pose.poses[0].position.y
		self.drone_z = pose.poses[0].position.z
	
	def get_yaw(self,yaw):
		
		self.yaw_z = yaw.data
	
	def image_callback(self,msg):
           global iktala
           if iktala==0:
		iktala=iktala+1
		# 'image' is now an opencv frame
		# You can run opencv operations on 'image'
		image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


		countg=0
		countb=0
		countr=0
		squares = []
		
		img=self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		i=0
	        j=0
   	        while (i < img.shape[0]):     
    			j=0
			while ( j < img.shape[1]):
				if img[i,j,0] > 5 or img[i,j,2] > 5:   
					img[i,j,2]=0
					img[i,j,0]=0
					img[i,j,1]=0
			
				j=j+1
			i = i + 1
		img = cv.GaussianBlur(img, (5, 5), 0)
    		     		
    		
    		for gray in cv.split(img):
        		for thrs in xrange(0, 255, 26):
            			if thrs == 0:
                			bin = cv.Canny(gray, 0, 50, apertureSize=5)
                			bin = cv.dilate(bin, None)
            			else:
                			_retval, bin = cv.threshold(gray, thrs, 255, cv.THRESH_BINARY)
            			bin, contours, _hierarchy = cv.findContours(bin, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
            			for cnt in contours:
           	 	    		cnt_len = cv.arcLength(cnt, True)
                			cnt = cv.approxPolyDP(cnt, 0.02*cnt_len, True)
                			if len(cnt) == 4 and cv.contourArea(cnt) > 100 and cv.isContourConvex(cnt):
                    				cnt = cnt.reshape(-1, 2)
                    				max_cos = np.max([angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)])
                    				if max_cos < 0.1:
                        				squares.append(cnt)
							countg=countg+1

		img=self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		i=0
	        j=0
   	        while (i < img.shape[0]):     
    			j=0
			while ( j < img.shape[1]):
				if img[i,j,1] > 5 or img[i,j,2] > 5:   
					img[i,j,2]=0
					img[i,j,0]=0
					img[i,j,1]=0
			
				j=j+1
			i = i + 1
		img = cv.GaussianBlur(img, (5, 5), 0)
    		     		
    		
    		for gray in cv.split(img):
        		for thrs in xrange(0, 255, 26):
            			if thrs == 0:
                			bin = cv.Canny(gray, 0, 50, apertureSize=5)
                			bin = cv.dilate(bin, None)
            			else:
                			_retval, bin = cv.threshold(gray, thrs, 255, cv.THRESH_BINARY)
            			bin, contours, _hierarchy = cv.findContours(bin, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
            			for cnt in contours:
           	 	    		cnt_len = cv.arcLength(cnt, True)
                			cnt = cv.approxPolyDP(cnt, 0.02*cnt_len, True)
                			if len(cnt) == 4 and cv.contourArea(cnt) > 100 and cv.isContourConvex(cnt):
                    				cnt = cnt.reshape(-1, 2)
                    				max_cos = np.max([angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)])
                    				if max_cos < 0.1:
                        				squares.append(cnt)
    							countb=countb+1

		img=self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		i=0
	        j=0
   	        while (i < img.shape[0]):     
    			j=0
			while ( j < img.shape[1]):
				if img[i,j,0] > 5 or img[i,j,1] > 5:   
					img[i,j,2]=0
					img[i,j,0]=0
					img[i,j,1]=0
			
				j=j+1
			i = i + 1
		img = cv.GaussianBlur(img, (5, 5), 0)
    		     		
    		
    		for gray in cv.split(img):
        		for thrs in xrange(0, 255, 26):
            			if thrs == 0:
                			bin = cv.Canny(gray, 0, 50, apertureSize=5)
                			bin = cv.dilate(bin, None)
            			else:
                			_retval, bin = cv.threshold(gray, thrs, 255, cv.THRESH_BINARY)
            			bin, contours, _hierarchy = cv.findContours(bin, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
            			for cnt in contours:
           	 	    		cnt_len = cv.arcLength(cnt, True)
                			cnt = cv.approxPolyDP(cnt, 0.02*cnt_len, True)
                			if len(cnt) == 4 and cv.contourArea(cnt) > 100 and cv.isContourConvex(cnt):
                    				cnt = cnt.reshape(-1, 2)
                    				max_cos = np.max([angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)])
                    				if max_cos < 0.1:
                        				squares.append(cnt)
    							countr=countr+1
		countb=countb/7
		countr=countr/7
		countg=countg/7
		print (countb)
		print (countr)
		print (countg)

		     		
		#image.shape[0]
		self.b=countb
		self.g=countg
		self.r=countr
		self.command_r.publish(self.r)
		self.command_b.publish(self.b)
		self.command_g.publish(self.g)
		cv2.waitKey(0) 
		cv2.destroyAllWindows()

	

		

if __name__ == '__main__':
   
	while not rospy.is_shutdown():
		temp = DroneFly()
		temp.init()

		temp.position_hold()
		
		rospy.spin()
	
