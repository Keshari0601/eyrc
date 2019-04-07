#!/usr/bin/env python

#Team Id : 3434
#Author List : Bhuvan Jhamb, Archit Chaudhary, Keshari Nath Tiwari, Parag Saroha
#File Name : task_1.1.py
#Theme : Pollinator Bee (PB)
#Functions : __init__(self),init(self),arm(self),def disarm(self), position_hold(self), get_data(self, req), def get_red(self, red), get_blue(self, blue), get_green(self, green):
#			 get_total(self, total), calc_pid(self), pid_roll(self), pid_pitch(self), pid_throt(self), pid_yaw(self), limit(self, input_value, max_value, min_value),
#	         set_pid_alt(self,pid_val), set_pid_roll(self,pid_val), set_pid_pitch(self,pid_val), set_pid_yaw(self,pid_val), get_pose(self,pose)
#Global Variables : pn




#




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
from plutodrone.srv import *
#pn : This is a varible to trigger drone for the next point

pn=0
#position: this is a variable to store the number of plant passed
position=0

#total: total number of plants we have

total=3

#xp: array of x coordinates of plants
xp=[5.8,0.2,-6.0,0]

#yp: array of y coordinates of plants

yp=[2.61,9.2,-5.5,-0]

#zp: array of z coordinates of plants

zp=[22.2,23,25,20]

	

class DroneFly():
	"""docstring for DroneFly"""
	
        def __init__(self):
		
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
		#rospy.Subscriber('drone_yaw', Float64, self.get_yaw)

		# To tune the drone during runtime
		rospy.Subscriber('/pid_tuning_altitude', PidTune, self.set_pid_alt)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.set_pid_roll)
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.set_pid_pitch)
		rospy.Subscriber('/pid_tuning_yaw', PidTune, self.set_pid_yaw)
		rospy.Subscriber('/yawttt', Float64, self.get_data)
		rospy.Subscriber('/red', Int32, self.get_red)
		rospy.Subscriber('/blue', Int32, self.get_blue)
		rospy.Subscriber('/green', Int32, self.get_green)
		rospy.Subscriber('/total', Int32, self.get_total)
		rospy.Subscriber('/n', Int32, self.get_n)
		self.cmd = PlutoMsg()

		# Position to hold.
		self.wp_x = 0.0
		self.wp_y = 0.0
		self.wp_z = 20.0
		self.wp_yaw = 141.0
	


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
		self.kp_roll = 16.0
		self.ki_roll = 22.0
		self.kd_roll = 545.0

		#PID constants for Pitch
		self.kp_pitch = 21.0
		self.ki_pitch = 17.0
		self.kd_pitch = 897.0
		
		#PID constants for Yaw
		self.kp_yaw = 75.0
		self.ki_yaw = 0.0
		self.kd_yaw =  96.0

		#PID constants for Throttle
		self.kp_throt = 120.0
		self.ki_throt = 2.0
		self.kd_throt = 10.0

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
		self.i=0
		self.j=0
		self.l=0
		self.t=0
		self.a=0
		
                #print(self.ithrot)
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


	'''
	* Function Name: position_hold
	* Input: self
	* Output: none
	* Logic: This function find the current position of drone and give drone commands by applying pid using other functions
	* 
	* Example Call: position_hold(self)
	'''


	def position_hold(self):

		rospy.sleep(2)
		global pn
		global position
		global total
		global xp
		global yp
		global zp
		
		#print "disarm"
		self.disarm()
		rospy.sleep(.2)
		print "start"
		self.arm()
		rospy.sleep(.1)
                self.iroll = 0.0
                self.ithrot = 0.0
                self.ipitch = 0.0

		while True:
			#this if case is for landing of drone on bee-hive
			if position==100:
			    if pn==-10:
				if self.drone_x > -1.0+self.wp_x and self.drone_x<1.0+self.wp_x and self.drone_y >-1.00+self.wp_y and self.drone_y <1.00+self.wp_y and self.drone_z>-1.0+self.wp_z and self.drone_z<1.0+self.wp_z:
					#print "one done"					
					pn=-11
					self.wp_z=25.0
					
			    if pn==-11:
				if self.drone_x > -0.9+self.wp_x and self.drone_x<0.9+self.wp_x and self.drone_y >-1.0+self.wp_y and self.drone_y <0.7+self.wp_y and self.drone_z>-0.8+self.wp_z and self.drone_z<0.8+self.wp_z:
					#print "two done"					
					pn=-12
					self.wp_z=27.0
			    if pn==-12:
				if self.drone_x > -0.8+self.wp_x and self.drone_x<0.8+self.wp_x and self.drone_y >-0.9+self.wp_y and self.drone_y <0.6+self.wp_y and self.drone_z>-0.8+self.wp_z and self.drone_z<0.8+self.wp_z:
					#print "three done"					
					pn=-13
					self.wp_z=28.0
			    if pn==-13:
				if self.drone_x > -0.7+self.wp_x and self.drone_x<0.7+self.wp_x and self.drone_y >-0.8+self.wp_y and self.drone_y <0.6+self.wp_y and self.drone_z>-0.7+self.wp_z and self.drone_z<0.7+self.wp_z:
					#print "four done"					
					pn=-14
					self.wp_z=29.0
			    if pn==-14:
				if self.drone_x > -0.5+self.wp_x and self.drone_x<0.5+self.wp_x and self.drone_y >-0.8+self.wp_y and self.drone_y <0.6+self.wp_y and self.drone_z>-0.5+self.wp_z and self.drone_z<0.5+self.wp_z:
					#print "five done"					
					pn=-15
					self.wp_z=29.5
					
			    if pn==-15:
				if self.drone_x > -0.5+self.wp_x and self.drone_x<0.5+self.wp_x and self.drone_y >-0.8+self.wp_y and self.drone_y <0.6+self.wp_y and self.drone_z>-0.5+self.wp_z and self.drone_z<0.5+self.wp_z:
										
					pn=-17
					
					
					
					#pn=1
					#self.wp_x=6.3
					#self.wp_y=2.5
					#print "one dealt"
			


			#self.a: A variable that stores stop condition
			if self.a==0:
			  if pn==0:
				if self.drone_x > -1.0+self.wp_x and self.drone_x<1.0+self.wp_x and self.drone_y >-1.00+self.wp_y and self.drone_y <1.00+self.wp_y and self.drone_z>-1.0+self.wp_z and self.drone_z<1.0+self.wp_z:
					#print "one done"					
					pn=-1
					self.wp_x=xp[position]
					self.wp_y=yp[position]
					
			  if pn==-1:
				if self.drone_x > -1.0+self.wp_x and self.drone_x<1.0+self.wp_x and self.drone_y >-1.0+self.wp_y and self.drone_y <1.0+self.wp_y and self.drone_z>-0.9+self.wp_z and self.drone_z<0.9+self.wp_z:
					#print "two done"					
					pn=-2
					self.wp_z=zp[position]
					
			  if pn==-2:
				if self.drone_x > -0.8+self.wp_x and self.drone_x<0.8+self.wp_x and self.drone_y >-1.0+self.wp_y and self.drone_y <1.0+self.wp_y and self.drone_z>-0.8+self.wp_z and self.drone_z<0.8+self.wp_z:
					#print "three done"					
					pn=-3
					self.wp_z=20
					
			  if pn==-3:
				if self.drone_x > -1.5+self.wp_x and self.drone_x<1.5+self.wp_x and self.drone_y >-1.5+self.wp_y and self.drone_y <1.5+self.wp_y and self.drone_z>-0.7+self.wp_z and self.drone_z<0.7+self.wp_z:
					#print "four done"					
					position=position+1
					if position<total:
						pn=0					
					else:
						position=300
						self.wp_x=0.0
						self.wp_y=0.0
						self.wp_z=20
						pn=-5555
		
			
			else:
				self.wp_x=0.0
				self.wp_y=0.0
				self.wp_z=20






			



			
			
			self.calc_pid()

			# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
                       
                        
			pitch_value = int(1535+self.correct_pitch)
			self.cmd.rcPitch = self.limit (pitch_value, 1635,1435)
			
                       												
			roll_value = int(1450+self.correct_roll)
			self.cmd.rcRoll = self.limit(roll_value, 1600,1300)
			
                        
			yaw_value = int(1700+self.correct_yaw)
			self.cmd.rcYaw = self.limit (yaw_value, 1750, 1400)
			
			throt_value = int(1600-self.correct_throt)
			self.cmd.rcThrottle = self.limit(throt_value, 2000,1450)
											
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
			#print self.yaw_z
			#print self.wp_yaw

	'''
	* Function Name: get_data
	* Input: self, req
	* Output: none
	* Logic: get the current yaw from a rospy typic
	* 
	* Example Call: get_data(self, req)
	'''




	def get_data(self, req):
		 self.yaw_z = req.data

	'''
	* Function Name: get_red
	* Input: self, red
	* Output: none
	* Logic: get the number of red leds on arena
	* 
	* Example Call: get_red(self, red)
	'''

	 
		 
	def get_red(self, red):
		self.j = red.data

	def get_blue(self, blue):
		self.i = blue.data
	
	def get_green(self, green):
		self.l = green.data

	def get_total(self, total):
		self.t = total.data

	def get_n(self, n):
		self.a = n.data
		if self.a==1:
			print 'stop'


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
		dt=0.032
		self.y=self.wp_y-self.drone_y
		#self.iroll=0.0
		self.iroll = self.iroll + self.y*dt
        	self.droll=(self.y-self.lastroll)
		self.lastroll=self.y
        	self.correct_roll=self.kp_roll*self.y+self.kd_roll*self.droll+(self.ki_roll*self.iroll/100)

	def pid_pitch(self):

		dt=0.032
		self.x=self.wp_x-self.drone_x
		#self.ipitch=0.0		
		self.ipitch = self.ipitch + self.x*dt
        	self.dpitch=(self.x-self.lastpitch)
		self.lastpitch=self.x
        	self.correct_pitch=self.kp_pitch*self.x+self.kd_pitch*self.dpitch+(self.ki_pitch*self.ipitch/100)

	def pid_throt(self):

		dt=0.032
		self.z=self.wp_z-self.drone_z		
		self.ithrot = (self.ithrot) + (self.z*dt)
         	self.dthrot=(self.z-self.lastthrot)/dt
		self.lastthrot=self.z
        	self.correct_throt=(self.kp_throt*self.z+self.kd_throt*self.dthrot+(self.ki_throt*self.ithrot/100))

	def pid_yaw(self):
		dt=0.032
		self.yawn=self.wp_yaw-self.yaw_z
		#self.iyaw=0.0
		self.iyaw = self.iyaw + self.yawn*dt
        	self.dyaw=(self.yawn-self.lastyaw)/dt
		self.lastyaw=self.yawn
        	self.correct_yaw=(self.kp_yaw*self.yawn+self.kd_yaw*self.dyaw+(self.ki_yaw*self.iyaw/20))/100
		#print(self.correct_yaw)

		
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
	
		
	

		

if __name__ == '__main__':
   
	while not rospy.is_shutdown():
		temp = DroneFly()
		temp.init()

		temp.position_hold()
		
		rospy.spin()
	
