#!/usr/bin/env python3
# Importing the required libraries
from luminosity_drone.msg import Biolocation
from swift_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from std_msgs.msg import Int16
from std_msgs.msg import Int64
#from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from imutils import contours
from skimage import measure
import numpy as np
import imutils
import cv2


class swift():
	"""docstring for swift"""
	def __init__(self):

		self.centroid_led_x= 0
		self.centroid_led_y= 0
		self.height=0
		self.perpendicular=0
		self.next_centre=0
		self.next_height=0

		self.dis=0
		rospy.init_node('drone_control')	# initializing ros node with name drone_control
		self.image=[]
		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.pub=0
		self.green=0
		self.drone_position = [0.0,0.0,0.0]	
		self.flag=[0,1,2,3,4,5,6,7,8,9]
		self.now = rospy.Time.now()
		# [x_setpoint, y_setpoint, z_setpoint]
		#self.setpoint = [[0, 0, 25],[5, 5, 25],[5,-5,20],[-5, -5, 20],[-6.5, 6, 20],[5, 5, 20]] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		#self.setpoint = [[0, 0, 25],[5, 5, 25],[5.5, 9.5, 25],[5,-9,25],[-3, -3, 25],[-6.5, -8, 25],[-3,0,25],[-6.5, 6, 25],[5, 5, 25]] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		self.setpoint = [[0,0,3],[0,0,17],[7.5,-7.5,25],[0,-7.5,25],[-7.5,-7.5,25],[-7.5,0,25],[-7.5,7.5,25],[0,7.5,25],[7.5,7.5,25],[0,0,17]] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

		#self.setpoint=[5.5,5.5,15]
		#self.target = self.setpoint[0]
		self.target = self.setpoint[0]
		self.index = 2
		self.n = 0
		self.min_err = [self.target[i] - 0.4 for i in range(3)]
		self.max_err = [self.target[i] + 0.4 for i in range(3)]
		self.No_of_led = []
		#Declaring a cmd of message type swift_msgs and initializing values
		self.cmd = swift_msgs()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500

		self.bridge = CvBridge()
		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [11.94, 11.94, 43.26]
		self.Ki = [0.0168, 0.0168, 0.036]
		self.Kd = [21.9, 21.9, 41.7]





		#-----------------------Add other required variables for pid here ----------------------------------------------
		self.organism_type = "hello"
		self.prev_error = [0, 0, 0]
		self.error_sum = [0, 0, 0]
		self.max_values = [2000, 2000, 2000]
		self.min_values = [1000, 1000, 1000]

		self.gothere_target=[0,0,0]
		

		self.sample_time = 0.060 # in seconds





		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)

		#------------------------Add other ROS Publishers here-------------------------

		self.alt_error_pub = rospy.Publisher('/alt_error',Float64, queue_size=1)
		self.pitch_error_pub = rospy.Publisher('/pitch_error',Float64, queue_size=1)
		self.roll_error_pub = rospy.Publisher('/roll_error',Float64, queue_size=1)
		self.publisher= rospy.Publisher('/astrobiolocation', Biolocation, queue_size=1)
		

		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------

		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber("/drone_display/image_view/output", Image, self.image_callback)
		#rospy.Subscriber("/whycon_display/image_view/output", Image, self.whycon_imgview_callback)



		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE
	def image_callback(self , img_msg):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(img_msg)
		except CvBridgeError as e:
			rospy.logerr("CvBridge Error: {0}".format(e))
		self.image= cv_image
		if self.drone_position[2] < 20:
			thresh_val=120
		else:
			thresh_val=200
		##cv2.imshow("Image Window", cv_image)
		##cv2.waitKey(3)
		gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray, (11, 11), 0)
			
		## threshold the image to reveal light regions in the blurred image
		thresh = cv2.threshold(blurred, thresh_val, 255, cv2.THRESH_BINARY)[1]
#		
		centroid_ledx=0
		centroid_ledy=0
		# perform a series of erosions and dilations to remove any small blobs of noise from the thresholded image
		thresh = cv2.erode(thresh, None, iterations=1)
		thresh = cv2.dilate(thresh, None, iterations=2)
		cv2.imshow("neq", blurred)
		cv2.imshow("neq2", thresh)
		cv2.waitKey(3)
		## perform a connected component analysis on the thresholded image, then initialize a mask to store only the "large" components
		labels = measure.label(thresh, connectivity=2, background=0)
		mask = np.zeros(thresh.shape, dtype="uint8")
		for label in np.unique(labels):
			#print("hello")
    	# if this is the background label, ignore it
			if label == 0:
				continue
			labelMask = np.zeros(thresh.shape, dtype="uint8")
			labelMask[labels == label] = 255
			numPixels = cv2.countNonZero(labelMask)
			if numPixels > 300:
				mask = cv2.add(mask, labelMask)
		contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		#contours = contours.sort_contours(contours)
		centroids = []
		areas = []
		print("green  "+ str(self.green))
		for i, contour in enumerate(contours):
			area = cv2.contourArea(contour)
			M = cv2.moments(contour)
			if M["m00"] != 0:
				#print("helllo")
				#if self.green==0:
					#self.green+=1
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
				cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)
				cv2.circle(cv_image, (cX, cY), 7, (0, 0, 255), -1)
				centroids.append((cX, cY))
				areas.append(area)
				self.No_of_led= len(centroids)
				centroid_ledx += cX
				centroid_ledy += cY 
				self.centroid_led_x = centroid_ledx/self.No_of_led
				self.centroid_led_y = centroid_ledy/self.No_of_led
			
			#print(self.centroid_led_x, self.centroid_led_y)
			
		#cv2.imwrite("led_detection_rets.png", )
		#print(len(centroids));
# Open a text file for writing
		self.No_of_led= len(centroids)
		
	
	
		#print(centroids)
		height = cv_image.shape[0]
		width = cv_image.shape[1]
		channels = cv_image.shape[2]








	def gothere(self):

		self.height= 37- self.drone_position[2]
		self.perpendicular= self.height*0.2
		self.next_centre = self.perpendicular/2
		self.next_height= (self.next_centre/0.2)-2


		if self.centroid_led_x < 353.6 and self.centroid_led_y< 353.6 :
			self.target= [-self.next_centre, -self.next_centre, self.next_height]
			print(1)
		if self.centroid_led_x > 353.6 and self.centroid_led_y< 353.6 :
			self.target= [self.next_centre, -self.next_centre, self.next_height]
			print(2)
		if self.centroid_led_x > 353.6 and self.centroid_led_y> 353.6 :
			self.target= [self.next_centre, self.next_centre, self.next_height]
			print(3)
		if self.centroid_led_x < 353.6 and self.centroid_led_y > 353.6 :
			self.target= [-self.next_centre, self.next_centre, self.next_height]
			print(4)
		self.min_err = [self.target[i] - 0.2 for i in range(3)]
		self.max_err = [self.target[i] + 0.2 for i in range(3)]
		print("hello")
		
	#def show_image(img):
		

		# loop over the unique components
		

		#cv2.imshow("Image Window", img)
		#cv2.waitKey(3)	



	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

	def disar_m(self):
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 0
		self.cmd.rcAUX1 = 0
		self.cmd.rcAUX2 = 0
		self.cmd.rcAUX3 = 0
		self.cmd.rcAUX4 = 0
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)

	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x
		
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z


	
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.06 #  # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.Ki * 0.0008 
		self.Kd[2] = alt.Kd * 0.3
		
		

		
	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
	def pitch_set_pid(self,alt):
		self.Kp[1] = alt.Kp * 0.06 # 120, 0, This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[1] = alt.Ki * 0.0008
		self.Kd[1] = alt.Kd * 0.3
		
		
	def roll_set_pid(self,alt):
		self.Kp[0] = alt.Kp * 0.06 #This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[0] = alt.Ki * 0.0008
		self.Kd[0] = alt.Kd * 0.3











	#----------------------------------------------------------------------------------------------------------------------

	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------
		if (0 and all(self.drone_position[i] <self.max_err[i] for i in range(3)) and all(self.drone_position[i]> self.min_err[i] for i in range(3)) and (rospy.Time.now() > self.now + rospy.Duration.from_sec(10))and self.n<8 and self.green<1): 
		#if (abs(self.drone_position[self.index] <self.max_err[self.index]) and (rospy.Time.now() > self.now + rospy.Duration.from_sec(10))and self.n<4):

			#print("Hello")
			#print(self.n)
			self.n += 1
			self.target = self.setpoint[self.n]
			self.min_err = [self.target[i] - 0.2 for i in range(3)]
			self.max_err = [self.target[i] + 0.2 for i in range(3)]
			#r.sleep_dur(0.2)
		error = [self.drone_position[i] - self.target[i] for i in range(3)]
		if (all(self.drone_position[i] <self.max_err[i] for i in range(3)) and all(self.drone_position[i]> self.min_err[i] for i in range(3)) and self.pub==0 and self.green==0):		

			self.green+=1
        # Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis
		p_term = [self.Kp[i] * error[i] for i in range(3)]
		d_term = [self.Kd[i] * (error[i] - self.prev_error[i]) / self.sample_time for i in range(3)]
		self.error_sum = [self.error_sum[i] + error[i] for i in range(3)]
		i_term = [self.Ki[i] * self.error_sum[i] * self.sample_time for i in range(3)]

        # Calculate the pid output required for each axis
		self.out_roll = p_term[1] + i_term[1] + d_term[1]
		self.out_pitch = p_term[0] + i_term[0] + d_term[0]
		self.out_throttle = p_term[2] + i_term[2] + d_term[2]

        # Reduce or add this computed output value on the avg value ie 1500
		self.cmd.rcRoll = int(1500 + self.out_roll)
		self.cmd.rcPitch = int(1500 + self.out_pitch)
		self.cmd.rcThrottle = int(1580 + self.out_throttle)

        # Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing
		if self.cmd.rcRoll > self.max_values[1]:
			self.cmd.rcRoll = self.max_values[1]
		elif self.cmd.rcRoll < self.min_values[1]:
			self.cmd.rcRoll = self.min_values[1]

		if self.cmd.rcPitch > self.max_values[0]:
			self.cmd.rcPitch = self.max_values[0]
		elif self.cmd.rcPitch < self.min_values[0]:
			self.cmd.rcPitch = self.min_values[0]
		
		if self.cmd.rcThrottle > self.max_values[2]:
			self.cmd.rcThrottle = self.max_values[2]
		elif self.cmd.rcThrottle < self.min_values[2]:
			self.cmd.rcThrottle = self.min_values[2]

        # Update previous errors and add error_sum
		self.prev_error = error

		if self.No_of_led==2:
			self.organism_type = "alien_a"
		if self.No_of_led==3:
			self.organism_type = "alien_b"
		if self.No_of_led==4:
			self.organism_type = "alien_c"
	#------------------------------------------------------------------------------------------------------------------------
		
		#print(self.drone_position, self.green, self.pub)

		if (all(self.drone_position[i] <self.max_err[i] for i in range(3)) and all(self.drone_position[i] >self.min_err[i] for i in range(3)) and self.green==1 and self.pub==0):
			self.min_err = [self.target[i] - 0.2 for i in range(3)]
			self.max_err = [self.target[i] + 0.2 for i in range(3)]
			#self.pub +=1
			#self.gothere()
		if (all(self.drone_position[i] <self.max_err[i] for i in range(3)) and all(self.drone_position[i] >self.min_err[i] for i in range(3))and self.green==1 and  self.pub<5):
			#self.publisher.publish(self.organism_type, self.drone_position[0],self.drone_position[1],self.drone_position[2])
			self.pub+=1
			self.gothere()
			
			#self.target = [10.5, 10.5, 37]
			#self.min_err = [self.target[i] - 0.2 for i in range(3)]
			#self.max_err = [self.target[i] + 0.2 for i in range(3)]
		if (all(self.drone_position[i] <self.max_err[i] for i in range(3)) and all(self.drone_position[i] >self.min_err[i] for i in range(3))and self.green==1 and self.pub==5):
			self.publisher.publish(self.organism_type, self.drone_position[0],self.drone_position[1],self.drone_position[2])
			self.pub+=1
			self.target = [11, 11, 37]
			self.min_err = [self.target[i] - 0.2 for i in range(3)]
			self.max_err = [self.target[i] + 0.2 for i in range(3)]
		#self.pub=2
#
		if (all(self.drone_position[i] < self.max_err[i] for i in range(3)) and all(self.drone_position[i] >self.min_err[i] for i in range(3)) and self.green==1 and self.pub==6):
			#self.disarm()
			self.dis+=1
			self.pub+=1
			#self.cmd.rcThrottle =1000
	#	if abs(self.centroid_led_x)<352 and abs(self.centroid_led_y)<352 and self.green==1:
	#		self.publisher.publish(self.organism_type, self.drone_position[0],self.drone_position[1],self.drone_position[2])
	#		self.green+=1





		self.height= 37- self.drone_position[2]
		self.perpendicular= self.height*0.2
		#self.next_perpendicular= self.perpendicular/2
#
		#self.next_height = 3 + self.next_perpendicular/0.2
		#self.centre= self.perpendicular * 1.414/2
		#self.next_centre= self.centre + self.drone_position[0]
#
		self.next_centre = self.perpendicular/2
		self.next_height= (self.next_centre/0.2)-2

		print(self.pub)
		self.command_pub.publish(self.cmd)
		self.alt_error_pub.publish(error[2])
		self.pitch_error_pub.publish(error[1])
		self.roll_error_pub.publish(error[0])
			#self.target = [11, 11, 37]
		print(self.drone_position)

if __name__ == '__main__':
	#dis=0
	swift_drone = swift()
	r = rospy.Rate(1/0.06) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		if swift_drone.dis ==0:
			swift_drone.pid()
		else:
			swift_drone.disar_m()
		r.sleep()
		
	