#!/usr/bin/env python3

'''

This python file runs a ROS-node of name drone_control which holds the position of Drone.
This node publishes and subscribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/astrobiolocation		/swift/camera_rgb/image_raw
								
'''

# Importing the required libraries

from swift_msgs.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from luminosity_drone.msg import Biolocation
import rospy
import time
import cv2
import numpy as np
from imutils import contours
from skimage import measure
import imutils


def within_bounds(error, bounds):
	"""returns whether error is within the bounds"""
	return abs(error[0]) < bounds[0] and abs(error[1]) < bounds[1] and abs(error[2]) < bounds[2]

def clamp(n, min, max):
	"""returns the value of n clamped between min and max"""
	if n < min:
		return min
	elif n > max:
		return max
	else:
		return n 

#generates search path as a list of coordinates in a spiral pattern in a grid
def generate_spiral_path(grid_size, grid_dim):

    # Initialize variables
    coordinates = []
    direction = 'up'
    x, y = 0, 0
    min_x, max_x, min_y, max_y = 0, 0, 0, 0

    # Helper function to update direction
    def change_direction():
        nonlocal direction, min_x, max_x, min_y, max_y
        if direction == 'right':
            direction = 'down'
            min_y -= grid_dim
        elif direction == 'down':
            direction = 'left'
            min_x -= grid_dim
        elif direction == 'left':
            direction = 'up'
            max_y += grid_dim
        elif direction == 'up':
            direction = 'right'
            max_x += grid_dim

    def move_one_step():
        nonlocal direction, x, y
        if direction == 'right':
            x += grid_dim
        elif direction == 'down':
        	y -= grid_dim
        elif direction == 'left':
            x -= grid_dim
        elif direction == 'up':
            y += grid_dim

    # Iterate through the grid
    for i in range(grid_size * grid_size):
        coordinates.append([x, y])
        if direction == 'right':
            if x < max_x:
                move_one_step()
            else:
                change_direction()
                move_one_step()
        elif direction == 'down':
            if y > min_y:
                move_one_step()
            else:
                change_direction()
                move_one_step()
        elif direction == 'left':
            if x > min_x:
                move_one_step()
            else:
                change_direction()
                move_one_step()
        elif direction == 'up':
            if y < max_y:
                move_one_step()
            else:
                change_direction()
                move_one_step()

    return coordinates

class swift():
	"""docstring for swift"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control


		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0,0.0,0.0]	

		self.search_height = 25;			#this is the height of hovering in searching
		self.search_grid_dimension = 4.5	#length of one step of movement
		self.search_grid_size = 5;			#this is the dimension of the grid search area in terms of grids
											#for example we are searhing in a 5x5 grid

		#Generating the coordinates for the search path
		search_path = generate_spiral_path(self.search_grid_size, self.search_grid_dimension)
		self.flightpath = [
			[
				point[0],
				point[1],
				self.search_height
			] for point in search_path
		]
		self.current_index = 0;  #index variable to keep track of points reached

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = self.flightpath[0]


		self.mission_phase = "searching"	#mission has been divided into 4 phases to keep track of drone operation
		self.research_station = [11,11,35]
		self.organism_list = {
			2: "alien_a",
			3: "alien_b",
			4: "alien_c"
		}

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

		#declaring astrobiolocation variable of Biolocation msg type
		self.astrobiolocation = Biolocation()
		self.astrobiolocation.organism_type = ""
		self.astrobiolocation.whycon_x = 0
		self.astrobiolocation.whycon_y = 0
		self.astrobiolocation.whycon_z = 37.22


		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [0.06*250,	0.06*300,	0.06*320]
		self.Ki = [0,			0,			0.0008*50]
		self.Kd = [0.3*1000,	0.3*900,	0.3*1600]
   
		#-----------------------Add other required variables for pid here ----------------------------------------------
		self.centroid_error = [0,0]
		self.error = [0,0,0]
		self.prev_error = [0,0,0]
		self.i_term = [0,0,0]

		self.out_roll = 0
		self.out_pitch = 0
		self.out_throttle = 0 

		self.max_values = [2000,2000,2000]
		self.min_values = [1000,1000,1000]

		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error
		self.command_pub = rospy.Publisher('/drone_command', swift_msgs, queue_size=1)
		self.biolocation_pub = rospy.Publisher('/astrobiolocation', Biolocation, queue_size=1)

	#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('swift/camera_rgb/image_raw', Image, self.image_callback)

		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE DRONE

	#this function lands the drone and disarms it
	def land(self):	
		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.disarm()

	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
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
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z

	
	#---------------------------------------------------------------------------------------------------------------

	# Image callback function
	# The function gets executed each time when /swift node publishes /swift/camera_rgb/image_raw
	def image_callback(self, msg):
		img = np.array([int(n) for n in msg.data], dtype=np.uint8)
		img.resize(msg.height, msg.width, 3)

		# convert it to grayscale, and blur it
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		blur = cv2.blur(gray, (9,9))

		# threshold the image to reveal light regions in the blurred image
		ret, thresh = cv2.threshold(blur, 150, 255, cv2.THRESH_BINARY)

		# perform a series of erosions and dilations to remove any small blobs of noise from the thresholded image
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
		eroded = cv2.erode(thresh, kernel, iterations = 3)
		dilated = cv2.dilate(eroded, kernel, iterations = 3)

		# cv2.imshow('Dilated', dilated)
		# cv2.waitKey(1)

		# perform a connected component analysis on the thresholded image, then initialize a mask to store only the "large" components
		labels = measure.label(dilated, connectivity=2, background=0)
		mask = np.zeros(thresh.shape, dtype="uint8")

		# loop over the unique components
		for label in np.unique(labels):
		    # print(label)
			# if this is the background label, ignore it
		    if label == 0:
		        continue
			# otherwise, construct the label mask and count the number of pixels 
		    else:
		        labelMask = np.zeros(thresh.shape, dtype="uint8")
		        labelMask[labels == label] = 255
		        numPixels = labelMask.sum() // 255
			# if the number of pixels in the component is sufficiently large, then add it to our mask of "large blobs"
		 
		    if numPixels > 150:
		        mask = cv2.add(mask, labelMask)
			
		# find the contours in the mask, then sort them from left to right
		contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		contours = imutils.grab_contours(contours)
		contours = sorted(contours, key=cv2.contourArea, reverse=True)

		# Initialize lists to store centroid coordinates
		centroid_list = []

		# Loop over the contours
		for i, contour in enumerate(contours): 

		    # Draw the bright spot on the image
		    M = cv2.moments(contour)
		    centroid_x = int(M["m10"]/M["m00"])
		    centroid_y = int(M["m01"]/M["m00"])
		    centroid = (centroid_x, centroid_y)
		    
		    # Append centroid coordinates to the list
		    centroid_list.append(centroid)

		# if   led detected       and  searching has begun
		if len(centroid_list) > 0 and self.current_index > 0:

			if self.mission_phase == "searching":
				self.mission_phase = "organism_detected"
				print("I saw something!!!")

			# identify the organism
			if (len(centroid_list) in self.organism_list.keys()):
				self.astrobiolocation.organism_type = self.organism_list[len(centroid_list)]

			# find the centroid of the centroids
			x_sum, y_sum = 0, 0
			for c in centroid_list:
				x_sum += c[0]
				y_sum += c[1]

			self.centroid_error = [
				x_sum/len(centroid_list) - msg.width/2,
				y_sum/len(centroid_list) - msg.height/2
			]

	#---------------------------------------------------------------------------------------------------------------

	def pid(self):

		# in searching mode, the drone follows the search path coordinates one be one
		if self.mission_phase == "searching":
			self.error[0] = self.drone_position[0] - self.setpoint[0]
			self.error[1] = self.drone_position[1] - self.setpoint[1]
			self.error[2] = self.drone_position[2] - self.setpoint[2]

			#if reached a search coordinate, move to the next one in the list
			if within_bounds(self.error, [0.5, 0.5, 1]):
				if self.current_index < len(self.flightpath)-1:
					self.current_index += 1
					self.setpoint = self.flightpath[self.current_index]
				else:
					print("Reached final search checkpoint: ", self.setpoint)

		# in this mode, PID tries to center the leds in the frame
		elif self.mission_phase == "organism_detected":
			self.error[0] = -self.centroid_error[0] / 200
			self.error[1] = -self.centroid_error[1] / 200
			self.error[2] = self.drone_position[2] - self.search_height

			# once, centered publish the astrobiolocation
			if within_bounds(self.error, [0.05, 0.05, 1]) and within_bounds(self.prev_error, [0.1, 0.1, 1]):
				print("Found a", self.astrobiolocation.organism_type)
				print("Returning to base")
				self.astrobiolocation.whycon_x = self.drone_position[0]
				self.astrobiolocation.whycon_y = self.drone_position[1]
				self.biolocation_pub.publish(self.astrobiolocation)
				self.mission_phase = "returning_to_base"

		#Returning to the base (self.research_station)
		elif self.mission_phase == "returning_to_base":
			self.error[0] = self.drone_position[0] - self.research_station[0]
			self.error[1] = self.drone_position[1] - self.research_station[1]
			self.error[2] = self.drone_position[2] - self.research_station[2]
			if within_bounds(self.error, [0.05, 0.05, 0.5]) and within_bounds(self.prev_error, [0.05, 0.05, 0.5]):
				print("Houston, we landed!")
				self.land()
				self.mission_phase = "mission_accomplished"
				return

		# once landed, do nothing
		elif self.mission_phase == "mission_accomplished":
			return

		#perform summation of integral term
		self.i_term[0] += self.error[0] if abs(self.error[0]) < 0.5 else 0;
		self.i_term[1] += self.error[1] if abs(self.error[1]) < 0.5 else 0;
		self.i_term[2] += self.error[2] if abs(self.error[2]) < 1 else 0;

		#calculate PID output
		self.out_roll = self.Kp[0]*(self.error[0]) + self.Kd[0]*(self.error[0] - self.prev_error[0]) + self.Ki[0]*self.i_term[0]
		self.out_pitch = self.Kp[1]*(self.error[1]) + self.Kd[1]*(self.error[1] - self.prev_error[1]) + self.Ki[1]*self.i_term[1]
		self.out_throttle = self.Kp[2]*(self.error[2]) + self.Kd[2]*(self.error[2] - self.prev_error[2]) + self.Ki[2]*self.i_term[2]

		#compute outputs
		self.cmd.rcPitch = 1500 + int(self.out_pitch)
		self.cmd.rcRoll = 1500 - int(self.out_roll)
		self.cmd.rcThrottle = 1586 + int(self.out_throttle)

		#limit the values of outputs
		self.cmd.rcRoll = clamp(self.cmd.rcRoll, self.min_values[0], self.max_values[0])
		self.cmd.rcPitch = clamp(self.cmd.rcPitch, self.min_values[1], self.max_values[1])
		self.cmd.rcThrottle = clamp(self.cmd.rcThrottle, self.min_values[2], self.max_values[2])

		#set previous errors
		self.prev_error[0] = self.error[0]
		self.prev_error[1] = self.error[1]
		self.prev_error[2] = self.error[2]

		self.command_pub.publish(self.cmd)

	#---------------------------------------------------------------------------------------------------------------

if __name__ == '__main__':

	swift_drone = swift()
	r = rospy.Rate(31) #specify rate in Hz based upon your desired PID sampling time, i.e. if desired sample time is 33ms specify rate as 30Hz
	while not rospy.is_shutdown():
		r.sleep()
		swift_drone.pid()