#!/usr/bin/env python
from sensor_msgs.msg import Image, CompressedImage
from duckietown_msgs.msg import Twist2DStamped
from fp_gt_and_oa.msg import awesome
from std_msgs.msg import Int16
import std_msgs
import math
import sys
import rospy
import cv2
import numpy as np
from collections import deque
from cv_bridge import CvBridge, CvBridgeError

class sub_class:
	def __init__(self):
		self.sub = rospy.Subscriber("/processed_image_msg", awesome, self.callback, queue_size=1)		
		self.pub_control= rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
		self.rate = rospy.Rate(10) # 10hz
        	self.msg = awesome()

	def callback(self, msg):
		
		control_msg = Twist2DStamped()
		
		if msg.x_go != -1:

			if msg.x_ob != -1:

				if math.fabs(msg.r_go-msg.r_ob) < 40:
				
					print "obstacle and goal are of comparable size"
					print "I will move based on their relative position in the processed image"

					if math.fabs(msg.x_go-125) < 20:

						if (msg.x_go-msg.x_ob) < -50:

							control_msg.v = 0.3
							control_msg.omega = 0.0

						elif (msg.x_go-msg.x_ob) > 50:

							control_msg.v = 0.3
							control_msg.omega = 0.0

						elif (msg.x_go < msg.x_ob) and math.fabs(msg.x_go-msg.x_ob) < 50:  #Not meeting the above 2 conditions
							control_msg.v = 0.1
							control_msg.omega = 0.3

						elif (msg.x_go > msg.x_ob) and math.fabs(msg.x_go-msg.x_ob) < 50:  #Not meeting the above 2 conditions
							control_msg.v = 0.1
							control_msg.omega = -0.3


					elif (msg.x_go-125)>20 :


						if (msg.x_go-msg.x_ob) < -50:

							control_msg.v = 0.2
							control_msg.omega = -0.4

						elif (msg.x_go-msg.x_ob) > 50:

							control_msg.v = 0.2
							control_msg.omega = -0.6

						elif (msg.x_go < msg.x_ob) and math.fabs(msg.x_go-msg.x_ob) < 50:  #Not meeting the above 2 conditions
							control_msg.v = 0.1
							control_msg.omega = -0.5

						elif (msg.x_go > msg.x_ob) and math.fabs(msg.x_go-msg.x_ob) < 50:  #Not meeting the above 2 conditions
							control_msg.v = 0.1
							control_msg.omega = -0.7

					elif (msg.x_go-125)<-20 :


						if (msg.x_go-msg.x_ob) < -50:

							control_msg.v = 0.2
							control_msg.omega = 0.6

						elif (msg.x_go-msg.x_ob) > 50:

							control_msg.v = 0.2
							control_msg.omega = 0.4

						elif (msg.x_go < msg.x_ob) and math.fabs(msg.x_go-msg.x_ob) < 50:  #Not meeting the above 2 conditions
							control_msg.v = 0.1
							control_msg.omega = 0.7

						elif (msg.x_go > msg.x_ob) and math.fabs(msg.x_go-msg.x_ob) < 50:  #Not meeting the above 2 conditions
							control_msg.v = 0.1
							control_msg.omega = 0.5

				elif math.fabs(msg.r_go-msg.r_ob) > 40:	


						if msg.r_go > 40:

							control_msg.v = 0.0
							control_msg.omega = 0.0
							print " I reached near the goal"
							print " Misson accomplished!! press 's' to stop"
	

						elif msg.r_ob > 40:

							control_msg.v = 0.05
							control_msg.omega = 0.8
							print " I am very near to the obstacle"	
							print " Scanning new direction for the goal"

			elif msg.r_go < 40:			

				if math.fabs(msg.x_go-125) < 20:
					v=0.4
					w=0
					print "goal is infront of me"	
					control_msg.v = .4
					control_msg.omega = 0
						
				elif (msg.x_go-125)<-20 and(msg.x_go-125)>-125 :
					#v=0.2
					#w=0.8
					print "goal is at the left of me"
					control_msg.v = .2
					control_msg.omega = 0.5
				elif (msg.x_go-125)>20 :
					#v=0.2
					#w=-0.5
					print "goal is at the right of me"
					control_msg.v = 0.2
					control_msg.omega = -0.5
			else:
				control_msg.v = 0.0
				control_msg.omega = 0.0
				print " I reached near the goal"
				print " Misson accomplished!! press 's' to stop"
		else:
			print "I can not find the goal in my FOV"
			print "I am searching for the goal"
			control_msg.v = 0.05
			control_msg.omega = 0.8

		
		
		self.pub_control.publish(control_msg)
		print (control_msg)
		






if __name__ == "__main__":
    rospy.init_node("goal_tracking_controller", anonymous=False)
    goal_tracking_controller = sub_class()
rospy.spin()
