#!/usr/bin/env python
from sensor_msgs.msg import Image, CompressedImage
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

#Define HSV colour range for certatn colour objects
#yellowLower = (20,110,120)
#yellowUpper = (45,255,255)
#orangeLower = (10,100, 50)
#orangeUpper = (25, 255, 255)

greenLower = (50,120, 80)
greenUpper = (80, 255, 255)

blueLower = (100,120, 100)
blueUpper = (140, 255, 255)


class cv_class:
    def __init__(self):
        self.pub_obstacle = rospy.Publisher("~obstacle_image", Image, queue_size=1)
	self.pub_goal = rospy.Publisher("~goal_image", Image, queue_size=1)

	self.pub_msg = rospy.Publisher("/processed_image_msg", awesome, queue_size=1)
    	#self.pub_go_x = rospy.Publisher("goal_center_x", awesome, queue_size=1)
	#self.pub_go_y = rospy.Publisher("goal_center_y", awesome, queue_size=1)

	#self.pub_ob_x = rospy.Publisher("ob_center_x", awesome, queue_size=1)
	#self.pub_ob_y = rospy.Publisher("ob_center_y", awesome, queue_size=1)

	#self.pub_ob_radius = rospy.Publisher("object_radius", Int16, queue_size=1)
	#self.pub_go_radius = rospy.Publisher("goal_radius", Int16, queue_size=1)

        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("~compressed", CompressedImage, self.cv_image, queue_size=1)
        self.rate = rospy.Rate(10) # 10hz
	self.message = awesome()
	

    # image color segmentation
    def image_filter(self, image, colorLower, colorUpper, flag):

	buffer = 20
	pts = deque(maxlen = buffer)
	counter = 0
	(dX, dY) = (0, 0)

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        color_filter = cv2.inRange(hsv, colorLower, colorUpper)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        color_erode = cv2.erode(color_filter, kernel, iterations=2)
        color_dilate = cv2.dilate(color_erode, kernel, iterations=2)

        #Find all contours in the masked image
        _,cnts,_ = cv2.findContours(color_dilate.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        #Define center of the ball to be detected as None
        center = None
        x_coor = -1
        y_coor = -1
        radius = -1.0

        #If any object is detected, then only proceed
        if(len(cnts) > 0):
            #Find the contour with maximum area
            c = max(cnts, key = cv2.contourArea)
            #Find the center of the circle, and its radius of the largest detected contour.
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            #Calculate the centroid of the ball, as we need to draw a circle around it.
            M = cv2.moments(c)

            x_coor = int(M['m10'] / M['m00'])
    	    y_coor = int(M['m01'] / M['m00'])
            center = (x_coor, y_coor)

            #Proceed only if a ball of considerable size is detected
            if radius > 5: #draw the circle and the center
		if flag:                
			cv2.circle(image, (int(x), int(y)), int(radius), (0,10,255), 2)
		        cv2.circle(image, center, 5, (0,10,255), -1)
		else:
			cv2.circle(image, (int(x), int(y)), int(radius), (0,255,255), 2)
		        cv2.circle(image, center, 5, (0,255,255), -1)

                #Append the detected object in the frame to pts deque structure
                pts.appendleft(center)

        #Using numpy arange function for better performance. Loop till all detected points
        for i in np.arange(1, len(pts)):
            #If no points are detected, move on.
            if(pts[i-1] == None or pts[i] == None):
                continue

            #If atleast 10 frames have direction change, proceed
            if counter >= 10 and i == 1 and pts[-10] is not None:
                #Calculate the distance between the current frame and 10th frame before
                dX = pts[-10][0] - pts[i][0]
                dY = pts[-10][1] - pts[i][1]
                (dirX, dirY) = ('', '')

                #If distance is greater than 100 pixels, considerable direction change has occured.
                if np.abs(dX) > 100:
                    dirX = 'West' if np.sign(dX) == 1 else 'East'

                if np.abs(dY) > 100:
                    dirY = 'North' if np.sign(dY) == 1 else 'South'

                #Set direction variable to the detected direction
                direction = dirX if dirX != '' else dirY

            #Draw a trailing red line to depict motion of the object.
            thickness = int(np.sqrt(buffer / float(i + 1)) * 2.5)
            cv2.line(image, pts[i - 1], pts[i], (0, 0, 255), thickness)

        return image, center, x_coor, y_coor, radius


    def cv_image(self, image_msg):
        np_arr = np.fromstring(image_msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        obstacle, ob_center, ob_x, ob_y, ob_radius = self.image_filter(image_np,blueLower,blueUpper,0)
	goal, go_center, go_x, go_y, go_radius = self.image_filter(image_np,greenLower,greenUpper,1)

	print "obstacle center:", ob_center
	print "goal center:", go_center
	#print type(ob_x)
	#print type(ob_y)
	#print type(ob_radius)
	#print type(ob_center)

	
	
	self.message.x_ob = ob_x
	self.message.y_ob = ob_y
	self.message.r_ob = ob_radius
	self.message.x_go = go_x
	self.message.y_go = go_y
	self.message.r_go = go_radius
	#print "Goal is not detected"

	self.pub_msg.publish(self.message)

        try:
            self.pub_obstacle.publish(self.bridge.cv2_to_imgmsg(obstacle, "bgr8"))  #8UC1

            self.pub_goal.publish(self.bridge.cv2_to_imgmsg(goal, "bgr8"))  #8UC1
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    rospy.init_node("obstacle_and_goal_filter", anonymous=False)
    obstacle_and_goal_filter = cv_class()
rospy.spin()
