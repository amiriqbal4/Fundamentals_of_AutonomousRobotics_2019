#!/usr/bin/env python
import sys
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

class lf_ht_mp3():

	def __init__(self):
        # subscribe compressed image (CompressedImage) from duckiebot
		self.sub_image = rospy.Subscriber("~compressed", CompressedImage, self.cb_lf_ht, queue_size=1)
		self.bridge = CvBridge()
        # publish images afte Hough transform (yellow and white lines)
		self.pub_white_ht = rospy.Publisher("~white_ht_lf_edges", Image, queue_size=1)
		self.pub_yellow_ht = rospy.Publisher("~yellow_ht_lf_edges", Image, queue_size=1)

	def get_lines(self,original_image, filtered_image):
		# do our hough transform on the white image
		# resolution: 1 pixel radius, 1 degree rotational
		r_res = 1
		theta_res = np.pi/180
		# threshold: number of intersections to define a line
		thresh = 10
		# min_length: minimum number of points to form a line
		min_length = 5
		# max_gap: maximum gap between two points to be considered a line
		max_gap = 6
		lines = cv2.HoughLinesP(filtered_image, r_res, theta_res, thresh, np.empty(1), min_length, max_gap)
		output = np.copy(original_image)

		if lines is not None:
            # grab the first line
			for i in range(len(lines)):
				print(lines[i])
				l = lines[i][0]
				cv2.line(output, (l[0],l[1]), (l[2],l[3]), (0,0,255), 3, cv2.LINE_AA)
		return output

	def cb_lf_ht(self,image):
		# convert image to numpy array data
		im_array_np = np.fromstring(image.data, np.uint8)

        # cv2 image for further operation
		im_cv = cv2.imdecode(im_array_np, cv2.IMREAD_COLOR)

		# The incoming image is BGR format, convert it to HSV
		hsv = cv2.cvtColor(im_cv, cv2.COLOR_BGR2HSV)

		# Filter for only white pixels. Experiment with values as needed

		w1 = cv2.inRange(hsv, (0,0,130), (35,45,255))
		w2 = cv2.inRange(hsv, (35,0,130), (255,45,255))
		mask = cv2.bitwise_or(w1, w2) # white not yellow!
		white_filter = mask #cv2.bitwise_and(image,image, mask=mask)

		#white_filter = cv2.inRange(hsv, (0,50,0), (255,180,255))
		#cv2.imshow("White Filter", white_filter)
		#cv2.imwrite("white_filter.png", white_filter)

		# Filter for only yellow pixels. Experiment with values as needed
		yellow_filter = cv2.inRange(hsv, (18,110,120), (42,255,255))

		#cv2.imshow("Yellow Filter", yellow_filter)
		#cv2.imwrite("yellow_filter.png", yellow_filter)

		# Create a kernel to dilate the image.
		# Experiment with the numbers in parentheses (optional)

		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))

		# Dilate both the white and yellow images.
		# No need to experiment here.

		white_dilate = cv2.dilate(white_filter, kernel)
		yellow_dilate = cv2.dilate(yellow_filter, kernel)

		#cv2.imshow("Dilate White", white_dilate)
		#cv2.imwrite("white_dilate.png", white_dilate)
		#cv2.imshow("Dilate Yellow", yellow_dilate)
		#cv2.imwrite("yellow_dilate.png", yellow_dilate)
		#Perform edge detection on the original image.
        
		#Experiment with the first two numbers. Aperture size experimentation optional
		edges = cv2.Canny(im_cv, 0, 300, apertureSize=3)

		# Use the edges to refine the lines in both white and yellow images
		# No need to experiment here

		white_edges = cv2.bitwise_and(white_dilate, edges)
		yellow_edges = cv2.bitwise_and(yellow_dilate, edges)

		#cv2.imshow("Yellow Edges", yellow_edges)
		#cv2.imwrite("yellow_edges.png", yellow_edges)
		#cv2.imshow("White Edges", white_edges)
		#cv2.imwrite("white_edges.png", white_edges)

		white_output = self.get_lines(im_cv, white_edges)
		yellow_output = self.get_lines(im_cv, yellow_edges)

		#cv2.imshow("White Output", white_output)
		#cv2.imwrite("white_output.png", white_output)
		#cv2.imshow("Yellow Output", yellow_output)
		#cv2.imwrite("yellow_output.png", yellow_output)

		try:
			self.pub_white_ht.publish(self.bridge.cv2_to_imgmsg(white_output, "bgr8"))
			self.pub_yellow_ht.publish(self.bridge.cv2_to_imgmsg(yellow_output, "bgr8"))
			print("I am okay!")			
		except CvBridgeError as e:
			print("CvBridgeError_a")
			print(e)

		# Wait for key press to close images
		#cv2.waitKey()


if __name__ == "__main__":

	rospy.init_node("amir_ht_LF", anonymous=False)  # adapted to sonjas default file

	amir_ht_LF = lf_ht_mp3()

rospy.spin()
