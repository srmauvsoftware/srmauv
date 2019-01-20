#!/usr/bin/env python
import cv2
import numpy as np
import math

from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16, Float64

bridge = CvBridge()

def callback(data): 
	try:
		img=bridge.imgmsg_to_cv2(data, "bgr8")
	except CvBridgeError as e:
		print(e)

	h, w, c = img.shape

	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	blur = cv2.GaussianBlur(hsv,(5,5),0)

	lower_red = np.array([0,0,0])
	upper_red = np.array([70,220,200])

	mask = cv2.inRange(blur, lower_red, upper_red)
	# res = cv2.bitwise_and(imgb, imgb, mask= mask)
	# res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
	# res = cv2.threshold(res, 0, 150, cv2.THRESH_BINARY)

	kernel = np.ones((4,4),np.uint8)
	res = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
	im2, contours, hierarchy = cv2.findContours(res, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	cnt = contours[0]

	M = cv2.moments(cnt)

	cx = int(M['m10']/M['m00'])
	cy = int(M['m01']/M['m00'])

	cv2.circle(img,(cx,cy), 4, (0,0,255), -1)
	cv2.circle(img,(int(w/2),int(h/2)), 4, (0,0,255), -1)
	# cv2.line(img,(cx,cy),(int(w/2), int(h/2)),(255,0,0),2)

	cv2.circle(img,(w,h), 4, (0,0,255), -1)
	# cv2.circle(img,(w,0), 4, (0,0,255), -1)
	# cv2.circle(img,(0,h), 4, (0,0,255), -1)
	cv2.circle(img,(0,0), 4, (0,0,255), -1)

	# cv2.drawContours(img, contours, -1, (0,255,0), 3)   
	rect = cv2.minAreaRect(cnt)
	(x,y),(width,height),theta = cv2.minAreaRect(cnt)
	box = cv2.boxPoints(rect)
	box = np.int0(box)

	angPub.publish(theta)

	cv2.drawContours(img,[box],0,(0,0,255),2)
	pt1 = (box[0][0], box[0][1])
	pt4 = (box[3][0], box[3][1])
	pt2 = (box[1][0], box[1][1])
	pt3 = (box[2][0], box[2][1])

	cv2.circle(img, pt1, 4, (100,205,220), 2)
	cv2.circle(img, pt2, 4, (100,205,220), 2)
	cv2.circle(img, pt3, 4, (100,205,220), 2)
	cv2.circle(img, pt4, 4, (100,205,220), 2)

	x1,y1 = (pt4[0] + pt1[0])/2.0, (pt4[1] + pt1[1])/2.0
	x2,y2 = (pt2[0] + pt3[0])/2.0, (pt2[1] + pt3[1])/2.0

	cv2.line(img,(int(x1),int(y1)),(int(x2),int(y2)),(255,0,0),2)
	cv2.line(img,(int(w/2),0),(int(w/2),h),(255,0,0),2)

	pt1 = (box[0][0], h-box[0][1])
	pt4 = (box[3][0], h-box[3][1])
	pt2 = (box[1][0], h-box[1][1])
	pt3 = (box[2][0], h-box[2][1])

	x1,y1 = (pt4[0] + pt1[0])/2.0, (pt4[1] + pt1[1])/2.0
	x2,y2 = (pt2[0] + pt3[0])/2.0, (pt2[1] + pt3[1])/2.0

	#angle = float(y2-y1)/float(x2-x1)
	#angle = math.atan2(y2-y1,x2-x1)

	imagePub.publish(bridge.cv2_to_imgmsg(img, "bgr8"))

if __name__=='__main__':
	rospy.init_node('pathDetect')
	angPub=rospy.Publisher('/heading_setpoint', Float64, queue_size=1)
	imagePub = rospy.Publisher("/path", Image, queue_size=1)
	imgSub = rospy.Subscriber('microsoft/image_raw', Image, callback)
	rospy.spin()

