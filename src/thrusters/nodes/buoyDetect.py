#!/usr/bin/env python
import cv2
import numpy as np
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64
import numpy as np
class BuoyDetect():
	def __init__(self):
		self.bridge = CvBridge()
		rospy.Subscriber('/camera/image_raw', Image, self.callback)
		self. headingPub = rospy.Publisher('/heading_setpoint', Float64, queue_size=10)
		self.depthPub = rospy.Publisher('/depth_setpoint', Float64, queue_size=10)
		self.imagePub = rospy.Publisher("/buoy", Image)
	
	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		template = cv2.imread('/home/nvidia/srmauv/src/1.png')
		w, h = template.shape[:-1]
		res = cv2.matchTemplate(cv_image, template, cv2.TM_CCOEFF_NORMED)
		threshold = 0.65
		dataSent = 0
		loc = np.where(res >= threshold)
		for pt in zip(*loc[::-1]):
			cv2.rectangle(cv_image, pt, (pt[0] + w, pt[1] + h), (0, 0, 255), 2)
			rospy.loginfo((pt[0]))
			rospy.loginfo((pt[1]))		

if __name__ == '__main__':
	try:
		rospy.init_node('buoy')
		buoy = BuoyDetect()
		rospy.spin()
	except rospy.ROSInterruptException: pass
