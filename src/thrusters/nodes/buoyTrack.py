#! /usr/bin/env python

import cv2
import numpy as np
import sys
import glob

from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16, Float64

class Vision:
    def __init__(self):
        self.imageSub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
        self.pressureSub = rospy.Subscriber("/pressure", Float64, self.pressureCallback)
        self.headingSub = rospy.Subscriber("/imu/Heading_degree/theta", Float64, self.headingCallback)
        self.bridge = CvBridge()
        self.cameraX = 640
        self.cameraY = 320
	self.xPub = rospy.Publisher("/x_setpoint", Float64, queue_size=1)
        self.center_x_pub = rospy.Publisher("/centerX", Float64, queue_size=1)
        self.center_y_pub = rospy.Publisher("/centerY", Float64, queue_size=1)
        self.yPub = rospy.Publisher("/y_setpoint", Float64, queue_size=1)
        self.imagePub = rospy.Publisher("/buoy", Image, queue_size=1)
        self.pressure = None
        self.heading = None
	self.trackerMedian = cv2.TrackerMedianFlow_create()
	self.trackerBoost = cv2.TrackerBoosting_create()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

	detecting = True
	tracking = False
	initTracker = False
        templates = glob.glob('/home/nvidia/Music/srmauv/src/thrusters/nodes/red_buoy_images/*.png')
        locs=[]
	ptx = 0.0
	pty = 0.0
        print("Callback")
	if detecting:
	    tracking = False
	    initTracker = False
	    for tem in templates:
	        template = cv2.imread(tem, 1)
		w, h = template.shape[:-1]
		res = cv2.matchTemplate(cv_image, template, cv2.TM_CCOEFF_NORMED)
		thresh = 0.65
		locs.append(np.where(res >= thresh))
	    ptf = []
	    if locs[0][0].any():
            	for loc in locs:
            	    for pt in zip(*loc[::-1]):
                    	ptf.append(pt)
                    	print("image found")

            	pt_len = len(ptf)

            	for x in ptf:
            	    ptx += x[0]
            	    pty += x[1]

            	ptx = int(ptx/pt_len)
            	pty = int(pty/pt_len)
            	print("POint UPdated")
            	cv2.rectangle(cv_image, pt, (ptx + w, pty + h), (0, 0, 255), 2)
            	cv2.line(cv_image, (ptx + (w/2), pty + (h/2)),(self.cameraX, self.cameraY),(255, 0, 0), 3)
		self.center_x_pub.publish(640)
                self.center_y_pub.publish(320)
                self.xPub.publish(ptx)
                self.yPub.publish(pty)
                print("setpoint published")
		tracking = True
		initTracker = True
	    
	if tracking:
	    if initTracker:
		bbox = (ptx, pty, 100, 100)
		ok = self.trackerMedian.init(cv_image, bbox)
		initTracker = False
	    else:
		ok, bbox1 = self.trackerMedian.update(cv_image)
		if ok:
		    p1 = (int[bbox1[0], int(bbox1[1])])
		    p2 = (int(bbox1[0] + bbox1[2]), int(bbox1[1] + bbox1[3]))
		    cv2.rectangle(cv_image, p1, p2, (255,0,0), 2, 1)
	self.imagePub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    def pressureCallback(self, data):
        self.pressure = data

    def headingCallback(self, data):
        self.heading = data


if __name__ == '__main__':
    Vision()
    rospy.init_node('buoyDetect')
    rospy.spin()
	
