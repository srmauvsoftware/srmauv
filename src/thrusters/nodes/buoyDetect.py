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

    def findOffsets(self, x, y):
        if( x < self.cameraX and y < self.cameraY ):
            offsetX = self.cameraX - x
            offsetY = self.cameraY - y
        elif( x > self.cameraX and y < self.cameraY ):
            offsetX = x - self.cameraX
            offsetY = self.cameraY - y
        elif( x < self.cameraX and y > self.cameraY):
            offsetX = self.cameraX - x
            offsetY = y - self.cameraY
        elif( x > self.cameraY and y > self.cameraY):
            offsetX = x - self.cameraX
            offsetY = y - self.cameraY
        return offsetX, offsetY

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        templates = glob.glob('/home/nvidia/Music/srmauv/src/thrusters/nodes/red_buoy_images/*.png')
        locs=[]
	print("Callback")
        for tem in templates:
            template = cv2.imread(tem, 1)
            w, h = template.shape[:-1]
            res = cv2.matchTemplate(cv_image, template, cv2.TM_CCOEFF_NORMED)
            thresh = 0.65
            locs.append(np.where(res >= thresh))    

        ptf = []

        for loc in locs:
            for pt in zip(*loc[::-1]):
                ptf.append(pt)
		print("image found")
        
        pt_len = len(ptf)
        ptx = 0.0
        pty = 0.0

        for x in ptf:
            ptx += x[0]
            pty += x[1]

        ptx = int(ptx/pt_len)
        pty = int(pty/pt_len)
	print("POint UPdated")
        cv2.rectangle(cv_image, pt, (ptx + w, pty + h), (0, 0, 255), 2)
        cv2.line(cv_image, (ptx + (w/2), pty + (h/2)),(self.cameraX, self.cameraY),(255, 0, 0), 3)
        #x, y = self.findOffsets(ptx + (w/2), pty + (h/2))
	if ptx:
		self.center_x_pub.publish(640)
		self.center_y_pub.publish(320)
		self.xPub.publish(ptx)
		self.yPub.publish(pty)
		print("setpoint published")
	print("Image published")
        self.imagePub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        #for pt in zip(*loc[::-1]):
        #    print("POint found")
        #    cv2.rectangle(cv_image, pt, (pt[0] + w, pt[1] + h), (0, 0, 255), 2)
        #    cv2.line(cv_image, (pt[0] + (w/2), pt[1] + (h/2)),(self.cameraX, self.cameraY),(255, 0, 0), 3)
        #    x, y = self.findOffsets(pt[0] + (w/2), pt[1] + (h/2))
        #    self.xPub.publish(x)
        #    print("Setpoint Sent")
        #    self.yPub.publish(y)
        #    self.imagePub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    def pressureCallback(self, data):
        self.pressure = data

    def headingCallback(self, data):
        self.heading = data


if __name__ == '__main__':
    Vision()
    rospy.init_node('buoyDetect')
    rospy.spin()
