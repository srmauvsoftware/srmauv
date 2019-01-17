#! /usr/bin/env python
import rospy
import smach

import numpy as np
import cv2
from matplotlib import pyplot as plt
from cv_bridge import CvBridge, CvBridgeError

from Depth import Depth
from Heading import Heading
# from darknet_ros_msgs.msg import BoundingBox
from std_msgs.msg import Float64
from sensor_msgs.msg import Image

class ImageTask():
    def __init__(self, smach_StateMachine, TASK):
        self.bridge = CvBridge()
        self.TASK = TASK
        self.depth_pub = rospy.Publisher('/depth', Float64, queue_size=10)
        self.heading_pub = rospy.Publisher('/heading', Float64, queue_size=10)
        rospy.Subscriber('/camera/image_raw', Image, self.callback)
        # rospy.Subscriber('/BoundingBox', BoundingBox, self.callback)
        sm_sub = smach.Concurrence(outcomes = ['DepthHeadingReached', 'DepthHeadingFailed'],
                                                default_outcome='DepthHeadingFailed',
                                                outcome_map={'DepthHeadingReached':
                                                {'DEPTH_CONCURRENT':'DepthReached',
                                                'HEADING_CONCURRENT':'HeadingReached'}})

        with sm_sub:
            smach.Concurrence.add('DEPTH_CONCURRENT', Depth(160, 'depth_success'))
            smach.Concurrence.add('HEADING_CONCURRENT', Heading(320, 'heading_success'))
        
        smach.StateMachine.add('IMAGETASK', sm_sub, transitions={
          'DepthHeadingFailed': 'IMAGETASK',
          'DepthHeadingReached': self.TASK
        })

    # def callback(data):
    #     self.box_centre_x = (data.xmax - data.xmin)/2
    #     self.box_centre_y = (data.ymax - data.ymin)/2
    #     self.depth_pub.publish(box_centre_y)
    #     self.heading_pub.publish(box_centre_x)

    def callback(self, data):
        rospy.loginfo("sub")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)
        kernel = np.ones((3,3),np.uint8)
        opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)
        # sure background area
        sure_bg = cv2.dilate(opening,kernel,iterations=3)
        # Finding sure foreground area
        dist_transform = cv2.distanceTransform(opening,cv2.DIST_L2,5)
        ret, sure_fg = cv2.threshold(dist_transform,0.7*dist_transform.max(),255,0)
        # Finding unknown region
        sure_fg = np.uint8(sure_fg)
        unknown = cv2.subtract(sure_bg,sure_fg)
        # Marker labelling
        ret, markers = cv2.connectedComponents(sure_fg)
        # Add one to all labels so that sure background is not 0, but 1
        markers = markers+1
        # Now, mark the region of unknown with zero
        markers[unknown==255] = 0
        markers = cv2.watershed(cv_image,markers)
        cv_image[markers == -1] = [255,0,0]
        

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        
        
