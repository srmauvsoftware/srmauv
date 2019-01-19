import cv2
import numpy as np
import sys
import glob

from sensor_msgs.msg import Image
import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int16

class Vision:
    def __init__(self):
        self.imageSub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
        self.pressureSub = rospy.Subscriber("/pressure", Float64, self.pressureCallback)
        self.headingSub = rospy.Subscriber("/imu/Heading_degree/theta", Float64, self.headingCallback)
        self.bridge = CvBridge()
        self.cameraX = 640
        self.cameraY = 320
        self.xPub = rospy.Publisher("/x_setpoint", Int16, queue_size=1)
        self.yPub = rospy.Publisher("/y_setpoint", Int16, queue_size=1)
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

        templates = glob.glob('*.png')
        for template in templates:
            tem = cv2.imread(template, 1)
            w, h = template.shape[:-1]
            res = cv2.matchTemplate(cv_image, template, cv2.TM_CCOEFF_NORMED)
            thresh = 0.65
            loc = np.where(res >= thresh)

        for pt in zip(*loc[::-1]):
            cv2.rectangle(img_rgb, pt, (pt[0] + w, pt[1] + h), (0, 0, 255), 2)
            cv2.line(cv_image, (pt[0] + (w/2), pt[1] + (h/2)),(self.cameraX, self.cameraY),(255, 0, 0), 3)
            x, y = self.findOffsets(pt[0] + (w/2), pt[1] + (h/2))
            self.xPub.publish(x)
            self.yPub.publish(y)
            self.imagePub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    def pressureCallback(self, data):
        self.pressure = data

    def headingCallback(self, data):
        self.heading = data


if __name__ == '__main__':
    Vision()
