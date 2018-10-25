#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import os

class helper:
    def __init__(self):
        rospy.Rate(10)
        rospy.Subscriber('/launcher', String, self.launcherCb)

    def launcherCb(self, data):
        rospy.loginfo(data)
        

if __name__ == '__main__':
    try:
        rospy.init_node('helper', anonymous=True)
        thruster = helper()
        rospy.spin()

    except rospy.ROSInterruptException: pass
