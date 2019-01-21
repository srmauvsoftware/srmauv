#!/usr/bin/env python

import roslib; roslib.load_manifest('mission_planner')
import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
import time

'''
Is callback updating pressure?
Dynamic Reconfigure for Kx and Ky consts.
'''
class Torpedo(smach.State):
    # def __init__(self, TASK):
    def __init__(self):
        self.tPub = rospy.Publisher("/torpedo", Bool, queue_size=100)
        smach.State.__init__(self, outcomes=['torpedo_success'])

    def execute(self, ud):
        rospy.loginfo("doing torpedo")
        
        self.tPub.publish(Bool(True))
        return 'torpedo_success'