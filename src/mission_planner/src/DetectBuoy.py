#!/usr/bin/env python

from std_msgs.msg import Float64
import roslib; roslib.load_manifest('mission_planner')
import rospy
import smach
import smach_ros
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
import math
from actions.msg import depthGoal, depthAction
from actions.msg import timeGoal, timeAction

'''
Is callback updating pressure?
Dynamic Reconfigure for Kx and Ky consts.
'''
class DetectBuoy(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['buoy_success','buoy_retry'])
        self.buoySuccess = False
        self.pressure = 0
        self.Kx = 0.1
        self.Ky = 0.05

    def pressureCallback(self, data):
        self.pressure = data.data

    def execute(self, ud):
        rospy.Subscriber("/BoundingBox", BoundingBoxes, self.bbCallback)
        rospy.Subscriber("/depth", Float64, self.pressureCallback)
        if self.buoySuccess:
            return 'buoy_success'
        else:
            return 'buoy_retry'

    def distance(self, buoy):
        return math.sqrt((buoy['x'] - 640)**2 + (buoy['y'] - 320)**2)

    def nearestBuoy(self, buoybbs):
        nearestBuoy_ = None
        minDistance = 1290
        for buoy in buoybbs:
            distance = self.distance(buoy)
            if (distance < minDistance):
                minDistance = distance
                nearestBuoy_ = buoy
        return nearestBuoy_
    
    def send_goal(self, server, actionName, actionGoal, goal):
        client = actionlib.SimpleActionClient(server, actionName)
        client.wait_for_server()
        client.send_goal(actionGoal)
        client.wait_for_result()
        result = client.get_state()
        if result == -1:
            return server+'Reached'
        elif result == 'preempted':
            return 'aborted'
        elif result == 'aborted':
            return 'aborted'
        else: return 'aborted'

    def bbCallback(self, data):
        rospy.loginfo("BBox callback")
	buoybbs = []
        bbs = data.bounding_boxes
        for bb in bbs:
            bb = BoundingBox()
            if bb['class'] == 0 and bb['probability'] > 0.5:
                buoybb = {'class': bb['class'], 
                          'probability': bb['probability'], 
                          'x': bb['x'], 'y': bb['y'],
                          'w': bb['w'], 'h': bb['h']}
                buoybbs.append(buoybb)
	
        if buoybbs:
            nearestBuoy = self.nearestBuoy(buoybbs)
	    rospy.loginfo("nearest buoy")
	    rospy.loginfo(nearestBuoy)
            goalX = (nearestBuoy['x'] - 640) * self.Kx
            goalY = (nearestBuoy['y'] - 320) * self.Ky + self.pressure 
            resultX = self.send_goal('depthServer', depthAction, depthGoal(depth_setpoint=goalY))
            resultY = self.send_goal('swayServer', timeAction, timeGoal(time_setpoint=goalX))
            if resultX == "depthServerReached" and resultY == "swayServerReached":
                self.buoySuccess = True
