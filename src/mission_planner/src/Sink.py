#! /usr/bin/env python
import rospy
from actions.msg import depthGoal, depthAction
import smach

from Depth import Depth

class Sink:

    def __init__(self,smach_StateMachine, INITIAL_PRESSURE, TASK):
        self.INITIAL_PRESSURE = INITIAL_PRESSURE
        sm_sub = smach.StateMachine(outcomes=['depth_success', 'aborted'])
        with sm_sub:
            depthTask = Depth(INITIAL_PRESSURE, 'depth_success')
            depthTask.addDepthAction(smach_StateMachine)
        smach.StateMachine.add('Sink', sm_sub,
                               transitions={'depth_success':'IMAGETASK'})
        

    def sink_callback(self, userdata, goal):
        rospy.loginfo('Executing State Sink')
        depthOrder = depthGoal()
        depthOrder.depth_setpoint = self.INITIAL_PRESSURE
        return depthOrder
