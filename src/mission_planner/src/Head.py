#! /usr/bin/env python
import rospy
from actions.msg import depthGoal, depthAction
import smach

from Heading import Heading

class Head:
    def __init__(self, smach_StateMachine, goalHeading, TASK):
        self.goal = goalHeading
        sm_sub = smach.StateMachine(outcomes=['heading_success','aborted'])
        with sm_sub:
            headTask = Heading(self.goal, 'heading_success')
            headTask.addHeadingAction(smach_StateMachine)
        smach.StateMachine.add('Head', sm_sub, transitions={'heading_success':TASK})
