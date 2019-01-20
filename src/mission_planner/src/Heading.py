#! /usr/bin/env python
import rospy
from smach import StateMachine
from actions.msg import headingGoal, headingAction
import smach
from smach_ros import SimpleActionState
import actionlib

class Heading(smach.State):
    def __init__(self, smach_StateMachine, NAME, HEADING, TASK):
        self.HEADING = HEADING
        self.smach_StateMachine = smach_StateMachine
        self.TASK = TASK
        self.name = NAME
        smach.State.__init__(self, outcomes=[self.TASK])
        self.smach_StateMachine.add(self.name, \
                        SimpleActionState('headingServer', \
                        headingAction, \
                        goal_cb=self.headingCallback), \
                        transitions={'succeeded':self.TASK,\
                                    'preempted': self.name,\
                                    'aborted':'aborted'})
        
    def headingCallback(self, userdata, goal):
        rospy.loginfo('Executing State New Heading')
        headingOrder = headingGoal()
        headingOrder.heading_setpoint = self.HEADING
        return headingOrder

    def execute(self, ud):
        rospy.loginfo("Executing State Concurrent Heading")
        client = actionlib.SimpleActionClient('headingServer',\
                                            headingAction)
        client.wait_for_server()
        goal = headingGoal(heading_setpoint=self.HEADING)
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_result()
        print(result)
        if result == -1:
            return 'HeadingReached'
        elif result == 'preempted':
            return 'aborted'
        elif result == 'aborted':
            return 'aborted'
        else: return 'aborted'

