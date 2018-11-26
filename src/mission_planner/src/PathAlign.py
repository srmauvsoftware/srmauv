#! /usr/bin/env python
import rospy
from smach import StateMachine
from actions.msg import headingGoal, headingAction
from opencv_apps import *
from opencv_apps.msg import RotatedRectArrayStamped
import smach
from smach_ros import SimpleActionState

class PathAlign(smach.State):
    def __init__(self, smach_StateMachine, TASK):
        self.TASK = TASK
        self.smach_StateMachine = smach_StateMachine
        self.rectSub = rospy.Subscriber("/general_contours/rectangles", RotatedRectArrayStamped, self.rectangleCb)
        smach.State.__init__(self, outcomes=[self.TASK])
        self.smach_StateMachine.add('PATHALIGN', \
                        SimpleActionState('headingServer', \
                        headingAction, \
                        goal_cb=self.headingCallback), \
                        transitions={'succeeded':self.TASK,\
                                    'preempted':'PATHALIGN',\
                                    'aborted':'aborted'})


    def rectangleCb(self, data):
        if (data.rects):
            self.rects = data.rects
        # self.rectSub.unregister()
        # print("Called")

    def headingCallback(self, userdata, goal):
        rospy.loginfo('Executing State New Heading')
        headingOrder = headingGoal()
        for i in self.rects:
            if (i.angle != -90 and i.angle != 0):
                self.HEADING = i.angle
                headingOrder.heading_setpoint = self.HEADING
                return headingOrder

    def execute(self, ud):
        rospy.loginfo("Executing State Concurrent Heading")
        client = actionlib.SimpleActionClient('headingServer',\
                                            actions.msg.headingAction)
        client.wait_for_server()
        goal = actions.msg.headingGoal(heading_setpoint=self.HEADING)
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
