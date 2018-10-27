#! /usr/bin/env python
import roslib; roslib.load_manifest('mission_planner')
import rospy
from smach import StateMachine
from actions.msg import depthGoal, depthAction
import actions.msg
from rospy.exceptions import ROSInitException
from rospy.timer import Rate
import smach
import actionlib
from smach_ros import SimpleActionState
import time

class Depth(smach.State):
    def __init__(self, PRESSURE, TASK):
        self.PRESSURE = PRESSURE
        self.TASK = TASK
        smach.State.__init__(self, outcomes=[self.TASK])

    def addDepthAction(self, sm):
        sm.add('DEPTH', \
                        SimpleActionState('depthServer', \
                        depthAction, \
                        goal_cb=self.depthCallback), \
                        transitions={'succeeded':self.TASK,\
                                    'preempted':'DEPTH',\
                                    'aborted':'aborted'})
        '''
        else:
            sm.add('DEPTH_CONCURRENT', \
                                SimpleActionState('depthServer', \
                                depthAction, \
                                goal_cb=self.depthCallback), \
                                transitions={'succeeded':self.TASK,\
                                            'preempted':'DEPTH_CONCURRENT',\
                                            'aborted':'aborted'})
        '''


    def depthCallback(self, userdata, goal):
        rospy.loginfo('Executing State New Depth')
        depthOrder = depthGoal()
        depthOrder.depth_setpoint = self.PRESSURE
        return depthOrder
