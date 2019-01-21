#! /usr/bin/env python
import rospy
from smach import StateMachine
from actions.msg import depthGoal, depthAction
import smach
from smach_ros import SimpleActionState
import actionlib

class Depth(smach.State):
    def __init__(self, PRESSURE, TASK):
        self.PRESSURE = PRESSURE
        # self.smach_StateMachine = smach_StateMachine
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

    def execute(self, ud):
        rospy.loginfo("Executing State Concurrent Depth")
        client = actionlib.SimpleActionClient('depthServer',\
                                            depthAction)
        client.wait_for_server()
        goal = depthGoal(depth_setpoint=self.PRESSURE)
        client.send_goal(goal)
        client.wait_for_result()
        result = client.get_state()
        if result == -1:
            return 'DepthReached'
        elif result == 'preempted':
            return 'aborted'
        elif result == 'aborted':
            return 'aborted'
        else: return 'aborted'
