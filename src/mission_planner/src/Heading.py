#! /usr/bin/env python
import rospy
from smach import StateMachine
from actions.msg import headingGoal, headingAction
import smach
from smach_ros import SimpleActionState

class Heading(smach.State):
    def __init__(self, HEADING, TASK):
        self.HEADING = HEADING
        self.TASK = TASK
        smach.State.__init__(self, outcomes=[self.TASK])


    def addHeadingAction(self, sm):
        sm.add('HEADING', \
                        SimpleActionState('headingServer', \
                        headingAction, \
                        goal_cb=self.headingCallback), \
                        transitions={'succeeded':self.TASK,\
                                    'preempted':'HEADING',\
                                    'aborted':'aborted'})
        '''
        else:
            smach.StateMachine.add('HEADING_CONCURRENT', \
                                SimpleActionState('headingServer', \
                                headingAction, \
                                goal_cb=self.headingCallback), \
                                transitions={'succeeded':self.TASK,\
                                            'preempted':'HEADING_CONCURRENT',\
                                            'aborted':'aborted'})
        '''


    def headingCallback(self, userdata, goal):
        rospy.loginfo('Executing State New Heading')
        headingOrder = headingGoal()
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

