#! usr/bin/env python

import rospy
import smach
from smach_ros import SimpleActionState
import actions.msg

class Forward:

    def __init__(self, smach_StateMachine, TIME, TASK):
      self.TIME_VALUE = TIME
      smach_StateMachine.add('FORWARD', \
          SimpleActionState('forwardServer', \
          actions.msg.timeAction, \
          goal_cb=self.goal_callback), \
          transitions={
            'succeeded': TASK,
            'preempted': 'FORWARD',
            'aborted': 'aborted'
          })

    def goal_callback(self, userdata, goal):
        rospy.loginfo("Executing State Forward")
        timeOrder = actions.msg.timeGoal()
        timeOrder.time_setpoint = self.TIME_VALUE
        return timeOrder
