#! /usr/bin/env python

import rospy
import actionlib
import time
from std_msgs.msg import Int16, Float64, Bool
import actions.msg
import subprocess
from thrusters.msg import VectorThrusterMsg

class swayAction(object):
    _feedback = actions.msg.timeFeedback()
    _result = actions.msg.timeResult()

    def __init__(self, name):
        self._da = name
        self.setpoint_pub = rospy.Publisher('/sway_scale_setpoint', Float64, queue_size=10)
        self.state_pub = rospy.Publisher('/sway_scale', Float64, queue_size=10)
        self.K = 1
        self._ds = actionlib.SimpleActionServer(
            self._da, \
            actions.msg.timeAction, \
            execute_cb = self.scaleCallback, \
            auto_start = False)
        self._ds.start()

    def scaleCallback(self, goal):
        self.currentState = 0
        success = False

        while True:
            self.state_pub.publish(Float64(self.currentState))
            self.setpoint_pub.publish(Float64(goal.time_setpoint))
            distance = abs(goal.time_setpoint - self.currentState)
            self._ds.publish_feedback(self._feedback)
            rospy.loginfo('%s : Going to Sway %f with Error : %f',\
                self._da, \
                goal.time_setpoint, \
                self._feedback.time_error)
            self._feedback.time_error = distance

            #Logic
            time.sleep(1)
            if goal.time_setpoint < 0:
                self.currentState = self.currentState - self.K
            elif goal.time_setpoint > 0:
                self.currentState = self.currentState + self.K
            if int(goal.time_setpoint) == int(self.currentState):
                rospy.loginfo("Goal Complete")
                success = True
                break

            if self._ds.is_preempt_requested():
                rospy.loginfo('%s : Preempted' % self._da)
                self._ds.set_preempted()
                success = False
                break

        if success:
            self._result.time_final = self._feedback.time_error
            rospy.loginfo('%s : Success' % self._da)
            self._ds.set_succeeded(self._result)

if __name__ == '__main__':
      rospy.init_node('swayServer')
      server = swayAction(rospy.get_name())
      rospy.spin()
