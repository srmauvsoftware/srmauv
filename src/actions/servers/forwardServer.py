#! /usr/bin/env python

import rospy
import actionlib
import time
from std_msgs.msg import Int16, Float64, Bool
import actions.msg
import subprocess

class forwardAction(object):
    _feedback = actions.msg.timeFeedback()
    _result = actions.msg.timeResult()

    def __init__(self, name):
        self._da = name
        self.headingControllerToggle = rospy.Publisher('/heading_controller/pid_enable', Bool, queue_size=10)
        self.setpoint_pub = rospy.Publisher('/time_setpoint', Float64, queue_size=10)
        self.state_pub = rospy.Publisher('/time', Float64, queue_size=10)
        self._ds = actionlib.SimpleActionServer(
            self._da, \
            actions.msg.timeAction, \
            execute_cb = self.timeCallback, \
            auto_start = False)
        self._ds.start()

    def timeCallback(self, goal):

        self.currentTime = int(time.time())
        time_setpoint = goal.time_setpoint
        epoch_setpoint = self.currentTime + time_setpoint

        while (int(time.time()) != epoch_setpoint):
            self.headingControllerToggle.publish(Bool(False))
            print("heading off")
            self.setpoint_pub.publish(Float64(time_setpoint))
            if self._ds.is_preempt_requested():
                rospy.loginfo('%s : Preempted' % self._da)
                self._ds.set_preempted()
                success = False
                break
            remaining_time = epoch_setpoint - int(time.time())
            self.state_pub.publish(Float64(goal.time_setpoint - remaining_time))
            # self._feedback.time_error = remaining_time
            # self._ds.publish_feedback(remaining_time)
            rospy.loginfo("Remaining time: %s" % remaining_time)

        # if success:
        self.headingControllerToggle.publish(Bool(True))
        self._result.time_final = self._feedback.time_error
        rospy.loginfo('%s : Success' % self._da)
        self._ds.set_succeeded(self._result)

if __name__ == '__main__':
      rospy.init_node('forwardServer')
      server = forwardAction(rospy.get_name())
      rospy.spin()
