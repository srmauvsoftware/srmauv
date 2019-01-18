#! /usr/bin/env python

import rospy
import actionlib
import time_
from std_msgs.msg import Int16, Float64
import actions.msg

class forwardAction(object):
    _feedback = actions.msg.timeFeedback()
    _result = actions.msg.timeResult()

    def __init__(self, name):
        self._da = name
        self.setpoint_pub = rospy.Publisher('/sway_time_setpoint', Float64, queue_size=10)
        self.state_pub = rospy.Publisher('/sway_time', Float64, queue_size=10)

        self._ds = actionlib.SimpleActionServer(
            self._da, \
            actions.msg.timeAction, \
            execute_cb = self.timeCallback, \
            auto_start = False)
        self._ds.start()

    def timeCallback(self, goal):
        epoch_setpoint = int(time.time()) + goal.time_setpoint
        self.setpoint_pub.publish(epoch_setpoint)

        while(int(time.time()) != epoch_setpoint):
            self.state_pub.publish(int(time.time()))

        #self._result.time_final = self._feedback.time_error
        #self.state_pub.publish(Float64(time_setpoint))
        rospy.loginfo('%s : Success' % self._da)
        self._ds.set_succeeded(self._result)


if __name__ == '__main__':
      rospy.init_node('swayServer')
      server = forwardAction(rospy.get_name())
      rospy.spin()
