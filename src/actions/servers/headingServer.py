#! /usr/bin/env python
import rospy
import actionlib
import actions.msg
from std_msgs.msg import Float64
import math
import time

class headingAction(object):
    _feedback = actions.msg.headingFeedback()
    _result = actions.msg.headingResult()

    def __init__(self, name):
        self.pub = rospy.Publisher('/heading_setpoint', Float64, queue_size=10)
        rospy.Subscriber("/imu/Heading_degree/theta", Float64, self.heading_cb)
        self._ha = name
        self._hs = actionlib.SimpleActionServer(
            self._ha, \
            actions.msg.headingAction, \
            execute_cb = self.headingCallback, \
            auto_start = False)
        self._hs.start()
	#self.heading_value

    def heading_cb(self, data):
	if data.data is None:
            return
        self.heading_value = data.data

    def headingCallback(self, goal):
        success = False
        successt = False
        while True:
            self.pub.publish(goal.heading_setpoint)
            self._feedback.heading_error = self.heading_value
            self._hs.publish_feedback(self._feedback)
            self._feedback.heading_error = self.heading_value - goal.heading_setpoint
            rospy.loginfo('%s : Going to Heading %f with Error : %f',\
                self._ha, \
                goal.heading_setpoint, \
                self._feedback.heading_error)

            # Logic
            start = int(time.time())
            while(abs(goal.heading_setpoint - self.heading_value) < 5):
                if(int(time.time()) == start + 5):
                    successt = True
                    rospy.loginfo("10 sec over")
                    break
            if(successt):
                success = True
                break

            if self._hs.is_preempt_requested():
                rospy.loginfo('%s : Preempted' % self._ha)
                self._hs.set_preempted()
                success = False
                break

        if success:
            self._result.heading_final = self._feedback.heading_error
            rospy.loginfo('%s : Success' % self._ha)
            self._hs.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('headingServer')
    server = headingAction(rospy.get_name())
    rospy.spin()
