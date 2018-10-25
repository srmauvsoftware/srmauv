#! /usr/bin/env python

import rospy
import actionlib
import actions.msg
from std_msgs.msg import Float64

class depthAction(object):
    _feedback = actions.msg.depthFeedback()
    _result = actions.msg.depthResult()

    def __init__(self, name):
        self.pub = rospy.Publisher('/depth_setpoint', Float64, queue_size=10)
        rospy.Subscriber("/depth", Float64, self.depth_cb)
        self._da = name
        self._ds = actionlib.SimpleActionServer(
            self._da, \
            actions.msg.depthAction, \
            execute_cb = self.depthCallback, \
            auto_start = False)
        self._ds.start()

    def depth_cb(self, data):
        self._depth = data.data

    def depthCallback(self, goal):
        r = rospy.Rate(10)
        success = True
        new_depth = goal.depth_setpoint
        while(goal.depth_setpoint != self._depth):
            self.pub.publish(new_depth)
            if self._ds.is_preempt_requested():
                rospy.loginfo('%s : Preempted' % self._da)
                self._ds.set_preempted()
                success = False
                break
            self._feedback.depth_error = self._depth
            self._ds.publish_feedback(self._feedback)
            self._feedback.depth_error = self._depth - goal.depth_setpoint
            rospy.loginfo('%s : Going to Depth %f with Error : %f',\
                self._da , \
                goal.depth_setpoint, \
                self._feedback.depth_error)
            r.sleep()

        if success:
            self._result.depth_final = self._feedback.depth_error
            rospy.loginfo('%s : Success' % self._da)
            self._ds.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('depthServer')
    server = depthAction(rospy.get_name())
    rospy.spin()
