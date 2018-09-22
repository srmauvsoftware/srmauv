#! /usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
import actions.msg

def depthClient():
    client = actionlib.SimpleActionClient('depth_server', \
    actions.msg.depthAction)
    client.wait_for_server()
    goal = actions.msg.depthGoal(depth_setpoint=100)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('depth_client')
        result = depthClient()
        print(result)
    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)
