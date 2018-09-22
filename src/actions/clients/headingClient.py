#! /usr/bin/env python
from __future__ import print_function
import rospy
import actionlib
import alpheus_actions.msg

def headingClient():
    client = actionlib.SimpleActionClient('headingServer', \
    alpheus_actions.msg.headingAction)
    client.wait_for_server()
    goal = alpheus_actions.msg.headingGoal(heading_setpoint=100)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('headingClient')
        result = headingClient()
        print(result)
    except rospy.ROSInterruptException:
        print("Program interrupted before completetion", file=sys.stderr)
