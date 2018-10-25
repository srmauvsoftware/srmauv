#! /usr/bin/env python
import roslib; roslib.load_manifest('mission_planner')
import rospy
from actions.msg import depthGoal, depthAction
import actions.msg
from rospy.exceptions import ROSInitException
from rospy.timer import Rate
import smach
import actionlib
from smach_ros import SimpleActionState
import time

class Sink:

    def __init__(self,smach_StateMachine, INITIAL_PRESSURE, TASK):
        self.INITIAL_PRESSURE = INITIAL_PRESSURE
        smach_StateMachine.add('SINK_ZARNA', \
                                SimpleActionState('depthServer', \
                                depthAction, \
                                goal_cb=self.sink_callback), \
                                transitions={'succeeded': TASK,\
                                            'preempted':'SINK_ZARNA',\
                                            'aborted':'aborted'})

    def sink_callback(self, userdata, goal):
        rospy.loginfo('Executing State Sink')
        depthOrder = depthGoal()
        depthOrder.depth_setpoint = self.INITIAL_PRESSURE
        return depthOrder


'''
class Sink(smach.State):
    def __init__(self, INITIAL_PRESSURE):
        smach.StateMachine.add('SINK_ALPHEUS', \
                                SimpleActionState('depthServer', \
                                alpheus_actions.msg.depthAction, \
                                goal_cb=self.callback), \
                                transitions={'succeeded':'sink',\
                                            'preempted':'not-sink',\
                                            'aborted':'not-sink'})

        self.INITIAL_PRESSURE = INITIAL_PRESSURE

    def callback(self, userdata):
        rospy.loginfo('Executing State Sink')
        depthOrder = depthGoal()
        depthOrder.depth_setpoint = INITIAL_PRESSURE
        return depthOrder
'''
'''
def main():
    rospy.init_node('sink_state')
    sm = smach.StateMachine(outcomes=['sink', 'not-sink'])

    with sm:
        def sink_goal_callback(self, goal):
            depthOrder = depthGoal()
            depthOrder.depth_setpoint = INITIAL_PRESSURE
            return depthOrder

        StateMachine.add('SINK_ALPHEUS', \
            SimpleActionState('depthServer',\
                alpheus_actions.msg.depthAction, \
                goal_cb = sink_goal_callback,), \
            transitions={'succeeded':'sink',\
                        'preempted':'not-sink',\
                        'aborted':'not-sink'})

    sis = IntrospectionServer('SINK_SERVER', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

if __name__ == '__main__':
    main()
'''
