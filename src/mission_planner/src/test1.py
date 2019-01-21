#! /usr/bin/env python
import rospy
from smach import StateMachine
import smach
from smach_ros import IntrospectionServer
from Sink import Sink
from Heading import Heading
from Forward import Forward
from Sway import Sway
import time
#from ImageTask import ImageTask

def main():
    rospy.init_node('mission_planner')
    sm = smach.StateMachine(outcomes=['mission_complete', 'mission_failed', 'aborted'])

    theta = 0

    with sm:

        Sink(sm, 'DEPTH1', 545, 'mission_complete')

        sis = IntrospectionServer('ZARNA_MISSION_PLANNER', sm, '/START_ZARNA')
        sis.start()
        outcome = sm.execute()

    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()
