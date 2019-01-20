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

    with sm:
        # Sink (sm, 'SINK1', 530, 'HEADING1')
        Sway(sm, 'SWAY1', -20, 'SWAY2')
        Sway(sm, 'SWAY2', 20, 'mission_complete')
        # Heading(sm, 'HEADING1', 90, 'FORWARD1')
        # Forward(sm, 'FORWARD1', 5, 'mission_complete')
        # Forward(sm, 'FORWARD1', -5, 'SWAYR1')

        #it = ImageTask() # Image Task should return User data which should be
        # further mapped to Heading etc states
        #it.init(sm)

        sis = IntrospectionServer('ZARNA_MISSION_PLANNER', sm, '/START_ZARNA')
        # start introspection server by - rosrun smach_viewer smach_viewer.py
        sis.start()
        outcome = sm.execute()


    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()
