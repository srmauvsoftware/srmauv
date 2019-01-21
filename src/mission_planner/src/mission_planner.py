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
from DetectBuoy import DetectBuoy
#from ImageTask import ImageTask

def main():
    rospy.init_node('mission_planner')
    sm = smach.StateMachine(outcomes=['mission_complete', 'mission_failed', 'aborted'])

    theta = 0

    with sm:
        # smach.StateMachine.add('TORPEDO', Torpedo(), transitions={'torpedo_success':'mission_complete'})
        # smach.StateMachine.add('DETECTBUOY', DetectBuoy(), transitions={'buoy_success':'mission_complete', 'buoy_retry': 'DETECTBUOY'})
        Sink (sm, 'SINK1', 530, 'HEADING1')
        Heading(sm, 'HEADING1', theta, 'FORWARD1')
        Forward(sm, 'FORWARD1', 14, 'DETECTBUOY')
        DetectBuoy(sm, 'DETECTBUOY', 'FORWARD2')
        Forward(sm, 'FORWARD2', 14, 'HEADING2')
        Heading(sm, 'HEADING2', theta + 45, 'FORWARD3')
        Forward(sm, 'FORWARD3', 14, 'SWAY1')
        #torpedo fire
        Sway(sm, 'SWAY1', -5, 'FORWARD4')
        Forward(sm, 'FORWARD4', 10, 'SINK2')
        Sink (sm, 'SINK2', 510, 'mission_complete') #resurface

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
