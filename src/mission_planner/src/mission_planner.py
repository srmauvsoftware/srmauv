#! /usr/bin/env python
import rospy
from smach import StateMachine
import smach
from smach_ros import IntrospectionServer
from Sink import Sink
from Forward import Forward
from Heading import Heading
#from ImageTask import ImageTask
from Backward import Backward

def main():
    rospy.init_node('mission_planner')
    sm = smach.StateMachine(outcomes=['mission_complete', 'mission_failed', 'aborted'])

    with sm:
        Sink (sm, 'SINK1', 530, 'HEADING1')
        Heading (sm, 'HEADING1',90, 'mission_complete')
        Forward(sm, 'FORWARD1', 7, 'mission_complete')
	#Backward(sm, 'BACKWARD1', 10, 'mission_complete')
	#Heading(sm, 'HEADING2', -20, 'FORWARD2')
	#Forward(sm, 'FORWARD2', 6, 'HEADING3')
	#Heading(sm, 'HEADING3', 60, 'FORWARD3')
	#Forward(sm, 'FORWARD3', 6, 'HEADING4')
	#Heading(sm, 'HEADING4', 160, 'mission_complete')
	#Backward(sm,'BACKWARD1', 3, 'mission_complete')
	#Backward(sm, 'BACKWARD1', 12, 'mission_complete')
	#Heading (sm, 'HEADING2',-98, 'FORWARD2')
	#Forward(sm,'FORWARD2', 5, 'HEADING3')
        #Heading (sm, 'HEADING3',-98, 'FORWARD3')
	#Forward(sm,'FORWARD3', 5, 'HEADING4')
        #Heading (sm, 'HEADING4',-98, 'FORWARD4')
	#Forward(sm,'FORWARD4', 5, 'HEADING5')
        #Heading (sm, 'HEADING5',-98, 'FORWARD5')
	#Forward(sm,'FORWARD5', 5, 'HEADING6')
        #Heading (sm, 'HEADING6',-98, 'mission_complete')

	#Heading(sm, 'HEADING2', 180, 'FORWARD2')
	#Forward(sm, 'FORWARD2', 10, 'HEADING3')
	#Heading(sm, 'HEADING3', -90, 'mission_complete')
        # Sink(sm, 530, 'FORWARD')
        # Head(sm, 90, 'FORWARD')
        # Forward(sm, 15, 'mission_complete')
        #PathAlign(sm, 'mission_complete')
        # ImageTask(sm, 'mission_complete')

        sis = IntrospectionServer('ZARNA_MISSION_PLANNER', sm, '/START_ZARNA')
        # start introspection server by - rosrun smach_viewer smach_viewer.py
        sis.start()
        outcome = sm.execute()


    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()
