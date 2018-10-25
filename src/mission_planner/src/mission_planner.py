#! /usr/bin/env python
import roslib; roslib.load_manifest('mission_planner')
import rospy
from smach import StateMachine
from rospy.exceptions import ROSInitException
from rospy.timer import Rate
import smach
import time
from smach_ros import IntrospectionServer
from Sink import Sink
# from Depth import Depth
# from Heading import Heading
# from DepthHeading import DepthHeading

def main():
    rospy.init_node('mission_planner')
    sm = smach.StateMachine(outcomes=['mission_complete', 'mission_failed', 'aborted'])

    with sm:
        # 15 is the inital pressure that acts as the setpoint
        Sink(sm, 15, 'mission_complete')
        # Sink(sm, 5, 'DEPTH')
	
	#state detectGate()
        # depthTask = Depth(100, 'HEADING')
        # depthTask.addDepthAction(sm)

        # headingTask = Heading(200, 'DEPTH+HEADING')
        # headingTask.addHeadingAction(sm)

        # depthHeadingTask = DepthHeading(350, 350, 'mission_complete')
        # depthHeadingTask.addDepthHeading(sm)
 

        sis = IntrospectionServer('ZARNA_MISSION_PLANNER', sm, '/START_ZARNA')
        # start introspection server by - rosrun smach_viewer smach_viewer.py
        sis.start()
        outcome = sm.execute()


    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()
