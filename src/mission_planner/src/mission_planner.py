#! /usr/bin/env python
import rospy
from smach import StateMachine
import smach
from smach_ros import IntrospectionServer
from Sink import Sink
from ImageTask import ImageTask
from darknet_ros.msg import BoundingBox

def main():
    rospy.init_node('mission_planner')
    sm = smach.StateMachine(outcomes=['mission_complete', 'mission_failed', 'aborted'])

    with sm:
        Sink(sm, 15, 'IMAGETASK')
        ImageTask(sm, 'mission_complete')
        
        
	
	#state detectGate()

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
