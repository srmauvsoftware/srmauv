#! /usr/bin/env python
import rospy
from smach import StateMachine
import smach
from smach_ros import IntrospectionServer
from Sink import Sink
from Heading import Heading
#from Torpedo import Torpedo
from Forward import Forward
from Sway import Sway
#from Surge import Surge
import time
from DetectBuoy import DetectBuoy
from std_msgs.msg import Float64

#from ImageTask import ImageTask

boolean = 0

def main():
    sm = smach.StateMachine(outcomes=['mission_complete', 'mission_failed', 'aborted'])

    theta = 0

    with sm:
        # smach.StateMachine.add('TORPEDO', Torpedo(), transitions={'torpedo_success':'mission_complete'})
       # smach.StateMachine.add('DETECTBUOY', DetectBuoy(), transitions={'buoy_success':'mission_complete', 'buoy_retry': 'DETECTBUOY'})
        Sink (sm, 'SINK1', 515, 'HEADING1')
        Heading(sm, 'HEADING1', 75,'FORWARD1')
        Forward(sm, 'FORWARD1', 6, 'FORWARD2')
	Forward(sm, 'FORWARD2', 12, 'mission_complete')
	#Heading(sm, 'HEADING2', 90, 'FORWARD2')
	#Forward(sm, 'FORWARD2', 10, 'HEADING3')
	#Heading(sm, 'HEADING3', 90, 'FORWARD3')
	#Forward(sm, 'FORWARD3', 15, 'mission_complete')
        #DetectBuoy(sm, 'DETECTBUOY', 'FORWARD2')
        #Forward(sm, 'FORWARD2', 14, 'HEADING2')
        #Heading(sm, 'HEADING2', theta + 45, 'FORWARD3')
        #Forward(sm, 'FORWARD3', 14, 'SWAY1')
        #torpedo fire
        #Sway(sm, 'SWAY1', -5, 'FORWARD4')
        #Forward(sm, 'FORWARD4', 10, 'SINK2')
        #Sink (sm, 'SINK2', 525, 'mission_complete') #resurface

        #it = ImageTask() # Image Task should return User data which should be
        # further mapped to Heading etc states
        #it.init(sm)

        sis = IntrospectionServer('ZARNA_MISSION_PLANNER', sm, '/START_ZARNA')
        # start introspection server by - rosrun smach_viewer smach_viewer.py
        sis.start()
        outcome = sm.execute()
	boolean = 1

    sis.stop()
    rospy.loginfo("Mission Complete")

def callback(data):
    rospy.loginfo(data.data)
    if(data.data == 1):
	if boolean == 0:
	    main()

def listener():
    rospy.init_node('mission_planner')
    rospy.Subscriber("/zsys", Float64, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
