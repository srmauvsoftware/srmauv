#! /usr/bin/env python
import rospy
import smach

from Depth import Depth 
from Heading import Heading

camera_centre_x = 320
camera_centre_y = 240
# depth constant
kd = 1.5
# heading constant
kh = 1.5

class ImageTask():
    def __init__(self, smach_StateMachine, TASK):
        self.TASK = TASK
        rospy.Subscriber('/BoundingBox', BoundingBox, self.callback)
        sm_sub = smach.Concurrence(outcomes = ['DepthHeadingReached', 'DepthHeadingFailed'],
                                                default_outcome='DepthHeadingFailed',
                                                outcome_map={'DepthHeadingReached':
                                                {'DEPTH_CONCURRENT':'DepthReached',
                                                'HEADING_CONCURRENT':'HeadingReached'}})

        with sm_sub:
            smach.Concurrence.add('DEPTH_CONCURRENT', Depth(mapped_depth))
            smach.Concurrence.add('HEADING_CONCURRENT', Heading(mapped_heading))
        
        smach.StateMachine.add('IMAGETASK', sm_sub, transitions={
          DepthHeadingFailed: 'IMAGETASK',
          DepthHeadingSuccess: self.TASK
        })

    def callback(data):
        self.box_centre = (data.xmax - data.xmin)/2
        self.box_centre_y = (data.ymax - data.ymin)/2
        self.difference_x = camera_centre_x - self.box_centre
        self.difference_y = camera_centre_y - self.box_centre_y
        # depth setpoint
        self.mapped_depth = kd * self.difference_y 
        # heading setpoint
        self.mapped_heading = kh * self.difference_x
