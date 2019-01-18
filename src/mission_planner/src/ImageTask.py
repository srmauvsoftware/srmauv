#! /usr/bin/env python
import rospy
import smach

from Depth import Depth
from Heading import Heading
from std_msgs.msg import Float64, Int16


class ImageTask():
    def __init__(self, smach_StateMachine, TASK):
        self.TASK = TASK
        self.xSub = rospy.Subscriber('/offsetX', Int16, self.xCallback)
        self.ySub = rospy.Subscriber('/offsetY', Int16, self.yCallback)
        self.y = None
        self.x = None
        sm_sub = smach.Concurrence(outcomes = ['DepthHeadingReached', 'DepthHeadingFailed'],
                                            default_outcome='DepthHeadingFailed',
                                            outcome_map={'DepthHeadingReached':
                                            {'DEPTH_CONCURRENT':'DepthReached',
                                            'HEADING_CONCURRENT':'HeadingReached'}})

        with sm_sub:
            smach.Concurrence.add('DEPTH_CONCURRENT', Depth(self.x, 'depth_success'))
            smach.Concurrence.add('HEADING_CONCURRENT', Heading(self.y, 'heading_success'))


        smach.StateMachine.add('IMAGETASK', sm_sub, transitions={
        'DepthHeadingFailed': 'IMAGETASK',
        'DepthHeadingReached': self.TASK
      })

    def xCallback(self, data):
        self.x = data.data

    def yCallback(self, data):
        self.y = data.data
