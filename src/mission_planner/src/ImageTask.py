#! /usr/bin/env python
import rospy
import smach

from Depth import Depth
from Heading import Heading
from darknet_ros_msgs.msg import BoundingBox
from std_msgs.msg import Float64
class ImageTask():
    def __init__(self, smach_StateMachine, OBJECT, TASK):
        self.TASK = TASK
        self.depth_pub = rospy.Publisher('/depth', Float64, queue_size=10)
        self.heading_pub = rospy.Publisher('/heading', Float64, queue_size=10)
        rospy.Subscriber('/BoundingBox', BoundingBox, self.callback)
        sm_sub = smach.Concurrence(outcomes = ['DepthHeadingReached', 'DepthHeadingFailed'],
                                                default_outcome='DepthHeadingFailed',
                                                outcome_map={'DepthHeadingReached':
                                                {'DEPTH_CONCURRENT':'DepthReached',
                                                'HEADING_CONCURRENT':'HeadingReached'}})

        with sm_sub:
            smach.Concurrence.add('DEPTH_CONCURRENT', Depth(160, 'depth_success'))
            smach.Concurrence.add('HEADING_CONCURRENT', Heading(320, 'heading_success'))
        
        smach.StateMachine.add('IMAGETASK', sm_sub, transitions={
          'DepthHeadingFailed': 'IMAGETASK',
          'DepthHeadingReached': self.TASK
        })

    def callback(data):
        self.box_centre_x = (data.xmax - data.xmin)/2
        self.box_centre_y = (data.ymax - data.ymin)/2
        self.depth_pub.publish(box_centre_y)
        self.heading_pub.publish(box_centre_x)
        
