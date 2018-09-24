#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from thrusters.msg import vectorThruster

def headingCb(data):
    msg = vectorThruster()
    msg.tfl += data.data
    msg.tfr -= data.data
    msg.trl += data.data
    msg.trr -= data.data
    pub.publish(msg)
    
def heading_controller():
    rospy.init_node('heading_controller', anonymous=True)
    pub = rospy.Publisher('/vector_thruster', vectorThruster, queue_size=10)
    rospy.Rate(10)
    rospy.Subscriber('/control_effort', Float64, headingCb)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        heading_controller()
    except rospy.ROSInterruptException: pass
