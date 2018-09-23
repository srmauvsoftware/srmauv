#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from thrusters.msg import depthThruster

def depthCb(data):
    msg = depthThruster()
    msg.td1 += data.data
    msg.td2 += data.data
    msg.td3 += data.data
    msg.td4 += data.data
    pub.publish(msg)
    
def depth_controller():
    rospy.init_node('depth_controller', anonymous=True)
    pub = rospy.Publisher('/depth_thruster', depthThruster, queue_size=10)
    rospy.Rate(10)
    rospy.Subscriber('/control_effort', Float64, depthCb)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
	depth_controller()
    except rospy.ROSInterruptException: pass
