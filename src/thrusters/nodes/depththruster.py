#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from thrusters.msg import DepthThrusterMsg
from dynamic_reconfigure.server import Server
from thrusters.cfg import DepthThrusterCfgConfig

class DepthThruster:
    def __init__(self):
        self.thrusterPub = rospy.Publisher('/depthThruster', DepthThrusterMsg, queue_size=10)
        rospy.Rate(10)
        rospy.Subscriber('/depthThruster/control_effort', Float64, self.thrusterCb)
        srv = Server(DepthThrusterCfgConfig, self.dynamicThrusterCb)

    def thrusterCb(self, data):
        msg = DepthThrusterMsg()
        msg.td1 = 1500 + data.data
        msg.td2 = 1500 + data.data
        msg.td3 = 1500 + data.data
        msg.td4 = 1500 + data.data
        self.thrusterPub.publish(msg)

    def dynamicThrusterCb(self, config, level):
        msg = DepthThrusterMsg()
        msg.td1 = config.td1
        msg.td2 = config.td2
        msg.td3 = config.td3
        msg.td4 = config.td4
        self.thrusterPub.publish(msg)
        return config

if __name__ == '__main__':
    try:
        rospy.init_node('depth_thruster', anonymous=True)
        thruster = DepthThruster()
        rospy.spin()

    except rospy.ROSInterruptException: pass