#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from thrusters.msg import VectorThrusterMsg
from dynamic_reconfigure.server import Server
from thrusters.cfg import VectorThrusterCfgConfig

class VectorThruster:
    def __init__(self):
        self.thrusterPub = rospy.Publisher('/vectorThruster', VectorThrusterMsg, queue_size=10)
        rospy.Rate(10)
        rospy.Subscriber('/vectorThruster/control_effort', Float64, self.thrusterCb)
        srv = Server(VectorThrusterCfgConfig, self.dynamicThrusterCb)

    def thrusterCb(self, data):
        msg = VectorThrusterMsg()
        msg.tfr = 1500 + data.data
        msg.tfl = 1500 - data.data
        msg.trr = 1500 + data.data
        msg.trl = 1500 - data.data
        self.thrusterPub.publish(msg)

    def dynamicThrusterCb(self, config, level):
        msg = VectorThrusterMsg()
        msg.tfr = config.tfr
        msg.tfl = config.tfl
        msg.trr = config.trr
        msg.trl = config.trl
        self.thrusterPub.publish(msg)
        return config

if __name__ == '__main__':
    try:
        rospy.init_node('vector_thruster', anonymous=True)
        thruster = VectorThruster()
        rospy.spin()

    except rospy.ROSInterruptException: pass
