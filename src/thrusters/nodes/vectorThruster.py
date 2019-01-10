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
        msg.tfr = 290 + data.data
        msg.tfl = 290 + data.data
        msg.trr = 290 + data.data
        msg.trl = 290 + data.data
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

        rospy.init_node('vectorThuster', anonymous=True)
        thruster = VectorThruster()
	msg = VectorThrusterMsg()
        msg.tfr = 290
        msg.tfl = 290
        msg.trr = 290
        msg.trl = 290
        thruster.thrusterPub.publish(msg)
        rospy.spin()

    except rospy.ROSInterruptException: pass
