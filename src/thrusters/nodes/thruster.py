#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from thrusters.msg import ThrusterMsg
from dynamic_reconfigure.server import Server
from thrusters.cfg import ThrusterCfgConfig

class Thruster:
    def __init__(self):
        self.thrusterPub = rospy.Publisher('thruster', ThrusterMsg, queue_size=10)
        rospy.Rate(10)
        rospy.Subscriber('control_effort', Float64, self.thrusterCb)
        srv = Server(ThrusterCfgConfig, self.dynamicThrusterCb)

    def thrusterCb(self, data):
        msg = ThrusterMsg()
        msg.t1 = data.data
        msg.t2 = data.data
        msg.t3 = data.data
        msg.t4 = data.data
        self.thrusterPub.publish(msg)

    def dynamicThrusterCb(self, config, level):
        msg = ThrusterMsg()
        msg.t1 = config.t1
        msg.t2 = config.t2
        msg.t3 = config.t3
        msg.t4 = config.t4
        self.thrusterPub.publish(msg)
        return config

if __name__ == '__main__':
    try:
        rospy.init_node('thruster', anonymous=True)
        thruster = Thruster()
        rospy.spin()

    except rospy.ROSInterruptException: pass
