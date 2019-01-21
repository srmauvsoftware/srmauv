#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, String
from thrusters.msg import VectorThrusterMsg
from dynamic_reconfigure.server import Server
from thrusters.cfg import VectorThrusterCfgConfig

class VectorThruster:
    def __init__(self):
        self.thrusterPub = rospy.Publisher('/vectorThruster', VectorThrusterMsg, queue_size=10)
        rospy.Rate(10)
        self.yaw = 0
        self.sway = 0
        self.surge = 0
        rospy.Subscriber('/yaw_controller/control_effort', Float64, self.yawCallback)
        rospy.Subscriber('/surge_controller/control_effort', Float64, self.surgeCallback)
        rospy.Subscriber('/sway_controller/control_effort', Float64, self.swayCallback)
        srv = Server(VectorThrusterCfgConfig, self.dynamicThrusterCb)

    def yawCallback(self, data):
        self.yaw = data.data

    def swayCallback(self, data):
        self.sway = data.data

    def surgeCallback(self, data):
        self.surge = data.data

    def maxmin(self, control_effort):
	if control_effort < -168:
	    control_effort = -168
	if control_effort > 168:
	    control_efffort = 168
	return control_effort

    def thrusterCb(self):
        msg = VectorThrusterMsg()
        msg.tfr = 290 + self.maxmin(-self.surge +self.sway -self.yaw)
        msg.tfl = 290 + self.maxmin( self.surge -self.sway -self.yaw)
        msg.trr = 290 + self.maxmin( self.surge +self.sway +self.yaw)
        msg.trl = 290 + self.maxmin(-self.surge -self.sway +self.yaw)
        self.thrusterPub.publish(msg)

    def vectorShutdown(self):
        msg = VectorThrusterMsg()
        msg.tfr = 290
        msg.tfl = 290
        msg.trr = 290
        msg.trl = 290
        self.thrusterPub.publish(msg)

    def dynamicThrusterCb(self, config, level):
        msg = VectorThrusterMsg()
        msg.tfr = config.tfr
        msg.tfl = config.tfl
        msg.trr = config.trr
        msg.trl = config.trl
        self.thrusterPub.publish(msg)
        return config

    def mapThrust(self, x, in_min=-504, in_max=504, out_min=-100, out_max=100):
        return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

if __name__ == '__main__':
    try:
        rospy.init_node('vectorThuster')
        thruster = VectorThruster()
        msg = VectorThrusterMsg()
        msg.tfr = 290
        msg.tfl = 290
        msg.trr = 290
        msg.trl = 290
        thruster.thrusterPub.publish(msg)
        while not rospy.is_shutdown():
            thruster.thrusterCb()
            rospy.sleep(1)
        rospy.on_shutdown(thruster.vectorShutdown)
    except rospy.ROSInterruptException: pass
