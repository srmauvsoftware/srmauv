#!/usr/bin/env python
'''
This file is responsible for publishing thruster values for each thruster.
'''

import rospy
from std_msgs.msg import Float64
from thrusters.msg import DepthThrusterMsg
from dynamic_reconfigure.server import Server
from thrusters.cfg import DepthThrusterCfgConfig

class DepthThruster:
    def __init__(self):
        self.thrusterPub = rospy.Publisher('/depthThruster', DepthThrusterMsg, queue_size=10)
        rospy.Rate(10)
        self.roll = 0
        self.pitch = 0
        self.heave = 0
        rospy.Subscriber('/roll_controller/control_effort', Float64, self.rollCallback)
        rospy.Subscriber('/pitch_controller/control_effort', Float64, self.pitchCallback)
        rospy.Subscriber('/heave_controller/control_effort', Float64, self.heaveCallback)
        srv = Server(DepthThrusterCfgConfig, self.dynamicThrusterCb)

    def rollCallback(self, data):
        self.roll = data.data

    def pitchCallback(self, data):
        self.pitch = data.data

    def heaveCallback(self, data):
        self.heave = data.data

    def thrusterCb(self):
        msg = DepthThrusterMsg()
        msg.td1 = 290 + self.mapThrust(self.roll + self.pitch + self.heave)
        msg.td2 = 290 + self.mapThrust(self.roll + self.pitch + self.heave)
        msg.td3 = 290 + self.mapThrust(self.roll + self.pitch + self.heave)
        msg.td4 = 290 + self.mapThrust(self.roll + self.pitch + self.heave)
        self.thrusterPub.publish(msg)

    def dynamicThrusterCb(self, config, level):
        msg = DepthThrusterMsg()
        msg.td1 = config.td1
        msg.td2 = config.td2
        msg.td3 = config.td3
        msg.td4 = config.td4
        self.thrusterPub.publish(msg)
        return config

    def depthShutdown(self):
        msg = DepthThrusterMsg()
        msg.td1 = 290
        msg.td2 = 290
        msg.td3 = 290
        msg.td4 = 290
        self.thrusterPub.publish(msg)

    def mapThrust(self, x, in_min=-504, in_max=504, out_min=-168, out_max=168):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

if __name__ == '__main__':
    try:
        rospy.init_node('depthThruster', anonymous=True)
        thruster = DepthThruster()
        msg = DepthThrusterMsg()
        msg.td1 = 290
        msg.td2 = 290
        msg.td3 = 290
        msg.td4 = 290
        thruster.thrusterPub.publish(msg)
        while not rospy.is_shutdown():
            thruster.thrusterCb()
            rospy.sleep(1)
        rospy.on_shutdown(thruster.depthShutdown)

    except rospy.ROSInterruptException: pass
