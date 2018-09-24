#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from thrusters.msg import ThrusterMsg

#from dynamic_reconfigure.server import Server
#from thrusters.cfg import ThrusterCfgConfig


class Thruster:
    def __init__(self):
        rospy.init_node('thruster', anonymous=True)
        self.thrusterPub = rospy.Publisher('/thruster', ThrusterMsg, queue_size=10)
        rospy.Rate(10)
        rospy.Subscriber('/control_effort', Float64, self.thrusterCb)
        #self.srv = Server(ThrusterCfgConfig, self.thrusterCb)
        rospy.spin()

    def thrusterCb(self, config, level=None):
        msg = ThrusterMsg()
        msg.t1 = config.t1
        msg.t2 = config.t2
        msg.t3 = config.t3
        msg.t4 = config.t4
        self.thrusterPub.publish(msg)

if __name__ == '__main__':
    try:
        thruster = Thruster()

    except rospy.ROSInterruptException: pass

#<node pkg="thrusters" type="thruster.py" name="thruster" output="screen" />
