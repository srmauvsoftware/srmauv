#! /usr/bin/env python
import rospy
from std_msgs.msg import Float64

def talker():
    pub = rospy.Publisher('/depth_setpoint', Float64, queue_size=10)
    rospy.init_node('depth_setpoint', anonymous=True)
    rate = rospy.Rate(10)
    i = 540	
    while not rospy.is_shutdown():
        pub.publish(i)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
