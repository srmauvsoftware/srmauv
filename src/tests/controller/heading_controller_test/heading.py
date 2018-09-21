#! /usr/bin/env python
import rospy
from std_msgs.msg import Float64

def talker():
    pub = rospy.Publisher('/imu/HeadingTrue_degree/theta', Float64, queue_size=10)
    rospy.init_node('heading_test', anonymous=True)
    rate = rospy.Rate(10)
    i = 0	
    while not rospy.is_shutdown():
        pub.publish(i)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
