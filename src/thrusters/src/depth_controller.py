import rospy
from std_msgs.msg import Float64
from thruster.msg import depthThruster

def depthCb(data):
    rospy.loginfo(data)
    pub = rospy.Publisher('/depth_thruster', depthThruster)
    r = rospy.Rate(10)
    msg = depthThruster()
    msg.td1 += data.data
    msg.td2 += data.data
    msg.td3 += data.data
    msg.td4 += data.data

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()
    
def depth_controller():

    rospy.init_node('depth_controller', anonymous=True)

    rospy.Subscriber("/control_effort", Float64, depthCb())

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    depth_controller()
