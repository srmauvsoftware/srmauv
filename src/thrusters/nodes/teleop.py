#!/usr/bin/env python
#import roslib; roslib.load_manifest('teleop')
import roslib
import rospy

from thrusters.msg import DepthThrusterMsg
from thrusters.msg import VectorThrusterMsg

import sys, select, termios, tty, threading

msg = """

  !depth!

td3--F--td1
 |       |
 |       |
 |       |
 |       |
td4-----td2

 !heading!

tfl--F--tfr
 |       |
 |       |
 |       |
 |       |
trl-----trr

Alpheus Testing Mode - Reading From Keyboard
--------------------
w - forward
a - left
s - backward
d - right
o - surface
l - sink
m - yaw-left
n - yaw-right
r - reset
q - quit
--------------------
"""


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    #print(key)
    return key

if __name__=="__main__":
    print(msg)
    settings = termios.tcgetattr(sys.stdin)

    depthPub = rospy.Publisher('/depthThruster', DepthThrusterMsg, queue_size=1)
    vectorPub = rospy.Publisher('/vectorThruster', VectorThrusterMsg, queue_size=1)
    rospy.init_node('teleop')

    try:
        while(1):
            key = getKey()

            dt = DepthThrusterMsg()
            vt = VectorThrusterMsg()

            if key == 'o':
                dt.td1 = 1450
                dt.td2 = 1450
                dt.td3 = 1450
                dt.td4 = 1450
                print("Moving Up")
                depthPub.publish(dt)

            elif key == 'l':
                dt.td1 = 1550
                dt.td2 = 1550
                dt.td3 = 1550
                dt.td4 = 1550
                print("Moving Down")
                depthPub.publish(dt)

            elif key == 'w':
                vt.tfr = 1550
                vt.tfl = 1550
                vt.trr = 1550
                vt.trl = 1550
                print("Moving forward")
                vectorPub.publish(vt)

            elif key == 'a':
                vt.tfr = 1550
                vt.tfl = 1450
                vt.trr = 1450
                vt.trl = 1550
                print("Moving left")
                vectorPub.publish(vt)

            elif key == 's':
                vt.tfr = 1450
                vt.tfl = 1450
                vt.trr = 1450
                vt.trl = 1450
                print("Moving backward")
                vectorPub.publish(vt)

            elif key == 'd':
                vt.tfr = 1450
                vt.tfl = 1550
                vt.trr = 1550
                vt.trl = 1450
                print("Moving right")
                vectorPub.publish(vt)

            elif key == 'q':
                print("resetting and quitting")
                dt.td1 = 1500
                dt.td2 = 1500
                dt.td3 = 1500
                dt.td4 = 1500
                vt.tfr = 1500
                vt.tfl = 1500
                vt.trr = 1500
                vt.trl = 1500
                break

            elif key == 'r':
                dt.td1 = 1500
                dt.td2 = 1500
                dt.td3 = 1500
                dt.td4 = 1500
                vt.tfr = 1500
                vt.tfl = 1500
                vt.trr = 1500
                vt.trl = 1500
                print("resetting vector and depth to 1500")
                vectorPub.publish(vt)
                depthPub.publish(dt)

            elif key == 'm':
		vt.tfr = 1450
		vt.tfl = 1550
		vt.trr = 1450
		vt.trl = 1550
		print("Yaw Right")
                vectorPub.publish(vt)

            elif key == 'n':
		vt.tfr = 1550
		vt.tfl = 1450
		vt.trr = 1550
		vt.trl = 1450
		print("Yaw Left")
                vectorPub.publish(vt)

            else:
                print("key not binded")
		print(key)

    except rospy.ROSInterruptException:
        pass

    finally:
        dt = ThrusterMsg()
        vt = ThrusterMsg()

        dt.td1 = 1500
        dt.td2 = 1500
        dt.td3 = 1500
        dt.td4 = 1500

        vt.tfr = 1500
        vt.tfl = 1500
        vt.trr = 1500
        vt.trl = 1500

        depthPub.publish(dt)
        vectorPub.publish(vt)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
