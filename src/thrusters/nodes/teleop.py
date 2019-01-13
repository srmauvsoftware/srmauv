#!/usr/bin/env python
import roslib
import rospy

from thrusters.msg import DepthThrusterMsg
from thrusters.msg import VectorThrusterMsg
from keyboard.msg import Key
from std_msgs.msg import Float64, Bool

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

Zarna Testing Mode - Reading From Keyboard
--------------------
w - forward
a - left
s - backward
d - right
o - surface
l - sink
m - yaw-right
n - yaw-left
r - reset
q - quit
--------------------
""" 


def keyUp(data):
    key = chr(data.code)
    print (key + ' key released')
    
    if key == 'o' or key == 'l':
        print ('Depth controller enabled.')
        depthSetpointPub.publish(Float64(depth))
        depthControllerToggle.publish(Bool(True))
    
    elif key == 'm' or key == 'n':
        print ('Heading controller enabled.')
        headingSetpointPub.publish(Float64(heading))
        headingControllerToggle.publish(Bool(True))


def keyDown(data):
    key = chr(data.code)
    print (key + ' key pressed')
    
    if key == 'o':
        depthControllerToggle.publish(Bool(False))
        print ('Depth controller disabled.')
        dt.td1 = 285
        dt.td2 = 285
        dt.td3 = 285
        dt.td4 = 285
        print("Moving Up")
        depthPub.publish(dt)

    elif key == 'l':
        depthControllerToggle.publish(Bool(False))
        print ('Depth controller disabled.')
        dt.td1 = 295
        dt.td2 = 295
        dt.td3 = 295
        dt.td4 = 295
        print("Moving Down")
        depthPub.publish(dt)
    
    elif key == 'w':
        vt.tfr = 295
        vt.tfl = 295
        vt.trr = 295
        vt.trl = 295
        print("Moving forward")
        vectorPub.publish(vt)

    elif key == 's':
        vt.tfr = 285
        vt.tfl = 285
        vt.trr = 285
        vt.trl = 285
        print("Moving backward")
        vectorPub.publish(vt)

    elif key == 'm':
        print ("Heading controller disabled.")
        headingControllerToggle.publish(Bool(False))
        vt.tfr = 285
        vt.tfl = 295
        vt.trr = 285
        vt.trl = 295
        print("Yaw right")
        vectorPub.publish(vt)

    elif key == 'n':
        print ("Heading controller disabled.")
        headingControllerToggle.publish(Bool(False))
        vt.tfr = 295
        vt.tfl = 285
        vt.trr = 295
        vt.trl = 285
        print("Yaw left")
        vectorPub.publish(vt)

    elif key == 'a':
        print ("Heading controller disabled.")
        headingControllerToggle.publish(Bool(False))
        vt.tfr = 330
        vt.tfl = 250
        vt.trr = 250
        vt.trl = 330
        print("Swaying left")
        vectorPub.publish(vt)


    elif key == 'd':
        print ("Heading controller disabled.")
        headingControllerToggle.publish(Bool(False))
        vt.tfr = 250
        vt.tfl = 330
        vt.trr = 330
        vt.trl = 250
        print("Swaying right")
        vectorPub.publish(vt)

    elif key == 'q':
        print("resetting and quitting")
        dt.td1 = 290
        dt.td2 = 290
        dt.td3 = 290
        dt.td4 = 290
        vt.tfr = 290
        vt.tfl = 290
        vt.trr = 290
        vt.trl = 290
        vectorPub.publish(vt)
        depthPub.publish(dt)
        exit()

    elif key == 'r':
        dt.td1 = 290
        dt.td2 = 290
        dt.td3 = 290
        dt.td4 = 290
        vt.tfr = 290
        vt.tfl = 290
        vt.trr = 290
        vt.trl = 290
        print("resetting vector and depth to 290")
        vectorPub.publish(vt)
        depthPub.publish(dt)

    else:
        print("key not binded")

def getDepth(data):
    global depth
    depth = data.data

def getHeading(data):
    global heading
    heading = data.data

if __name__=="__main__":
    
    print(msg)
    
    depthPub = rospy.Publisher('/depthThruster', DepthThrusterMsg, queue_size=10)
    vectorPub = rospy.Publisher('/vectorThruster', VectorThrusterMsg, queue_size=10)

    dt = DepthThrusterMsg()
    vt = VectorThrusterMsg()

    depthSetpointPub = rospy.Publisher('/depth_setpoint', Float64, queue_size=10)
    depthControllerToggle = rospy.Publisher('/depth_controller/pid_enable', Bool, queue_size=10)
    
    headingSetpointPub = rospy.Publisher('/heading_setpoint', Float64, queue_size=10)
    headingControllerToggle = rospy.Publisher('/heading_controller/pid_enable', Bool, queue_size=10)

    rospy.Subscriber('/keyboard/keyup', Key, keyUp)
    rospy.Subscriber('/keyboard/keydown', Key, keyDown)

    rospy.Subscriber('/depth', Float64, getDepth)
    rospy.Subscriber('/imu/HeadingTrue_degree/theta', Float64, getHeading)
    
    rospy.init_node('teleop')
    
    rospy.spin()
