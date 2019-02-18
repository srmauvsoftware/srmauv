#!/usr/bin/env python
import roslib
import rospy
import time

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
t - torpedo
x - dropper
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

    elif key == 'a' or key == 'd':
	print ('Heading controller enabled.')
	headingControllerToggle.publish(Bool(True))

    elif key == 'w' or key == 's':
	print ('Heading controller enabled.')
	headingControllerToggle.publish(Bool(True))



def keyDown(data):
    key = chr(data.code)
    print (key + ' key pressed')
    
    if key == 'o':
        depthControllerToggle.publish(Bool(False))
        print ('Depth controller disabled.')
        dt.td1 = 280
       	dt.td2 = 280
        dt.td3 = 280
        dt.td4 = 280
        print("Moving Up")
        depthPub.publish(dt)

    elif key == 'l':
        depthControllerToggle.publish(Bool(False))
        print ('Depth controller disabled.')
        dt.td1 = 300
        dt.td2 = 300
        dt.td3 = 300
        dt.td4 = 300
        print("Moving Down")
        depthPub.publish(dt)
    
    elif key == 'w':
	headingControllerToggle.publish(Bool(False))
        vt.tfr = 320
        vt.tfl = 320
        vt.trr = 320
        vt.trl = 320
        print("Moving forward")
        vectorPub.publish(vt)

    elif key == 's':
        headingControllerToggle.publish(Bool(False))
	vt.tfr = 330
        vt.tfl = 250
        vt.trr = 250
        vt.trl = 330
        print("Moving backward")
        vectorPub.publish(vt)

    elif key == 'm':
        print ("Heading controller disabled.")
        headingControllerToggle.publish(Bool(False))
        vt.tfr = 300
        vt.tfl = 300
        vt.trr = 280
        vt.trl = 280
        print("Yaw right")
        vectorPub.publish(vt)

    elif key == 'n':
        print ("Heading controller disabled.")
        headingControllerToggle.publish(Bool(False))
        vt.tfr = 280
        vt.tfl = 280
        vt.trr = 300
        vt.trl = 300
        print("Yaw left")
        vectorPub.publish(vt)

    elif key == 'a':
        print ("Heading controller disabled.")
        headingControllerToggle.publish(Bool(False))
        vt.tfr = 280
        vt.tfl = 300
        vt.trr = 280
        vt.trl = 300
        print("Swaying left")
        vectorPub.publish(vt)


    elif key == 'd':
        print ("Heading controller disabled.")
        headingControllerToggle.publish(Bool(False))
        vt.tfr = 300
        vt.tfl = 280
        vt.trr = 300
        vt.trl = 280
        print("Swaying right")
        vectorPub.publish(vt)

    elif key == 't':
        print ("Firing torpedo")
        torpedoPub.publish(Bool(True))
        time.sleep(.500)
        torpedoPub.publish(Bool(False))
        
    
    elif key == 'x':
        print ("Actuating dropper")
        dropperPub.publish(Bool(True))
        time.sleep(1)
        dropperPub.publish(Bool(False))

    elif key == 'q':
        print("resetting vector")
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
        headingControllerToggle.publish(Bool(False))
	depthControllerToggle.publish(Bool(False))
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
    depthControllerToggle = rospy.Publisher('/heave_controller/pid_enable', Bool, queue_size=10)
    
    headingSetpointPub = rospy.Publisher('/heading_setpoint', Float64, queue_size=10)
    headingControllerToggle = rospy.Publisher('/yaw_controller/pid_enable', Bool, queue_size=10)

    torpedoPub = rospy.Publisher('/torpedo', Bool, queue_size=10)
    dropperPub = rospy.Publisher('/dropper', Bool, queue_size=10)

    rospy.Subscriber('/keyboard/keyup', Key, keyUp)
    rospy.Subscriber('/keyboard/keydown', Key, keyDown)

    rospy.Subscriber('/depth', Float64, getDepth)
    rospy.Subscriber('/imu/Heading_degree/theta', Float64, getHeading)

    rospy.init_node('teleop')
    
    rospy.spin()
