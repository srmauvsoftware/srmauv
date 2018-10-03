#!/usr/bin/env python
#import roslib; roslib.load_manifest('teleop')
import roslib
import rospy

from thrusters.msg import ThrusterMsg

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
o - ascend
l - descend
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

    depthPub = rospy.Publisher('/depthThruster', ThrusterMsg, queue_size=1)
    vectorPub = rospy.Publisher('/vectorThruster', ThrusterMsg, queue_size=1)
    rospy.init_node('teleop')

    try:
        while(1):
            key = getKey()

            dt = ThrusterMsg()
            vt = ThrusterMsg()

            if key == 'o':
                dt.t1 = 1450
                dt.t2 = 1450
                dt.t3 = 1450
                dt.t4 = 1450
                print("Moving Up")
                depthPub.publish(dt)

            elif key == 'l':
                dt.t1 = 1550
                dt.t2 = 1550
                dt.t3 = 1550
                dt.t4 = 1550
                print("Moving Down")
                depthPub.publish(dt)

            elif key == 'w':
                vt.t1 = 1550
                vt.t2 = 1550
                vt.t3 = 1550
                vt.t4 = 1550
                print("Moving forward")
                vectorPub.publish(vt)

            elif key == 'a':
                vt.t1 = 1550
                vt.t2 = 1450
                vt.t3 = 1450
                vt.t4 = 1550
                print("Moving left")
                vectorPub.publish(vt)

            elif key == 's':
                vt.t1 = 1450
                vt.t2 = 1450
                vt.t3 = 1450
                vt.t4 = 1450
                print("Moving backward")
                vectorPub.publish(vt)

            elif key == 'd':
                vt.t1 = 1450
                vt.t2 = 1550
                vt.t3 = 1550
                vt.t4 = 1450
                print("Moving right")
                vectorPub.publish(vt)

            elif key == 'q':
                print("resetting and quitting")
                dt.t1 = 1500
                dt.t2 = 1500
                dt.t3 = 1500
                dt.t4 = 1500
                vt.t1 = 1550
                vt.t2 = 1550
                vt.t3 = 1550
                vt.t4 = 1550
                break

            elif key == 'r':
                dt.t1 = 1500
                dt.t2 = 1500
                dt.t3 = 1500
                dt.t4 = 1500
                vt.t1 = 1550
                vt.t2 = 1550
                vt.t3 = 1550
                vt.t4 = 1550
                print("resetting vector and depth to 1500")
                vectorPub.publish(vt)
                depthPub.publish(dt)

            else:
                print("key not binded")

    except rospy.ROSInterruptException:
        pass

    finally:
        dt = ThrusterMsg()
        vt = ThrusterMsg()

        dt.t1 = 1500
        dt.t2 = 1500
        dt.t3 = 1500
        dt.t4 = 1500

        vt.t1 = 1500
        vt.t2 = 1500
        vt.t3 = 1500
        vt.t4 = 1500

        depthPub.publish(dt)
        vectorPub.publish(vt)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)





'''
# -*- coding: utf-8 -*-
from __future__ import absolute_import, division, unicode_literals, print_function

import tty, termios
import sys
if sys.version_info.major < 3:
    import thread as _thread
else:
    import _thread
import time


try:
    from msvcrt import getch  # try to import Windows version
except ImportError:
    def getch():   # define non-Windows version
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def keypress():
    global char
    char = getch()

def main():
    global char
    char = None
    _thread.start_new_thread(keypress, ())

    while True:
        if char is not None:
            try:
                print("Key pressed is " + char.decode('utf-8'))
            except UnicodeDecodeError:
                print("character can not be decoded, sorry!")
                char = None
            _thread.start_new_thread(keypress, ())
            if char == 'q' or char == '\x1b':  # x1b is ESC
                exit()
            char = None
        print("Program is running")
        time.sleep(1)

if __name__ == "__main__":
    main()
'''
