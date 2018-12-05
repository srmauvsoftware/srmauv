#!/usr/bin/env python
# Software License Agreement (BSD License)

# Sparton Digital Compass ROS Driver for AHRS-8/GEDC-6
# Copyright (c) 2013, Cheng-Lung Lee, University of Detroit Mercy.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Sparton Digital Compass ROS Driver for AHRS-8/GEDC-6
# Copyright (c) 2013, Cheng-Lung Lee, University of Detroit Mercy.

# Changelog

# 2013.01.06 Add IMU message
# 2012.12.13 Use Pos2D message, normalized to 0 ~ 2*PI
#


import roslib; roslib.load_manifest('SpartonCompassIMU')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float64
import serial, string, math, time, calendar

#import tf
from tf.transformations import euler_from_quaternion , quaternion_from_euler
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


def wrapTo2PI(theta):
    '''Normalize an angle in radians to [0, 2*pi]
    '''
    return theta % (2.*math.pi)

def wrapToPI(theta):
    '''Normalize an angle in radians to [-pi, pi]
    '''
    return (wrapTo2PI(theta+math.pi) - math.pi)

def Spartonshutdownhook():
    global D_Compass
    global myStr1
    print "Sparton shutdown time!"
    D_Compass.write(myStr1) # stop data stream before close port
    D_Compass.flush() # flush data out
    rospy.loginfo('Closing Digital Compass Serial port')
    D_Compass.close() #Close D_Compass serial port
    # rospy.on_shutdown(Spartonshutdownhook)
if __name__ == '__main__':
    global D_Compass
    global myStr1
    rospy.init_node('SpartonDigitalCompassIMU')
    Pos_pub  = rospy.Publisher('imu/HeadingTrue', Pose2D)
    PosD_pub = rospy.Publisher('imu/HeadingTrue_degree', Pose2D)
    PosD_pubD = rospy.Publisher('imu/Heading_degree/theta', Float64)
    Imu_pub = rospy.Publisher('imu/data', Imu)
    SpartonPose2D=Pose2D()
    SpartonPose2D.x=float(0.0)
    SpartonPose2D.y=float(0.0)
    SpartonPose2D_D=SpartonPose2D
    #Init D_Compass port
    D_Compassport = rospy.get_param('~port','/dev/ttyUSB0')
    D_Compassrate = rospy.get_param('~baud',115200)
    #<!--printmodulus 60:10Hz 40:15~17  35:17~18Hz 30:21Hz 25:23~27Hz ,20: 30~35Hz,15:35~55Hz 10: 55~76 Hz,  5: 70~100 Hz, 1:70~100 Hz -->
    D_Compassprintmodulus = rospy.get_param('~printmodulus',1)
    
    #http://www.ngdc.noaa.gov/geomag-web/#declination -->
    # Detroit Magnetic declination           : 2013-06-10 	7 30' 6"  W changing by  2.7' W per year = -7 30'6"=-7.501666666666667 degree ,
    # Oakland University Magnetic declination: 2013-06-10 	7 27' 46" W changing by  2.7' W per year =         =-7.462777777777778 degree
    #http://www.firefightermath.org/index.php?option=com_content&view=article&id=56&Itemid=70
    #in NED , North-East-Down system
    #Magnetic = true + westerly declination , True= Magnetic - westerly declination
    #Magnetic = true - easterly declination , True= Magnetic + easterly declination
    
    
    #Digital compass heading offset in degree   
    # D_Compass_offset = rospy.get_param('~offset',0.)
    D_Compass_declination = rospy.get_param('~declination',-7.462777777777778)* (math.pi/180.0)
    # By defaule IMU use Megnatic North as zero degree in Quaternion
    # If we want to use it directly with GPS-UTM x,y as Global Heading, East is our zero 
    D_Compass_UseEastAsZero = rospy.get_param('~UseEastAsZero',True)
    
    # use this try to control miss sync in USB-serial, when it happends, must restart
    Checksum_error_limits   =rospy.get_param('~Checksum_error_limits', 10.)
    checksum_error_counter=0
    
    imu_data = Imu()
    imu_data = Imu(header=rospy.Header(frame_id="SpartonCompassIMU"))
    
    #TODO find a right way to convert imu acceleration/angularvel./orientation accuracy to covariance
    imu_data.orientation_covariance = [1e-6, 0, 0, 
                                       0, 1e-6, 0, 
                                       0, 0, 1e-6]
    
    imu_data.angular_velocity_covariance = [1e-6, 0, 0,
                                            0, 1e-6, 0, 
                                            0, 0, 1e-6]
    
    imu_data.linear_acceleration_covariance = [1e-6, 0, 0, 
                                               0, 1e-6, 0, 
                                               0, 0, 1e-6]
    myStr1='\r\n\r\nprinttrigger 0 set drop\r\n'
    # this is with true north setting
    # myStr2='printmask gyrop_trigger accelp_trigger or quat_trigger or yawt_trigger or time_trigger or set drop\r\n'
    # this is output magnetic north 
    myStr2='printmask gyrop_trigger accelp_trigger or quat_trigger or time_trigger or set drop\r\n'
        # set the number high to get lower update rate , the IMU data is 100Hz rate , the string is 130 byte with 10 bit/byte , the max sampling rate is 88Hz
        # printmodulus 60:10Hz 40:15~17  35:17~18Hz 30:21Hz 25:23~27Hz ,20: 30~35Hz,15:35~55Hz 10: 55~76 Hz,  5: 70~100 Hz, 1:70~100 Hz
    myStr_printmodulus=('printmodulus %i set drop\r\n' % D_Compassprintmodulus  )
    myStr3='printtrigger printmask set drop\r\n'

    rospy.on_shutdown(Spartonshutdownhook)

    try:
        #talker()
        #ReadCompass()
        #Setup Compass serial port
        D_Compass = serial.Serial(port=D_Compassport, baudrate=D_Compassrate, timeout=.5)
        # Stop continus mode
        D_Compass.write(myStr1)
        D_Compass.flush() # flush data out
        time.sleep(0.5)
        # readout all data, if any
        rospy.loginfo("Send Stop Continus mode to Digital Compass Got bytes %i" % D_Compass.inWaiting() ) # should got OK here
        if (D_Compass.inWaiting() >0):
                #read out all datas, the response shuldbe OK
                data=D_Compass.read(D_Compass.inWaiting())
                print("Send to Digital Compass: %s Got: %s" % (myStr1 ,data)) # should got OK here

        else:
                #sned error no data in buffer error
                rospy.logerr('[Sparton][1]Received No data from DigitalCompass. Shutdown!')
                rospy.signal_shutdown('Received No data from DigitalCompass')
        D_Compass.write(myStr2) # send printmask
        data = D_Compass.readline()
#        rospy.loginfo("Send to Digital Compass: %s" % myStr2 ) # should got OK here
#        rospy.loginfo("Send to Digital Compass Got: %s" % data ) # should got OK here
        if (len(data) >0):
                #read out all datas, the response shuldbe OK
                rospy.loginfo("Send to Digital Compass: %s" % myStr2 ) # should got OK here
                rospy.loginfo("Send to Digital Compass. Got: %s" % data ) # should got OK here
                D_Compass.write(myStr_printmodulus) # setup printmodule
                data = D_Compass.readline()
                rospy.loginfo("Send to Digital Compass: %s " % myStr_printmodulus) # should got OK here
                rospy.loginfo("Send to Digital Compass. Got: %s" % data) # should got OK here

                D_Compass.write(myStr3) # start the data streaming
                data = D_Compass.readline()
                rospy.loginfo("Send to Digital Compass: %s " % myStr3 ) # should got OK here
                rospy.loginfo("Send to Digital Compass. Got: %s" % data) # should got OK here
                rospy.loginfo('Digital Compass Setup Complete!')
        else:
                #sned error no data in buffer
                rospy.logerr('[Sparton][2]Received No data from DigitalCompass. Shutdown!')
                rospy.signal_shutdown('Received No data from DigitalCompass')

        #### loop-back test only
        #Testdata='P:,%i,ap,-6.34,-22.46,1011.71,gp,0.00,0.00,-0.00,yt,342.53,q,0.98,-0.01,0.01,-0.15\n'
        #          0  1  2  3     4      5       6  7    8    9     10 11     1213   14    15   16  
        #Testdata='P:,%i,ap,-6.34,-22.46,1011.71,gp,0.00,0.00,-0.00,q,0.98,-0.01,0.01,-0.15\n'
        #Testdata='P:,%i,ap,-6.34,-22.46,1011.71,gp,0.00,0.00,-0.00,q,1.,0.,0.,0.\n'
        #          0  1  2  3     4      5       6  7    8    9    10 11   12    13   14  
        #i=0
        ### end of loop-back test
        while not rospy.is_shutdown():
            
            ### loop-back test only
            #i+=1
            #D_Compass.write(Testdata % i) # send testdata and do loop-back in RS232 for debug
            ### end of loop-back test
            
            data = D_Compass.readline()
            #rospy.loginfo("Received a sentence: %s" % data)

            #if not check_checksum(data):
            #    rospy.logerr("Received a sentence with an invalid checksum. Sentence was: %s" % data)
            #    continue

            #DatatimeNow = rospy.get_rostime()
            DataTimeSec=rospy.get_time()
            fields = data.split(',')
            #print fields[0]+fields[2]+fields[6]+fields[10] #P:apgpq

            try:
                if len(fields)>14:
                            if 'P:apgpq' == (fields[0]+fields[2]+fields[6]+fields[10]):

                                #      0  1 mSec 2  3Ax  4Ay     5Az     5  7Gx  8Gy  9G    10 11YawT 1213w  14x   15y  16z
                                #data='P:,878979,ap,-6.34,-22.46,1011.71,gp,0.00,0.00,-0.00,yt,342.53,q,0.98,-0.01,0.01,-0.15'
                                #data='P:,%i,ap,-6.34,-22.46,1011.71,gp,0.00,0.00,-0.00,q,0.98,-0.01,0.01,-0.15\n'
                                #      0  1  2  3     4      5       6  7    8    9    10 11   12    13   14  

                                Ax=float(fields[3])/1000.*9.81 # convert to m/s^2 from mg/s
                                Ay=float(fields[4])/1000.*9.81
                                Az=float(fields[5])/1000.*9.81
                                Gx=float(fields[7]) * (math.pi/180.0) # convert to radians from degrees
                                Gy=float(fields[8]) * (math.pi/180.0)
                                Gz=float(fields[9]) * (math.pi/180.0)
                                w =float(fields[11])
                                x =float(fields[12])
                                y =float(fields[13])
                                z =float(fields[14])
                                
                                #imu_data.header.stamp = rospy.Time.now() # Should add an offset here
                                imu_data.header.stamp = rospy.Time.from_sec(DataTimeSec-len(data)/11520.) # this is timestamp with a bit time offset 10bit per byte @115200bps
                                imu_data.orientation = Quaternion()
                                # IMU outputs [w,x,y,z] NED, convert to [x,y,z,w] ENU
                                imu_data.orientation.x = y
                                imu_data.orientation.y = x
                                imu_data.orientation.z = -z
                                imu_data.orientation.w = w
                                # right now already in ENU but still use 0 is still at Magnetic north
                                
                                
                                #Convert to euler
                                #(r,p,y)=euler_from_quaternion(quaternion, axes='sxyz')
                                #print "Start here"
                                #NED=  [x,y,z,w] ; print NED
                                ENU=  [y,x,-z,w] ; #print ENU
                                #angle_NED=euler_from_quaternion(NED, axes='sxyz'); print angle_NED
                                angle_ENU=euler_from_quaternion(ENU, axes='sxyz'); #print angle_ENU
                                
                                #add 90 degree and -declination , ROS= angle_ENU +90 degree - declination
                                angle_ROS=(angle_ENU[0],angle_ENU[1],angle_ENU[2]+math.pi/2.- D_Compass_declination);     #print angle_ROS
                                q_ROS=quaternion_from_euler(angle_ROS[0],angle_ROS[1],angle_ROS[2], axes='sxyz');         #print q_ROS

                                #(r, p, y) = euler_from_quaternion([imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w])
                                #print (r,p,y)
                                # Heading = 90 degree + Yaw - Magnetic declination.
                                #yaw_ros= math.pi/2. +y- D_Compass_declination
                                #print (yaw_ros)
                                #q = quaternion_from_euler(ai, aj, ak, axes='sxyz') ,ai, aj, ak : Euler's roll, pitch and yaw angles 
                                
                                if D_Compass_UseEastAsZero :
                                
                                    #(imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w)=quaternion_from_euler(r, p, yaw_ros, axes='sxyz')
                                    (imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w)=q_ROS
                                    #print (imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w)
                                    #print imu_data.orientation
                                else:
                                    # if not use East As Zero, just use compass's origional data
                                    pass
                                # again note NED to ENU converstion
                                imu_data.angular_velocity.x = Gy
                                imu_data.angular_velocity.y = Gx
                                imu_data.angular_velocity.z = -Gz
                                # again note NED to ENU converstion
                                imu_data.linear_acceleration.x = Ay
                                imu_data.linear_acceleration.y = Ax
                                imu_data.linear_acceleration.z = -Az

                                Imu_pub.publish(imu_data)

                                #SpartonPose2D.y=1000./(float(fields[1])-SpartonPose2D.x) # put update rate here for debug the update rate
                                #SpartonPose2D.x=float(fields[1]) # put mSec tick here for debug the speed
                                #SpartonPose2D.theta = wrapToPI(math.radians(90.-float(fields[11])-D_Compass_offset))
                                #SpartonPose2D.theta = wrapToPI(yaw_ros)
                                SpartonPose2D.theta = wrapToPI(angle_ROS[2])
				
                                #print SpartonPose2D.theta/math.pi *180.
                                Pos_pub.publish(SpartonPose2D)
                                SpartonPose2D_D.theta =SpartonPose2D.theta/math.pi *180.
                                PosD_pub.publish(SpartonPose2D_D)
                                PosD_pubD.publish(SpartonPose2D_D.theta)
                                # reset checksum_error_counter when you have good data
                                checksum_error_counter=0

                            else:
                                rospy.logerr("[Sparton][3]Received a sentence but not correct. Sentence was: %s" % data)
                                # Usually when USB-Serial miss sync wiill cause error
                                checksum_error_counter+=1
                                if (checksum_error_counter > Checksum_error_limits ):
                                    #shutdown DGPS node
                                    #raise SystemExit, 0
                                    rospy.logfatal('[Sparton Compass] Too much back to back checksumn error in Sparton Compass data. Shutdown!')
                                    rospy.signal_shutdown('Too much back to back checksum error in Sparton Compass data')
                                    


                else:
                        rospy.logerr("[Sparton][4]Received a sentence but not correct. Sentence was: %s" % data)
                        # Usually when USB-Serial miss sync wiill cause error
                        checksum_error_counter+=1
                        if (checksum_error_counter > Checksum_error_limits ):
                            #shutdown DGPS node
                            #raise SystemExit, 0
                            rospy.logfatal('[Sparton Compass] Too much back to back checksumn error in Sparton Compass data. Shutdown!')
                            rospy.signal_shutdown('Too much back to back checksum error in Sparton Compass data')

            except ValueError as e:
                rospy.logwarn("Value error, likely due to missing fields in the data messages.Sentence was: %s" % data)

            # no loop, delay, ROSspin() here, we try to read all the data asap
        D_Compass.write(myStr1) # stop data stream before close port
        D_Compass.flush() # flush data out

        rospy.loginfo('Closing Digital Compass Serial port')
        D_Compass.close() #Close D_Compass serial port
    except rospy.ROSInterruptException:
        pass
