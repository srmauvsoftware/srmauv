/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include "depth.h"
#include <thrusters/ThrusterMsg.h>
#include <Servo.h>

ros::NodeHandle  nh;
thrusters::ThrusterMsg thruster;/
Servo t1;
Servo t2;
Servo t3;
Servo t4;

void messageCb( const thrusters::ThrusterMsg& msg){
  t1.write(msg.t1);
  t2.write(msg.t2);
  t3.write(msg.t3);
  t4.write(msg.t4);
}

ros::Subscriber<thrusters::ThrusterMsg> sub("/depthThruster", &messageCb );

void setup()
{ 
  t1.attach(TD1);
  t2.attach(TD2);
  t3.attach(TD3);
  t4.attach(TD4);
  t1.write(1500);
  t2.write(1500);
  t3.write(1500);
  t4.write(1500);
  delay(1000);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

