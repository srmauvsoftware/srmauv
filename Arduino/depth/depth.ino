#include <ros.h>
#include "depth.h"
#include <thrusters/ThrusterMsg.h>
#include <Servo.h>

ros::NodeHandle nh;
thrusters::ThrusterMsg depthtThruster;
thrusters::ThrusterMsg vectorThruster;

Servo td1;
Servo td2;
Servo td3;
Servo td4;

Servo tfr;
Servo tfl;
Servo trr;
Servo trl;

void depthMessageCb( const thrusters::ThrusterMsg& msg){
  td1.write(msg.t1);
  td2.write(msg.t2);
  td3.write(msg.t3);
  td4.write(msg.t4);
}

void vectorMessageCb( const thrusters::ThrusterMsg& msg)
{
  tfr.write(msg.t1);
  tfl.write(msg.t2);
  trr.write(msg.t3);
  trl.write(msg.t4);
}

ros::Subscriber<thrusters::ThrusterMsg> depthSub("/depthThruster", &depthMessageCb );
ros::Subscriber<thrusters::ThrusterMsg> vectorSub("/vectorThruster", &vectorMessageCb );


void setup()
{ 
  td1.attach(TD1);
  td2.attach(TD2);
  td3.attach(TD3);
  td4.attach(TD4);
  td1.write(1500);
  td2.write(1500);
  td3.write(1500);
  td4.write(1500);
  
  delay(1000);
  
  tfr.attach(TFR);
  tfl.attach(TFL);
  trr.attach(TRR);
  trl.attach(TRL);
  tfr.write(1500);
  tfl.write(1500);
  trr.write(1500);
  trl.write(1500);
  
  delay(1000);
  nh.initNode();
  nh.subscribe(depthSub);
  nh.subscribe(vectorSub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

