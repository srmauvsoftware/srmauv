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

int t1 = 1500;
int t2 = 1500;
int t3 = 1500;
int t4 = 1500;

int t5 = 1500;
int t6 = 1500;
int t7 = 1500;
int t8 = 1500;


void depthMessageCb( const thrusters::ThrusterMsg& msg){
  t1 = msg.t1;
  t2 = msg.t2;
  t3 = msg.t3;
  t4 = msg.t4;
}

void vectorMessageCb( const thrusters::ThrusterMsg& msg)
{
  t5 = msg.t1;
  t6 = msg.t2;
  t7 = msg.t3;
  t8 = msg.t4;
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
  
  td1.write(t1);
  td2.write(t2);
  td3.write(t3);
  td4.write(t4);
  
  tfr.write(t5);
  tfl.write(t6);
  trr.write(t7);
  trl.write(t8);
  
  nh.spinOnce();
  delay(1);
}

