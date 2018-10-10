#include <ros.h>
#include "depth.h"
#include <thrusters/DepthThrusterMsg.h>
#include <thrusters/VectorThrusterMsg.h>
#include <Servo.h>

ros::NodeHandle nh;
thrusters::DepthThrusterMsg depthThruster;
thrusters::VectorThrusterMsg vectorThruster;

Servo td1;
Servo td2;
Servo td3;
Servo td4;

Servo tfr;
Servo tfl;
Servo trr;
Servo trl;

void depthMessageCb( const thrusters::DepthThrusterMsg& msg){
  depthThruster = msg;
}

void vectorMessageCb( const thrusters::VectorThrusterMsg& msg){
  vectorThruster = msg;
}

ros::Subscriber<thrusters::DepthThrusterMsg> depthSub("/depthThruster", &depthMessageCb );
ros::Subscriber<thrusters::VectorThrusterMsg> vectorSub("/vectorThruster", &vectorMessageCb );

void setup()
{
  initThrusters();
  initNodes();
}

void loop()
{
  runDepthThrusters();
  runVectorThrusters();
  
  nh.spinOnce();
  delay(1);
}

void runVectorThrusters(){
  tfr.write(vectorThruster.tfr);
  tfl.write(vectorThruster.tfl);
  trr.write(vectorThruster.trr);
  trl.write(vectorThruster.trl);
}

void runDepthThrusters(){
  td1.write(depthThruster.td1);
  td2.write(depthThruster.td2);
  td3.write(depthThruster.td3);
  td4.write(depthThruster.td4);
}

void initNodes(){
  nh.initNode();
  nh.subscribe(depthSub);
  nh.subscribe(vectorSub);
}
void initThrusters(){
  td1.attach(TD1);
  td1.write(1500);
  delay(1000);
  td2.attach(TD2);
  td2.write(1500);
  delay(1000);
  td3.attach(TD3);
  td3.write(1500);
  delay(1000);
  td4.attach(TD4);
  td4.write(1500);
  delay(1000);

  tfr.attach(TFR);
  tfr.write(1500);
  delay(1000);
  tfl.attach(TFL);
  tfl.write(1500);
  delay(1000);
  trr.attach(TRR);
  trr.write(1500);
  delay(1000);
  trl.attach(TRL);
  trl.write(1500);
  delay(1000);
}
