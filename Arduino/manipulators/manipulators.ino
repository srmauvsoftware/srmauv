#include <ros.h>
#include <Servo.h>
#include <ArduinoHardware.h>
#include "manipulators.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
//#include <alpheus_msgs/depthThruster.h> not being used

Servo tfr;
Servo tfl;
Servo trr;
Servo trl;

ros::NodeHandle nh;
std_msgs::Bool emergency;
int control_effort;

//void emergencyCallback(const std_msgs::Bool &msg);
void torpedoCallback(const std_msgs::Bool &msg);
void dropperCallback(const std_msgs::Bool &msg);
void getThrusterControlEffort(const std_msgs::Float64 &msg);

//ros::Subscriber<std_msgs::Bool> emergency_sub("emergency",emergencyCallback);
ros::Subscriber<std_msgs::Float64>thruster_sub("/vectorThruster", getThrusterControlEffort);
ros::Subscriber<std_msgs::Bool> torpedo_sub("torpedo",torpedoCallback);
ros::Subscriber<std_msgs::Bool> dropper_sub("dropper",dropperCallback);

boolean kill = false;
static uint32_t currentTime,loopTime, fast_loop,time_elapsed, medium_loop, slow_loop;

void setup(){
  initThrusters();
  initManipulators();
  nh.initNode();
  initTopics();
  time_elapsed=0;
  currentTime=millis();
  loopTime=currentTime;
}

void loop(){
  currentTime = millis();
  if(currentTime >= (fast_loop + 50)){
    runThrusters();
    fast_loop = currentTime;
    }
  nh.spinOnce();
}

void initThrusters(){
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

void initManipulators(){
   pinMode(DROPPER,OUTPUT);
   pinMode(TORPEDO,OUTPUT);
   digitalWrite(DROPPER,HIGH);
   digitalWrite(TORPEDO,HIGH);
}

void initTopics(){
  //nh.subscribe(emergency_sub);
  nh.subscribe(torpedo_sub);
  nh.subscribe(dropper_sub);
}

void runThrusters(){
  if(kill){
    tfr.write(1500);
    tfl.write(1500);
    trr.write(1500);
    trl.write(1500);
  }
  tfr.write(1500 + control_effort);
  tfl.write(1500 + control_effort);
  trr.write(1500 - control_effort);
  trl.write(1500 - control_effort);
}

void getThrusterControlEffort(const std_msgs::Float64 &msg){
  control_effort = msg.data;
}

void torpedoCallback(const std_msgs::Bool &msg){
   if(msg.data==true){
     digitalWrite(TORPEDO,LOW);
     delay(500);
     digitalWrite(TORPEDO,HIGH);
  }
}

void dropperCallback(const std_msgs::Bool &msg){
   if(msg.data==true){
       digitalWrite(DROPPER,LOW);
       delay(2500);
       digitalWrite(DROPPER,HIGH);
  }
}

void emergencyMode(){
  emergency.data = true;
  while(true){
    //emergency_pub.publish(&emergency);
    tfr.write(1500);
    tfl.write(1500);
    trr.write(1500);
    trl.write(1500);
  }
 }
