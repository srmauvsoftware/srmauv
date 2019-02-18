#include <Servo.h>
#include "depth.h"
#include <ros.h>
#include <SimpleKalmanFilter.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

ros::NodeHandle(nh);
SimpleKalmanFilter pressureKF(1, 1, 0.01);
Adafuit_BNO055 bno = Adafuit_BNO055(28);

void torpedoCb( const std_msgs::Bool& msg){
  
  if (msg.data) {
    digitalWrite(4,0);
    digitalWrite(7,0);
  } else {
    digitalWrite(4,1);
    digitalWrite(7,1);
  }  
}

void dropperCb( const std_msgs::Bool& msg){
  
  if (msg.data) {
    digitalWrite(8,0);
//    digitalWrite(8,0);
  } else {
    digitalWrite(8,1);
    delay(1000);
    digitalWrite(9,0);
    delay(1000);
    digitalWrite(9,1);
  }  
}


std_msgs::Float64 depth;
ros::Publisher depth_pub("/depth", &depth);
ros::Subscriber<std_msgs::Bool> tsub("/torpedo", &torpedoCb );
ros::Subscriber<std_msgs::Bool> dsub("/dropper", &dropperCb );
std_msgs::Float64 p;
std_msgs::Float64 i;
std_msgs::Float64 d;
ros::Publisher p_pub("/p", &p);
ros::Publisher i_pub("/i", &i);
ros::Publisher d_pub("/d", &d);

void setup(){
  pinMode(9, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(7,OUTPUT);
  
  digitalWrite(4,1);
  digitalWrite(7,1);
  digitalWrite(8,1);
  digitalWrite(9,1);
  
  nh.initNode();
  nh.subscribe(tsub);
  nh.subscribe(dsub);
  nh.advertise(depth_pub);
}
 
void loop(){
  int pressure = analogRead(A0);
  depth.data = pressureKF.updateEstimate(pressure);
  depth_pub.publish(&depth);
  p_pub.publish(&p);
  i_pub.publish(&i);
  d_pub.publish(&d);
  nh.spinOnce();
}
