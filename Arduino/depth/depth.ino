#include <Servo.h>
#include "depth.h"
#include <ros.h>
#include <SimpleKalmanFilter.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

ros::NodeHandle(nh);
SimpleKalmanFilter pressureKF(1, 1, 0.01);

void messageCb( const std_msgs::Bool& msg){
  Serial.println("ABCD");
  if (msg.data) {
    digitalWrite(4,0);
    digitalWrite(7,0);
    digitalWrite(8,0);
    digitalWrite(13,0);
  } else {
    digitalWrite(4,1);
    digitalWrite(7,1);
    digitalWrite(8,1);
    digitalWrite(13,1);
  }

  
  //digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  
}

std_msgs::Float64 depth;
ros::Publisher depth_pub("/depth", &depth);
ros::Subscriber<std_msgs::Bool> sub("/torpedo", &messageCb );

void setup(){
  pinMode(13, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(8,OUTPUT);
  
  pinMode(7,OUTPUT);
  delay(1000);
  Serial.begin(9600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(depth_pub);
}
 
void loop(){
  int pressure = analogRead(A0);
  depth.data = pressureKF.updateEstimate(pressure);
  depth_pub.publish(&depth);
  nh.spinOnce();
}
