#include <Servo.h>
#include "depth.h"
#include <ros.h>
#include <SimpleKalmanFilter.h>
#include <std_msgs/Float64.h>

ros::NodeHandle(nh);
SimpleKalmanFilter pressureKF(1, 1, 0.01);

std_msgs::Float64 depth;
ros::Publisher depth_pub("/depth", &depth);

void setup(){
  delay(1000);
  nh.initNode();
  nh.advertise(depth_pub);
}
 
void loop(){
  int pressure = analogRead(A0);
  depth.data = pressureKF.updateEstimate(pressure);
  depth_pub.publish(&depth);
  nh.spinOnce();
}
