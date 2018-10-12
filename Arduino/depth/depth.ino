#include <Servo.h>
#include "depth.h"
#include <ros.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include "MS5837.h"
#include <std_msgs/Float64.h>

ros::NodeHandle(nh);
SimpleKalmanFilter pressureKF(1, 1, 0.01);
MS5837 pressure_sensor;

std_msgs::Float64 depth;
ros::Publisher depth_pub("/depth", &depth);

void setup(){
  delay(1000);
  nh.initNode();
  Wire.begin();
  pressure_sensor.init();
  pressure_sensor.setFluidDensity(997);
}

void loop(){
  pressure_sensor.read();
  depth.data = pressureKF.updateEstimate(pressure_sensor.depth());
  depth_pub.publish(&depth);
  nh.spinOnce();
}
