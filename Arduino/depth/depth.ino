#include <Servo.h>
#include "depth.h"
#include <ros.h>
#include <ArduinoHardware.h>
#include <thrusters/ThrusterMsg.h>
// #include <alpheus_msgs/depth.h>
#include <std_msgs/Float64.h>
#include <SimpleKalmanFilter.h>
#include <std_msgs/Bool.h>


boolean kill = false;
Servo td1; // Thruster Depth
Servo td2;
Servo td3;
Servo td4;
SimpleKalmanFilter pressureKF(1, 1, 0.01);

ros::NodeHandle(nh);
std_msgs::Float64 depth;
// alpheus_msgs::depth depth;
thrusters::ThrusterMsg thruster;
std_msgs::Bool emergency;
int pressure;
static uint32_t currentTime,loopTime, fast_loop,time_elapsed, medium_loop, slow_loop;


// void getThrusterPWM(const alpheus_msgs::depthThruster &msg);
void getThrusterPWM(const Thrusters.msg::depthThruster &msg);
void getPressure(const alpheus_msgs::depth &depth);

//ros::Subscriber<> emergency_pub("/emergency",&emergency);
ros::Subscriber<thrusters::ThrusterMsg>thruster_sub("/depthThruster", getThrusterPWM);
ros::Publisher depth_pub("/depth", &depth);

void setup(){

  delay(500);

  initThrusters();
  //pinMode(LCD,OUTPUT);
  //digitalWrite(LCD,LOW);
  nh.initNode();
  initPressure();
  initTopics();
  time_elapsed=0;
  currentTime=millis();
  loopTime=currentTime;

}

void loop(){

  currentTime=millis();

  // Every 10 ms read Pressure
  if(currentTime >= (fast_loop + 10)){
    getPressure();
    fast_loop=currentTime;
  }

  // Every 50 ms publish pressure and run thrusters
  if(currentTime >= (medium_loop + 50)){

    runThrusters();

    depth.depth = pressure;
    // depth.pressure = pressure;
    depth_pub.publish(&depth);

    nh.spinOnce();

    medium_loop = currentTime;
  }

  currentTime=millis();
  //
  if(currentTime >= ( slow_loop + 333)){
   detectLeak();
   slow_loop = currentTime;
  }
}

void initPressure(){
  pressure = analogRead(PRESSURE);
  delay(1000);
}

void initTopics(){
  nh.subscribe(thruster_sub);
  //nh.advertise(emergency_pub);
  nh.advertise(depth_pub);
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
}

void runThrusters(){
  if(kill){
    td1.write(1500);
    td2.write(1500);
    td3.write(1500);
    td4.write(1500);
  }
  td1.write(thruster.td1);
  td2.write(thruster.td2);
  td3.write(thruster.td3);
  td4.write(thruster.td4);
}

void getThrusterPWM(const alpheus_msgs::depthThruster &msg){
  thruster = msg;
}

void getPressure(){
  int new_pressure = analogRead(PRESSURE);
  pressure = pressureKF.updateEstimate(new_pressure);
}

void detectLeak(){
  if(analogRead(LEAK) < WATER_LEAK_THRESH){
    emergencyMode();
    kill = true;
  }
}

void emergencyMode(){
  emergency.data = true;
  while(true){
    //emergency_pub.publish(&emergency);
    td1.write(1500);
    td2.write(1500);
    td3.write(1500);
    td4.write(1500);
  }
 }
