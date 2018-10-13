#include "ros/ros.h"
#include "thrusters/DepthThrusterMsg.h"
#include "thrusters/VectorThrusterMsg.h"

#include <JHPWMPCA9685.h>

int t1;
int t2;
int t3;
int t4;

int t5;
int t6;
int t7;
int t8;

void depthThrusterCallback(const thrusters::DepthThrusterMsg::ConstPtr& msg);
void vectorThrusterCallback(const thrusters::VectorThrusterMsg::ConstPtr& msg);

void depthThrusterCallback(const thrusters::DepthThrusterMsg::ConstPtr& msg){
  ROS_INFO("%d",msg->td1);
  t1 = msg->td1;
  t2 = msg->td2;
  t3 = msg->td3;
  t4 = msg->td4;
}

void vectorThrusterCallback(const thrusters::VectorThrusterMsg::ConstPtr& msg){
  t5 = msg->tfr;
  t6 = msg->tfl;
  t7 = msg->trl;
  t8 = msg->trr;
}

int main(int argc, char **argv){
  PCA9685 *pca9685 = new PCA9685(0x70);
  int err = pca9685->openPCA9685();
  if (err < 0){
        ROS_INFO("Error: %d", pca9685->error);
    } else {
        ROS_INFO("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
        pca9685->setAllPWM(0,0) ;
        pca9685->reset() ;
        pca9685->setPWMFrequency(50) ;
        ros::init(argc, argv, "jetsonPCA9685");
        ros::NodeHandle n;
        ros::Subscriber depthSubscriber = n.subscribe("/depthThruster", 1000, depthThrusterCallback);
        ros::Subscriber vectorSubscriber = n.subscribe("/vectorThruster", 1000, vectorThrusterCallback);

        ros::Rate r(2);
        while(ros::ok()){
	pca9685->setPWM(12, 0, t1);
	}

        ros::spin();
  }
return 0;
}
