#include "ros/ros.h"
#include "thrusters/DepthThrusterMsg.h"
#include "thrusters/VectorThrusterMsg.h"

#include <JHPWMPCA9685.h>

PCA9685 pca9685;

void depthThrusterCallback(const thrusters::DepthThrusterMsg::ConstPtr& msg){
  pca9685->setPWM(0, 0, msg.td1);
  pca9685->setPWM(1, 0, msg.td2);
  pca9685->setPWM(2, 0, msg.td3);
  pca9685->setPWM(3, 0, msg.td4);
}

void vectorThrusterCallback(const thrusters::DepthThrusterMsg:;ConstPtr& msg){
  pca9685->setPWM(4, 0, msg.tfr);
  pca9685->setPWM(5, 0, msg.tfl);
  pca9685->setPWM(6, 0, msg.trr);
  pca9685->setPWM(7, 0, msg.trl);
}

int main(int argc, char **argv){
  *pca9685 = new PCA9685(0x70);
  int err = pca9685->openPCA9685();
  if (err < 0){
        ROS_INFO("Error: %d", pca9685->error);
    } else {
        ROS_INFO("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
        pca9685->setAllPWM(0,0) ;
        pca9685->reset() ;
        pca9685->setPWMFrequency(60) ;
        ros::init(argc, argv, "jetsonPCA9685");
        ros::NodeHandle n;
        ros::Subscriber depthSubscriber = n.subscribe("/depthThruster", 1000, depthThrusterCallback);
        ros::Subscriber vectorSubscriber = n.subscribe("/vectorThruster", 1000, vectorThrusterCallback);
        ros::spin();
  }
return 0;
}
