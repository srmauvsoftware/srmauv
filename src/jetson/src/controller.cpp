#include "ros/ros.h"
#include "thrusters/DepthThrusterMsg.h"
#include "thrusters/VectorThrusterMsg.h"
#include <JHPWMPCA9685.h>

class Controller
{
public:
  Controller();
  void depthThrusterCallback(const thrusters::DepthThrusterMsg::ConstPtr& msg);
  void vectorThrusterCallback(const thrusters::VectorThrusterMsg::ConstPtr& msg);
  void updateDepth();
  void updateVector();
  int td1 = 209;
  int td2 = 209;
  int td3 = 209;
  int td4= 209;
  int trl= 209;
  int tfl= 209;
  int tfr= 209;
  int trr= 209;
  PCA9685 *pca9685;
};
Controller::Controller()
{
  PCA9685 *pca9685 = new PCA9685(0x70);
  int err = pca9685->openPCA9685();
  if(err < 0){
    ROS_INFO("Error: %d", pca9685->error);
  }
  else{
    ROS_INFO("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
    pca9685->setAllPWM(0,0) ;
    pca9685->reset() ;
    pca9685->setPWMFrequency(50) ;

    pca9685->setPWM(8, 0, 290);
  pca9685->setPWM(14, 0, 290);
  pca9685->setPWM(6, 0, 290);
  pca9685->setPWM(0, 0, 290);
pca9685->setPWM(2, 0, 290);
  pca9685->setPWM(4, 0, 290);
  pca9685->setPWM(10, 0, 290);
  pca9685->setPWM(12, 0, 290);
  }
}
void Controller::depthThrusterCallback(const thrusters::DepthThrusterMsg::ConstPtr& msg)
{
  td1 = msg->td1;
  td2 = msg->td2;
  td3 = msg->td3;
  td4 = msg->td4;
  ROS_INFO("ValueL %d", msg->td1);
}
void Controller::vectorThrusterCallback(const thrusters::VectorThrusterMsg::ConstPtr& msg)
{
  ROS_INFO("ValueL %d", msg->trr);
  trl = msg->trl;
  tfl = msg->tfl;
  tfr = msg->tfr;
  trr = msg->trr;
}

void Controller::updateDepth()
{ 
  pca9685->setPWM(8, 0, td1);
  pca9685->setPWM(14, 0, td2);
  pca9685->setPWM(6, 0, td3);
  pca9685->setPWM(0, 0, td4);
}

void Controller::updateVector()
{
  pca9685->setPWM(2, 0, trl);
  pca9685->setPWM(4, 0, tfl);
  pca9685->setPWM(10, 0, tfr);
  pca9685->setPWM(12, 0, trr);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jetson");
  ros::NodeHandle n;

  Controller controller;
  ros::Subscriber depth_sub = n.subscribe("/depthThruster", 1000, &Controller::depthThrusterCallback, &controller);
  ros::Subscriber vector_sub = n.subscribe("/vectorThruster", 1000, &Controller::vectorThrusterCallback, &controller);

  ros::Rate r(2);
  while(ros::ok()){
	controller.updateDepth();
        controller.updateVector();
   }

  ros::spin();

  return 0;
}
