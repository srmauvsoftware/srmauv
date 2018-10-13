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
};

void Controller::Controller()
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
  }
}
void Controller::depthThrusterCallback(const thrusters::DepthThrusterMsg::ConstPtr& msg)
{
  ROS_INFO("ValueL %d", msg->td1);
}
void Controller::vectorThrusterCallback(const thrusters::VectorThrusterMsg::ConstPtr& msg)
{
  ROS_INFO("ValueL %d", msg->trr);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "jetson");
  ros::NodeHandle n;

  Controller controller = new Controller();
  ros::Subscriber sub = n.subscribe("/depthThruster", 1000, &Controller::depthThrusterCallback, &controller);
  ros::Subscriber sub = n.subscribe("/vectorThruster", 1000, &Controller::vectorThrusterCallback, &controller);

  ros::spin();

  return 0;
}
