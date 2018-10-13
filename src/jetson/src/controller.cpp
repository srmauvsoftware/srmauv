#include "controller.h"


Controller::Controller(){
	controller = new PCA9685(0x70);
	controller->setAllPWM(0,0) ;
	controller->reset();
 	controller->setPWMFreq(50);

	sub_depthThruster = node.subscribe("/depthThruster", 100, &Controller::depthThrusterCb, this);
	sub_vectorThruster = node.subscribe("/vectorThruster", 100, &Controller::vectorThrusterCb, this);
	ROS_INFO("Servo controller is ready...");
	controller->setPWM(8, 0, 290);
    	controller->setPWM(14, 0, 290);
    	controller->setPWM(6, 0, 290);
    	controller->setPWM(0, 0, 290);
    	controller->setPWM(2, 0, 290);
    	controller->setPWM(4, 0, 290);
    	controller->setPWM(10, 0, 290);
    	controller->setPWM(12, 0, 290);
}

void Controller::depthThrusterCb (const thrusters::DepthThrusterMsg::ConstPtr& msg){
	controller->setPWM(8, 0, msg->td1);
    	controller->setPWM(14, 0, msg->td2);
    	controller->setPWM(6, 0, msg->td3);
    	controller->setPWM(0, 0, msg->td4);
}

void Controller::vectorThrusterCb (const thrusters::VectorThrusterMsg::ConstPtr& msg){
	controller->setPWM(2, 0, msg->trl);
    	controller->setPWM(4, 0, msg->tfl);
    	controller->setPWM(10, 0, msg->tfr);
    	controller->setPWM(12, 0, msg->trr);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "jetson");
	Controller c;
    	ros::spin();
}


