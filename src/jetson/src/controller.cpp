#include "controller.h"
#include <time.h>
#define TFL 2
#define TFR 4 
#define TRL 10
#define TRR 12 
#define TD1 8  
#define TD2 14 
#define TD3 6  
#define TD4 0

Controller::Controller(){
	controller = new PCA9685(0x70);
	controller->openPCA9685();
	controller->setAllPWM(0,0) ;
	controller->reset();
 	controller->setPWMFrequency(50);
	sleep(1);
	controller->setPWM(TD1, 0, 290);sleep(1);	
    	controller->setPWM(TD2, 0, 290);sleep(1);
        controller->setPWM(TD3, 0, 290);sleep(1);
    	controller->setPWM(TD4, 0, 290);sleep(1);
    	controller->setPWM(TFL, 0, 290);sleep(1);
    	controller->setPWM(TFR, 0, 290);sleep(1);
    	controller->setPWM(TRR, 0, 290);sleep(1);
	controller->setPWM(TRL, 0, 290);sleep(1);
	sub_depthThruster = node.subscribe("/depthThruster", 100, &Controller::depthThrusterCb, this);
	sub_vectorThruster = node.subscribe("/vectorThruster", 100, &Controller::vectorThrusterCb, this);
	ROS_INFO("Servo controller is ready...");
	
	ROS_INFO("Servo 12 Initialized...");

}

Controller::~Controller() {
controller->closePCA9685();
}

void Controller::depthThrusterCb (const thrusters::DepthThrusterMsg::ConstPtr& msg){
    	controller->setPWM(TD1, 0, msg->td1);
	controller->setPWM(TD2, 0, msg->td2);
        controller->setPWM(TD3, 0, msg->td3);
    	controller->setPWM(TD4, 0, msg->td4);
	ROS_INFO("Hello Depth Thrusters");
}

void Controller::vectorThrusterCb (const thrusters::VectorThrusterMsg::ConstPtr& msg){
	controller->setPWM(TRL, 0, msg->trl);
    	controller->setPWM(TFL, 0, msg->tfl);
    	controller->setPWM(TFR, 0, msg->tfr);
    	controller->setPWM(TRR, 0, msg->trr);
	//ROS_INFO("Hello Vector Thrusters");
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "jetson");
	Controller c;
    	ros::spin();

}


