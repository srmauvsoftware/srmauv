#ifndef CONTROLLER_SUB_HPP_
#define CONTROLLER_SUB_HPP_

#include <ros/ros.h>
#include "thrusters/DepthThrusterMsg.h"
#include "thrusters/VectorThrusterMsg.h"
#include <JHPWMPCA9685.h>

class Controller {
	public:
		Controller();
		PCA9685 *controller;

	private:

		ros::NodeHandle node;
		

		ros::Subscriber sub_depthThruster;
                ros::Subscriber sub_vectorThruster;
		
		void depthThrusterCb (const thrusters::DepthThrusterMsg::ConstPtr& msg);
		void vectorThrusterCb (const thrusters::VectorThrusterMsg::ConstPtr& msg);

};

#endif /* CONTROLLER_SUB_HPP_ */
