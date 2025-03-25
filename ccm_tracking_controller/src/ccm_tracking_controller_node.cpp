/*
	FILE: ccmtracking_controller_node.cpp
	---------------------------------
	tracking controller for px4-based quadcopter
*/
#include <ccm_tracking_controller/CCMtrackingController.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "ccm_tracking_controller_node");
	ros::NodeHandle nh;
	controller::trackingController tc (nh);
	ros::spin();

	return 0;
}