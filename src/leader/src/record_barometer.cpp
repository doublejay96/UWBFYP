//This is an EXPERIMENT ONLY node to record how much the barometer readings fluctuate in place.
#include "ros/ros.h" //all headers necessary for ROS functions
#include <mavros_msgs/CommandBool.h>//following header files define the ROS message objects in respective namespaces
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <iostream>

std::ofstream output_file;
void position_cb(const geometry_msgs::PoseStamped msg) {
	output_file << msg.header.seq << " " << msg.pose.position.z << std::endl;
	ROS_INFO("sequence number is %d", msg.header.seq);
	if (msg.header.seq == 100) output_file.close();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "record_barometer");//initialise the node, name it
	ros::NodeHandle nh; //construct the first NodeHandle to fully initialise, handle contains communication fns
	//LISTEN TO THE CURRENT PX POSITION ESTIMATE
	output_file.open("barometer_readings.txt", std::ios::out | std::ios::trunc);//will be saved to ~/.ros
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, position_cb);
	ros::Rate rate(10.0);
	while (ros::ok()) {
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
