//This node is meant to simulate the UWB node readings. It does this by getting the model state from Gazebo and calculating the position of the leader's tag relative to the follower node, and publishing to the '/uwb_node_reading' topic, hence replacing the 'uwb_node_reader' node entirely.
//TO DO: implement D, P readings, account for orientation of follower, add realistic noise based on experimental data to make it not perfectly accurate
#include "ros/ros.h" //all headers necessary for ROS functions
#include <gazebo_msgs/GetModelState.h>
#include <mavros_msgs/PositionTarget.h>
#include "follower/uwb_node_reading.h"//defines the uwb_node_reading object, in 'follower' namespace
#include <math.h>//math functions for calculating distance, adding noise

//Function to update known relative positions at rate of 10 Hz
//iris0 (leader) at index 2, followers at subsequent indices
//Account for conversion between absolute Gazebo model state reference frame and the body-fixed UWB reference frame (F-L)
//convert from given metres to publishing in cm

int main(int argc, char** argv) {
	double Xcm_sim, Ycm_sim;//The current known relative position of the leader to the follower (in cm)
	ros::init(argc, argv, "uwb_simulator");//initialise the node and name it
	ros::NodeHandle nh; //construct the first NodeHandle to fully initialise, handle contains communication fns
	ros::ServiceClient model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
	ros::Publisher reading_pub = nh.advertise<follower::uwb_node_reading>("follower1/uwb_node_reading", 1000);
	ros::Rate rate(10);//same as the actual uwb_node_reader, 10 Hz
	gazebo_msgs::GetModelState leader_relative_state;
	leader_relative_state.request.model_name = "iris0";//get the state of iris0, the leader
	leader_relative_state.request.relative_entity_name = "iris1";//relative to iris1, the follower
	follower::uwb_node_reading message;//create the message to publish
	message.D = 0;//dummy info
	message.P = 0;
	while (ros::ok()) {
		if (model_state_client.call(leader_relative_state)) {
			Xcm_sim = leader_relative_state.response.pose.position.x * 100;//get relative position (convert to cm)
			Ycm_sim = leader_relative_state.response.pose.position.y * 100;
			message.Xcm = (int) Xcm_sim;//cast to int, put in message
			message.Ycm = (int) Ycm_sim;
			message.D = (int) sqrt(pow(Xcm_sim, 2) + pow(Ycm_sim,2));//calculate distance from doubles
			reading_pub.publish(message);
			//ROS_INFO("UWB readings are Xcm: %d, Ycm: %d", Xcm_sim, Ycm_sim);
		} else {
			ROS_ERROR("uwb_simulator: could not get relative leader position");
		}
		ros::spinOnce();//call all the model_states callbacks
		rate.sleep();
	}
	return 0;
}
