//This node is meant to simulate the UWB node readings. It does this by getting the model state from Gazebo and calculating the position of the leader's tag relative to the follower node, and publishing to the '/uwb_node_reading' topic, hence replacing the 'uwb_node_reader' node entirely.
//TO DO: implement D, P readings, add realistic noise based on experimental data to make it not perfectly accurate
#include "ros/ros.h" //all headers necessary for ROS functions
#include <gazebo_msgs/ModelStates.h>
#include <mavros_msgs/PositionTarget.h>
#include "follower/uwb_node_reading.h"//defines the uwb_node_reading object, in 'follower' namespace 

int Xcm_sim, Ycm_sim;
void modelStateCallback(const gazebo_msgs::ModelStates message) {//iris1 is at index 2, iris0 at index 3
    Xcm_sim = (message.pose[2].position.x - message.pose[3].position.x) * 100;//calculate the relative position (cast to int)
    Ycm_sim = (message.pose[2].position.y - message.pose[3].position.y) * 100;
    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "uwb_simulator");//initialise the node and name it
    ros::NodeHandle nh; //construct the first NodeHandle to fully initialise, handle contains communication fns
    ros::Subscriber model_states_sub = nh.subscribe<gazebo_msgs::ModelStates>("gazebo/model_states", 50, modelStateCallback);
    ros::Publisher reading_pub = nh.advertise<follower::uwb_node_reading>("uwb_node_reading", 1000);
    ros::Rate loop_rate(10);//same as the actual uwb_node_reader, 10 Hz
    follower::uwb_node_reading message;
    message.D = 0;//dummy info
    message.P = 0;
    while (ros::ok()) {
	ros::spinOnce();//call all the model_states callbacks
	message.Xcm = Xcm_sim;
	message.Ycm = Ycm_sim;
    	reading_pub.publish(message);
	ROS_INFO("UWB readings are Xcm: %d, Ycm: %d", Xcm_sim, Ycm_sim);
        loop_rate.sleep();
    }
    return 0;
}
