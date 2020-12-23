//This is an SIMULATION ONLY node to record a quadcopter's position in the simulation, to investigate the control dynamics.
//This node will only be launched if the "log_position" param is set to true
#include "ros/ros.h" //all headers necessary for ROS functions
#include <gazebo_msgs/GetModelState.h>//following header files define the ROS message objects in respective namespaces
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>

int main(int argc, char** argv) {
	ros::init(argc, argv, "position_logger");//initialise the node, name it
	ros::NodeHandle nh; //construct the first NodeHandle to fully initialise, handle contains communication fns
	bool log_position = false;
	nh.getParam("log_position", log_position);
	if (!log_position) {//if not logging, exit immediately
		return 0;
	}
	ros::Rate rate(10.0); //setpoint publishing rate MUST be faster than 2 Hz
	//POSITION LOGGING
	std::string log_position_path;
	std::ofstream output_file;
	double log_until_seconds;
	nh.getParam("log_position_path", log_position_path);//where to save the CSV file
	nh.getParam("log_until_seconds", log_until_seconds);//how long to log the position for in seconds in Gazebo
	ROS_INFO("Position logging enabled for %f seconds, output to %s", log_until_seconds, log_position_path.c_str());
	output_file.open(log_position_path.c_str());
	bool override_pid_constants = false;
	nh.getParam("follower1/override_PID_constants", override_pid_constants);
	if (override_pid_constants) {//if this is the follower quadcopter and we are passing in pid constants from yaml file
		double Kp = 0, Ki = 0, Kd = 0;//p, i and d gains (placeholder values of 0)
        nh.getParam("follower1/PID_Kp", Kp);
        nh.getParam("follower1/PID_Ki", Ki);
        nh.getParam("follower1/PID_Kd", Kd);
		output_file << Kp << "," << Ki << "," << Kd << std::endl;//record the Kp, Ki, Kd values for analysis
	}
	ros::ServiceClient model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
	gazebo_msgs::GetModelState iris0_state, iris1_state;
	iris0_state.request.model_name = "iris0";
	iris1_state.request.model_name = "iris1";
	while (ros::ok() && ros::Time::now().toSec() <= log_until_seconds) {//run this loop until log_until_seconds
		if (model_state_client.call(iris0_state)) {//Note: ros::Time::now output here is the same as Gazebo time shown in the GUI
			//ROS_INFO("Time: %f, Position: %f, %f", ros::Time::now().toSec(), model_state.response.pose.position.x*100, model_state.response.pose.position.y*100);
			output_file << ros::Time::now() << ","  << iris0_state.response.pose.position.x*100 << "," << iris0_state.response.pose.position.y*100;//record absolute position (in cm)
		}
		if (model_state_client.call(iris1_state)) {//Note: ros::Time::now output here is the same as Gazebo time shown in the GUI
			//ROS_INFO("Time: %f, Position: %f, %f", ros::Time::now().toSec(), model_state.response.pose.position.x*100, model_state.response.pose.position.y*100);
			output_file << ","  << iris1_state.response.pose.position.x*100 << "," << iris1_state.response.pose.position.y*100 << std::endl;//record absolute position (in cm)
		}
		ros::spinOnce();
		rate.sleep();
	}
	output_file.close();//close the output file stream gracefully
	ROS_INFO("Finished position logging after %f seconds", log_until_seconds);
	return 0;
}
