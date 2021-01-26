//This ROS node logs several things IN THE SIMULATION: the Gazebo model position of leader and follower, the UWB node readings, the filtered UWB readings, the x,y,z error calculated by the PID controller, and the corresponding x,y,z velocity setpoints given. It is the equivalent of super_logger for simulation purposes and should not be called together with it.
//NOTE: node should be called outside of follower1 namespace to correctly access Gazebo services.
#include "ros/ros.h" //all headers necessary for ROS functions
#include "ros/package.h"
#include "follower/uwb_node_reading.h"//defines the uwb_node_reading object, in 'follower' namespace
#include "follower/filtered_reading.h"
#include "follower/flight_status.h"
#include "follower/state_error.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <gazebo_msgs/GetModelState.h>
#include <fstream>
#include <iostream>

//GLOBAL VARIABLES ARE THOSE BEING LOGGED
//set initialised values to -1 to tell if they are not being updated
double gazebo_F_x_pos = -1.0, gazebo_F_y_pos = -1.0, gazebo_F_z_pos = -1.0;
double gazebo_L_x_pos = -1.0, gazebo_L_y_pos = -1.0, gazebo_L_z_pos = -1.0;
int UWB_Xcm = -1, UWB_Ycm = -1;
float avg_Xcm = -1.0, avg_Ycm = -1.0;
int flightStage = -1;
double state_x_error = -1.0, state_y_error = -1.0, state_z_error = -1.0;
double offb_x_vel = -1.0, offb_y_vel = -1.0, offb_z_vel = -1.0;
//Callbacks for the global variables
void logUWBNodeReading (const follower::uwb_node_reading message) {
	UWB_Xcm = message.Xcm;
	UWB_Ycm = message.Ycm;
}
void logFilteredReading (const follower::filtered_reading message) {
	avg_Xcm = message.Xcm_fil;
	avg_Ycm = message.Ycm_fil;
}
void logFlightStatus (const follower::flight_status message) {
	flightStage = message.stage;
}
void logStateError (const follower::state_error message) {
	state_x_error = message.x_error;
	state_y_error = message.y_error;
	state_z_error = message.z_error;
}
void logOffbVel (const mavros_msgs::PositionTarget message) {
	offb_x_vel = message.velocity.x;
	offb_y_vel = message.velocity.y;
	offb_z_vel = message.velocity.z;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "sim_logger");//initialise the node, name it "sim_logger"
	ros::NodeHandle nh; //construct the first NodeHandle to fully initialise, handle contains communication fns
	ros::Rate rate(10);//the rate this thing runs (10 Hz, same as UWB serial port input)
	bool super_logger = false;
	nh.param("follower1/super_logger", super_logger, false);//if not set, default to FALSE
	if (super_logger == false) {
		ROS_INFO("Super logger disabled");
		return 0;//exit the node immediately
	} else {
		ROS_INFO("Super logger enabled");
	}
	bool override_PID_constants = false;
	nh.param("follower1/override_PID_constants", override_PID_constants, false);//if not set, default to FALSE
	if (override_PID_constants) {
		double Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y, Kp_z, Ki_z, Kd_z;
		nh.getParam("follower1/PID_Kp_x", Kp_x);
		nh.getParam("follower1/PID_Ki_x", Ki_x);
		nh.getParam("follower1/PID_Kd_x", Kd_x);
		nh.getParam("follower1/PID_Kp_y", Kp_y);
		nh.getParam("follower1/PID_Ki_y", Ki_y);
		nh.getParam("follower1/PID_Kd_y", Kd_y);
		nh.getParam("follower1/PID_Kp_z", Kp_z);
		nh.getParam("follower1/PID_Ki_z", Ki_z);
		nh.getParam("follower1/PID_Kd_z", Kd_z);
		std::string super_logger_constants_path;
		nh.getParam("follower1/super_logger_constants_path", super_logger_constants_path);
		super_logger_constants_path = ros::package::getPath("follower") + "/../.." + super_logger_constants_path;
		std::ofstream constants_file;
		constants_file.open(super_logger_constants_path.c_str());
		constants_file << "Kp_x: " << Kp_x << ", Ki_x: " << Ki_x << ", Kd_x: " << Kd_x << std::endl;
		constants_file << "Kp_y: " << Kp_y << ", Ki_y: " << Ki_y << ", Kd_y: " << Kd_y << std::endl;
		constants_file << "Kp_z: " << Kp_z << ", Ki_z: " << Ki_z << ", Kd_z: " << Kd_z << std::endl;
		constants_file.close();
	}
	std::string super_logger_path;
	std::ofstream super_logger_file;
	nh.getParam("follower1/super_logger_path", super_logger_path);
	super_logger_path = ros::package::getPath("follower") + "/../.." + super_logger_path;
	super_logger_file.open(super_logger_path.c_str());
	super_logger_file << "Time (s), Gazebo_follower_X_pos (m), Gazebo_follower_Y_pos (m), Gazebo_follower_Z_pos (m), Gazebo_leader_X_pos (m), Gazebo_leader_Y_pos (m), Gazebo_leader_Z_pos (m), UWB_Xcm, UWB_Ycm, avg_Xcm, avg_Ycm, flightStage, state_x_error (m), state_y_error (m), state_z_error (m), Offb_x_vel (m/s), Offb_y_vel (m/s), Offb_z_vel (m/s)" << std::endl;
	//Create the Gazebo services
	ros::ServiceClient model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
	gazebo_msgs::GetModelState leader_state;
    leader_state.request.model_name = "iris0";//get the state of iris0, the leader
	gazebo_msgs::GetModelState follower_state;
	follower_state.request.model_name = "iris1";//get the state of iris1, the follower	
	//Create the many subscribers
	ros::Subscriber reading_sub = nh.subscribe<follower::uwb_node_reading>("follower1/uwb_node_reading", 100, logUWBNodeReading);
	ros::Subscriber filtered_sub = nh.subscribe<follower::filtered_reading>("follower1/filtered_reading", 100, logFilteredReading);
	ros::Subscriber flight_status_sub = nh.subscribe<follower::flight_status>("follower1/flight_status", 10, logFlightStatus);
	ros::Subscriber state_error_sub = nh.subscribe<follower::state_error>("follower1/state_error", 100, logStateError);
	ros::Subscriber offb_vel_sub = nh.subscribe<mavros_msgs::PositionTarget>("follower1/mavros/setpoint_raw/local", 100, logOffbVel);
	ros::Time start = ros::Time::now();//define this as a starting point in time
	while (ros::ok()) {
		if (flightStage == 2) {
			if (model_state_client.call(leader_state)) {
				gazebo_L_x_pos = leader_state.response.pose.position.x;
				gazebo_L_y_pos = leader_state.response.pose.position.y;
				gazebo_L_z_pos = leader_state.response.pose.position.z;
			}
			if (model_state_client.call(follower_state)) {
				gazebo_F_x_pos = follower_state.response.pose.position.x;
				gazebo_F_y_pos = follower_state.response.pose.position.y;
				gazebo_F_z_pos = follower_state.response.pose.position.z;
			}
			super_logger_file << ros::Time::now() - start << ",";
			super_logger_file << gazebo_F_x_pos << "," << gazebo_F_y_pos << "," << gazebo_F_z_pos << ",";
			super_logger_file << gazebo_L_x_pos << "," << gazebo_L_y_pos << "," << gazebo_L_z_pos << ",";
			super_logger_file << UWB_Xcm << "," << UWB_Ycm << ",";
			super_logger_file << avg_Xcm << "," << avg_Ycm << ",";
			super_logger_file << flightStage << ",";
			super_logger_file << state_x_error << "," << state_y_error << "," << state_z_error << ",";
			super_logger_file << offb_x_vel << "," << offb_y_vel << "," << offb_z_vel;
			super_logger_file << std::endl;//all of the above is one line
			super_logger_file.flush();//ensure it is written not buffered
		}
		ros::spinOnce(); //call any callbacks waiting
		rate.sleep(); //sleep for appropriate amt of time to maintain rate
	}
	return 0;
}
