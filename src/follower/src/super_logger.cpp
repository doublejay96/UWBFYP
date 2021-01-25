//This ROS node logs several things: the VICON position of leader and follower, the UWB node readings, the filtered UWB readings, the x,y,z error calculated by the PID controller, and the corresponding x,y,z velocity setpoints given.
#include "ros/ros.h" //all headers necessary for ROS functions
#include "ros/package.h"
#include "follower/uwb_node_reading.h"//defines the uwb_node_reading object, in 'follower' namespace
#include "follower/filtered_reading.h"
#include "follower/flight_status.h"
#include "follower/PID_error.h"
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <fstream>
#include <iostream>

//GLOBAL VARIABLES ARE THOSE BEING LOGGED
//set initialised values to -1 to tell if they are not being updated
float vicon_F_x_pos = -1.0, vicon_F_y_pos = -1.0, vicon_F_z_pos = -1.0;
float vicon_L_x_pos = -1.0, vicon_L_y_pos = -1.0, vicon_L_z_pos = -1.0;
int UWB_Xcm = -1, UWB_Ycm = -1;
float avg_Xcm = -1.0, avg_Ycm = -1.0;
int flightStage = -1;
float PID_x_error = -1.0, PID_y_error = -1.0, PID_z_error = -1.0;
float offb_x_vel = -1.0, offb_y_vel = -1.0, offb_z_vel = -1.0;
//Callbacks for the global variables
void logFollowerViconPos (const geometry_msgs::PoseStamped message) {
	vicon_F_x_pos = message.pose.position.x;
	vicon_F_y_pos = message.pose.position.y;
	vicon_F_z_pos = message.pose.position.z;
}
void logLeaderViconPos (const geometry_msgs::PoseStamped message) {
	vicon_L_x_pos = message.pose.position.x;
	vicon_L_y_pos = message.pose.position.y;
	vicon_L_z_pos = message.pose.position.z;
}
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
void logPIDError (const follower::PID_error message) {
	PID_x_error = message.x_error;
	PID_y_error = message.y_error;
	PID_z_error = message.z_error;
}
void logOffbVel (const mavros_msgs::PositionTarget message) {
	offb_x_vel = message.velocity.x;
	offb_y_vel = message.velocity.y;
	offb_z_vel = message.velocity.z;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "super_logger");//initialise the node, name it "super_logger"
	ros::NodeHandle nh; //construct the first NodeHandle to fully initialise, handle contains communication fns
	ros::Rate rate(10);//the rate this thing runs (10 Hz, same as UWB serial port input)
	bool super_logger = false;
	nh.param("super_logger", super_logger, false);//if not set, default to FALSE
	if (super_logger == false) {
		ROS_INFO("Super logger disabled");
		return 0;//exit the node immediately
	} else {
		ROS_INFO("Super logger enabled");
	}
	bool override_PID_constants = false;
	nh.param("override_PID_constants", override_PID_constants, false);//if not set, default to FALSE
	if (override_PID_constants) {
		double Kp_x, Ki_x, Kd_x, Kp_y, Ki_y, Kd_y, Kp_z, Ki_z, Kd_z;
		nh.getParam("PID_Kp_x", Kp_x);
		nh.getParam("PID_Ki_x", Ki_x);
		nh.getParam("PID_Kd_x", Kd_x);
		nh.getParam("PID_Kp_y", Kp_y);
		nh.getParam("PID_Ki_y", Ki_y);
		nh.getParam("PID_Kd_y", Kd_y);
		nh.getParam("PID_Kp_z", Kp_z);
		nh.getParam("PID_Ki_z", Ki_z);
		nh.getParam("PID_Kd_z", Kd_z);
		std::string super_logger_constants_path;
		nh.getParam("super_logger_constants_path", super_logger_constants_path);
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
	nh.getParam("super_logger_path", super_logger_path);
	super_logger_path = ros::package::getPath("follower") + "/../.." + super_logger_path;
	super_logger_file.open(super_logger_path.c_str());
	super_logger_file << "Time (s), VICON_follower_X_pos (m), VICON_follower_Y_pos (m), VICON_follower_Z_pos (m), VICON_leader_X_pos (m), VICON_leader_Y_pos (m), VICON_leader_Z_pos (m), UWB_Xcm, UWB_Ycm, avg_Xcm, avg_Ycm, flightStage, PID_x_error (m), PID_y_error (m), PID_z_error (m), Offb_x_vel (m/s), Offb_y_vel (m/s), Offb_z_vel (m/s)" << std::endl;
	//Create the many subscribers
	ros::Subscriber follower_vicon_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 100, logFollowerViconPos);//remapped
	ros::Subscriber leader_vicon_sub = nh.subscribe<geometry_msgs::PoseStamped>("vrpn_client_node/UWBFYP_leader/pose", 100, logLeaderViconPos);
	ros::Subscriber reading_sub = nh.subscribe<follower::uwb_node_reading>("uwb_node_reading", 100, logUWBNodeReading);
	ros::Subscriber filtered_sub = nh.subscribe<follower::filtered_reading>("filtered_reading", 100, logFilteredReading);
	ros::Subscriber flight_status_sub = nh.subscribe<follower::flight_status>("flight_status", 10, logFlightStatus);
	ros::Subscriber PID_error_sub = nh.subscribe<follower::PID_error>("PID_error", 100, logPIDError);
	ros::Subscriber offb_vel_sub = nh.subscribe<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 100, logOffbVel);
	ros::Time start = ros::Time::now();//define this as a starting point in time
	while (ros::ok()) {
		if (flightStage == 2) {
			super_logger_file << ros::Time::now() - start << ",";
			super_logger_file << vicon_F_x_pos << "," << vicon_F_y_pos << "," << vicon_F_z_pos << ",";
			super_logger_file << vicon_L_x_pos << "," << vicon_L_y_pos << "," << vicon_L_z_pos << ",";
			super_logger_file << UWB_Xcm << "," << UWB_Ycm << ",";
			super_logger_file << avg_Xcm << "," << avg_Ycm << ",";
			super_logger_file << flightStage << ",";
			super_logger_file << PID_x_error << "," << PID_y_error << "," << PID_z_error << ",";
			super_logger_file << offb_x_vel << "," << offb_y_vel << "," << offb_z_vel;
			super_logger_file << std::endl;//all of the above is one line
			super_logger_file.flush();//ensure it is written not buffered
		}
		ros::spinOnce(); //call any callbacks waiting
		rate.sleep(); //sleep for appropriate amt of time to maintain rate
	}
	return 0;
}
