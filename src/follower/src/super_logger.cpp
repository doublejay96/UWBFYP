//This ROS node logs several things: the VICON position of leader and follower, the UWB node readings, the filtered UWB readings, the x,y,z error calculated by the PID controller, and the corresponding x,y,z velocity setpoints given.
#include "ros/ros.h" //all headers necessary for ROS functions
#include "follower/uwb_node_reading.h"//defines the uwb_node_reading object, in 'follower' namespace
#include "follower/filtered_reading.h"
#include "follower/flight_status.h"
#include "follower/PID_error.h"
#include <fstream>
#include <iostream>

//GLOBAL VARIABLES ARE THOSE BEING LOGGED
//set initialised values to -1 to tell if they are not being updated
int UWB_Xcm = -1, UWB_Ycm = -1;
float avg_Xcm = -1.0, avg_Ycm = -1.0;
int flightStage = -1;
float PID_x_error = -1.0, PID_y_error = -1.0, PID_z_error = -1.0;
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
void logPIDError (const follower::PID_error message) {
	PID_x_error = message.x_error;
	PID_y_error = message.y_error;
	PID_z_error = message.z_error;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "uwb_node_reader");//initialise the node, name it "uwb_node_reader"
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
	std::string super_logger_path;
	std::ofstream super_logger_file;
	nh.getParam("super_logger_path", super_logger_path);
	super_logger_file.open(super_logger_path.c_str());
	super_logger_file << "Time (s), UWB_Xcm, UWB_Ycm, avg_Xcm, avg_Ycm, flightStage, PID_x_error (m), PID_y_error (m), PID_z_error (m)";
	//Create the many subscribers
	ros::Subscriber reading_sub = nh.subscribe<follower::uwb_node_reading>("uwb_node_reading", 100, logUWBNodeReading);
	ros::Subscriber filtered_sub = nh.subscribe<follower::filtered_reading>("filtered_reading", 100, logFilteredReading);
	ros::Subscriber flight_status_sub = nh.subscribe<follower::flight_status>("flight_status", 10, logFlightStatus);
	ros::Subscriber PID_error_sub = nh.subscribe<follower::PID_error>("PID_error", 100, logPIDError);
	ros::Time start = ros::Time::now();//define this as a starting point in time
	while (ros::ok()) {
		super_logger_file << ros::Time::now() - start << "," << UWB_Xcm << "," << UWB_Ycm << ",";
		super_logger_file << avg_Xcm << "," << avg_Ycm << ",";
		super_logger_file << flightStage << ",";
		super_logger_file << PID_x_error << "," << PID_y_error << "," << PID_z_error;
		super_logger_file << std::endl;//all of the above is one line
		super_logger_file.flush();//ensure it is written not buffered
		ros::spinOnce(); //call any callbacks waiting
		rate.sleep(); //sleep for appropriate amt of time to maintain rate
	}
	return 0;
}
