//This ROS node reads the filtered UWB readings, converts that into appropriate offboard commands in velocity using the PID controller, and sends it to the mavros_node node by publishing to 'mavros/setpoint_raw/local' topic. It is always calculating, but only starts sending offboard commands when the 'flight_status' topic lets it know that the previous mavros_takeoff node is done.
#include "ros/ros.h" //all headers necessary for ROS functions
#include <mavros_msgs/CommandBool.h>//following header files define the ROS message objects in respective namespaces
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include "follower/flight_status.h"//defines the flight_status object, in 'follower' namespace
#include "follower/filtered_reading.h"//defines the filtered_reading object
#include <fstream>
#include <iostream>

mavros_msgs::State current_state;//global var to monitor the current state (of the connection)
uint8_t flightStage = 0;//the current flight status, to activate the sending of offboard commands
float Xcm, Ycm;//LAST KNOWN position of UWB tag relative to node (in cm, F-L reference axes)(message def is float32s)
float X_offset = 0, Y_offset = 0;//distance to keep the leader at relative to folllower (in cm, F-L reference axes)
double Kp = 0.5, Ki = 0.125, Kd = 0.75;//P, I and D gains (placeholder values)
const float dt = 0.1;//dt used to differentiate/integrate, reciprocal of 10 Hz update rate from UWB node

//callbacks update the relevant global var when receiving on topics "mavros/state", "filtered_reading", "flight_status"
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}
void filteredReadingReceivedCallback(const follower::filtered_reading message) {
    Xcm = message.Xcm_fil;//update our last known relative position of leader
    Ycm = message.Ycm_fil;
}
void flightStatusReceivedCallback(const follower::flight_status message) {
	flightStage = message.stage;//update the known stage we are at
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mavros_offboard");//initialise the node, name it "mavros_offboard"  
    ros::NodeHandle nh; //construct the first NodeHandle to fully initialise, handle contains communication fns
    ros::Subscriber filtered_sub = nh.subscribe<follower::filtered_reading>("filtered_reading", 1000, filteredReadingReceivedCallback);//subscribe to the filtered readings published by the ma_filter node
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher target_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 100);
	ros::Subscriber flight_status_sub = nh.subscribe<follower::flight_status>("flight_status", 10, flightStatusReceivedCallback);
	ros::Rate rate(10.0); //setpoint publishing rate MUST be faster than 2 Hz
    while (flightStage != 2) {//while the quadcopter hasn't reached flight stage 2 (still connecting or taking off)
        ros::spinOnce(); //call any callbacks waiting
        rate.sleep();
    }
    mavros_msgs::PositionTarget positionTarget;//This is the setpoint we want to move to
    positionTarget.coordinate_frame = 8;//Set the reference frame of the command to body (FLU) frame
	//positionTarget.type_mask = 0b110111111000;//set positions only, 3576
    positionTarget.type_mask = 0b110111000111;//set velocities only, 3527
	//PID Controller. Consider x and y separately (assume no yaw)
	float x_error = 0;//the error value for that instant 
	float y_error = 0;
	long double integral_x = 0;//integral over time, watch out for overflow
	long double integral_y = 0;
	float prev_x_error = 0;//store the immediately preceding value to calculate derivative
	float prev_y_error = 0;
	double derivative_x = 0;//the calculated derivative (not stored across time),
	double derivative_y = 0;//but declare here to avoid re-declaring inside loop
	bool override_PID_constants = false;//whether to use the Kp, Ki, Kd values in the param file or not
	nh.param("override_PID_constants", override_PID_constants, false);//if parameter not set, default to FALSE, use the original values specified above
	if (override_PID_constants) {//if enabled, read the Kp, Ki, Kd values loaded from the parameter file
		nh.getParam("PID_Kp", Kp);
		nh.getParam("PID_Ki", Ki);
		nh.getParam("PID_Kd", Kd);
		ROS_INFO("Using PID constants from param file, Kp: %f, Ki: %f, Kd: %f", Kp, Ki, Kd);
	}
	bool log_errors = false;//whether to log the errors in the PID controller or not
	std::string log_errors_path;
	std::ofstream output_file;
	nh.param("log_errors", log_errors, false);//if parameter not set, default to FALSE, dont log errors
	if (log_errors) {
		nh.getParam("log_errors_path", log_errors_path);
		ROS_INFO("PID position error logging enabled for follower, output to %s", log_errors_path.c_str());
		output_file.open(log_errors_path.c_str());
		output_file << Kp << "," << Ki << "," << Kd << std::endl;
	}
    while (ros::ok() && current_state.connected && flightStage == 2) {//once at flight stage 2, begin PID controller
		x_error = (Xcm - X_offset)/100;//calculate error in X, convert to m
		y_error = (Ycm - Y_offset)/100;
		integral_x += x_error * dt;//add the most recent error to the integral
		integral_y += y_error * dt;
		//integral_x += (prev_x_error + x_error) / 2 * dt;//add the most recent error to the integral
		//integral_y += (prev_y_error + y_error) / 2 * dt;
		derivative_x = (x_error - prev_x_error) / dt;//calculate the derivative from the previous error value
		derivative_y = (y_error - prev_y_error) / dt;
		positionTarget.velocity.x = (Kp * x_error) + (Ki * integral_x) + (Kd * derivative_x);//combine P,I,D terms to get the output control var
    	positionTarget.velocity.y = (Kp * y_error) + (Ki * integral_y) + (Kd * derivative_y);
		target_pos_pub.publish(positionTarget);
		if (log_errors) output_file << x_error << "," << y_error << std::endl;
		//ROS_INFO("For x, P: %f, I: %Lf, D:%f, output velocity: %f", x_error, integral_x, derivative_x, positionTarget.velocity.x);
		//ROS_INFO("For y, P: %f, I: %Lf, D:%f, output velocity: %f", y_error, integral_y, derivative_y, positionTarget.velocity.y);
		prev_x_error = x_error;//store the error value for the next loop
		prev_y_error = y_error;
		ros::spinOnce(); //call any callbacks waiting
		rate.sleep();
    }
	ROS_INFO("Flight stage is now %d, the mavros_offboard node is exiting now", flightStage);
	output_file.close();
    return 0;
}
