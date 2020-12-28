//This is an ACTUAL FLIGHT ONLY node to demonstrate sending offboard commands to the PX4 in Gazebo to control the leader's flight.
#include "ros/ros.h" //all headers necessary for ROS functions
#include <mavros_msgs/CommandBool.h>//following header files define the ROS message objects in respective namespaces
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include "follower/flight_status.h"//defines the flight_status object, in 'follower' namespace

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}
uint8_t flightStage = 0;//the current flight status, to activate the sending of offboard commands
void flightStatusReceivedCallback(const follower::flight_status message) {
    flightStage = message.stage;//update the known stage we are at
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mavros_direct_offboard");//initialise the node, name it
	ros::NodeHandle nh; //construct the first NodeHandle to fully initialise, handle contains communication fns
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber flight_status_sub = nh.subscribe<follower::flight_status>("flight_status", 10, flightStatusReceivedCallback);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	ros::Rate rate(10.0); //setpoint publishing rate MUST be faster than 2 Hz
	while (ros::ok() && !current_state.connected) {
		ros::spinOnce(); //call any callbacks waiting
		rate.sleep();
	}
	//OFFBOARD BY SETPOINT_POSITION
	//ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
	//geometry_msgs::PoseStamped pose;
	//pose.pose.position.x = 0;
	//pose.pose.position.y = 0;
	//pose.pose.position.z = 1;
	//OFFBOARD BY SETPOINT_RAW/LOCAL
	ros::Publisher target_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
	mavros_msgs::PositionTarget positionTarget;
	positionTarget.coordinate_frame = 8; //FRAME_BODY_NED
	positionTarget.type_mask = 0b111111111000;//set positions only, 3576
	//positionTarget.type_mask = 0b110111000111;//set velocities only, 3527
	positionTarget.position.z = 1;
	//for (int i = 20; ros::ok() && i > 0; --i) { //send a few setpoints before starting
	//	//local_pos_pub.publish(pose);
	//	ros::spinOnce();
	//	rate.sleep();
	//}
	while (ros::ok() && flightStage != 2) {//while the quadcopter hasn't reached flight stage 2 (still connecting or taking off)
		target_pos_pub.publish(positionTarget);
		ros::spinOnce(); //call any callbacks waiting
		rate.sleep();
	}
	//mavros_msgs::SetMode offb_set_mode;
	//offb_set_mode.request.custom_mode = "OFFBOARD";
	//mavros_msgs::CommandBool arm_cmd;
	//arm_cmd.request.value = true;
	//ros::Time last_request = ros::Time::now();
	//while (ros::ok()) {
	while (ros::ok() && current_state.connected && flightStage == 2) {//once at flight stage 2, begin offboard control
		//if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) { //if not in OFFBOARD mode already, 5s since last request
		//	if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {//if setting to offboard mode is successful
		//		ROS_INFO("Offboard enabled");
		//	}
		//	last_request = ros::Time::now();
		//} else {
		//	if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {//if not armed, 5s since last request
		//		if (arming_client.call(arm_cmd) && arm_cmd.response.success) {//if arming call succeeded
		//			ROS_INFO("Vehicle armed");
		//		}
		//		last_request = ros::Time::now();
		//	}
		//}
		//STOP publishing, we will do it from command line
		//local_pos_pub.publish(pose);
		target_pos_pub.publish(positionTarget);//publish the latest position_Target
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
