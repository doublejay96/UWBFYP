//This ROS node handles arming and taking off the leader initially to a starting hover, before it stops publishing setpoints and allows mavros_offboard node to do that instead for the control logic. Publishes to the 'flight_status' topic, to let mavros_offboard know when to take over.
#include "ros/ros.h" //all headers necessary for ROS functions
#include <mavros_msgs/CommandBool.h>//following header files define the ROS message objects in respective namespaces
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
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
	ros::init(argc, argv, "mavros_takeoff_land");//initialise the node, name it
	ros::NodeHandle nh; //construct the first NodeHandle to fully initialise, handle contains communication fns
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Publisher target_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	ros::Rate rate(10.0); //setpoint publishing rate MUST be faster than 2 Hz
	ros::Publisher flight_status_pub = nh.advertise<follower::flight_status>("flight_status", 10);
	ros::Subscriber flight_status_sub = nh.subscribe<follower::flight_status>("flight_status", 10, flightStatusReceivedCallback);
	follower::flight_status flightStatus;//the flight_status object to be published;
	flightStatus.stage = 0;//not yet connected to PX4
	flight_status_pub.publish(flightStatus);//update the flightStatus
	while (ros::ok() && !current_state.connected) {
		ros::spinOnce(); //call any callbacks waiting
		rate.sleep();
	}
	flightStatus.stage = 1;//connected, going to arm and take off
	flight_status_pub.publish(flightStatus);//update the flightStatus
	//mavros_msgs::PositionTarget startingPosition;
	//startingPosition.coordinate_frame = 8; //FRAME_BODY_NED
	//startingPosition.type_mask = 0b110111111000;//set positions only, 3576
	//startingPosition.type_mask = 0b110111000111;//set velocities only, 3527
	//startingPosition.position.z = 2;
	//for (int i = 10; ros::ok() && i > 0; --i) { //send a few setpoints before starting
	//	target_pos_pub.publish(startingPosition);
	//	ros::spinOnce();
	//	rate.sleep();
	//}
	mavros_msgs::SetMode autotakeoff_set_mode;
	autotakeoff_set_mode.request.custom_mode = "AUTO.TAKEOFF";
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	ros::Time last_request = ros::Time::now();
	ros::Time time_armed;
	while (ros::ok()) {
		if (current_state.mode != "AUTO.TAKEOFF" && (ros::Time::now() - last_request > ros::Duration(5.0))) { //if not in TAKEOFF mode already, 5s since last request
			if (set_mode_client.call(autotakeoff_set_mode) && autotakeoff_set_mode.response.mode_sent) {//if setting to offboard mode is successful
				ROS_INFO("Auto Takeoff mode enabled");
			}
			last_request = ros::Time::now();
		} else {//in AUTO.TAKEOFF mode
			if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {//if not armed, 5s since last request
				if (arming_client.call(arm_cmd) && arm_cmd.response.success) {//if arming call succeeded
					ROS_INFO("Vehicle armed, changing flight status to 2 in 10 seconds");
					time_armed = ros::Time::now();//record this time since arm, pass over control
				}
				last_request = ros::Time::now();
			} else if (current_state.armed && (ros::Time::now() - time_armed > ros::Duration(10.0))) {//if armed AND 10s have passed
				ROS_INFO("Assumed reached starting position, going to control logic now");
				break;
			}
		}
		//target_pos_pub.publish(startingPosition);//publish the initial position again
		ros::spinOnce();
		rate.sleep();
	}
	flightStatus.stage = 2;//at starting position, go to control logic by mavros_offboard
	flight_status_pub.publish(flightStatus);//update the flightStatus
	ROS_INFO("Flight stage is now %d, mavros_takeoff_land node handing control to mavros_offboard node", flightStatus.stage);
	while (ros::ok()) {//flight stage is 2, mavros_offboard node controlling, wait
		if (flightStage == 3) break;
		ros::spinOnce();
		rate.sleep();
	}
	mavros_msgs::SetMode autoland_set_mode;
	autoland_set_mode.request.custom_mode = "AUTO.LAND";
	while (ros::ok()) {
		if (current_state.mode != "AUTO.LAND") {
			if (set_mode_client.call(autoland_set_mode) && autoland_set_mode.response.mode_sent) {//if setting to auto land mode successfull
				ROS_INFO("Set to automatic landing mode enabled");
				break;
			}
		}
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("mavros_takeoff_land node is exiting");
	return 0;
}
