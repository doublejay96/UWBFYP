//This is an ACTUAL FLIGHT ONLY node to demonstrate sending offboard commands to the PX4 in Gazebo to control the leader's flight.
#include "ros/ros.h" //all headers necessary for ROS functions
#include <mavros_msgs/CommandBool.h>//following header files define the ROS message objects in respective namespaces
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include "follower/flight_status.h"//defines the flight_status object, in 'follower' namespace
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <cstdlib>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}
uint8_t flightStage = 0;//the current flight status, to activate the sending of offboard commands
void flightStatusReceivedCallback(const follower::flight_status message) {
	flightStage = message.stage;//update the known stage we are at
}
struct waypoint {
	double x, y, z, duration;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "mavros_direct_offboard");//initialise the node, name it
	ros::NodeHandle nh; //construct the first NodeHandle to fully initialise, handle contains communication fns
	bool followingWaypoints = true;
	nh.param("waypoints", followingWaypoints, true);
	if (nh.getParam("waypoints", followingWaypoints)) ROS_INFO("Reading from waypoints file");
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber flight_status_sub = nh.subscribe<follower::flight_status>("flight_status", 10, flightStatusReceivedCallback);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	ros::Rate rate(10.0); //setpoint publishing rate MUST be faster than 2 Hz
	std::vector<waypoint> waypoints;
	std::ifstream waypoints_file;
	if (followingWaypoints) {
		waypoints_file.open("../UWBFYP/data/waypoints.txt");
		char line[256];
		while (waypoints_file.getline(line, 256)) {
			char* p;
			waypoint Waypoint;
			Waypoint.x = atof(line);
			p = strstr(line, ",") + 1;//next char after delimiter
			Waypoint.y = atof(p);
			p = strstr(p, ",") + 1;
			Waypoint.z = atof(p);
			p = strstr(p, ",") + 1;
			Waypoint.duration = atof(p);
			waypoints.push_back(Waypoint);
			ROS_INFO("Read waypoint: (%.3f, %.3f, %.3f), duration %.3f seconds", Waypoint.x, Waypoint.y, Waypoint.z, Waypoint.duration);
		}
		waypoints_file.close();
	}
	while (ros::ok() && !current_state.connected) {
		ros::spinOnce(); //call any callbacks waiting
		rate.sleep();
	}
	//OFFBOARD BY SETPOINT_RAW/LOCAL
	ros::Publisher target_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
	mavros_msgs::PositionTarget positionTarget;
	positionTarget.coordinate_frame = 8; //FRAME_BODY_NED
	positionTarget.type_mask = 0b111111111000;//set positions only, 3576
	positionTarget.position.z = 1;
	bool override_home_pos = false;//default is to assume home position is 0,0
    nh.param("override_home_pos", override_home_pos, false);//if needed, override to new home position from VICON
    if (override_home_pos) {
        float home_x = 0, home_y = 0;//the new home position
        nh.getParam("home_x", home_x);
        nh.getParam("home_y", home_y);
        positionTarget.position.x = home_x;
        positionTarget.position.y = home_y;
    }
	while (ros::ok() && flightStage != 2) {//while the quadcopter hasn't reached flight stage 2 (still connecting or taking off)
		target_pos_pub.publish(positionTarget);
		ros::spinOnce(); //call any callbacks waiting
		rate.sleep();
	}
	ros::Time last_waypoint_time = ros::Time::now();//time at which last waypoint was 'reached'
	std::vector<waypoint>::iterator currentWaypoint = waypoints.begin();//iterator, first waypoint
	ros::Duration currentWaypointDuration = ros::Duration(0.0);//ros::Duration variable saves on recalculation
	if (followingWaypoints) {
		positionTarget.position.x = currentWaypoint->x;
		positionTarget.position.y = currentWaypoint->y;
		positionTarget.position.z = currentWaypoint->z;
		currentWaypointDuration = ros::Duration(currentWaypoint->duration);
		ROS_INFO("Moving to first waypoint: (%.3f, %.3f, %.3f), duration %.3f seconds", currentWaypoint->x, currentWaypoint->y, currentWaypoint->z, currentWaypoint->duration);
		last_waypoint_time = ros::Time::now();//start time of first waypoint at time of arming
	}
	while (ros::ok() && current_state.connected && flightStage == 2) {//once at flight stage 2, begin offboard control
		if (ros::Time::now() - last_waypoint_time > currentWaypointDuration && currentWaypoint < --waypoints.end()) {
			currentWaypoint++;//if not yet at the end, increment the iterator to point to next waypoint
			positionTarget.position.x = currentWaypoint->x;//update setpoint coordinates to publish
			positionTarget.position.y = currentWaypoint->y;
			positionTarget.position.z = currentWaypoint->z;
			currentWaypointDuration = ros::Duration(currentWaypoint->duration);//update the duration
			last_waypoint_time = ros::Time::now();//reset the time to now
			ROS_INFO("Moving to next waypoint: (%.3f, %.3f, %.3f), duration %.3f seconds", currentWaypoint->x, currentWaypoint->y, currentWaypoint->z, currentWaypoint->duration);
		}
		target_pos_pub.publish(positionTarget);//publish the latest position_Target
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("Exiting mavros_direct_offboard, should go to automatic landing");
	return 0;
}
