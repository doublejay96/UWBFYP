//This is an SIMULATION ONLY node to demonstrate sending offboard commands to the PX4 in Gazebo to control the leader's flight.
#include "ros/ros.h" //all headers necessary for ROS functions
#include <mavros_msgs/CommandBool.h>//following header files define the ROS message objects in respective namespaces
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
	current_state = *msg;
}

struct waypoint {
	double x, y, z, duration;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "mavros_direct_offboard");//initialise the node, name it
	ros::NodeHandle nh; //construct the first NodeHandle to fully initialise, handle contains communication fns
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
	ros::Rate rate(10.0); //setpoint publishing rate MUST be faster than 2 Hz
	std::vector<waypoint> waypoints;//x, y, z, time for each waypoint
	std::ifstream waypoints_file;
	waypoints_file.open("../UWBFYP/data/waypoints.txt");
	char line[256];
	while (waypoints_file.getline(line, 256)) {
		char* p;
		waypoint Waypoint;
		Waypoint.x = atof(line);
		p = strstr(line, ",") + 1;//delimiter
		Waypoint.y = atof(p);
		p = strstr(p, ",") + 1;
		Waypoint.z = atof(p);
		p = strstr(p, ",") + 1;
		Waypoint.duration = atof(p);
		waypoints.push_back(Waypoint);
		ROS_INFO("Read waypoint: (%.3f, %.3f, %.3f), duration %.3f seconds", Waypoint.x, Waypoint.y, Waypoint.z, Waypoint.duration);
	}
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
	//OFFBOARD BY SETPOINT_RAW/ATTITUDE
	//ros::Publisher target_att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
	//mavros_msgs::AttitudeTarget attitudeTarget;
	//attitudeTarget.type_mask = 0b00111;//set attitude and orientation only
	//attitudeTarget.orientation.x = 0;
	//attitudeTarget.orientation.y = 0;
	//attitudeTarget.orientation.z = 0;
	//attitudeTarget.orientation.w = 0;
	//attitudeTarget.thrust = 0.5;//set thrust to 0.5 of maximum
	for (int i = 10; ros::ok() && i > 0; --i) { //send a few setpoints before starting
		//local_pos_pub.publish(pose);
		target_pos_pub.publish(positionTarget);
		//target_att_pub.publish(attitudeTarget);
		ros::spinOnce();
		rate.sleep();
	}
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = true;
	ros::Time last_request = ros::Time::now();
	ros::Time last_waypoint_time = ros::Time::now();//time at which last waypoint was 'reached'
	std::vector<waypoint>::iterator currentWaypoint = waypoints.begin();//iterator, first waypoint
	positionTarget.position.x = currentWaypoint->x;
	positionTarget.position.y = currentWaypoint->y;
	positionTarget.position.z = currentWaypoint->z;
	ros::Duration currentWaypointDuration = ros::Duration(currentWaypoint->duration);//ros::Duration variable saves on recalculation
	while (ros::ok()) {//nested if statements :((
		if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) { //if not in OFFBOARD mode already, 5s since last request
			if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {//if setting to offboard mode is successful
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		} else if (current_state.mode == "OFFBOARD") {//if in OFFBOARD mode already
			if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {//if not armed, 5s since last request
				if (arming_client.call(arm_cmd) && arm_cmd.response.success) {//if arming call succeeded
					ROS_INFO("Vehicle armed");
					ROS_INFO("Moving to first waypoint: (%.3f, %.3f, %.3f), duration %.3f seconds", currentWaypoint->x, currentWaypoint->y, currentWaypoint->z, currentWaypoint->duration);
					last_waypoint_time = ros::Time::now();//start time of first waypoint at time of arming
				}
				last_request = ros::Time::now();
			} else if (current_state.armed && ros::Time::now() - last_waypoint_time > currentWaypointDuration) {//if armed, and time to go to next waypoint
				if (currentWaypoint < --waypoints.end()) {
					currentWaypoint++;//if not yet at the end, increment the iterator to point to next waypoint
					positionTarget.position.x = currentWaypoint->x;//update setpoint coordinates to publish
					positionTarget.position.y = currentWaypoint->y;
					positionTarget.position.z = currentWaypoint->z;
					currentWaypointDuration = ros::Duration(currentWaypoint->duration);//update the duration
					last_waypoint_time = ros::Time::now();//reset the time to now
					ROS_INFO("Moving to next waypoint: (%.3f, %.3f, %.3f), duration %.3f seconds", currentWaypoint->x, currentWaypoint->y, currentWaypoint->z, currentWaypoint->duration);
				}
			}
		}
		//STOP publishing, we will do it from command line
		//local_pos_pub.publish(pose);
		target_pos_pub.publish(positionTarget);//publish the latest positionTarget
		//target_att_pub.publish(attitudeTarget);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
