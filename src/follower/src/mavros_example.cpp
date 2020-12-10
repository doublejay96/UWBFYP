//This is an EXAMPLE ONLY node to demonstrate sending offboard commands to the PX4 in Gazebo.
#include "ros/ros.h" //all headers necessary for ROS functions
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
//#include "follower/uwb_node_reading.h"//defines the uwb_node_reading object, in 'follower' namespace 
//#include "follower/filtered_reading.h"//defines the filtered_reading object

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mavros_offboard");//initialise the node, name it "mavros_offboard"
    ros::NodeHandle nh; //construct the first NodeHandle to fully initialise, handle contains communication fns
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::Publisher target_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Rate rate(10.0); //setpoint publishing rate MUST be faster than 2 Hz
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce(); //call any callbacks waiting
        rate.sleep();
    }
    mavros_msgs::PositionTarget positionTarget;
    positionTarget.coordinate_frame = 8; //FRAME_BODY_NED
    positionTarget.type_mask = 8+16+32+64+128+256+1024+2048;//set positions only
    positionTarget.position.z = 2;
    for (int i = 100; ros::ok() && i > 0; --i) { //send a few setpoints before starting
        //local_pos_pub.publish(pose);
	target_pos_pub.publish(positionTarget);
        ros::spinOnce();
        rate.sleep();
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    ros::Time arm_time;
    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
		    arm_time = ros::Time::now();
                }
                last_request = ros::Time::now();
            } else if (current_state.armed && (ros::Time::now() - arm_time > ros::Duration(20.0))) {
		positionTarget.type_mask = 1+2+4+64+128+256+1024+2048;//set velocities only
    		positionTarget.velocity.x = 1;
    		positionTarget.velocity.y = 1;
	    }
        }
        //local_pos_pub.publish(pose);
	target_pos_pub.publish(positionTarget);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
