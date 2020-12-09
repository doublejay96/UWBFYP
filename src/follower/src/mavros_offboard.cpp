#include "ros/ros.h" //all headers necessary for ROS functions
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
//#include "follower/uwb_node_reading.h"//defines the uwb_node_reading object, in 'follower' namespace 
#include "follower/filtered_reading.h"//defines the filtered_reading object

mavros_msgs::State current_state;//global var to monitor the current state (of the connection)
float Xcm, Ycm;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {//update the global var when receiving on topic "mavros/state"
    current_state = *msg;
}

void filteredReadingReceivedCallback(const follower::filtered_reading message) {//called on receiving a filtered reading from ma_filter
    Xcm = message.Xcm_fil;
    Ycm = message.Ycm_fil;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mavros_offboard");//initialise the node, name it "mavros_offboard"
    ros::NodeHandle nh; //construct the first NodeHandle to fully initialise, handle contains communication fns
    ros::Subscriber filtered_sub = nh.subscribe<follower::filtered_reading>("filtered_reading", 1000, filteredReadingReceivedCallback);//subscribe to the filtered readings published by the ma_filter node
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Rate rate(20.0); //setpoint publishing rate MUST be faster than 2 Hz
    while (ros::ok() && !current_state.connected) {//while ros running, but not yet connected to px4
        ros::spinOnce(); //call any callbacks waiting
        rate.sleep();
    }
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    for (int i = 100; ros::ok() && i > 0; --i) { //send a few setpoints before starting
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
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
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}