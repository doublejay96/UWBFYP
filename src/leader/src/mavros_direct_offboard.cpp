//This is an SIMULATION ONLY node to demonstrate sending offboard commands to the PX4 in Gazebo to control the leader's flight.
#include "ros/ros.h" //all headers necessary for ROS functions
#include <mavros_msgs/CommandBool.h>//following header files define the ROS message objects in respective namespaces
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mavros_direct_offboard");//initialise the node, name it
    ros::NodeHandle nh; //construct the first NodeHandle to fully initialise, handle contains communication fns
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
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
    positionTarget.type_mask = 0b110111111000;//set positions only, 3576
    //positionTarget.type_mask = 0b110111000111;//set velocities only, 3527
    positionTarget.position.z = 2;
    for (int i = 10; ros::ok() && i > 0; --i) { //send a few setpoints before starting
	target_pos_pub.publish(positionTarget);
        ros::spinOnce();
        rate.sleep();
    }
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) { //if not in OFFBOARD mode already, 5s since last request
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {//if setting to offboard mode is successful
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) {//if not armed, 5s since last request
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {//if arming call succeeded
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
	    }
        }
	//STOP publishing, we will do it from command line
	//target_pos_pub.publish(positionTarget);//publish the latest position_Target
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}