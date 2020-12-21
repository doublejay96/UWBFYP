//This ROS node reads the filtered UWB readings, converts that into appropriate offboard commands in velocity (using just proportional control), and sends it to the mavros_node node by publishing to 'mavros/setpoint_raw/local' topic. It is always calculating, but only starts sending offboard commands when the 'flight_status' topic lets it know that the previous mavros_takeoff node is done.
#include "ros/ros.h" //all headers necessary for ROS functions
#include <mavros_msgs/CommandBool.h>//following header files define the ROS message objects in respective namespaces
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include "follower/flight_status.h"//defines the flight_status object, in 'follower' namespace
#include "follower/filtered_reading.h"//defines the filtered_reading object

mavros_msgs::State current_state;//global var to monitor the current state (of the connection)
uint8_t flightStage = 0;//the current flight status, to activate the sending of offboard commands
float Xcm, Ycm;//LAST KNOWN position of UWB tag relative to node (in cm, F-L reference axes)
float X_offset = 0, Y_offset = 100;//distance to keep the leader at relative to folllower (in cm, F-L reference axes)
const float VELOCITY_MULTIPLIER = 1;//distance divided by this gives velocity to set. Cover the distance in this amount of seconds?

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
    while (ros::ok() && current_state.connected && flightStage == 2) {
		positionTarget.velocity.x = ((Xcm - X_offset)/100) * VELOCITY_MULTIPLIER; //get the x-distance to move (convert from cm to m)
    	positionTarget.velocity.y = ((Ycm - Y_offset)/100) * VELOCITY_MULTIPLIER; //get the y-distance to move
		target_pos_pub.publish(positionTarget);
		ros::spinOnce(); //call any callbacks waiting
		rate.sleep();
    }
	ROS_INFO("Flight stage is now %d, the mavros_offboard node is exiting now", flightStage);
    return 0;
}
