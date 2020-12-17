//This is an ACTUAL FLIGHT node to take local position estimate (barometric-altitude + IMU attitude) from the PX4, feed it back as a fake vicon position to enable flight without any local position estimate (GPS/VICON)
#include "ros/ros.h" //all headers necessary for ROS functions
#include <mavros_msgs/CommandBool.h>//following header files define the ROS message objects in respective namespaces
#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped current_pose;
double z_offset_sum = 0;
double z_offset = 0;
int count_offset_samples = 0;
void position_cb(const geometry_msgs::PoseStamped msg) {
	if (count_offset_samples < 5) {
		z_offset_sum += msg.pose.position.z;
		count_offset_samples++;
	} else if (count_offset_samples == 5) {
		z_offset = z_offset_sum / 5;
		count_offset_samples++;
	}
	current_pose.pose.position.x = 0;
	current_pose.pose.position.y = 0;
	current_pose.pose.position.z = msg.pose.position.z;
	current_pose.pose.orientation.x = msg.pose.orientation.x;
	current_pose.pose.orientation.y = msg.pose.orientation.y;
	current_pose.pose.orientation.z = msg.pose.orientation.z;
	current_pose.pose.orientation.w = msg.pose.orientation.w;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "feedback_fake_vicon");//initialise the node, name it
	ros::NodeHandle nh; //construct the first NodeHandle to fully initialise, handle contains communication fns
	//LISTEN TO THE CURRENT PX POSITION ESTIMATE
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, position_cb);
	//ECHO BACK INTO VISION ESTIMATE
	ros::Publisher vision_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 100);
	ros::Rate rate(20.0); //messages to be streamed at 30-50 Hz
	while (ros::ok()) {
		if (count_offset_samples >= 5) {
			nh.setParam("/z_offset_barometer", z_offset);
		}
		//PUBLISH current known pose back into vision position estimate
		current_pose.header.stamp = ros::Time::now();
		vision_pos_pub.publish(current_pose);
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
