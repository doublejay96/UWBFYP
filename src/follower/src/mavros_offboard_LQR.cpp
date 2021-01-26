//This ROS node reads the filtered UWB readings, converts that into appropriate offboard commands in velocity using the LQR controller, and sends it to the mavros_node node by publishing to 'mavros/setpoint_raw/local' topic. It is always calculating, but only starts sending offboard commands when the 'flight_status' topic lets it know that the previous mavros_takeoff node is done.
#include "ros/ros.h" //all headers necessary for ROS functions
#include "ros/package.h"
#include <mavros_msgs/CommandBool.h>//following header files define the ROS message objects in respective namespaces
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>//for listening to vision_pose/pose topic
#include "follower/flight_status.h"//defines the flight_status object, in 'follower' namespace
#include "follower/filtered_reading.h"//defines the filtered_reading object
#include "follower/state_error.h"
#include "follower/sim_altitude.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

mavros_msgs::State current_state;//global var to monitor the current state (of the connection)
uint8_t flightStage = 0;//the current flight status, to activate the sending of offboard commands
float Xcm, Ycm;//LAST KNOWN position of UWB tag relative to node (in cm, F-L reference axes)(message def is float32s)
float desired_z = 1, z_error;//Z-displacement of follower from desired altitude (obtained directly from VICON)
float X_offset = 0, Y_offset = 300;//distance to keep the leader at relative to folllower (in cm, F-L reference axes)

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
void altitudeCallback(const geometry_msgs::PoseStamped message) {
	z_error = desired_z - message.pose.position.z;
}
void simAltitudeCallback(const follower::sim_altitude message) {
    z_error = desired_z - message.altitude;
}

//The actual LQR function
Eigen::Vector3f LQR(Eigen::Vector3f state_error, Eigen::Matrix3f Q, Eigen::Matrix3f R, Eigen::Matrix3f A, Eigen::Matrix3f B) {
    int N = 50;//time-horizon
    std::vector<Eigen::Matrix3f> P_vec (N+1);//Positive,definite,symmetric solution to DARE
    P_vec[N] = Q;//terminal condition
    //Pre-compute transposes of A, B to save function calls, A,B do not vary
    Eigen::Matrix3f A_t = A.transpose();
    Eigen::Matrix3f B_t = B.transpose();
    for (int i = N; i > 0; i--) {//Iterate backwards in time
        //Discrete-time Algebraic Riccati Equation
        P_vec[i-1] = (A_t * P_vec[i] * A) - (A_t * P_vec[i] * B) * (R + (B_t * P_vec[i] * B)).inverse() * (B_t * P_vec[i] * A) + Q;
    }
    std::vector<Eigen::Matrix3f> K_vec (N);//feedback gain matrix K
    std::vector<Eigen::Vector3f> inputs_vec (N);//control inputs
    for (int i = 0; i < N; i++) {//Iterate forwards in time
        K_vec[i] = -(R + (B_t * P_vec[i+1] * B)).inverse() * B_t * P_vec[i+1] * A;
        inputs_vec[i] = K_vec[i] * state_error;
    }
    Eigen::Vector3f optimal_input = inputs_vec[N-1];
    return optimal_input;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mavros_offboard_LQR");//initialise the node, name it "mavros_offboard_LQR"  
	ros::NodeHandle nh; //construct the first NodeHandle to fully initialise, handle contains communication fns
	int use_controller = 0;
    nh.param("use_controller", use_controller, 0);//if not set, default to 0 (PID)
    if (use_controller != 1) {
        ROS_INFO("Not using LQR controller");
        return 0;//exit the node immediately
    } else {
        ROS_INFO("Using LQR controller");
    }
	ros::Subscriber filtered_sub = nh.subscribe<follower::filtered_reading>("filtered_reading", 1000, filteredReadingReceivedCallback);//subscribe to the filtered readings published by the ma_filter node
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	ros::Subscriber altitude_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 100, altitudeCallback);
	ros::Subscriber sim_altitude_sub = nh.subscribe<follower::sim_altitude>("sim_altitude", 100, simAltitudeCallback);
	ros::Publisher target_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 100);
	ros::Subscriber flight_status_sub = nh.subscribe<follower::flight_status>("flight_status", 10, flightStatusReceivedCallback);
	ros::Publisher state_error_pub = nh.advertise<follower::state_error>("state_error", 100);
	ros::Rate rate(10.0); //setpoint publishing rate MUST be faster than 2 Hz
	bool override_desired_offsets = false;
	nh.param("override_desired_offsets", override_desired_offsets, false);//if parameter not set, default to FALSE, use the original values specified above
	if (override_desired_offsets) {
		nh.getParam("X_offset", X_offset);
		nh.getParam("Y_offset", Y_offset);
		ROS_INFO("Follower offset override: set to X_offset %f, Y_offset %f", X_offset, Y_offset);
	}
	//LQR Controller.
	float x_error = 0;//the error value for that instant 
	float y_error = 0;
	Eigen::Matrix3f A, B, Q, R;
	Eigen::Vector3f state, control;
	//Initialise the A, B matrices from state space model file
    std::string SSmodel_path = "state_space_model.csv";
	nh.getParam("SSmodel_path", SSmodel_path);
	SSmodel_path = ros::package::getPath("follower") + "/../.." + SSmodel_path;
	std::ifstream SSmodel_file;
    SSmodel_file.open(SSmodel_path.c_str());
    std::vector<float> SSmodel_A, SSmodel_B;//temp vector storage for the floats
    char line[256];
    char* p;
    for (int l = 0; l < 6; l++) {
        SSmodel_file.getline(line, 256);
        p = line;
        for (int i = 0; i < 3; i++) {
            if (l < 3) {
                SSmodel_A.push_back(atof(p));
            } else {
                SSmodel_B.push_back(atof(p));
            }
            p = strstr(p, ",") + 1;
        }
    }
	A = Eigen::Map<Eigen::Matrix3f>(SSmodel_A.data());
    B = Eigen::Map<Eigen::Matrix3f>(SSmodel_B.data());
    //Initialise the Q, R matrices
    std::string QR_path = "Q_R.csv";
	nh.getParam("QR_path", QR_path);
	QR_path = ros::package::getPath("follower") + "/../.." + QR_path;
	std::ifstream QR_file;
    QR_file.open(QR_path.c_str());
    std::vector<float> QR_Q, QR_R;//temp vector storage for the floats
    for (int l = 0; l < 6; l++) {
        QR_file.getline(line, 256);
        p = line;
        for (int i = 0; i < 3; i++) {
            if (l < 3) {
                QR_Q.push_back(atof(p));
            } else {
                QR_R.push_back(atof(p));
            }
            p = strstr(p, ",") + 1;
        }
    }
	Q = Eigen::Map<Eigen::Matrix3f>(QR_Q.data());	
	R = Eigen::Map<Eigen::Matrix3f>(QR_R.data());	
	//Create state_error publishing message
	follower::state_error state_error_message;
	while (flightStage != 2) {//while the quadcopter hasn't reached flight stage 2 (still connecting or taking off)
		ros::spinOnce(); //call any callbacks waiting
		rate.sleep();
	}
	mavros_msgs::PositionTarget positionTarget;//This is the setpoint we want to move to
	positionTarget.coordinate_frame = 8;//Set the reference frame of the command to body (FLU) frame
	//positionTarget.type_mask = 0b110111111000;//set positions only, 3576
	positionTarget.type_mask = 0b101111000111;//set velocities only, 3527
	positionTarget.yaw = 0;//yaw locked to 0 (absolute values accepted)
	while (ros::ok() && current_state.connected) {//begin LQR controller
		if (flightStage == 3) break;
		x_error = (Ycm - Y_offset)/100;//calculate error in X, convert to m
		y_error = (Xcm - X_offset)/100;
		//z_error is calculated in the callback from the VICON
		state << x_error, y_error, z_error;//update the current state with the UWB readings
		control = LQR(state, Q, R, A, B);//Run the LQR
		positionTarget.velocity.x = -control(0);//update positionTarget with optimal controls
		positionTarget.velocity.y = -control(1);
		positionTarget.velocity.z = -control(2);
		//Implement 'dead zone' in which no velocity commands are given
		//if (x_error < 0.1 && x_error > -0.1) {
		//	positionTarget.velocity.x = 0;
		//}
		//if (y_error < 0.1 && y_error > -0.1) {
		//	positionTarget.velocity.y = 0;
		//}
		if (flightStage == 2) target_pos_pub.publish(positionTarget);
		//ROS_INFO("For x, P: %f, I: %Lf, D:%f, output velocity: %f", x_error, integral_x, derivative_x, positionTarget.velocity.x);
		//ROS_INFO("For y, P: %f, I: %Lf, D:%f, output velocity: %f", y_error, integral_y, derivative_y, positionTarget.velocity.y);
		//ROS_INFO("For z, P: %f, I: %Lf, D:%f, output velocity: %f", z_error, integral_z, derivative_z, positionTarget.velocity.z);
		state_error_message.x_error = x_error;
		state_error_message.y_error = y_error;
		state_error_message.z_error = z_error;
		state_error_pub.publish(state_error_message);
		ros::spinOnce(); //call any callbacks waiting
		rate.sleep();
	}
	ROS_INFO("Flight stage is now %d, the mavros_offboard node is exiting now", flightStage);
	return 0;
}
