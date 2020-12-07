#include "ros/ros.h" //all headers necessary for ROS functions
#include "follower/uwb_node_reading.h"//defines the uwb_node_reading object, in 'follower' namespace 
#include <string> //strstr find substring function
//Header files for serial port
#include <termios.h> //terminal control and struct
#include <fcntl.h> //file control
#include <errno.h> //error functions
#include <unistd.h> //POSIX OS API, read/write/close functions

//this function reads the JSON frame (in string form) and writes the important info (D,P,Xcm,Ycm) into the uwb_node_reading object pointer
void readPDoAFrame(char* read_buf, follower::uwb_node_reading* output_info, char* D_field, char* P_field, char* Xcm_field, char* Ycm_field) {//pass in all these to avoid re-declaring
    char* p; //pointer for incrementing along the string
    p = strstr(read_buf, D_field) + 4; //find first occurence of "D" in read_buf, +4 to get to the start of the value, store pointer to this char in p
    output_info->D = atoi(p); //convert the remaining array (substring) pointed to by p to an integer, giving the D value
    p = strstr(p, P_field) + 4; //continue from the previous pointer, find first occurence of "P" in the remaining substring pointed to by p
    output_info->P = atoi(p);
    p = strstr(p, Xcm_field) + 6; //+6 bc "Xcm": is 6 bytes (chars) long
    output_info->Xcm = atoi(p);
    p = strstr(p, Ycm_field) + 6;
    output_info->Ycm = atoi(p);
    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "uwb_node_reader");//initialise the node, name it "uwb_node_reader"
    ros::NodeHandle nh; //construct the first NodeHandle to fully initialise, handle contains communication fns
    ros::Publisher reading_pub = nh.advertise<follower::uwb_node_reading>("uwb_node_reading", 1000);//advertise that we want to publish on topic 'uwb_node_reading', 1000 message queue
    ros::Rate loop_rate(10);//the rate this thing runs (10 Hz, same as UWB serial port input)
    //Open the serial port
    int serial_port = open("/dev/ttyACM0", O_RDWR); //returns the file descriptor (usually 3), set O_RDWR for both read and write, O_RDONLY for read only access
    if (serial_port < 0) {
        ROS_INFO("Error opening serial port, returned %d\n", serial_port);
    } else {
        ROS_INFO("Serial port opened successfully\n");
    }
    struct termios tty;
    if (tcgetattr(serial_port, &tty) != 0) {//get parameters associated with the object in serial_port, stores in termios struct, returns 0 on success
        ROS_INFO("Error reading atributes from file descriptor to termios struct\n");
    }
    tty.c_lflag &= ICANON; //set canonical mode, input becomes available line-by-line. Default should be canonical alr, but just make sure.
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {//get parameters associated with the object in serial_port, stores in termios struct, returns 0 on success
        ROS_INFO("Error setting attributes from file descriptor to termios struct");
    }
    char read_buf[256];//buffer to store what we read from the serial port (assume each line less than 256 chars)
    int n = 0;//number of bytes read from the serial port (1 char is 1 byte)
    n = read(serial_port, &read_buf, sizeof(read_buf)); //read the first (usually bugged) line to discard
    char D_field[] = "\"D\"", P_field[] = "\"P\"", Xcm_field[] = "\"Xcm\"", Ycm_field[] = "\"Ycm\""; //store all these strings to be searched for later
    while (ros::ok()) {
        n = read(serial_port, &read_buf, sizeof(read_buf)); //read the first (usually bugged) line to discard (BLOCKING READ)
        if (n == 1) continue; //if it is the second of a double newline, continue (without sleeping)
        read_buf[n-1] = '\0'; //replace the final character, newline, with null to make C-style string
        follower::uwb_node_reading message;//declare a uwb_node_reading object named 'message'
        readPDoAFrame(read_buf, &message, D_field, P_field, Xcm_field, Ycm_field);//this function reads the frame in string form, puts important info into pdoa_Info
        //ROS_INFO("PDoA distance: %d, phase: %d, Xcm: %d, Ycm: %d", message.D, message.P, message.Xcm, message.Ycm);//display extracted frame data to rosconsole for debugging
        //ROS_INFO("%s", read_buf); //display raw frame to rosconsole for debugging
        reading_pub.publish(message); //publish the message object
        ros::spinOnce(); //call any callbacks waiting (none currently)
        loop_rate.sleep(); //sleep for appropriate amt of time to maintain rate
    }
    close(serial_port);//close the file descriptor gracefully
    return 0;
}
