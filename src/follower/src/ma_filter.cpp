#include "ros/ros.h" //all headers necessary for ROS functions
#include "follower/uwb_node_reading.h"//defines the uwb_node_reading object, in 'follower' namespace 
#include "follower/filtered_reading.h"//defines the filtered_reading object

//length-6 moving average filter
class maFilter {
    private:
        int prev_vals[4][6] = {0}; //1st dim is D, P, X, Y, 2nd is the 6 previous values
        int sums[4] = {0}; //cumulative sum of 6 values for D, P, Xcm, Ycm
        int buffer_pos = 0;
        float averages[4] = {0};
    public:
        ros::Publisher filtered_pub;
        void readingReceivedCallback(const follower::uwb_node_reading message) {
            int i = 0;
            follower::filtered_reading message_out;//declare a filtered_reading object to be the outgoing message
            for (i = 0; i < 4; i++) sums[i] -= prev_vals[i][buffer_pos];//subtract the reading to be overwritten from the cumulative sum
            prev_vals[0][buffer_pos] = message.D;
            prev_vals[1][buffer_pos] = message.P;
            prev_vals[2][buffer_pos] = message.Xcm;
            prev_vals[3][buffer_pos] = message.Ycm;
            for (i = 0; i < 4; i++) {
                sums[i] += prev_vals[i][buffer_pos];//add the new readings to the cumulative sum
                averages[i] = (float) sums[i] / 6;//divide to get the moving average, store in averages[]
            }
            message_out.D_fil = averages[0];//write the averages to the message fields
            message_out.P_fil = averages[1];
            message_out.Xcm_fil = averages[2];
            message_out.Ycm_fil = averages[3];
            filtered_pub.publish(message_out);//publish the outgoing message of filtered values
            buffer_pos = (buffer_pos+1)%6;//increment the buffer position to the next one (circular)
            //ROS_INFO("Buffer position is %d", buffer_pos);
            //ROS_INFO("Averages are D: %f, P: %f, Xcm: %f, Ycm: %f", averages[0], averages[1], averages[2], averages[3]);
            //ROS_INFO("PDoA distance: %d, phase: %d, Xcm: %d, Ycm: %d", message.D, message.P, message.Xcm, message.Ycm);//display extracted frame data to rosconsole for debugging
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ma_filter");//initialise the node, name it "uwb_node_reader"
    ros::NodeHandle nh; //construct the first NodeHandle to fully initialise, handle contains communication fns
    maFilter maFilter;
    ros::Subscriber reading_sub = nh.subscribe("uwb_node_reading", 1000, &maFilter::readingReceivedCallback, &maFilter);//subscribe to topic 'uwb_node_reading', 1000 message queue, pass messages to readingReceivedCallback function
    maFilter.filtered_pub = nh.advertise<follower::filtered_reading>("filtered_reading", 1000);
    ros::spin();//enter loop and pump callbacks if received
    return 0;
}
