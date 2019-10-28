/**Copyright (c) 2019   Hao Da (Kevin) Dong
 * @file       talker.cpp
 * @date       10/27/2019
 * @brief      Simple ROS publisher from the beginner tutorials
 * @license    This project is released under the BSD-3-Clause License.
 */

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv) {

    ros::init(argc, argv, "talker");

    // Create node handle and publisher to publish to chatter topic
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    // Set publishing rate to 10 Hz
    ros::Rate loop_rate(10);

    // Counter to keep track of the number of published messages
    int count = 0;

    while (ros::ok()) {
        // Create String message to publish
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Number of published messages: " << count;
        msg.data = ss.str();

        // Print message string to terminal
        ROS_INFO("%s", msg.data.c_str());

        // Publish message
        chatter_pub.publish(msg);

        ros::spinOnce();

        // Pause the program until it is time to publish again
        loop_rate.sleep();
        ++count;
    }

    return 0;
}