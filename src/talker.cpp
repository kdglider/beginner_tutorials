/**Copyright (c) 2019   Hao Da (Kevin) Dong
 * @file       talker.cpp
 * @date       10/27/2019
 * @brief      Simple ROS publisher from the beginner tutorials
 * @license    This project is released under the BSD-3-Clause License. See full details in LICENSE.
 */

#include <sstream>
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "beginner_tutorials/printString.h"

// String that will be published and modified by the printString service
std::string serviceMsg = "Default Message";

/**
 * @brief printString service callback function that returns a sentence with the name given in the request
 * @param req Service request
 * @param res Service response
 * @return None
 */
bool printString(beginner_tutorials::printString::Request  &req, beginner_tutorials::printString::Response &res) {
    res.returnMsg = "This ROS service exists to serve the master: " + req.name;
    serviceMsg = res.returnMsg;
    ROS_DEBUG_STREAM("Request: name = " + req.name);
    ROS_DEBUG_STREAM("Response Preview: returnMsg = " + res.returnMsg);
    return 1;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "talker");

    // Record publishing rate input from user (Hz)
    int pubRate = atoi(argv[1]);

    // Log warnings or errors if the publishing rate is too fast or negative
    if (pubRate > 1000) {
        ROS_ERROR_STREAM("Publishing rate over 1000 Hz detected (WAY too fast)");
    } 
    else if (pubRate > 100) {
        ROS_WARN_STREAM("Publishing rate over 100 Hz detected (may be too fast)");
    }
    else if (pubRate <= 0) {
        ROS_FATAL_STREAM("Negative publishing rate detected");
    }

    // Create node handle and publisher to publish to chatter topic
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    // Set publishing rate 
    ros::Rate loop_rate(pubRate);

    // Create service and advertise it over ROS
    ros::ServiceServer service = n.advertiseService("printString", printString);

    // Counter to keep track of the number of published messages
    int count = 0;

    while (ros::ok()) {
        // Create String message to publish
        std_msgs::String msg;
        std::stringstream ss;
    
        // Update stringstream and String msg objects
        ss << serviceMsg << ", Count: " << count;
        msg.data = ss.str();

        // Print message string to terminal
        ROS_INFO_STREAM("Published by talker: [" << msg.data.c_str() << "]");

        // Publish message
        chatter_pub.publish(msg);

        ros::spinOnce();

        // Pause the program until it is time to publish again
        loop_rate.sleep();
        count++;
    }

    return 0;
}
