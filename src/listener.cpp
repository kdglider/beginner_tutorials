/**
 * Copyright (c) 2019   Hao Da (Kevin) Dong
 * @file       listener.cpp
 * @date       2019/11/10
 * @brief      Simple ROS subscriber from the beginner tutorials
 * @license    This project is released under the BSD-3-Clause License. See full details in LICENSE.
 */

#include "std_msgs/String.h"
#include "ros/ros.h"

 /**
 * @brief Callback function to print chatter topic messages to the terminal
 * @param msg Topic message
 * @return None
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO_STREAM("Read by listener: [" << msg->data.c_str() << "]");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");

    // Create node handle and subscriber to subscribe to chatter topic
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    ros::spin();

    return 0;
}
