// imu_node.cpp
// Environment: ubuntu 20.04, ros noetic, webots 2021b
// Created by Liu, Yuming on Spet. 21st, 2021
//

#include "ros/ros.h"
#include "webots_communication_pkg/imu_node.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;
    IMUNode imuNode(nh);
    return 0;
}