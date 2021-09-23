// lidar_node.cpp
// Environment: ubuntu 20.04, ros noetic, webots 2021b
// Created by Liu, Yuming on Spet. 22nd, 2021
//

#include "ros/ros.h"
#include "webots_communication_pkg/lidar_node.hpp"

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_node");
    ros::NodeHandle nh;
    LidarNode lidarNode(nh);
    return 0;
}