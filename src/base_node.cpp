// base_node.cpp
// Environment: ubuntu 20.04, ros noetic
// Created by Liu, Yuming on Spet. 19th, 2021.
//

#include "ros/ros.h"
#include "webots_communication_pkg/base_node.hpp"

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "base_node");
    ros::NodeHandle nh;
    BaseNode baseNode(nh);
    return 0;
}