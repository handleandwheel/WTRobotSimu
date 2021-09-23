// camera_node.cpp
// Environment: ubuntu 20.04, ros noetic
// Created by Liu, Yuming on Spet. 20th, 2021.
//

#include "webots_communication_pkg/camera_node.hpp"

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;
    CameraNode cameraNode(nh);
    return 0;
}