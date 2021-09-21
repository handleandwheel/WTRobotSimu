// base_node_configure.h
// This file defines the parameters used in camera_node
// Environment: ubuntu 20.04, ros noetic, webots 2021b
// Created by Liu, Yuming on Spet. 20th, 2021
//

#ifndef WEBOTS_WS_CAMERA_NODE_CONFIGURE_H
#define WEBOTS_WS_CAMERA_NODE_CONFIGURE_H

#include "ros/ros.h"

using namespace std;

class CameraNodeConfig
{
public:
    CameraNodeConfig(){}
    void set(const ros::NodeHandle &);
    double exposure;
    double focalDistance;
    double fov;
    std::vector<int> resolution = {10, 2}; //width, length
    int samplingPeriod;
};

void CameraNodeConfig::set(const ros::NodeHandle &nh)
{
    nh.param("camera_node/exposure", exposure, 1.0);
    // 1.0 is the default param in webots
    nh.param("camera_node/focalDistance", focalDistance, 2.0);
    // The max focal distance is 4.0, and the min focal distance is 0
    nh.param("camera_node/fov", fov, 2.0);
    // The max FOV is 2, ang the min FOV is 0.3
    nh.param("camera_node/resolution",resolution, resolution);
    // The max resolution is 1280*1280, so it's meaningless to set larger numbers
    nh.param("camera_node/samplingPeriod", samplingPeriod, 2);
}

#endif //WEBOTS_WS_CAMERA_NODE_CONFIGURE_H
