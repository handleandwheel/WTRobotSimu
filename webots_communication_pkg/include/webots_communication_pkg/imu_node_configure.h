// imu_node_configure.h
// Environment: ubuntu 20.04, ros noetic
// Created by handleandwheel on 2021/9/21.
//

#ifndef WEBOTS_WS_IMU_NODE_CONFIGURE_H
#define WEBOTS_WS_IMU_NODE_CONFIGURE_H

#include "ros/ros.h"

using namespace std;

class IMUNodeConfig
{
public:
    IMUNodeConfig(){}
    void set(const ros::NodeHandle &);
    int sampling_time_period;
};

void IMUNodeConfig::set(const ros::NodeHandle &nh)
{
    nh.param("imu_node/sampling_time_period", sampling_time_period, 64);
}

#endif //WEBOTS_WS_IMU_NODE_CONFIGURE_H
