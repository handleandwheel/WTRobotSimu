// lidar_node_configure.h
// This file defines the parameters used in lidar_node
// Environment: ubuntu 20.04, ros noetic, webots 2021b
// Created by Liu, Yuming on Spet. 22nd, 2021
//

#ifndef WEBOTS_WS_LIDAR_NODE_CONFIGURE_H
#define WEBOTS_WS_LIDAR_NODE_CONFIGURE_H

#include "ros/ros.h"

using namespace std;

class LidarNodeConfig
{
public:
	LidarNodeConfig(){}
	void set(const ros::NodeHandle &);
	int sampling_time_period;
	double frequency;
};

void LidarNodeConfig::set(const ros::NodeHandle &nh)
{
	nh.param("/lidar_node/sampling_time_period", sampling_time_period, 64);
	// min fre is 1, max fre is 25
	nh.param("/lidar_node/frequency", frequency, 10.0);
}

#endif //WEBOTS_WS_LIDAR_NODE_CONFIGURE_H
