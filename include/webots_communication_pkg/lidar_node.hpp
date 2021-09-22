// lidar_node.hpp
// This file defines class LidarNode, which is used to communicate with the lidar of WTRobotSimu PROTO of webtos
// Environment: ubuntu 20.04, ros noetic, webots 2021b, webots_ros 4.1.0,
// Created by Liu, Yuming on Spet. 22nd, 2021
//

#ifndef WEBOTS_WS_LIDAR_NODE_HPP
#define WEBOTS_WS_LIDAR_NODE_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "webots_ros/set_int.h"
#include "webots_ros/get_float.h"
#include "webots_ros/set_float.h"
#include "webots_communication_pkg/lidar_node_configure.h"

using namespace std;

class LidarNode
{
public:
    LidarNode(ros::NodeHandle &);
private:
	void enable(ros::ServiceClient &, ros::ServiceClient &, int &);
    void model_name_callback(const std_msgs::String::ConstPtr &);
    void lidar_callback(const sensor_msgs::LaserScan::ConstPtr &);
    ros::ServiceClient robotBasicTimeStepClient;
    ros::ServiceClient lidarEnableClient;
    ros::ServiceClient lidarFrequencyClient;
    ros::Subscriber modelNameSub;
    ros::Subscriber lidarScanSub;
    ros::Publisher lidarScanPub;
    LidarNodeConfig lidarNodeConfig;
    string model_name = "empty";
};

LidarNode::LidarNode(ros::NodeHandle &nh)
{
 	lidarNodeConfig.set(nh);
	
	modelNameSub = nh.subscribe<std_msgs::String>("/model_name", 1, &LidarNode::model_name_callback, this);

    //model name varies every time simulation runs
    while(model_name == "empty") ros::spinOnce();
    ROS_INFO("[LIDAR_NODE_INFO] Model name loaded.");

    robotBasicTimeStepClient = nh.serviceClient<webots_ros::get_float>(model_name + "/robot/get_basic_time_step");
    lidarEnableClient = nh.serviceClient<webots_ros::set_int>(model_name + "/lidar/enable");
    
    enable(robotBasicTimeStepClient, lidarEnableClient, lidarNodeConfig.sampling_time_period);
    
    lidarFrequencyClient = nh.serviceClient<webots_ros::set_float>(model_name + "/lidar/set_frequency");
    
    webots_ros::set_float freq_srv;
    freq_srv.request.value = lidarNodeConfig.frequency;
    lidarFrequencyClient.call(freq_srv);
    
    lidarScanSub = nh.subscribe<sensor_msgs::LaserScan>(model_name + "/lidar/laser_scan/layer0", 1, &LidarNode::lidar_callback, this);
    lidarScanPub = nh.advertise<sensor_msgs::LaserScan>("WTRobotSimu/lidar", 1);
    
    ros::spin();

    ros::waitForShutdown();
}

void LidarNode::model_name_callback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data.find_first_of("WTRobotSimu") == 0 && model_name == "empty") model_name = msg->data;
}

void LidarNode::enable(ros::ServiceClient &basicTimeStepClient, ros::ServiceClient &enableClient, int &sampling_period)
{
	double basic_time_step = 0.0;
	while (abs(basic_time_step - 0.0) < 1e-3)
	{
		webots_ros::get_float time_step_srv;
		time_step_srv.request.ask = true;
		basicTimeStepClient.call(time_step_srv);
		basic_time_step = time_step_srv.response.value;
	}
	
	webots_ros::set_int sampling_period_srv;
	if (sampling_period % int(basic_time_step) == 0)
	{
		sampling_period_srv.request.value = sampling_period;
		enableClient.call(sampling_period_srv);
		if (sampling_period_srv.response.success)
			ROS_INFO("[LIDAR_NODE_INFO] Sensor enabled.");
		else
		{
			ROS_WARN("[LIDAR_NODE_INFO] Enable failed. Trying again...");
			for (int i = 0; i < 20; i++)
			{
				if (sampling_period_srv.response.success)
				{
					ROS_INFO("[LIDAR_NODE_INFO] Sensor enabled.");
					break;
				}
				if(i = 19) ROS_ERROR("[LIDAR_NODE_INFO] Enable failed. Please restart the node");
			}
		}
	}
	else if (basic_time_step <= 1)
	{
		sampling_period_srv.request.value = 1;
		enableClient.call(sampling_period_srv);
		if (sampling_period_srv.response.success)
			ROS_WARN("[LIDAR_NODE_INFO] Invalid sampling time. Sampling time is set to 1ms");
		else
		{
			ROS_WARN("[LIDAR_NODE_INFO] Enable failed. Trying again...");
			for (int i = 0; i < 20; i++)
			{
				if (sampling_period_srv.response.success)
				{
					ROS_WARN("[LIDAR_NODE_INFO] Invalid sampling time. Sampling time is set to 1ms");
					break;
				}
				if(i = 19) ROS_ERROR("[LIDAR_NODE_INFO] Enable failed. Please restart the node");
			}
		}
	}
	else
	{
		sampling_period_srv.request.value = int(basic_time_step);
		enableClient.call(sampling_period_srv);
		if (sampling_period_srv.response.success)
			ROS_WARN("[LIDAR_NODE_INFO] Invalid sampling time. Sampling time is set to robot basic time step:%d.",
					 int(basic_time_step));
		else
		{
			ROS_WARN("[LIDAR_NODE_INFO] Enable failed. Trying again...");
			for (int i = 0; i < 20; i++)
			{
				if (sampling_period_srv.response.success)
				{
					ROS_WARN(
							"[LIDAR_NODE_INFO] Invalid sampling time. Sampling time is set to robot basic time step:%d.",
							int(basic_time_step));
					break;
				}
				if(i = 19) ROS_ERROR("[LIDAR_NODE_INFO] Enable failed. Please restart the node");
			}
		}
	}
}

void LidarNode::lidar_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	sensor_msgs::LaserScan lidar_msg;
	lidar_msg = *msg;
	lidarScanPub.publish(lidar_msg);
}

#endif //WEBOTS_WS_LIDAR_NODE_HPP
