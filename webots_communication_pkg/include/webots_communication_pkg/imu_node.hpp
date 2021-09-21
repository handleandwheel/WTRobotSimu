// imu_node.hpp
// This file mix InertialUnit, Accelerometer and Gyro in webots, and publish them as imu message.
// Environment: ubuntu 20.04, ros noetic, webots 2021b, webots-ros 4.1.0
// Created by Liu, Yuming on Spet. 21st, 2021

#ifndef WEBOTS_WS_IMU_NODE_HPP
#define WEBOTS_WS_IMU_NODE_HPP

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "webots_ros/set_int.h"
#include "webots_ros/get_float.h"
#include "webots_communication_pkg/imu_node_configure.h"

using namespace std;

class IMUNode
{
public:
    IMUNode(ros::NodeHandle &);
private:
    void model_name_callback(const std_msgs::String::ConstPtr &);
    void enable(ros::ServiceClient &, ros::ServiceClient &, int &);
    void imu_ang_pos_callback(const sensor_msgs::Imu::ConstPtr &);
    void imu_lin_acc_callback(const sensor_msgs::Imu::ConstPtr &);
    void imu_ang_vel_callback(const sensor_msgs::Imu::ConstPtr &);
    void timer_callback(const ros::TimerEvent &);
    ros::ServiceClient robotBasicTimeStepClient;
    ros::ServiceClient inertialUnitEnableClient;
    ros::ServiceClient accelerometerEnableClient;
    ros::ServiceClient gyroEnableClient;
    ros::Subscriber modelNameSub;
    ros::Subscriber inertialUnitSub;
    ros::Subscriber accelerometerSub;
    ros::Subscriber gyroSub;
    ros::Publisher imuPub;
    IMUNodeConfig imuNodeConfig;
    sensor_msgs::Imu imu_msg;
    string model_name = "empty";
};

IMUNode::IMUNode(ros::NodeHandle &nh)
{
    imuNodeConfig.set(nh);

    modelNameSub = nh.subscribe<std_msgs::String>("/model_name", 1, &IMUNode::model_name_callback, this);

    //model name varies every time simulation runs
    while(model_name == "empty") ros::spinOnce();
    ROS_INFO("[IMU_NODE_INFO] Model name loaded.");

    // enable
    robotBasicTimeStepClient = nh.serviceClient<webots_ros::get_float>(model_name + "/robot/get_basic_time_step");
    inertialUnitEnableClient = nh.serviceClient<webots_ros::set_int>(model_name + "/imu_ang_pos/enable");
    accelerometerEnableClient = nh.serviceClient<webots_ros::set_int>(model_name + "/imu_lin_acc/enable");
    gyroEnableClient = nh.serviceClient<webots_ros::set_int>(model_name + "/imu_ang_vel/enable");

    enable(robotBasicTimeStepClient, inertialUnitEnableClient, imuNodeConfig.sampling_time_period);
    enable(robotBasicTimeStepClient, accelerometerEnableClient, imuNodeConfig.sampling_time_period);
    enable(robotBasicTimeStepClient, gyroEnableClient, imuNodeConfig.sampling_time_period);

    inertialUnitSub = nh.subscribe<sensor_msgs::Imu>(model_name + "/imu_ang_pos/quaternion", 1, &IMUNode::imu_ang_pos_callback, this);
    accelerometerSub = nh.subscribe<sensor_msgs::Imu>(model_name + "/imu_lin_acc/values", 1, &IMUNode::imu_lin_acc_callback, this);
    gyroSub = nh.subscribe<sensor_msgs::Imu>(model_name + "/imu_ang_vel/values", 1, &IMUNode::imu_ang_vel_callback, this);
    imuPub = nh.advertise<sensor_msgs::Imu>("/WTRobotSimu/imu", 1);

    ros::Timer controlThread = nh.createTimer(ros::Duration(imuNodeConfig.sampling_time_period*1e-3), &IMUNode::timer_callback, this);

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
}

void IMUNode::model_name_callback(const  std_msgs::String::ConstPtr &msg)
{
    if (msg->data.find_first_of("WTRobotSimu") == 0 && model_name == "empty") model_name = msg->data;
}

void IMUNode::enable(ros::ServiceClient &basicTimeStepClient, ros::ServiceClient &enableClient, int &sampling_period)
{
    static int sensor_part = 0;
    double basic_time_step;
    while(abs(basic_time_step - 0.0) < 1e-3)
    {
        webots_ros::get_float time_step_srv;
        time_step_srv.request.ask = true;
        basicTimeStepClient.call(time_step_srv);
        basic_time_step = time_step_srv.response.value;
    }

    webots_ros::set_int sampling_period_srv;
    if(sampling_period % int(basic_time_step) == 0)
    {
        sampling_period_srv.request.value = sampling_period;
        enableClient.call(sampling_period_srv);
    }
    else if(basic_time_step <= 1)
    {
        sampling_period_srv.request.value = 1;
        enableClient.call(sampling_period_srv);
        ROS_INFO("[IMU_NODE_INFO][part %d] Invalid sampling time. Sampling time is set to 1ms", sensor_part);
    }
    else
    {
        sampling_period_srv.request.value = int(basic_time_step);
        enableClient.call(sampling_period_srv);
        ROS_INFO("[IMU_NODE_INFO][part %d] Invalid sampling time. Sampling time is set to robot basic time step:%d.", sensor_part, int(basic_time_step));
    }
    sensor_part++;
}

void IMUNode::imu_ang_pos_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu_msg.orientation.x = msg->orientation.x;
    imu_msg.orientation.y = msg->orientation.y;
    imu_msg.orientation.z = msg->orientation.z;
    imu_msg.orientation.w = msg->orientation.w;
}

void IMUNode::imu_lin_acc_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu_msg.linear_acceleration.x = msg->linear_acceleration.x;
    imu_msg.linear_acceleration.y = msg->linear_acceleration.y;
    imu_msg.linear_acceleration.z = msg->linear_acceleration.z;
}

void IMUNode::imu_ang_vel_callback(const sensor_msgs::Imu::ConstPtr &msg)
{

    imu_msg.angular_velocity.x = msg->angular_velocity.x;
    imu_msg.angular_velocity.y = msg->angular_velocity.y;
    imu_msg.angular_velocity.z = msg->angular_velocity.z;
}

void IMUNode::timer_callback(const ros::TimerEvent &evt)
{
    static long int seq = 1;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.seq = seq;
    imu_msg.header.frame_id = "WTRobotSimu/imu";
    imuPub.publish(imu_msg);
}

#endif //WEBOTS_WS_IMU_NODE_HPP
