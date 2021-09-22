// base_node.hpp
// This file defines class BaseNode, which is used as a low level program to communication with webots
// WARNING: Do Not Change The Order In Which The Clients Are Handled
// If the position is put before the velocity, there will be a tremendously large acceleration,
// and the wheels of the robot will fall apart for a short time, which will cause CHAOS!
// Environment: ubuntu 20.04, ros noetic, webots 2021b, webots-ros 4.1.0
// Created by Liu, Yuming on Spet. 19th, 2021

#ifndef WEBOTS_WS_BASE_NODE_HPP
#define WEBOTS_WS_BASE_NODE_HPP

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "webots_ros/set_float.h"
#include "webots_ros/motor_set_control_pid.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "webots_communication_pkg/base_node_configure.h"

using namespace std;

class BaseNode
{
public:
    BaseNode(ros::NodeHandle &);
private:
    void model_name_callback(const std_msgs::String::ConstPtr &);
    void vel_callback(const geometry_msgs::Twist::ConstPtr &);
    ros::ServiceClient frontLeftMotorPosClient, frontRightMotorPosClient;
    ros::ServiceClient backLeftMotorPosClient, backRightMotorPosClient;
    ros::ServiceClient frontLeftMotorVelClient, frontRightMotorVelClient;
    ros::ServiceClient backLeftMotorVelClient, backRightMotorVelClient;
    ros::ServiceClient frontLeftMotorPIDClient, frontRightMotorPIDClient;
    ros::ServiceClient backLeftMotorPIDClient, backRightMotorPIDClient;
    ros::ServiceClient frontLeftMotorTorqueClient, frontRightMotorTorqueClient;
    ros::ServiceClient backLeftMotorTorqueClient, backRightMotorTorqueClient;
    ros::Subscriber modelNameSub;
    string model_name = "empty";
    BaseNodeConfig baseNodeConfig;
    const double maxLinearVel = 0.5;
    const double maxAngularVel = 3.14;
};

BaseNode::BaseNode(ros::NodeHandle &nh)
{
    baseNodeConfig.set(nh);

    modelNameSub = nh.subscribe<std_msgs::String>("/model_name", 1, &BaseNode::model_name_callback, this);

    //model name varies every time simulation runs
    while(model_name == "empty") ros::spinOnce();
    ROS_INFO("[BASE_NODE_INFO] Model name loaded.");

    //velocity clients
    frontLeftMotorVelClient = nh.serviceClient<webots_ros::set_float>(model_name + "/frontLeftMotor/set_velocity");
    frontRightMotorVelClient = nh.serviceClient<webots_ros::set_float>(model_name + "/frontRightMotor/set_velocity");
    backLeftMotorVelClient = nh.serviceClient<webots_ros::set_float>(model_name + "/backLeftMotor/set_velocity");
    backRightMotorVelClient = nh.serviceClient<webots_ros::set_float>(model_name + "/backRightMotor/set_velocity");

    //set initial velocity
    webots_ros::set_float vel_srv;
    vel_srv.request.value = 0.0;
    frontLeftMotorVelClient.call(vel_srv);
    vel_srv.request.value = 0.0;
    frontRightMotorVelClient.call(vel_srv);
    vel_srv.request.value = 0.0;
    backLeftMotorVelClient.call(vel_srv);
    vel_srv.request.value = 0.0;
    backRightMotorVelClient.call(vel_srv);
    //DO NOT CHANGE THE ORDER OF THE POS AND VEL!

    //pos clients
    frontLeftMotorPosClient = nh.serviceClient<webots_ros::set_float>(model_name + "/frontLeftMotor/set_position");
    frontRightMotorPosClient = nh.serviceClient<webots_ros::set_float>(model_name + "/frontRightMotor/set_position");
    backLeftMotorPosClient = nh.serviceClient<webots_ros::set_float>(model_name + "/backLeftMotor/set_position");
    backRightMotorPosClient = nh.serviceClient<webots_ros::set_float>(model_name + "/backRightMotor/set_position");

    //set max pos
    webots_ros::set_float pos_srv;
    pos_srv.request.value = INFINITY;//baseNodeConfig.frontLeftMotorPos;
    frontLeftMotorPosClient.call(pos_srv);
    pos_srv.request.value = INFINITY;//baseNodeConfig.frontRightMotorPos;
    frontRightMotorPosClient.call(pos_srv);
    pos_srv.request.value = INFINITY;//baseNodeConfig.backLeftMotorPos;
    backLeftMotorPosClient.call(pos_srv);
    pos_srv.request.value = INFINITY;//baseNodeConfig.backRightMotorPos;
    backRightMotorPosClient.call(pos_srv);
    //DO NOT CHANGE THE ORDER OF THE POS AND VEL!

    //pid clients
    frontLeftMotorPIDClient = nh.serviceClient<webots_ros::motor_set_control_pid>(model_name + "/frontLeftMotor/set_control_pid");
    frontRightMotorPIDClient = nh.serviceClient<webots_ros::motor_set_control_pid>(model_name + "/frontRightMotor/set_control_pid");
    backLeftMotorPIDClient = nh.serviceClient<webots_ros::motor_set_control_pid>(model_name + "/backLeftMotor/set_control_pid");
    backRightMotorPIDClient = nh.serviceClient<webots_ros::motor_set_control_pid>(model_name + "/backRightMotor/set_control_pid");

    //set pid
    webots_ros::motor_set_control_pid pid_srv;
    pid_srv.request.controlp = baseNodeConfig.frontLeftMotorPID[0];
    pid_srv.request.controli = baseNodeConfig.frontLeftMotorPID[1];
    pid_srv.request.controld = baseNodeConfig.frontLeftMotorPID[2];
    frontLeftMotorPIDClient.call(pid_srv);
    pid_srv.request.controlp = baseNodeConfig.frontRightMotorPID[0];
    pid_srv.request.controli = baseNodeConfig.frontRightMotorPID[1];
    pid_srv.request.controld = baseNodeConfig.frontRightMotorPID[2];
    frontRightMotorPIDClient.call(pid_srv);
    pid_srv.request.controlp = baseNodeConfig.backLeftMotorPID[0];
    pid_srv.request.controli = baseNodeConfig.backLeftMotorPID[1];
    pid_srv.request.controld = baseNodeConfig.backLeftMotorPID[2];
    backLeftMotorPIDClient.call(pid_srv);
    pid_srv.request.controlp = baseNodeConfig.backRightMotorPID[0];
    pid_srv.request.controli = baseNodeConfig.backRightMotorPID[1];
    pid_srv.request.controld = baseNodeConfig.backRightMotorPID[2];
    backRightMotorPIDClient.call(pid_srv);

    //torque clients
    frontLeftMotorTorqueClient = nh.serviceClient<webots_ros::set_float>(model_name + "/frontLeftMotor/set_available_torque");
    frontRightMotorTorqueClient = nh.serviceClient<webots_ros::set_float>(model_name + "/frontRightMotor/set_available_torque");
    backLeftMotorTorqueClient = nh.serviceClient<webots_ros::set_float>(model_name + "/backLeftMotor/set_available_torque");
    backRightMotorTorqueClient = nh.serviceClient<webots_ros::set_float>(model_name + "/backRightMotor/set_available_torque");

    //set available torque
    webots_ros::set_float torque_srv;
    torque_srv.request.value = baseNodeConfig.frontLeftMotorTorque;
    frontLeftMotorTorqueClient.call(torque_srv);
    torque_srv.request.value = baseNodeConfig.frontRightMotorTorque;
    frontRightMotorTorqueClient.call(torque_srv);
    torque_srv.request.value = baseNodeConfig.backLeftMotorTorque;
    backLeftMotorTorqueClient.call(torque_srv);
    torque_srv.request.value = baseNodeConfig.backRightMotorTorque;
    backRightMotorTorqueClient.call(torque_srv);

    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &BaseNode::vel_callback, this);

    ros::spin();

    ros::waitForShutdown();
}

void BaseNode::model_name_callback(const std_msgs::String::ConstPtr &msg)
{
    if (msg->data.find_first_of("WTRobotSimu") == 0) model_name = msg->data;
}

void BaseNode::vel_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
    double vForward = msg->linear.x * maxLinearVel;
    double vHorizontal = msg->linear.y * maxLinearVel;
    double vRotation = msg->angular.z * maxAngularVel;
    webots_ros::set_float vel_srv;
    vel_srv.request.value = (-1.414 * vForward - 1.414 * vHorizontal + 0.15554 * vRotation) / 0.055;
    frontLeftMotorVelClient.call(vel_srv);
    vel_srv.request.value = (1.414 * vForward - 1.414 * vHorizontal + 0.15554 * vRotation) / 0.055;
    frontRightMotorVelClient.call(vel_srv);
    vel_srv.request.value = (-1.414 * vForward + 1.414 * vHorizontal + 0.15554 * vRotation) / 0.055;
    backLeftMotorVelClient.call(vel_srv);
    vel_srv.request.value = (1.414 * vForward + 1.414 * vHorizontal + 0.15554 * vRotation) / 0.055;
    backRightMotorVelClient.call(vel_srv);
}

#endif //WEBOTS_WS_BASE_NODE_HPP