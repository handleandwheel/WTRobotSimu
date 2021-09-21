// base_node_configure.h
// This file defines the parameters used in base_node
// The default parameters are those of DJI M3508 using C620 speed controller or default in webots
// Environment: ubuntu 20.04, ros noetic, webots 2021b
// Created by Liu, Yuming on Spet. 19th, 2021
//

#ifndef WEBOTS_WS_BASE_NODE_CONFIGURE_H
#define WEBOTS_WS_BASE_NODE_CONFIGURE_H

#include "ros/ros.h"
#include "cmath"

using namespace std;

class BaseNodeConfig
{
public:
    BaseNodeConfig(){}
    void set(const ros::NodeHandle &);
    double frontLeftMotorTorque;
    double frontRightMotorTorque;
    double backLeftMotorTorque;
    double backRightMotorTorque;
    std::vector<double> frontLeftMotorPID = {10.0, 0.0, 0.0};
    std::vector<double> frontRightMotorPID = {10.0, 0.0, 0.0};
    std::vector<double> backLeftMotorPID = {10.0, 0.0, 0.0};
    std::vector<double> backRightMotorPID = {10.0, 0.0, 0.0};
    double frontLeftMotorPos;
    double frontRightMotorPos;
    double backLeftMotorPos;
    double backRightMotorPos;
};

void BaseNodeConfig::set(const ros::NodeHandle &nh)
{
    nh.param("base_node/frontLeftMotor/torque", frontLeftMotorTorque, 3.0);
    nh.param("base_node/frontRightMotor/torque", frontRightMotorTorque, 3.0);
    nh.param("base_node/backLeftMotor/torque", backLeftMotorTorque, 3.0);
    nh.param("base_node/backRightMotor/torque", backRightMotorTorque, 3.0);
    nh.param("base_node/frontLeftMotor/pid", frontLeftMotorPID, frontLeftMotorPID);
    nh.param("base_node/frontRightMotor/pid", frontRightMotorPID, frontRightMotorPID);
    nh.param("base_node/backLeftMotor/pid", backLeftMotorPID, backLeftMotorPID);
    nh.param("base_node/backRightMotor/pid", backRightMotorPID, backRightMotorPID);
    nh.param("base_node/frontLeftMotor/pos", frontLeftMotorPos, (double)INFINITY);
    nh.param("base_node/frontRightMotor/pos", frontRightMotorPos, (double)INFINITY);
    nh.param("base_node/backLeftMotor/pos", backLeftMotorPos, (double)INFINITY);
    nh.param("base_node/backRightMotor/pos", backRightMotorPos, (double)INFINITY);
}

#endif //WEBOTS_WS_BASE_NODE_CONFIGURE_H
