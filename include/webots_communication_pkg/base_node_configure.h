// base_node_configure.h
// This file defines the parameters used in base_node
// The default parameters are those of DJI M3508 using C620 speed controller or default in webots
// Environment: ubuntu 20.04, ros noetic, webots 2021b
// Created by Liu, Yuming on Spet. 19th, 2021
//

#ifndef WEBOTS_WS_BASE_NODE_CONFIGURE_H
#define WEBOTS_WS_BASE_NODE_CONFIGURE_H

#include "ros/ros.h"

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
    double frontLeftMotorPID[4];
    double frontRightMotorPID[4];
    double backLeftMotorPID[4];
    double backRightMotorPID[4];
    double frontLeftMotorPos;
    double frontRightMotorPos;
    double backLeftMotorPos;
    double backRightMotorPos;
};

void BaseNodeConfig::set(const ros::NodeHandle &nh)
{
    nh.param("/base_node/frontLeftMotor/torque", frontLeftMotorTorque, 3.0);
    nh.param("/base_node/frontRightMotor/torque", frontRightMotorTorque, 3.0);
    nh.param("/base_node/backLeftMotor/torque", backLeftMotorTorque, 3.0);
    nh.param("/base_node/backRightMotor/torque", backRightMotorTorque, 3.0);
    nh.param("/base_node/frontLeftMotor/pid/KP", frontLeftMotorPID[0], 10.0);
	nh.param("/base_node/frontLeftMotor/pid/KI", frontLeftMotorPID[1], 0.0);
	nh.param("/base_node/frontLeftMotor/pid/KD", frontLeftMotorPID[2], 0.0);
    nh.param("/base_node/frontRightMotor/pid/KP", frontRightMotorPID[0], 10.0);
	nh.param("/base_node/frontRightMotor/pid/KI", frontRightMotorPID[1], 0.0);
	nh.param("/base_node/frontRightMotor/pid/KD", frontRightMotorPID[2], 0.0);
    nh.param("/base_node/backLeftMotor/pid/KP", backLeftMotorPID[0], 10.0);
	nh.param("/base_node/backLeftMotor/pid/KI", backLeftMotorPID[1], 0.0);
	nh.param("/base_node/backLeftMotor/pid/KD", backLeftMotorPID[2], 0.0);
    nh.param("/base_node/backRightMotor/pid/KP", backRightMotorPID[0], 10.0);
	nh.param("/base_node/backRightMotor/pid/KI", backRightMotorPID[1], 0.0);
	nh.param("/base_node/backRightMotor/pid/KD", backRightMotorPID[2], 0.0);
    //nh.param("/base_node/frontLeftMotor/pos", frontLeftMotorPos, (double)INFINITY);
    //nh.param("/base_node/frontRightMotor/pos", frontRightMotorPos, (double)INFINITY);
    //nh.param("/base_node/backLeftMotor/pos", backLeftMotorPos, (double)INFINITY);
    //nh.param("/base_node/backRightMotor/pos", backRightMotorPos, (double)INFINITY);
	nh.param("/base_node/backLeftMotor/pid/KP", backLeftMotorPID[1], 0.0);
	nh.param("/base_node/backLeftMotor/pid/KP", backLeftMotorPID[2], 0.0);
    nh.param("/base_node/backRightMotor/pid/KP", backRightMotorPID[0], 10.0);
	nh.param("/base_node/backRightMotor/pid/KP", backRightMotorPID[1], 0.0);
	nh.param("/base_node/backRightMotor/pid/KP", backRightMotorPID[2], 0.0);
    nh.param("/base_node/frontLeftMotor/pos", frontLeftMotorPos, (double)INFINITY);
    nh.param("/base_node/frontRightMotor/pos", frontRightMotorPos, (double)INFINITY);
    nh.param("/base_node/backLeftMotor/pos", backLeftMotorPos, (double)INFINITY);
    nh.param("/base_node/backRightMotor/pos", backRightMotorPos, (double)INFINITY);
}

#endif //WEBOTS_WS_BASE_NODE_CONFIGURE_H
