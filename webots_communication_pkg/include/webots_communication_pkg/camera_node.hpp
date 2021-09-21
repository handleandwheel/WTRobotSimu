// camera_node.hpp
// This file defines class CameraNode, which is used to communicate with the camera of WTRobotSimu PROTO of webtos
// Environment: ubuntu 20.04, ros noetic, webots 2021b, webots-ros 4.1.0, opencv 4.2.0, cv_bridge 1.15.0
// Created by Liu, Yuming on Spet. 20th, 2021.
//

#ifndef WEBOTS_WS_CAMERA_NODE_HPP
#define WEBOTS_WS_CAMERA_NODE_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "webots_ros/get_float.h"
#include "webots_ros/set_float.h"
#include "webots_ros/set_int.h"
#include "webots_ros/camera_get_info.h"
#include "opencv2/opencv.hpp"
#include "opencv2/opencv_modules.hpp"
#include "cv_bridge/cv_bridge.h"
#include "webots_communication_pkg/camera_node_configure.h"

using namespace std;

class CameraNode
{
public:
    CameraNode(ros::NodeHandle &);
private:
    void model_name_callback(const std_msgs::String::ConstPtr &);
    void img_callback(const sensor_msgs::Image::ConstPtr &);
    ros::ServiceClient robotBasicTimeStepClient;
    ros::ServiceClient cameraEnableClient;
    ros::ServiceClient cameraExposureClient;
    ros::ServiceClient cameraFocalDistanceClient;
    ros::ServiceClient cameraFovClient;
    ros::ServiceClient cameraInfo;
    ros::Subscriber modelNameSub;
    ros::Subscriber imgSub;
    ros::Publisher imgPub;
    CameraNodeConfig cameraNodeConfig;
    double basic_time_step = 0.0;
    string model_name = "empty";
};

CameraNode::CameraNode(ros::NodeHandle &nh)
{
    cameraNodeConfig.set(nh);

    modelNameSub = nh.subscribe<std_msgs::String>("/model_name", 1, &CameraNode::model_name_callback, this);

    //model name varies every time simulation runs
    while(model_name == "empty") ros::spinOnce();
    ROS_INFO("[CAMERA_NODE_INFO] Model name loaded.");

    // enable
    robotBasicTimeStepClient = nh.serviceClient<webots_ros::get_float>(model_name + "/robot/get_basic_time_step");
    cameraEnableClient = nh.serviceClient<webots_ros::set_int>(model_name + "/camera/enable");

    // Compare the basic time step and the sampling period, since the sampling period must be divisible by basic time step.
    while(abs(basic_time_step-0.0)<1e-3)
    {
        webots_ros::get_float time_step_srv;
        time_step_srv.request.ask = true;
        robotBasicTimeStepClient.call(time_step_srv);
        basic_time_step = time_step_srv.response.value;
    }

    webots_ros::set_int sampling_period_srv;
    if(cameraNodeConfig.samplingPeriod % int(basic_time_step) == 0)
    {
        sampling_period_srv.request.value = cameraNodeConfig.samplingPeriod;
        cameraEnableClient.call(sampling_period_srv);
    }
    else if(basic_time_step <= 1)
    {
        sampling_period_srv.request.value = 1;
        cameraEnableClient.call(sampling_period_srv);
        ROS_INFO("[CAMERA_NODE_INFO] Invalid sampling time. Sampling time is set to 1ms");
    }
    else
    {
        sampling_period_srv.request.value = int(basic_time_step);
        cameraEnableClient.call(sampling_period_srv);
        ROS_INFO("[CAMERA_NODE_INFO] Invalid sampling time. Sampling time is set to robot basic time step:%d.", int(basic_time_step));
    }

    // exposure
    cameraExposureClient = nh.serviceClient<webots_ros::set_float>(model_name + "/camera/set_exposure");
    webots_ros::set_float exposure_srv;
    exposure_srv.request.value = cameraNodeConfig.exposure;
    cameraExposureClient.call(exposure_srv);

    // focal distance
    cameraFocalDistanceClient = nh.serviceClient<webots_ros::set_float>(model_name + "/camera/set_focal_distance");
    webots_ros::set_float focal_dis_srv;
    focal_dis_srv.request.value = cameraNodeConfig.focalDistance;
    cameraFocalDistanceClient.call(focal_dis_srv);

    // fov
    cameraFovClient = nh.serviceClient<webots_ros::set_float>(model_name + "/camera/set_fov");
    webots_ros::set_float fov_srv;
    fov_srv.request.value = cameraNodeConfig.fov;
    cameraFovClient.call(fov_srv);

    cameraInfo = nh.serviceClient<webots_ros::camera_get_info>(model_name + "/camera/get_info");
    imgSub = nh.subscribe<sensor_msgs::Image>(model_name + "/camera/image", 1, &CameraNode::img_callback, this);
    imgPub = nh.advertise<sensor_msgs::Image>("WTRobotSimu/img", 1);

    ros::spin();
    ros::waitForShutdown();

}

void CameraNode::model_name_callback(const  std_msgs::String::ConstPtr &msg)
{
    if (msg->data.find_first_of("WTRobotSimu") == 0 && model_name == "empty") model_name = msg->data;
}

void CameraNode::img_callback(const sensor_msgs::Image::ConstPtr &msg)
{
    // make a mask and get roi of the img to prevent distortion due to the mismatch of the input and output height and width.
    webots_ros::camera_get_info info_srv;
    info_srv.request.ask = 1;
    cameraInfo.call(info_srv);
    int rec_width, rec_height;
    if(info_srv.response.width/cameraNodeConfig.resolution[0] > info_srv.response.height/cameraNodeConfig.resolution[1])
    {
        rec_height = info_srv.response.height;
        rec_width = int(cameraNodeConfig.resolution[0] * info_srv.response.width / cameraNodeConfig.resolution[1]);
    }
    else
    {
        rec_width = info_srv.response.width;
        rec_height = int(cameraNodeConfig.resolution[1] * info_srv.response.width / cameraNodeConfig.resolution[0]);
    }
    int up_left_x, up_left_y;
    up_left_x = int((info_srv.response.width - rec_width)/2);
    up_left_y = int((info_srv.response.height - rec_height)/2);
    cv::Rect rect(up_left_x, up_left_y, rec_width, rec_height);
    cv_bridge::CvImagePtr cv_img_ptr;
    cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv_img_ptr->image = cv_img_ptr->image(rect);
    cv::resize(cv_img_ptr->image, cv_img_ptr->image, cv::Size(cameraNodeConfig.resolution[0], cameraNodeConfig.resolution[1]), cv::INTER_LINEAR);
    imgPub.publish(cv_img_ptr->toImageMsg());
}

#endif //WEBOTS_WS_CAMERA_NODE_HPP