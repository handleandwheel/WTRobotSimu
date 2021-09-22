# WTRobotSimu

>[Introduction](#introduction)

>[Installation](#installation)

>>[Webots](#webots)

>>[webots_ros](#webots_ros)

>>[OpenCV](#opencv)

>>[WtrobotSimu](#wtrobotsimu)

>[Tutorial](#tutorial)

>>[Basic use](#basicuse)

>>[Parameters](#parameters)

>>[proto](#proto)

## Introduction

This repository contains a robot PROTO named WTRobotSimu for webots and the corresponding ros package. The WTRobotSimu robot equipped with 4 omni wheels, a lidar, an imu and a RGB camera. 

## Installation

### Webots

Webots is the simulation software we use. The easiest way to install webots is using apt

```shell
sudo apt install webots
```

But this way may cause bugs (so far), so it's recommended to install the latest release in https://github.com/cyberbotics/webots/releases. You should download the right package for you system. Let's assume you have downloaded the package to `/home/username/webots/`, you should do as follows.

```shell
cd ~/webots
tar xjf webots-*-.tar.bz2
```

Open `~/.bashrc`, and add the following line at the bottom of the file then save (remember to change to your own path)

```shell
export WEBOTS_HOME=/home/username/webots
```

And execute the following commands.

```shell
sudo apt update
sudo apt install ffmeg libavcodec-extra
sudo apt install ubuntu-restricted-extras
```

Additionally, if you are using Anaconda, execute the following command.

```shell
conda install x264 ffmpeg -c conda-forge
```

Finally you can run `webots` by clicking the file in you install directory. If you want to run `webots` by just run `webots` command in the terminal, you can add it to your environment path in `./bashrc` by adding the first following line to the bottom line of this file. You can run the second line or reboot to make the change take effect.

```shell
% add this to your file
alias webots="${WEBOTS_HOME}/webots"
% run this
source ~/.bashrc
```

If you have any problem see at https://www.cyberbotics.com/doc/guide/installation-procedure#installing-the-tarball-package.

### webots_ros

`webots_ros` is a default ros controller of webots.  If you are using ros-noetic, install the package this way:

```shell
sudo apt install ros-noetic-webots-ros
```

### OpenCV

OpenCV is used in `camera_node`. 

```shell
sudo apt install libopencv-dev
```

### WTRobotSimu

Create you own workspace:

```shell
mkdir webots_ws/src
cd webots_ws
```

and clone the `webots_communication_pkg` folder in this repository into `src`, then execute the following command in `webots_ws`

```shell
catkin_make
```

## Tutorial

### Basic use

`cd` into your workspace and run these commands in your ternimal.

```shell
source ./devel/setup.bash
roslaunch webots_communication webots_communication_launch.launch
```

Hopefully, there should be no errors, and the topics of the robot are as follows.

| Topic                  | Type                     | Explanation                      |
| ---------------------- | ------------------------ | -------------------------------- |
| `/WTRobotSimu/cmd_vel` | `geometry_msgs::Twist`   | Subscribe the velocity messege.  |
| `/WTRobotSimu/img`     | `sensor_msgs::Image`     | Publish the image of the camera. |
| `/WTRobotSimu/imu`     | `sensor_msgs::Imu`       | Publish the imu messeage.        |
| `/WTRobotSimu/lidar`   | `sensor_msgs::LaserScan` | Publish the lidar messege.       |

### Parameters

There are some parameters in the `param` directory, where you can adjust them. And there's no need to recompile after adjusting the parameters.

### Proto

There is a proto file named `WTRobotSimu.proto`is the `protos`directory, it's the same robot used in `navi_world.wbt`. You can load this proto into your own world.
