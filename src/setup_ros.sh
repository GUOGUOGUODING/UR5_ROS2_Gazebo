#!/bin/bash
set -e 

echo "Start"

# 更新系统索引
sudo apt update

# ROS2 控制框架（ros2_control + 通用控制器）
sudo apt install -y ros-humble-ros2-control \
                    ros-humble-ros2-controllers \
                    ros-humble-gripper-controllers

# Gazebo 仿真环境 + ROS2 集成
sudo apt install -y gazebo \
                    ros-humble-gazebo-ros-pkgs \
                    ros-humble-gazebo-ros2-control

# 常用 ROS 工具包（xacro, DDS, sensor_msgs）
sudo apt install -y ros-humble-xacro \
                    ros-humble-rmw-cyclonedds-cpp \
                    ros-humble-sensor-msgs

echo "Finished！"
