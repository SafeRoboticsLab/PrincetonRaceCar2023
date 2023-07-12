#!/bin/bash

CUR_DIR=$(pwd)

git submodule update --init --recursive

# SLAM message
cd $CUR_DIR/ROS_Core/py_ws/src/Modules/AprilTagSLAM_ROS
git checkout msg_only
git pull

# Interface
cd $CUR_DIR/ROS_Core/py_ws/src/Modules/Interface
git checkout main
git pull

# # Planning
# cd $CUR_DIR/ROS_Core/src/Utility/Labs/Lab1
# git checkout main
# git pull


# Message
cd $CUR_DIR/ROS_Core/py_ws/src/Modules/ROS_msgs
git checkout main
git pull

# Routing
cd $CUR_DIR/ROS_Core/py_ws/src/Modules/Routing
git checkout pylanelet
git pull

cd $CUR_DIR