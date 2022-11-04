#!/bin/bash
roslaunch imu_launch imu_msg.launch
roslaunch scout_bringup open_rslidar.launch
roslaunch realsense2_camera rs_camera.launch
