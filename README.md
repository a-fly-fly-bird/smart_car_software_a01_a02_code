# genetic_recombination
## 基本信息
#### 介绍
基因重组
A01 & A02 智能小车代码仓库

#### Team Member
* E2019102
* E2019093
* E2019163
* E2019084

#### 目录结构
* code: 非catkin的源文件
* src: catkin的源文件

## 一些命令
```shell
# 开启 imu
roslaunch imu_launch imu_msg.launch

# 开启 LiDar
roslaunch scout_bringup open_rslidar.launch

# 开启深度相机
roslaunch realsense2_camera rs_camera.launch

# enable can
rosrun scout_bringup setup_can2usb.bash

# 指定编译功能包
catkin_make -DCATKIN_WHITELIST_PACKAGES="package1 package2"

```

## 松灵源码网站
* https://github.com/agilexrobotics
