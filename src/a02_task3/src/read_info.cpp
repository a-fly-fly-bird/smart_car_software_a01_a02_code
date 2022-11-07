#include <memory>
#include <iostream>

#include "ugv_sdk/mobile_robot/scout_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

using namespace westonrobot;

std::unique_ptr<ScoutRobot> robot;

int main(int argc, char **argv)
{
  ProtocolDectctor detector;
  try
  {
    detector.Connect("can0");
    auto proto = detector.DetectProtocolVersion(5);
    if (proto == ProtocolVersion::AGX_V1)
    {
      std::cout << "Detected protocol: AGX_V1" << std::endl;
      robot = std::unique_ptr<ScoutRobot>(
          new ScoutRobot(ProtocolVersion::AGX_V1, is_scout_mini));
    }
    else if (proto == ProtocolVersion::AGX_V2)
    {
      std::cout << "Detected protocol: AGX_V2" << std::endl;
      robot = std::unique_ptr<ScoutRobot>(
          new ScoutRobot(ProtocolVersion::AGX_V2, is_scout_mini));
    }
    else
    {
      std::cout << "Detected protocol: UNKONWN" << std::endl;
      return -1;
    }
    if (robot == nullptr)
      std::cout << "Failed to create robot object" << std::endl;
  }
  catch (const std::exception error)
  {
    ROS_ERROR("please bringup up can or make sure can port exist");
    ros::shutdown();
  }
  // 获取小车的状态
  ScoutCoreState scout_core_tate = robot->GetRobotState();
  ScoutActuatorState actuator_state = robot->GetActuatorState();
  std::cout << "前灯状态:" << scout_core_tate.light_state.front_light << std::endl;
  std::cout << "线速度:" << scout_core_tate.motion_state.linear_velocity << std::endl;
  std::cout << "角速度:" << scout_core_tate.motion_state.angular_velocity << std::endl;
  std::cout << "电池百分比:" << scout_core_tate.system_state.battery_voltage << std::endl;
  std::cout << "电机rpm:" << actuator_state.actuator_hs_state->rpm << std::endl;
  return 0;
}
