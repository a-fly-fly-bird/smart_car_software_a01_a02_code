#include <memory>
#include <iostream>

#include "ugv_sdk/mobile_robot/scout_robot.hpp"
#include "ugv_sdk/utilities/protocol_detector.hpp"

using namespace westonrobot;

int main(int argc, char **argv)
{
    std::unique_ptr<ScoutRobot> scout;

    std::string device_name;

    ProtocolDectctor detector;
    if (detector.Connect(device_name))
    {
        auto proto = detector.DetectProtocolVersion(5);
        if (proto == ProtocolVersion::AGX_V1)
        {
            std::cout << "Detected protocol: AGX_V1" << std::endl;
            scout = std::unique_ptr<ScoutRobot>(
                new ScoutRobot(ProtocolVersion::AGX_V1, is_scout_mini));
        }
        else if (proto == ProtocolVersion::AGX_V2)
        {
            std::cout << "Detected protocol: AGX_V2" << std::endl;
            scout = std::unique_ptr<ScoutRobot>(
                new ScoutRobot(ProtocolVersion::AGX_V2, is_scout_mini));
        }
        else
        {
            std::cout << "Detected protocol: UNKONWN" << std::endl;
            return -1;
        }
    }
    else
    {
        return -1;
    }

    if (scout == nullptr)
        std::cout << "Failed to create robot object" << std::endl;

    // bind car
    scout->Connect(device_name);

    if (scout->GetParserProtocolVersion() == ProtocolVersion::AGX_V2)
    {
        scout->EnableCommandedMode();
    }
    // 前后车灯 mode 是统一的
    // light control
    std::cout << "Light: const off" << std::endl;
    scout->SetLightCommand(CONST_OFF, 0, CONST_OFF, 0);
    sleep(3);
    std::cout << "Light: const on" << std::endl;
    scout->SetLightCommand(CONST_ON, 0, CONST_ON, 0);
    sleep(3);
    std::cout << "Light: breath" << std::endl;
    scout->SetLightCommand(BREATH, 0, BREATH, 0);
    sleep(3);
    std::cout << "Light: custom 30-40" << std::endl;
    scout->SetLightCommand(CUSTOM, 30, CUSTOM, 40);
    sleep(3);
    scout->SetLightCommand(CONST_OFF, 0, CONST_OFF, 0);
}