#include <iostream>
#include <string>
#include <cstdlib>

int main()
{
    std::string network_name = "netplan-wlan0-Raspberry";
    std::string mode = "up";

    std::string command = "/home/vboxuser/catkin_ws/src/swarms/bash/network_control.sh " + network_name + " " + mode;
    system(command.c_str());

    return 0;
}