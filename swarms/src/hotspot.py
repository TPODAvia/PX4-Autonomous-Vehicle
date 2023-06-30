import subprocess

network_name = "netplan-wlan0-Raspberry"
mode = "up"

command = "/home/vboxuser/catkin_ws/src/swarms/bash/network_control.sh {} {}".format(network_name, mode)
subprocess.run(command, shell=True)