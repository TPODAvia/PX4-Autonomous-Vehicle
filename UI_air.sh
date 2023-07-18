#!/bin/bash

action=$(zenity --list --text="Select bash script to run" --radiolist --column "Pick" --column "Action" TRUE "Script 1" FALSE "Script 2" FALSE "Script 3")

if [ -n "${action}" ];then
    case "${action}" in
    "Script 1")
        # Call your bash script 1 here
        echo "Running script 1"
        bash /home/vboxuser/catkin_ws/src/bash/missions_air/delivery.sh
        ;;
    "Script 2")
        # Call your bash script 2 here
        echo "Running script 2"
        bash /home/vboxuser/catkin_ws/src/bash/missions_air/delivery.sh
        ;;
    "Script 3")
        # Call your bash script 3 here
        echo "Running script 3"
        bash /home/vboxuser/catkin_ws/src/bash/missions_air/delivery.sh
        ;;
    esac
else
    echo "No action selected"
fi
