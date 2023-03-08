#!/bin/bash
# Script setup for enabling/disabling REAL motors while in HITL simulation

motors_on="armed.lockdown = true;"
motors_off="//armed.lockdown = true;"
state=''
count=1
prev_line=''

# read state_machine_helper.cpp
cd ~/PX4-Autopilot/src/modules/commander
filename='state_machine_helper.cpp'

# checking current motor state
while read line; do 

if [ "$prev_line" = '/* enforce lockdown in HIL */' ]; then 
    case $line in 
        "$motors_on") state='enabled'
        No=$count
        break
        ;;
        "$motors_off") state='disabled'
        No=$count
        break
        ;;
    esac
fi

prev_line=$line
count=$((count+1))
# echo $count
done < $filename 

echo $count
echo $No
read -n1 -p "Motors are currently $state. Do you want to change its state? (y/n): "  answ
echo

# changing motor state or exiting script 
case $answ in 
    "y") echo "Changing state"
        if [ "$state" = enabled ] ; then
        sed -i ' '$No' c\'"$motors_off"'' $filename
        echo Changed to Disabled
        else
        sed -i ' '$No' c\'"			$motors_on"'' $filename
        echo Changed to Enabled
        fi
    ;;
    "n") echo "Exiting script"
        exit 2
    ;;
    *) echo "Incorrect argument. Closing script"
        exit 1
    ;;
esac
prev_line=""

while read line; do 
    if [ "$prev_line" = '/* enforce lockdown in HIL */' ]; then 
        case $line in 
        "$motors_on") state='enabled'
        echo motors on
        break
        ;;
        "$motors_off") state='disabled'
        echo motors off
        break
        ;;
        esac
    fi  
prev_line=$line
done < $filename 

exit 0
