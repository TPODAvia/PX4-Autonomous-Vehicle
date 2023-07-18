#!/bin/bash

network_name="\$1"
mode="\$2"

if [ "$mode" == "up" ]; then
    sudo nmcli con up "$network_name"
elif [ "$mode" == "down" ]; then
    sudo nmcli con down "$network_name"
else
    echo "Invalid mode. Use 'up' or 'down'."
fi