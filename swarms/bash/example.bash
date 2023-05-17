#!/bin/bash

# sudo apt install hostapd dnsmasq
# sudo apt install wavemon

WIFI_INTERFACE="<WIFI_CARD_NAME>"
HOTSPOT_SSID="MyHotspot"
HOTSPOT_PASS="12345678"
STATUS_FILE="wifi_status.txt"
PRIORITY_LIST_FILE="wifi_priority_list.txt"

function check_wifi() {
    # Get a list of WiFi networks in the priority list
    IFS=$'\n' read -d '' -r -a priority_list < "$PRIORITY_LIST_FILE"
    # Get a list of available WiFi networks
    available_networks=$(nmcli -f SSID,BSSID,ACTIVE,BARS,SECURITY dev wifi | sort -k4nr)
    # Check if the computer is connected to a WiFi network in the priority list
    for network in "${priority_list[@]}"
    do
        if echo "$available_networks" | grep -q "$network:.*yes"; then
            return 0
        fi
    done
    return 1
}

function check_hotspot() {
    nmcli -t -f NAME,DEVICE connection | grep "$HOTSPOT_SSID:$WIFI_INTERFACE" > /dev/null 2>&1
}

function create_hotspot() {
    if ! nmcli connection show | grep -q "$HOTSPOT_SSID"; then
        nmcli con add type wifi ifname "$WIFI_INTERFACE" con-name "$HOTSPOT_SSID" autoconnect no ssid "$HOTSPOT_SSID"
        nmcli con modify "$HOTSPOT_SSID" wifi-sec.key-mgmt wpa-psk
        nmcli con modify "$HOTSPOT_SSID" wifi-sec.psk "$HOTSPOT_PASS"
    fi
    nmcli connection up "$HOTSPOT_SSID"
}

function stop_hotspot() {
    if check_hotspot; then
        nmcli connection down "$HOTSPOT_SSID"
    fi
}

# Create hotspot connection if it doesn't exist
create_hotspot

while true; do
    if check_wifi; then
        echo "Wi-Fi connected."
        stop_hotspot
        echo "False" > "$STATUS_FILE"
    else
        echo "Wi-Fi not connected. Creating hotspot..."
        create_hotspot
        echo "True" > "$STATUS_FILE"
    fi
    sleep 60
done