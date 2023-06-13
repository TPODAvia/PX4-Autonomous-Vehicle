

   #!/bin/bash
   
   # Enable WiFi hotspot
   nmcli device wifi hotspot ifname <DEVICE_NAME> ssid <SSID> password <PASSWORD>

   #!/bin/bash
   
   # Disable WiFi hotspot
   nmcli connection delete <SSID>

   $ chmod +x wifi-hotspot.sh
   $ ./wifi-hotspot.sh enable
   $ ./wifi-hotspot.sh disable


   #!/bin/bash
   
   # Check if the correct number of arguments is passed
   if [ "$#" -ne 1 ]; then
       echo "Usage: \$0 enable|disable"
       exit 1
   fi
   
   # Enable or disable WiFi hotspot
   if [ "\$1" = "enable" ]; then
       nmcli device wifi hotspot ifname <DEVICE_NAME> ssid <SSID> password <PASSWORD>
   elif [ "\$1" = "disable" ]; then
       nmcli connection delete <SSID>
   else
       echo "Usage: \$0 enable|disable"
       exit 1
   fi
