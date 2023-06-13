# swarms

![iq](docs/imgs/iq.JPG)

This package hosts a collection of software designed to help drone developers make their application come to life. The code in this repo is complimented by the [iq_tutorials](https://github.com/Intelligent-Quads/iq_tutorials) repo, which explains how to set up your dev enviorment as well as teaches basic drone programming fundamentals. 

## Example Code (C++)

### avoidance_sol.cpp
Example obstacle avoidance program utilizing the potential field method.

To create a bash script that enables or disables the WiFi hotspot on Ubuntu, follow these steps:

    Open a terminal window and create a new file using your favorite text editor. For example, you can use nano:

   $ nano wifi-hotspot.sh

    Add the following code to the file to create a script that enables the WiFi hotspot:

   #!/bin/bash
   
   # Enable WiFi hotspot
   nmcli device wifi hotspot ifname <DEVICE_NAME> ssid <SSID> password <PASSWORD>

Replace <DEVICE_NAME> with your WiFi device name, <SSID> with the name of your WiFi hotspot, and <PASSWORD> with the password for your WiFi hotspot. Save the file.

    Add the following code to the file to create a script that disables the WiFi hotspot:

   #!/bin/bash
   
   # Disable WiFi hotspot
   nmcli connection delete <SSID>

Replace <SSID> with the name of your WiFi hotspot. Save the file.

    Make the script executable:

   $ chmod +x wifi-hotspot.sh

    To enable the WiFi hotspot, run the script as follows:

   $ ./wifi-hotspot.sh enable

    To disable the WiFi hotspot, run the script as follows:

   $ ./wifi-hotspot.sh disable

    You can also add some error checking to the script to make sure that the correct arguments are passed. For example:

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

This code checks if the correct number of arguments is passed and if the argument is either "enable" or "disable". If not, it displays the usage message and exits with an error code.

## Related Repos

[iq_tutorials](https://github.com/Intelligent-Quads/iq_tutorials) - Repo hosting associated tutorials for swarms

[iq_sim](https://github.com/Intelligent-Quads/iq_sim) - Repo hosing the simulation wolds designed to help develop drone gnc missions



