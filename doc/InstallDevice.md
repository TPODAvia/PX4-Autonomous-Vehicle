
# Working with Raspberry Pi

1) install the image ubuntu 20.04 server for raspberry
https://cdimage.ubuntu.com/releases/focal/release/

or install my image for raspberry pi 4

2) To connecto to local wifi network run:

sudo nano /etc/netplan/50-cloud-init.yaml

add this line to the file:

wifis:
  wlan0:
    optional: true
    access-points:
        "your-ssid":
            password: "pass"
    dhcp4: true

3) We should check the sintax for the errors

sudo netplan -debug generate

4) We need to mofify terminal UI for the colorful visualization:

 nano ~/.bashrc
  ucomment line force_color_prompt=yes
press "ctrl + x", press "y", press "enter", write in terminal: exit

5) Encrease swap file:
https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-20-04

5) git clone this repo

6) remove unnessary files

7) rm -d -r px4_sim



#### PX4 launch command
roslaunch mavros px4.launch

#### PX4 launch file (default fcu_url worked fine for Pi 4 running Ubuntu server)
https://github.com/mavlink/mavros/blob/master/mavros/launch/px4.launch

If using the mini USB connector on Pi 4 you can use the default fcu_url
<arg name="fcu_url" default="/dev/ttyACM0:57600" />

If using the telemetry 2 port on Pixhawk you need this.
<arg name="fcu_url" default="/dev/ttyS0:921600" />

For using the telemetry 2 port you need the following params as specified at the bottom of this page:
https://docs.px4.io/v1.9.0/en/peripherals/mavlink_peripherals.html

If you want to override the fcu_url value in the launch file you can do:
roslaunch mavros px4.launch fcu_url:=/dev/ttyS0:921600

#### Broadcast to QGroundControl running on another computer on same network
<arg name="gcs_url" default="udp://:14556@192.168.43.40:14550" />

#### Change ROS_MASTER_URI to connect to ROS master on another computer (on the same network)
export ROS_MASTER_URI=http://192.168.86.38:11311

#### Sample change flight mode
rosrun mavros mavsys mode -b 80 # stablize disarmed
rosrun mavros mavsys mode -b 64 # manual disarmed

#### Get diagnostics
rostopic echo /diagnostics

#### Get status
rostopic echo /mavros/state

#### Get/set airframe type 0=generic micro air vehicle, 2=quadrotor
https://docs.px4.io/v1.9.0/en/advanced_config/parameter_reference.html
rosrun mavros mavparam get MAV_TYPE
rosrun mavros mavparam set MAV_TYPE 2

#### Dump params to a text file
rosrun mavros mavparam dump params.txt

#### Arm
rosrun mavros mavsafety arm

#### Takeoff from current position (work in progress)
rosrun mavros mavcmd takeoffcur 0 0 0
rosrun mavros mavcmd local_takeoff <yaw> <z>

#### Land
rosrun mavros mavcmd local_land <x> <y> <yaw> <z>