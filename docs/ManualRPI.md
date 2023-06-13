# Working with Raspberry Pi

#### How to use

To ensure that the package is connected and working properly, it is recommended to run the launch file first. There are some useful tools available for debugging purposes such as `rqt_graph`, `rostopic echo`, and `rviz`.

Here are the steps to use the package:

1. Launch the mavros node using the following commands:
```bash
roslaunch px4_ground 0main_camera.launch
roslaunch px4_ground 0mavros.launch
```

2. Launch the drone setup using the following command:
```bash
roslaunch px4_ground 1setup.launch
```

3. Launch the control missions using the following commands:
```bash
roslaunch px4_ground 2mission_ManualOffb.launch
roslaunch px4_ground 3mission_QGCaruco.launch
roslaunch px4_ground 4mission_SingleOffb.launch
roslaunch px4_ground 5mission_SingleOffb_Avoid.launch
roslaunch px4_ground 6mission_FollowMe.launch
```

#### PX4 launch command
```bash
roslaunch mavros px4.launch
```
#### PX4 launch file (default fcu_url worked fine for Pi 4 running Ubuntu server)
https://github.com/mavlink/mavros/blob/master/mavros/launch/px4.launch

If using the USB connector on Pi 4 you can use the default fcu_url
```bash
<arg name="fcu_url" default="/dev/ttyACM0:57600" />
```
If using the telemetry 2 port on Pixhawk you need this.
```bash
<arg name="fcu_url" default="/dev/ttyS0:921600" />
```
For using the telemetry 2 port you need the following params as specified at the bottom of this page:
https://docs.px4.io/v1.9.0/en/peripherals/mavlink_peripherals.html

> **Note** If you want to override the fcu_url value in the launch file you can do:
```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyS0:921600
```
#### Broadcast to QGroundControl running on another computer on same network
```bash
<arg name="gcs_url" default="udp://:14556@192.168.43.40:14550" />
```
#### Change ROS_MASTER_URI to connect to ROS master on another computer (on the same network)
```bash
export ROS_MASTER_URI=http://192.168.1.6:11311
```
#### Sample change flight mode
```bash
rosrun mavros mavsys mode -b 80 # stablize disarmed
rosrun mavros mavsys mode -b 64 # manual disarmed
```
#### Get diagnostics
```bash
rostopic echo /diagnostics
```
#### Get status
```bash
rostopic echo /mavros/state
```
#### Get/set airframe type 0=generic micro air vehicle, 2=quadrotor
https://docs.px4.io/v1.9.0/en/advanced_config/parameter_reference.html
```bash
rosrun mavros mavparam get MAV_TYPE
rosrun mavros mavparam set MAV_TYPE 2
```
#### Dump params to a text file
```bash
rosrun mavros mavparam dump params.txt
```
#### Arm
```bash
rosrun mavros mavsafety arm
```
#### Takeoff from current position (work in progress)
```bash
rosrun mavros mavcmd takeoffcur 0 0 0

rosrun mavros mavcmd local_takeoff <yaw> <z>
```
#### Land
```bash
rosrun mavros mavcmd local_land <x> <y> <yaw> <z>
```