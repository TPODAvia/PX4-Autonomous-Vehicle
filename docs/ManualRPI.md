# Working with Raspberry Pi

#### Setup Autopilot

QGroundControl offers a multitude of parameters for customization. Below are the fundamental steps required for setting it up:

```bash
###############################################
# Attitude & rate gains
###############################################
# Roll
param set-default MC_ROLL_P 6.5
param set-default MC_ROLLRATE_P 0.0630
param set-default MC_ROLLRATE_I 0.0840
param set-default MC_ROLLRATE_D 0.0014
param set-default MC_ROLLRATE_MAX 220

# Pitch
param set-default MC_PITCH_P 6.5
param set-default MC_PITCHRATE_P 0.0660
param set-default MC_PITCHRATE_I 0.0860
param set-default MC_PITCHRATE_D 0.0014
param set-default MC_PITCHRATE_MAX 220

# Yaw
param set-default MC_YAW_P 6.5
param set-default MC_YAWRATE_P 0.07
param set-default MC_YAWRATE_I 0.06
param set-default MC_YAWRATE_D 0
param set-default MC_YAWRATE_FF 0
param set-default MC_YAWRATE_MAX 200

###############################################
# Multirotor Position Gains
###############################################
param set-default MPC_ACC_HOR 8
param set-default MPC_THR_MIN 0.06
param set-default MPC_THR_HOVER 0.3
# altitude control gains
param set-default MPC_Z_P 1
param set-default MPC_Z_VEL_P_ACC 4
param set-default MPC_Z_VEL_I_ACC 0.4
# position control gains
param set-default MPC_XY_P 0.9500
param set-default MPC_XY_VEL_P_ACC 1.8
param set-default MPC_XY_VEL_I_ACC 0.4
param set-default MPC_XY_VEL_D_ACC 0.2
# etc gains
param set-default MPC_TKO_RAMP_T 0.0
param set-default MPC_TKO_SPEED 1.5
param set-default MPC_VEL_MANUAL 5


#####################################
# EKF
#####################################
# fusion: flow + vis pos + vis yaw
param set-default EKF2_AID_MASK 26
param set-default EKF2_OF_DELAY 0
param set-default EKF2_OF_QMIN 10
param set-default EKF2_OF_N_MIN 0.05
param set-default EKF2_OF_N_MAX 0.2

param set-default EKF2_OF_POS_X 0.12
param set-default EKF2_OF_POS_Y 0.0
param set-default EKF2_OF_POS_Z -0.09

# height est EKF2_HGT_MODE: 0 = baro, 1 = gps, 2 = range, 3 = vision
param set-default EKF2_HGT_MODE 3

# maximum fusion in (m) for the distance sensor (Aruco/April tag)
param set-default EKF2_RNG_A_HMAX 4
param set-default EKF2_EVA_NOISE 0.1
param set-default EKF2_EVP_NOISE 0.1
param set-default EKF2_EV_DELAY 0

# ALT Mode: 0 alt following, 1 terrain following, 2 terrain hold
param set-default MPC_ALT_MODE 2

# # Optical Flow parameters
param set-default SENS_FLOW_ROT 0
param set-default SENS_FLOW_MINHGT 0.0
param set-default SENS_FLOW_MAXHGT 4.0
param set-default SENS_FLOW_MAXR 10.0

# Filter settings
param set-default IMU_DGYRO_CUTOFF 90
param set-default IMU_GYRO_CUTOFF 100

# Minimum take off altitude
param set-default MIS_TAKEOFF_ALT 1

# Time out of auto disarm
param set-default COM_DISARM_LAND 1.0

# enable offboard flights without rc
param set-default COM_RCL_EXCEPT 4

# LPE: Flow-only mode
param set-default LPE_FUSION 242
param set-default LPE_FAKE_ORIGIN 1

param set-default LPE_FLW_SCALE 1.0
param set-default LPE_FLW_R 0.2
param set-default LPE_FLW_RR 0.0
param set-default LPE_FLW_QMIN 10
param set-default LPE_VIS_DELAY 0.0
param set-default LPE_VIS_Z 0.1
param set-default LPE_FUSION 86 # flow + vis + land detector + gyro comp

# Manual control parameters
# param set-default COM_RC_IN_MODE 3
param set-default RC2_TRIM 1000
param set-default COM_FLTMODE1 2
param set-default RC_MAP_ROLL 1
param set-default RC_MAP_THROTTLE 2
param set-default RC_MAP_PITCH 3
param set-default RC_MAP_YAW 4
param set-default RC_MAP_FLTMODE 5
param set-default RC_CHAN_CNT 8

# Set to Quadrotor
set MAV_TYPE 2

set MIXER quad_x

```

#### Self check

The RPi image contains a tool for automatic checking the correctness of all the settings and subsystems of the drone – selfcheck.py.

It is generally a good idea to perform this check before flight, especially before an autonomous one.

In order to run it, enter the following command in the Raspberry Pi console:

```bash

rosrun px4_air selfcheck.py

```

Description of some checks:

```bash

FCU – checks for proper connection with the flight controller;
IMU – checks whether the data from from IMU is sane;
Local position – checks presence of local position data;
Velocity estimation – checks whether drone velocity estimation is sane(autonomous flight is not to be performed if this check fails!);
Global position (GPS) — checks for presence of global position data (GPS module is required for this check);
Camera — checks for proper operation of the Raspberry camera.
ArUco — checks whether ArUco detection is working
VPE — checks whether VPE data is published.
Rangefinder — checks whether rangefinder data is published.
RPi health – checks the onboard computer status.
CPU usage – checks the CPU load of the onboard computer.

```

Check the MAVROS state:

```bash
rostopic echo -n 1 /dignostics
```
The heardbeat should be 1HZ
The voltage should be show up "normal"

Run the RVIZ:

The rviz tool allows real-time visualization of all components of the robotic system —the system of coordinates, moving parts, sensors, camera images — on the 3D stag

In the terminal where drone Ubuntu's Running excecute this line:
```bash

export ROS_IP=10.100.190.146

``` 

Open your desktop Ubuntu so we can be able to open rviz and type this scripts:

```bash

ROS_MASTER_URI=http://10.100.190.146
rviz # or rqt

```

#### Camera calibration

Camera calibration can significantly improve the quality of nodes related to computer vision: ArUco markers detection and optical flow.

See the tutorial for more informations:
http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

```bash

rosdep install camera_calibration

```
```bash

rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/camera/image_raw camera:=/camera

```

#### Software autorun
===

> **Note** In the image version **0.20** `clever` package and service was renamed to `clover`. See [previous version of the article](https://github.com/CopterExpress/clover/blob/v0.19/docs/en/autolaunch.md) for older images.

systemd
---

Main documentation: https://wiki.archlinux.org/title/Systemd.

All automatically started Clover software is launched as a `clover.service` systemd service.

The service may be restarted by the `systemctl` command:

```(bash)
sudo systemctl restart clover
```

Text output of the software can be viewed using the `journalctl` command:

```(bash)
journalctl -u clover
```

To run Clover software directly in the current console session, you can use the `roslaunch` command:

```(bash)
sudo systemctl restart clover
roslaunch clover clover.launch
```

You can disable Clover software autolaunch using the `disable` command:

```(bash)
sudo systemctl disable clover
```

roslaunch
---

Main documentation: http://wiki.ros.org/roslaunch.

The list of nodes / programs declared for running is specified in file `/home/pi/catkin_ws/src/clover/clover/launch/clover.launch`.

You can add your own node to the list of automatically launched ones. To do this, place your executable file (e.g. `my_program.py`) into folder `/home/pi/catkin_ws/src/clover/clover/src`. Then add the start of your node to `clover.launch`, for example:

```xml
<node name="my_program" pkg="clover" type="my_program.py" output="screen"/>
```

The started file must have *permission* to run:

```bash
chmod +x my_program.py
```

When scripting languages are used, a <a href="https://en.wikipedia.org/wiki/Shebang_(Unix)">shebang</a> should be placed at the beginning of the file, for example:

```bash
#!/usr/bin/env python3
```


#### How to use

To ensure that the package is connected and working properly, it is recommended to run the launch file first. There are some useful tools available for debugging purposes such as `rqt_graph`, `rostopic echo`, and `rviz`.

Here are the steps to use the package:

#### 1. Launch the mavros node using the following commands:
```bash
roslaunch px4_air 0main_camera.launch
roslaunch px4_air 0mavros.launch
```

#### 2. Launch the drone setup using the following command:
```bash
roslaunch px4_air 1setup.launch
```

#### 3. Launch the control missions using the following commands:
```bash
roslaunch px4_air 2mission_ManualOffb.launch
roslaunch px4_air 3mission_QGCaruco.launch
roslaunch px4_air 4mission_SingleOffb.launch
roslaunch px4_air 5mission_SingleOffb_Avoid.launch
roslaunch px4_air 6mission_FollowMe.launch
```

#### 4. Swarms

This project need to use WiFi hotspot for drone collaboration. Make sure that network is properly setup:

[Setting_Swarms](../swarms/README.md)

[Setting_Hotspot](../swarms/docs/Setting_Hotspot.md)

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
<arg name="gcs_url" default="udp://:14556@192.168.1.196:14550" />
```
#### Change ROS_MASTER_URI to connect to ROS master on another computer (on the same network)
```bash
export ROS_MASTER_URI=http://192.168.1.6:11311
```
### Check if your camera is recognized and working

Run this command to check if your camera is recognized by the system:

```bash
ls -l /dev/video*
```
Camera is assigned as `/dev/video0` by default. If it is absent in the list you get in terminal after running previous command you need to follow next steps: 
1. Open `config.txt` for editing:
```bash
sudo nano /boot/firmware/config.txt
```
2. Add the following line: 
```bash
start_x=1
```
Hit <kbd>Ctrl</kbd>+<kbd>O</kbd> to save, then <kbd>Ctrl</kbd>+<kbd>X</kbd> to exit.
3. Reboot your system.
4. Update your system to install the necessary drivers:
```bash
sudo apt update
```
It may take a few minutes. After its all done you can use `rqt` or `rviz` to visualize your camera topic.


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
