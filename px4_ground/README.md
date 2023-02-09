# This repository contains Gazebo SITL control for multiple vehicles

## Air airframes:
- iris multicopter
- standard vtol
## Ground airframes:
- rover ackermann drive
- r1_rover differencial drive
- vector (Experimantal robot for SLAM algorithms)


# Starting Guide
### 0) The prerequiment


### 1) Create your workspace first.
mkdir test_ws && cd test_ws && mkdir src && cd src

### 2) Init your workspace.
catkin_init_workspace src

### 3) Clone this repository
git clone http://git.promcars.ru/promavto-drone/px4-repository.git

### 4) cd ..
### 5) catkin_make
### 6) source devel/setup.bash 

### 7) Now the enviroment is done. Test the sdf models first.
In the terminal 1 run:
    roscore

run the launch file with your defined vehicle
In the terminal 2 run:
    source devel/setup.bash 
    roslaunch px4_ground 0launch_urdf_only.launch
You can change the vehicle type <arg name="vehicle" default="r1_rover"/>
Now, If everything is fine close all and moving to the next step.

### 8) Run with PX4 SITL.
In the terminal 1 run:
    roscore
In the terminal 2 run:
    source devel/setup.bash 
    roslaunch px4_ground 1mavros_posix_sitl.launch
   
Remember that you can change the vehicle type.
<arg name="vehicle" default="vector"/>
Open Qground Control(QGC) or run ./QGroundControl.AppImage

```Note: for r1_rover you need to change some parameters in QGC. Now you can run a PX4 vehicle in Gazebo enviroment.```

### 9) Run the predefined mission

In QGC you can add a predefined mission in the path ~/px4_ground/mission and run the mission as usual.

### 10) Run in mission mode with mavros.
In the terminal 1 run:
    roscore

In the terminal 2 run:
    source devel/setup.bash 
    roslaunch px4_ground 1mavros_posix_sitl.launch

Now go to the path ../px4_ground/src you need to get the permision to all pythons files. I mean chmod +x to all python file in the /src path.
Example: chmod +x control_vel.py
In the terminal 3 run:
    source devel/setup.bash
    rosrun px4_ground wind.py
    rosrun px4_ground control_vel.py
    rosrun px4_ground mavros_offboard_posctl_test.py
```Note: others python scripts is experimental. If the OFFBOARD is running, you won't be able to run a QGC mission plan.```

The option is to run launch file instead of python node.
In the terminal 3 run: (Option)
    roslaunch px4_ground 3mission_multi_offb.launch
Now, If everything is fine close all and moving to the next step.

### 11) Run the SLAM Navigation
In the terminal 1 run:
    roscore

For air vehicle run:
In the terminal 2 run:
    roslaunch px4_ground 2obs_avoidance_air.launch

For ground vehicle run:
In the terminal 2 run: (Option)
    roslaunch px4_ground 2obs_avoidance_ground2.launch

### 12) Run multiple UAV with mavros.
roslaunch px4_ground 4multi_uav_mavros_sitl.launch
Still in development...
