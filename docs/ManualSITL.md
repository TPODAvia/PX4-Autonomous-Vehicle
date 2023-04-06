# Starting Guide

#### Adding airframe configurations

To add sitl configuration it's simple to add this lines of codes. But it's will be clean if you rebuil your sitl-gazebo project in PX4-Autopilot

```s
ln -fs ~/catkin_ws/src/px4_sim/airframes_sitl/* ~/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/
```
#### Another option:

The steps I took for creating a custom Gazebo model and PX4 SITL airframe with the adjusted gains:

- Create a folder under Tools/sitl_gazebo/models for your model, let’s call it my_vehicle

- Create the following files under Tools/sitl_gazebo/models/my_vehicle: model.config and my_vehicle.sdf (these can be based off the iris or solo models in Tools/sitl_gazebo/models)

- Create a world file in Tools/sitl_gazebo/worlds called my_vehicle.world (Again, this can be based off the iris or solo world files)

- Create an airframe file under ROMFS/px4fmu_common/init.d-posix (Yet again, this can be based off the iris or solo airframe files), give it a number (for example 4229) and name it 4229_my_vehicle

- Add the airframe name (my_vehicle) to the file platforms/posix/cmake/sitl_target.cmake in the command set(models …

- Now you are free to tweak any of the model values in the SDF file and any of the controller gains in the airframe file.
You can launch SITL with your model with “make px4_sitl gazebo_my_vehicle”



A couple of things to keep in mind:

- When increasing the mass of the vehicle, also increase the inertias. I found that Gazebo does not like really small inertias.

- With the larger vehicle, tweak the motor values (but you have done this already)

- If the quad is unstable, it is probably due to bad controller gains. Tweak them for the larger vehicle.


#### 1) Test the sdf models.

Go to the catkin_ws
```s
cd ~/catkin_ws
```
In the terminal 1 run:
```s
roscore
```
Run the launch file with your defined vehicle

In the terminal 2 run:
```s
roslaunch px4_sim 0launch_model_only.launch
```
You can change the vehicle type 
```s
<arg name="vehicle" default="diff_rover"/>
```
Change world if necessary
```s
<arg name="world" default="$(find px4_sim)/worlds/empty.world"/>
```

#### 2) Run with PX4 SITL.
In the terminal 1 run:
```s
roscore
```
In the terminal 2 run:
```s
roslaunch px4_sim 1mavros_posix_sitl.launch
```

Remember that you can change the vehicle type.
```s
<arg name="vehicle" default="frame450"/>
```
Now we can use Qground Control(QGC) or run ./QGroundControl.AppImage

#### 3) Run the predefined mission

In QGC you can add a predefined mission in the path ~/px4_sim/mission and run the mission as usual.

#### 4) Run in mission mode with mavros.
In the terminal 1 run:
```s
roscore
```
In the terminal 2 run:
```s
roslaunch px4_sim 1mavros_posix_sitl.launch
```
> **Note** Now go to the path ../px4_sim/src you need to get the permision to all pythons files. I mean chmod +x to all python file in the /src path.

Example: 
```s
chmod +x control_vel.py
```

In the terminal 3 run:
```s
source devel/setup.bash
rosrun mavros mavsys mode -c OFFBOARD
rosrun mavros mavsafety arm
rosrun px4_sim wind.py
rosrun px4_sim mavros_vel_test.py
rosrun px4_sim mavros_offboard_posctl_test.py
```
> **Note**  others python scripts is experimental. If the OFFBOARD is running, you won't be able to run a QGC mission plan.

The option is to run launch file instead of python node.
In the terminal 3 run: (optional)
```s
roslaunch px4_sim 3mission_qgcaruco_sitl.launch
```
Now, If everything is fine close all and moving to the next step.

#### 5) Run the Avoidance Navigation
In the terminal 1 run:
```s
roscore
```
For air vehicle run:
In the terminal 2 run:
```s
roslaunch px4_sim 2obs_avoidance_air_local.launch
```

For ground vehicle run:
In the terminal 2 run: (optional)
```s
roslaunch px4_sim 2obs_avoidance_car_lidar.launch
```
In the terminal 3 run:
```s
rosrun mavros mavsys mode -c OFFBOARD
rosrun mavros mavsafety arm
```

#### 6) Run Missions with collision avoidance
In the terminal run:
```s
roslaunch px4_sim 4mission_followme_sitl.launch
```
    
#### 7) Run multiple UAVs with mavros.
In the terminal run:
```s
roslaunch px4_sim 5multi_uav_auto_sitl.launch
```