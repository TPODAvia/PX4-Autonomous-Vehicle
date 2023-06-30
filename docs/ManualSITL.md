# Starting Guide

#### Adding airframe configurations

To add sitl configuration it's simple to add this lines of codes. But it's will be clean if you rebuil your sitl-gazebo project in PX4-Autopilot

```bash
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
```bash
cd ~/catkin_ws
```

Run the launch file with your defined vehicle

In the terminal run:
```bash
roslaunch px4_sim 0launch_model_only.launch
```
You can change the vehicle type 
```bash
<arg name="vehicle" default="diff_rover"/>
```
Change world if necessary
```bash
<arg name="world" default="$(find px4_sim)/worlds/empty.world"/>
```

#### 2) Run with PX4 SITL.

To setup your SITL simulation make sure to provide symbolic link to your airframe configuration file.
```bash
ln -fs ~/catkin_ws/src/px4_sim/airframes_sitl/* ~/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/
```

In the terminal run:
```bash
roslaunch px4_sim 1mavros_posix_sitl.launch
```

Remember that you can change the vehicle type.
```bash
<arg name="vehicle" default="frame450"/>
```
Now we can use Qground Control(QGC) or run ./QGroundControl.AppImage

#### 3) Run the predefined mission

In QGC you can add a predefined mission in the path ~/px4_sim/mission and run the mission as usual.

#### 4) Run in mission mode with mavros.

In the terminal run:
```bash
roslaunch px4_sim 1mavros_posix_sitl.launch
```
> **Note** Now go to the path ../px4_sim/scripts you need to get the permision to all pythons files. I mean chmod +x to all python file in the /scripts path.

Example: 
```bash
chmod +x control_vel.py
```

In the second terminal run:
```bash
rosrun mavros mavsys mode -c OFFBOARD
rosrun mavros mavsafety arm
rosrun px4_sim wind.py
rosrun px4_sim mavros_vel_test.py
rosrun px4_sim mavros_offboard_posctl_test.py
```
> **Note**  others python scripts is experimental. If the OFFBOARD is running, you won't be able to run a QGC mission plan.

The option is to run launch file instead of python node.
In the second terminal run: (optional)
```bash
roslaunch px4_sim 3mission_qgcaruco_sitl.launch
```
Now, If everything is fine close all and moving to the next step.

#### 5) Run the Avoidance Navigation

For air vehicle run:
```bash
roslaunch px4_sim 2obs_avoidance_air_local.launch
```

For ground vehicle run: (optional)
```bash
roslaunch px4_sim 2obs_avoidance_car_lidar.launch
```
In the second terminal run:
```bash
rosrun mavros mavsys mode -c OFFBOARD
rosrun mavros mavsafety arm
```

#### 6) Run Missions with precision landing
In the terminal run:
```bash
roslaunch px4_sim 3mission_qgcaruco_sitl.launch
```

#### 7) Run Missions with collision avoidance
In the terminal run:
```bash
roslaunch px4_sim 4mission_car_singleoffb_avoid_sitl.launch
```
    
#### 8) Run multiple UAVs with mavros.
In the terminal run:
```bash
roslaunch px4_sim 5multi_uav_qgc_sitl.launch
```

#### 9) Run multiple UAVs missions.
In the terminal run:
```bash
roslaunch px4_sim 5multi_uav_swarms.launch
```

#### 10) Upload flight plan via QGroundControl

Once both the Gazebo and QGroundControl windows have appeared (QGroundControl should show the drone location near San
Carlos airport), use QGroundControl to upload the sample `~px4_sim/mission/MC_mission_box.plan` flight plan that is included inside the
Docker container, and then start the mission.

#### 11) Simulate GPS failure

Wait until the drone has risen to its final mission altitude. You should see a visualization of the GISNav-estimated
field of view projected on the ground appear. You can then try disabling GPS through your [MAVLink Shell][5]
*(accessible e.g. through QGroundControl > Analyze Tools > MAVLink Console)*:

```bash
failure gps off
```

The drone should now continue to complete its mission *GNSS-free* with GISNav substituting for GPS.

You can check if PX4 is receiving the mock GPS position estimates by typing the following in the MAVLink shell:

```bash
listener sensor_gps
```

If the printed GPS message has a `satellites_used` field value of `255`, your PX4 is receiving the mock GPS node output
as expected. QGroundControl will most likely show 0 satellites used next to the GPS icon as shown in the demo video.

[5]: https://docs.px4.io/main/en/debug/mavlink_shell.html#qgroundcontrol
