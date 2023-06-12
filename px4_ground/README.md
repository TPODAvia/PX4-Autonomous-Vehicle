The package is designed to run on the companion computer for rovers. 

## How to use

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
