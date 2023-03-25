# This is the project contains Gazebo SITL control for multiple vehicles

# User Guide

[Installing a Desktop Ubuntu for Simulation](doc/InstallDesktop.md)

[Installing a Server Ubuntu for Raspberry Pi](doc/InstallDevice.md)

[Working with SITL](doc/ManualSITL.md)

[Working with HITL](doc/ManualHITL.md)

[Working with Raspberry Pi](doc/ManualRPI.md)

# Inroduction

This project contains autonomous UAVs and cars with PX4 autopilot software.
The folder is provided SITL(Software in the loop), HITL(Hardware in the loop) and folder for the real autonomous too. All the setup is for Ubuntu 20.04.5 and ROS Noetic.

Some realization in this project:

- custom models and configuration in Gazebo for PX4
- Q Ground Control compatible and automatic mission accomplishment
- Atomatic mission with avoidance algorithms
- Swarm mission

Examples:

UAV with avoidance module
![alt text](./doc/uav.jpg)

Ground vehicle with avoidance module
![alt text](./doc/car.jpg)


UAV or ground vehicle swarm
![alt text](./doc/swarm.jpg)


UAV pipeline
![alt text](./doc/uav_pipeline.jpg)


Ground vehicle pipeline
![alt text](./doc/car_pipeline.jpg)


# Licence

Free to use free to modify

