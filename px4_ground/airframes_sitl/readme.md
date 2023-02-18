The steps I took for creating a custom Gazebo model and PX4 SITL airframe with the adjusted gains:

1
Create a folder under Tools/sitl_gazebo/models for your model, let’s call it my_vehicle

2 
Create the following files under Tools/sitl_gazebo/models/my_vehicle: model.config and my_vehicle.sdf (these can be based off the iris or solo models in Tools/sitl_gazebo/models)

3 
Create a world file in Tools/sitl_gazebo/worlds called my_vehicle.world (Again, this can be based off the iris or solo world files)

4 
Create an airframe file under ROMFS/px4fmu_common/init.d-posix (Yet again, this can be based off the iris or solo airframe files), give it a number (for example 4229) and name it 4229_my_vehicle

5 
Add the airframe name (my_vehicle) to the file platforms/posix/cmake/sitl_target.cmake in the command set(models …

6
Now you are free to tweak any of the model values in the SDF file and any of the controller gains in the airframe file.
You can launch SITL with your model with “make px4_sitl gazebo_my_vehicle”



A couple of things to keep in mind:

1 
When increasing the mass of the vehicle, also increase the inertias. I found that Gazebo does not like really small inertias.

2 
With the larger vehicle, tweak the motor values (but you have done this already)

3 
If the quad is unstable, it is probably due to bad controller gains. Tweak them for the larger vehicle.






To setup your SITL simulation you need to provide symbolic link to your airframe configuration file.
It can be done by using this comand:

ln -fs ~/catkin_ws/src/px4_ground/airframes_sitl/* ~/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/

ln -s ~/catkin_ws/src/px4_ground/airframes_sitl/* /home/vboxuser/.ros/etc/init.d-posix/airframes