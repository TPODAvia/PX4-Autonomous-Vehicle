To setup your SITL simulation you need to provide symbolic link to your airframe configuration file.
It can be done by using this command:

ln -fs ~/catkin_ws/src/px4_sim/airframes_sitl/* ~/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/