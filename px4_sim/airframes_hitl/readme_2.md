To setup your HITL simulation you need to provide symbolic link to your airframe configuration file.
It can be done by using this comand:

ln -fs ~/catkin_ws/src/px4_sim/airframes_hitl/* ~/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/




To make our airframes workable we need to delete other files:

rm -i ~/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/6011_typhoon_h480