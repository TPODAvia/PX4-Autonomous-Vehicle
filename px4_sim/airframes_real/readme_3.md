For a new airframe belonging to an existing group, you don't need to do anything more than provide documentation in the airframe description located at ROMFS/px4fmu_common/init.d.


this should be a copy
ln -fs ~/catkin_ws/src/px4_sim/airframes_real/50008_aoaoaoao ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/

cd PX4-Autopilot

add airframe
add cmake

make px4_fmu-v6c

go to the QGC

flash