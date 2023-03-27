For a new airframe belonging to an existing group, you don't need to do anything more than provide documentation in the airframe description located at ROMFS/px4fmu_common/init.d.


#### Step 1:
ln -fs ~/catkin_ws/src/px4_sim/airframes_real/* ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/

#### Step 2:
cd ~/PX4-Autopilot

#### Step 3:
Go to the CmakeList and add all models there

#### Step 4:
make px4_(your board)
make px4_fmu-v6c

#### Step 5:
go to the QGC and flash to controller