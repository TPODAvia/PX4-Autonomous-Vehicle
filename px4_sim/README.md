The simulation package contains several vehicle models that can be used for simulating different missions. 

The "airframe" folder contains the necessary parameters and files required for simulating the vehicles. These files are specific to each vehicle model. 

To simulate a mission, the launch file needs to be executed. This file contains the list of all the vehicle missions that can be simulated. 

To add a new vehicle model, the corresponding startup script needs to be added to the package. This script can be found in the ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/ directory.

## Air frames:
- frame450
- frame550
- typhoon_h480
- iris
- osprey_vtol
- avia_vtol


## Ground frames:
- diff_rover
- acker_rover
- tank
- balance_bot


## Worlds:
- city
- empty
- flat
- forest
- office
- parcking
- playground
- playpen
- village
