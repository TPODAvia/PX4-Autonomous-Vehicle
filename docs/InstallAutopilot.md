
# Working with Autonomous Device

## Install Autopilot controllers

In your desktop Ubuntu go to the PX4-Autopilot folder then build and upload firmware to a flight controler.

cd ~/PX4-Autopilot

The drone can be landed by typing `commander land` and the whole simulation can be stopped by doing **CTRL+C** (or by entering `shutdown`).

Flying the simulation with the ground control station is closer to the real operation of the vehicle.
Click on a location in the map while the vehicle is flying (takeoff flight mode) and enable the slider. 
This will reposition the vehicle.


> **Note**  PX4 can be used with a number of other Simulators, including Gazebo and Gazebo Classic. These are also started with *make* - e.g.

```s
make px4_sitl gazebo-classic
```

#### NuttX / Pixhawk Based Boards

#### Building for NuttX

To build for NuttX- or Pixhawk- based boards, navigate into the **PX4-Autopilot** directory and then call `make` with the build target for your board.

For example, to build for Pixhawk 4 hardware you could use the following command:
```s
cd PX4-Autopilot
make px4_fmu-v5_default
```

A successful run will end with similar output to:
```s
-- Build files have been written to: /home/youruser/src/PX4-Autopilot/build/px4_fmu-v4_default
[954/954] Creating /home/youruser/src/PX4-Autopilot/build/px4_fmu-v4_default/px4_fmu-v4_default.px4
```

The first part of the build target `px4_fmu-v4` indicates the firmware for a particular flight controller hardware.
The following list shows the build commands for the Pixhawk standard boards:

-Holybro Pixhawk 6X (FMUv6X): `make px4_fmu-v6x_default`

-Holybro Pixhawk 6C (FMUv6C): `make px4_fmu-v6c_default`

-Holybro Pix32 v6 (FMUv6C): `make px4_fmu-v6c_default`

-Holybro Pixhawk 5X (FMUv5X): `make px4_fmu-v5x_default`

-Pixhawk 4 (FMUv5): `make px4_fmu-v5_default`

-Pixhawk 4 Mini (FMUv5): `make px4_fmu-v5_default`

-CUAV V5+ (FMUv5): `make px4_fmu-v5_default`

-CUAV V5 nano (FMUv5): `make px4_fmu-v5_default`

-Pixracer (FMUv4): `make px4_fmu-v4_default`

-Pixhawk 3 Pro: `make px4_fmu-v4pro_default`

-Pixhawk Mini: `make px4_fmu-v3_default`

-Pixhawk 2 (Cube Black) (FMUv3): `make px4_fmu-v3_default`

-mRo Pixhawk (FMUv3): `make px4_fmu-v3_default` (supports 2MB Flash)

-Holybro pix32 (FMUv2): `make px4_fmu-v2_default`

-Pixfalcon (FMUv2): `make px4_fmu-v2_default`

-Dropix (FMUv2): `make px4_fmu-v2_default`

-Pixhawk 1 (FMUv2): `make px4_fmu-v2_default`

-Pixhawk 1 with 2 MB flash: `make px4_fmu-v3_default`

  > **Warning**  You **must** use a supported version of GCC to build this board (e.g. the same as used by CI/docker or remove modules from the build. Building with an unsupported GCC may fail, as PX4 is close to the board's 1MB flash limit.

- Pixhawk 1 with 2 MB flash: `make px4_fmu-v3_default`

Build commands for non-Pixhawk NuttX fight controllers (and for all other-boards) are provided in the documentation for the individual flight controller boards.


> **Note**  The `_default` suffix is the firmware _configuration_.
This is optional (i.e. you can also build using `make px4_fmu-v4`, `make bitcraze_crazyflie`, etc.).


#### Uploading Firmware (Flashing the board)

Append `upload` to the make commands to upload the compiled binary to the autopilot hardware via USB.
For example

```s
make px4_fmu-v4_default upload
```

A successful run will end with this output:

```s
Erase  : [====================] 100.0%
Program: [====================] 100.0%
Verify : [====================] 100.0%
Rebooting.

[100%] Built target upload
```

#### Other Boards

Build commands for other boards are given the board-specific flight controller pages (usually under a heading *Building Firmware*).

You can also list all configuration targets using the command:
```s
make list_config_targets
```


#### Compiling in a Graphical IDE

VSCode is the officially supported (and recommended) IDE for PX4 development.
It is easy to set up and can be used to compile PX4 for both simulation and hardware environments.
