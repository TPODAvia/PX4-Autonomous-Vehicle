# This is the project contains Gazebo SITL control for multiple vehicles

# Inroduction

This project contains autonomous UAVs and cars with PX4 autopilot software.
The folder is provided SITL(Software in the loop), HITL(Hardware in the loop) and folder for the real autonomous too.

Some realization in this project:

- custom model and configuration in Gazebo for PX4
- Q Ground Control compatible and automatic mission accomplishment
- Atomatic mission with avoidance algorithms
- Swarm mission

Examples:
![alt text](./doc/uav.jpg)
UAV with avoidance module

![alt text](./doc/car.jpg)
Ground vehicle with avoidance module

![alt text](./doc/swarm.jpg)
UAV or ground vehicle swarm

![alt text](./doc/uav_pipeline.jpg)
UAV pipeline

![alt text](./doc/car_pipeline.jpg)
Ground vehicle pipeline

# Installing Guide

In this txt file manually change the the ubuntu name for your pc. In this example is "vboxuser"

If the problem appeard with permission for user:
open the terminal:

```
su root
nano /etc/sudoers
```
Write in there and save it
```
vboxuser ALL=(ALL:ALL) ALL
%vboxuser ALL=(ALL) ALL
```
Close the teminal. Then in the new terminal run

### Install ROS
```
sudo apt install git python3-pip -y
```
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/TPODAvia/ROS1-installation.git
chmod +x ROS1-installation/ROS.sh
sudo ./ROS1-installation/ROS.sh
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
```
source /opt/ros/noetic/setup.bash
sudo apt install build-essential git python3-pip python3-rosdep -y
sudo apt install libpcl1 ros-noetic-octomap-* -y
```
### Install ORB-SLAM dependencies
```
sudo apt install libeigen3-dev -y
sudo apt install ros-noetic-hector-trajectory-server -y
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make
sudo make install
sudo ldconfig
```

### Install YOLOv7 dependencies

```
pip3 install numpy==1.21
pip3 install torch torchvision pandas

sudo apt-get install python3-scipy -y
sudo apt-get install ros-noetic-vision-msgs -y
sudo apt-get install ros-noetic-geometry-msgs -y
sudo apt-get install ros-noetic-usb-cam -y
```

### Install the workspace
```
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
```
cd ~/catkin_ws/src
git clone https://github.com/thien94/orb_slam3_ros.git
cd ../
catkin_make

```
```
cd ~/catkin_ws/src
git init
git remote add origin http://git.promcars.ru/promavto-drone/px4-repository.git
git pull origin main
```

Restart the PC then continue

```
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
sudo rosdep init
```
```
rosdep update
rosdep install --from-paths src --ignore-src -y

sudo /usr/bin/python3 -m pip install -r ~/catkin_ws/src/requirements.txt
sudo /usr/bin/python3 -m pip install -r ~/catkin_ws/src/yolov7-ros/requirements.txt
```
### Install PX4-Autopilot
```
git clone --recursive --depth 1 --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot
ln -s ~/PX4-Autopilot ~/catkin_ws/src/
ln -s ~/PX4-Autopilot/Tools/sitl_gazebo ~/catkin_ws/src/
ln -s ~/PX4-Autopilot/mavlink ~/catkin_ws/src/

cd ~/catkin_ws/src/PX4-Autopilot/Tools/setup
sudo ./ubuntu.sh
```
```
pip3 install --user toml
```
```
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
cd ~/catkin_ws
source devel/setup.bash
catkin_make
```
if catkin_make fails just catkin_make again :)

### Install QGroundControl
```
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 -y
xdg-open https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
cd
```
wait for some second
```
cp ./Downloads/QGroundControl.AppImage ~/QGroundControl.AppImage
chmod +x ./QGroundControl.AppImage
```

Fixing Octomap server problem
```
sudo apt install libopencv-dev python-jinja2 protobuf-compiler -y
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/avoidance/sim/models
export QT_X11_NO_MITSHM=1
```
### Install SITL enviroment
And this is the final step
```
cd ~/PX4-Autopilot
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11
make px4_sitl gazebo
```
Close the gazebo simulation then restart your Ubuntu

# Starting Guide

### 1) Now the enviroment is done. Test the sdf models first.

Go to the catkin_ws first
```
cd ~/catkin_ws
```
In the terminal 1 run:
```
roscore
```
run the launch file with your defined vehicle
In the terminal 2 run:
```
source devel/setup.bash 
roslaunch px4_ground 0launch_model_only.launch
```
You can change the vehicle type <arg name="vehicle" default="r1_rover"/>
Now, If everything is fine close all and moving to the next step.

### 2) Run with PX4 SITL.
In the terminal 1 run:
```
roscore
```
In the terminal 2 run:
```
source devel/setup.bash 
roslaunch px4_ground 1mavros_posix_sitl.launch
```

Remember that you can change the vehicle type.
<arg name="vehicle" default="vector"/>
Open Qground Control(QGC) or run ./QGroundControl.AppImage

``Note: for r1_rover you need to change some parameters in QGC. Now you can run a PX4 vehicle in Gazebo enviroment.``

### 3) Run the predefined mission

In QGC you can add a predefined mission in the path ~/px4_ground/mission and run the mission as usual.

### 4) Run in mission mode with mavros.
In the terminal 1 run:
```
roscore
```
In the terminal 2 run:
```
source devel/setup.bash 
roslaunch px4_ground 1mavros_posix_sitl.launch
```
Now go to the path ../px4_ground/src you need to get the permision to all pythons files. I mean chmod +x to all python file in the /src path.

Example: chmod +x control_vel.py

In the terminal 3 run:
```
source devel/setup.bash
rosrun mavros mavsys mode -c OFFBOARD
rosrun mavros mavsafety arm
rosrun px4_ground wind.py
rosrun px4_ground control_vel.py
rosrun px4_ground mavros_offboard_posctl_test.py
```
``Note: others python scripts is experimental. If the OFFBOARD is running, you won't be able to run a QGC mission plan.``

The option is to run launch file instead of python node.
In the terminal 3 run: (Options)
```
roslaunch px4_ground 3mission_multi_offb.launch
```
Now, If everything is fine close all and moving to the next step.

### 5) Run the SLAM Navigation
In the terminal 1 run:
```
roscore
```
For air vehicle run:
In the terminal 2 run:
```
roslaunch px4_ground 2obs_avoidance_air.launch
```

For ground vehicle run:
In the terminal 2 run: (Option)
```
roslaunch px4_ground 2obs_avoidance_ground2.launch
```
In the terminal 3 run:
```
source devel/setup.bash
rosrun mavros mavsys mode -c OFFBOARD
rosrun mavros mavsafety arm
```

### 6) Run ORB-SLAM navigation
In one terminal:
```
roslaunch orb_slam3_ros euroc_mono_inertial.launch
```
In another terminal:
```
rosbag play MH_01_easy.bag
```

Note: MH_01_easy cab be download here: http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag
    
### 7) Run multiple UAV with mavros.
In the terminal 1 run:
```
roslaunch px4_ground 4multi_uav_mavros_sitl.launch
```
Still in development...
