# Install software for the computer 

## Build from Raspberry Pi image

There are several 2 ways to build raspberry pi system. Easiest of courece is to download the image that we are provided.

We are tested and provided several images:
- Raspberry Pi Model 4B 4G RAM
- Raspberry Pi Model 3B

## Build from Source(optional)

#### install the image ubuntu 20.04 server for raspberry
https://cdimage.ubuntu.com/releases/focal/release/


#### To connecto to local wifi network run:

sudo nano /etc/netplan/50-cloud-init.yaml

add this line to the file:
```
wifis:
  wlan0:
    optional: true
    access-points:
        "your-ssid":
            password: "pass"
    dhcp4: true
```
#### We should check the sintax for the errors
```
sudo netplan -debug generate
```
#### We need to mofify terminal UI for the colorful visualization:
```
 nano ~/.bashrc
  ucomment line force_color_prompt=yes
```
press "ctrl + x", press "y", press "enter", write in terminal: exit

#### Encrease swap file:
https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-20-04

Restart the device

### SSH control

Instal Putty to your desktop
https://www.putty.org/

Connect to ssh with ip: 192.168.1.6

#### Install ROS
```
sudo apt update
```
```
sudo apt install git python3-pip -y
```
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/TPODAvia/ROS1-installation.git
chmod +x ROS1-installation/ROS_Raspbian.sh
sudo ./ROS1-installation/ROS_Raspbian.sh
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/noetic/setup.bash
```
```
sudo apt install build-essential python3-rosdep -y
sudo apt-get install ros-noetic-hector-slam -y
sudo apt install libpcl1 ros-noetic-octomap-* -y
```

#### Install YOLOv8 dependencies

```
sudo apt-get install python3-scipy -y
sudo apt-get install ros-noetic-vision-msgs -y
sudo apt-get install ros-noetic-geometry-msgs -y
```

#### Install the workspace
```
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
```
cd ~/catkin_ws/src

# git clone https://github.com/TPODAvia/yolov8_ros.git
git clone https://github.com/okalachev/vl53l1x_ros
git clone https://github.com/machinekoder/ar_track_alvar.git -b noetic-devel

git init
git remote add origin https://github.com/TPODAvia/PX4-Autonomous-Vehicle.git
git pull origin main
```

Restart the PC then continue

```
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
sudo rosdep init
```
Now we need to delete gazebo-related packege because it's not neeeded here
```
sudo rm -r px4_sim
sudo rm -r world_sim
sudo rm -r learning_tf2
```
```
rosdep update
rosdep install --from-paths src --ignore-src -y
```
```
sudo /usr/bin/python3 -m pip install -r ~/catkin_ws/src/requirements.txt
sudo /usr/bin/python3 -m pip install -r ~/catkin_ws/src/yolov8_ros/requirements.txt
```

```
cd ~/catkin_ws
source devel/setup.bash
catkin_make
```

#### Install QGroundControl
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```
# cd
# sudo usermod -a -G dialout $USER
# sudo apt-get remove modemmanager -y
# sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
# sudo apt install libqt5gui5 -y
# sudo apt install libfuse2 -y
# xdg-open https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
```

# Custom image building for Ubuntu Server

go to the cmd and type
```
"C:\Program Files\Oracle\VirtualBox\VBoxManage" internalcommands createrawvmdk -filename "c:/Hard Disks/sdcard.vmdk" -rawdisk "\\.\PHYSICALDRIVE1"

```

install GParted in ubuntu
```
sudo apt install gparted
```
Resize image there

Install win32 disk imager

create image there

