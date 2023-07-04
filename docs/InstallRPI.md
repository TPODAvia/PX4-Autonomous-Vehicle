# Install Ubuntu 20.04 Server for the companion computer 

## Build from our Raspberry Pi images

There are several 2 ways to build raspberry pi system. Easiest of courece is to download the image that we are provided.

We are tested and provided several images (https://drive.google.com/drive/folders/1ff6Cl5y6kWvpjLPnML8qoepN1H--mvBu?usp=share_link):
- Raspberry Pi Model 4B 4G RAM
- Raspberry Pi Model 3B

## Build from Source(optional)

#### Install the image ubuntu 20.04 server for raspberry
https://cdimage.ubuntu.com/releases/focal/release/


#### To connect to local wifi network run:

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```
add this line to the file:
```bash
wifis:
  wlan0:
    optional: true
    access-points:
        "your-ssid":
            password: "pass"
    dhcp4: true
```
the setting can be modified:
```bash
wifis:
  wlan0:
    optional: true
    dhcp4: false
    dhcp6: false
    addresses: [10.100.190.50/24]
    nameservers:
      addresses: [10.11.12.2]
    access-points:
        "your-ssid":
            password: "pass"
    routes:
      - to: default
        via: 10.100.190.1
```

The yaml should look like this:

![alt text](./wifi.jpeg)

#### We should check the sintax for the errors
```bash
sudo netplan -debug generate
```

#### To connect to desktop with Ethernet cable:
Both desktop and Raspberry Pi should have IP adresses from the range 169.254.x.x to be ready to connect via ssh. You can verify IP adresses with this commands:
for Windows
```bash
ipconfig
```
and for Linux (may require net-tools to be installed)
```bash
ifconfig -a
```
other options:
```bash
ip a
ip addr show
hostname -I
```

#### We need to mofify terminal UI for the colorful visualization:
```bash
ls -la ~/ | more
 nano ~/.bashrc
```
uncomment the line: 
```bash
force_color_prompt=yes
```
press "ctrl + x", press "y", press "enter", write in terminal: exit

#### Encrease swap file:
See this instructions for the details
https://www.digitalocean.com/community/tutorials/how-to-add-swap-space-on-ubuntu-20-04

Restart the device
```bash
sudo reboot
```

### SSH control

Install the Putty software to your desktop
https://www.putty.org/

Connect to ssh with IP: 192.168.1.6

![alt text](./putty.jpeg)

#### Install ROS
```bash
sudo apt update
```
```bash
sudo apt install git python3-pip python3-schedule -y
```
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/TPODAvia/ROS1-installation.git
chmod +x ROS1-installation/ROS_server.sh
sudo ./ROS1-installation/ROS_server.sh
```
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/noetic/setup.bash
sudo apt install xterm pi-bluetooth -y
sudo apt install build-essential python3-rosdep ros-noetic-hector-slam libpcl1 ros-noetic-octomap-* -y
# sudo apt-get install ros-noetic-hector-slam -y
# sudo apt install libpcl1 ros-noetic-octomap-* -y
```

#### Install YOLOv8 dependencies

```bash
sudo apt-get install python3-scipy ros-noetic-vision-msgs ros-noetic-geometry-msgs -y
# sudo apt-get install ros-noetic-vision-msgs -y
# sudo apt-get install ros-noetic-geometry-msgs -y
```

#### Install the workspace
```bash
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
```bash
cd ~/catkin_ws/src

git clone https://github.com/TPODAvia/yolov8_ros.git
git clone https://github.com/okalachev/vl53l1x_ros
git clone https://github.com/machinekoder/ar_track_alvar.git -b noetic-devel

git init
git remote add origin https://github.com/TPODAvia/PX4-Autonomous-Vehicle.git
git pull origin main
```

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
sudo rosdep init
```
Now we need to delete gazebo-related package because it's not neeeded here
```bash
sudo rm -r src/px4_sim
sudo rm -r src/world_sim
sudo rm -r src/mavros_humantracking
```
```bash
rosdep update
rosdep install --from-paths src --ignore-src -y
```
```bash
sudo /usr/bin/python3 -m pip install -r ~/catkin_ws/src/requirements.txt
sudo /usr/bin/python3 -m pip install -r ~/catkin_ws/src/yolov8_ros/requirements.txt
```

```bash
cd ~/catkin_ws
source devel/setup.bash
catkin_make -j1
```

#### Install QGroundControl

```bash
cd
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

#### WIFI setup

```bash
cd ~/catkin_ws/src/swarms/
sudo wifi-hotspot.sh
```

#### I2C communication setup

[Setting 9250 IMU module with I2C](docs/IMU9250I2C.md)

#### Flash Arduino via ssh

[Setting Raspberry Pi with Arduino](docs/InstallArduino.md)

#### Custom image building for the Ubuntu Server

For Linux users there are really simple instructions on the internet how to do it but for Windows users there are some "hacks" how we can create an image backup for our companion computer

Go to the cmd and type:
```
"C:\Program Files\Oracle\VirtualBox\VBoxManage" internalcommands createrawvmdk -filename "c:/Hard Disks/sdcard.vmdk" -rawdisk "\\.\PHYSICALDRIVE1"
```
![alt text](./cmd.jpeg)

Go to Ubuntu:
install GParted
```bash
sudo apt install gparted
```
Resize image there

![alt text](./gparted.jpeg)

Switch back to Windows and install "win32 disk imager". It's a simple way to create backup file.

Remember to check "Read Only Allocated Partitions"
![alt text](./win32.jpeg)


#### Delete disk location

To delete the link to an SD card in VirtualBox, you can follow these steps:

- Open the VirtualBox Disk Manager.

- Locate the sdcard.vmdk file in the list.

- Select the sdcard.vmdk file and click on the "Remove" or "Delete" button.

![alt text](./Vbox.jpeg)

From cmd excecute this line of code:

"C:\Program Files\Oracle\VirtualBox\VBoxManage" closemedium "C:\Hard Disks\sdcard.vmdk" --delete


#### Always On mode

To disable the sleep mode and allow automatic login in Ubuntu using the terminal, follow the steps below:

1. **Disable Sleep Mode**

To disable the sleep mode in Ubuntu, you can use the `systemctl` command to mask the sleep, suspend, hibernate, and hybrid-sleep targets. This essentially links these unit files to /dev/null, making it impossible for the system to start them [Source 2](https://askubuntu.com/questions/47311/how-do-i-disable-my-system-from-going-to-sleep).

You can execute the following command:

```bash
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
```

Keep in mind that this command will completely disable sleep mode, meaning your system will stay awake indefinitely until you manually put it to sleep or shut it down.

2. **Enable Automatic Login**

To enable automatic login, you'll need to edit the `custom.conf` file in the `/etc/gdm3/` directory. Here is how you can do it:

First, open the file using a text editor such as `nano`.

```bash
sudo nano /etc/gdm3/custom.conf
```

Find the section in the file labeled `[daemon]`. In this section, uncomment (or add if it's not there) the line `AutomaticLoginEnable=true` and the line `AutomaticLogin=[your username]`, replacing `[your username]` with your actual username.

Here is how it should look:

```bash
[daemon]
AutomaticLoginEnable=true
AutomaticLogin=john_doe
```

After making the changes, press `Ctrl+X` to exit and `Y` to save the changes. Then, restart your system for the changes to take effect.

Please note that enabling automatic login can pose a security risk as anyone who can access your computer will be able to turn it on and have immediate access to your files and data.

3. **Handle Lid Close Action**

In some cases, you might want to change the action that Ubuntu takes when the laptop lid is closed. To do this, you can edit the `/etc/systemd/logind.conf` file [Source 9](https://www.dell.com/support/kbdoc/en-us/000179566/how-to-disable-sleep-and-configure-lid-power-settings-for-ubuntu-or-red-hat-enterprise-linux-7).

Open the file using a text editor:

```bash
sudo nano /etc/systemd/logind.conf
```

Find the line that starts with `#HandleLidSwitch=suspend` and change it to `HandleLidSwitch=ignore`. This will make Ubuntu do nothing when the laptop lid is closed.

Here is how it should look:

```bash
HandleLidSwitch=ignore
```

After making the change, save and close the file, then restart the `systemd-logind` service by running:

```bash
sudo systemctl restart systemd-logind
```

Please keep in mind that these changes should be made carefully as they can significantly alter the behavior of your system. Always make sure to backup any files you edit in case you need to revert the changes.
