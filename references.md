---
layout: default
title: Notes
nav_order: 200
---
# **Reference file for Ubuntu / Linux solutions**

## Install bash_notes
**__setup __bash_notes**
```bash
git clone https://github.com/Magnushhoie/bash_notes.git
chmod +x bash_notes/setup.sh
cd bash_notes/
./setup.sh
```

## Install llvm and clang
**__setup __clang __llvm __cpp_compiler**
```bash
sudo apt install build-essential xz-utils curl
curl -SL $URL | tar -xJC .
mv $LONG_FILE clang_x.x.x
sudo mv clang_x.x.x /usr/local
export PATH=/usr/local/clang_x.x.x/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/clang_x.x.x/lib:$LD_LIBRARY_PATH
```



## Configure PRUs on Beagleboard Blue
**__setup __bbb __PRU**

Test if PRUS are working

```bash 
rc_test_drivers
```

Both of these commands have fixed issues (first one seems more legit)

```bash 
sudo /opt/scripts/tools/update_kernel.sh --ti-rt-channel --lts-4_19
```

```bash 
sudo dd if=/dev/zero of=/dev/mmcblk1 bs=1M count=10
```

## Configure WiFi on Beagleboard Blue/Black
**__setup __bbb __WIFI**
```bash
connmanctl
	enable wifi
	tether wifi disable
	scan wifi
	services
	agent on
	connect wifi_$WIFI_ID
		$PASSWORD
	quit
```
## Upgrade Beagleboard Blue/Black
**__setup __bbb __upgrade**
```bash
cd /opt/scripts
git pull
sudo tools/update_kernel.sh
sudo shutdown -r now
sudo apt update
sudo apt upgrade
sudo /opt/scripts/tools/version.sh
```
## Install ROS on Beagleboard Blue
**__setup __bbb __ros __install**
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt-get update
sudo apt install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake

sudo rosdep init
rosdep update

mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws

rosinstall_generator ros_base --rosdistro noetic --deps --wet-only --tar > noetic-ros_base-wet.rosinstall

wstool init src noetic-ros_base-wet.rosinstall
rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster

sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic
```
## Install ROS2 on Ubuntu 20.04
**__setup __ros2 __rosinstall_generator**
```bash
rosinstall_generator --deps --tar --rosdistro=foxy ros_base > foxy_ros_base.rosinstall
mkdir -p foxy_ros_base_ws/src
cd foxy_ros_base_ws/src
vcs import < /path/to/foxy_ros_base.rosinstall
```

