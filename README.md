# PieMoPER
Pika Teleoperation for Mobile PiPER (**PieMoPER**) is a simple and convenient teleoperation system designed for the **MoPER** robot, and it can be easily intergrated into other mobile robots. The system is built upon **AgileX Pika**, which uses multiple VIVE trackers for localization and maps the teleoperator's movements onto the robot.

<p align="center">
  <img src="image/moper_v1_side.jpg" height="400">
  <img src="image/demo_open_drawer_pick_bottle_v1.gif" height="400">
</p>

Follow the steps below to set up and use PieMoPER.

## Guides
- [Hardware Requirements](#hardware-requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Miscellaneous](#miscellaneous)

## Hardware Requirements
- **Mobile Robot:** We use our **MoPER** robot, equipped with two PiPER arms. 
- **Laptop:** Running **Ubuntu 20.04** with at least **3x USB3.0 ports**.
- **Docking Station:** Provides at least **7× USB3.0 ports**. A docking station with individual port switchs is recommended.
- **AgileX Pika:** 2× Pika Sense, 2× Pika Gripper, 2× Pika Station.
- **VIVE Tracker:** 1× VIVE tracker for robot base localization.

## Installation
PieMoPER is built upon [pika_ros](https://github.com/agilexrobotics/pika_ros), [pika_sdk](https://github.com/agilexrobotics/pika_sdk), [PikaAnyArm](https://github.com/agilexrobotics/PikaAnyArm).
Please refer to the original repositories for detailed setup guides. 
Currently, the installation process is somewhat tedious, but ensures optimal result. We plan to improve it in the future.

-  Clone repo (install under ~ for documentation consistency):
	```bash
	cd ~ && git clone
	```
-  Set Up the Environment and Install Dependencies
	1. Create a virtual environment:
		```bash
		conda create -n piemoper python=3.9.18
		conda activate piemoper
		conda install pinocchio==3.2.0 casadi==3.6.7 -c conda-forge
		pip3 install meshcat rospkg pyyaml piper-sdk opencv-python netifaces ur-rtde
		```
	2. Install ROS-noetic following [the official documentation](https://wiki.ros.org/noetic/Installation/Ubuntu)
	3. Install other dependencies:
		```bash
		sudo apt-get update && sudo apt install \
		v4l-utils libudev-dev libeigen3-dev libjsoncpp-dev ros-noetic-ddynamic-reconfigure \
		libpcap-dev  ros-noetic-serial ros-noetic-ros-numpy ros-noetic-librealsense2 \
		python3-pcl libqt5serialport5-dev build-essential zlib1g-dev libx11-dev \
		libusb-1.0-0-dev freeglut3-dev liblapacke-dev libopenblas-dev libatlas-base-dev \
		cmake git libssl-dev pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev \
		libglu1-mesa-dev g++ python3-pip libopenvr-dev ethtool can-utils
		```
	4. Install RealSense SDK:
		1. Extract package files:
			```bash
			unzip source/librealsense-2.55.1.zip -d source/librealsense-2.55.1
			mkdir -p source/curl-7.75.0 && tar -zxvf source/curl-7.75.0.tar.gz -C source/curl-7.75.0
			```
		2. Edit file path:
			```bash
			gedit librealsense-2.55.1/CMake/external_libcurl.cmake
			```
			Update the curl path at **line 21** to your extracted path, e.g.:
			```bash
			/home/<user>/pika_ros/source/curl-7.75.0
			```
		3. Install package:
			```bash
			cd librealsense-2.55.1 && bash install.bash
			```
		4. Verify installation:
			```bash
			realsense-viewer
			```
			If the GUI displays normally, the installation is successful.
	5. Install libsurvive
	```bash
	git clone https://github.com/cntools/libsurvive.git
	cd libsurvive
	sudo cp ./useful_files/81-vive.rules /etc/udev/rules.d/
	sudo udevadm control --reload-rules && sudo udevadm trigger
	make
	```
	6. Install pika sdk:
	```bash
	pip3 install agx-pypika 
	# wxpython may take insanely long time to install
	```
-  Install PieMoPER:
	```bash
	unzip source/install.zip -d install
	catkin_make install -DCATKIN_WHITELIST_PACKAGES=""
	chmod +x -R install/
	```
	
## Usage
Please refer to the [PieMoPER documentation](PieMoPER_Documentation.md) for detailed usage instructions.
After completing the setup, you will be able to teleoperate your own robot like this:

<p align="center">
  <img src="image/sense_test_real_quick.gif" height="400">
</p>

## Miscellaneous
1. We built our own mobile teleoperation code (the AgileX official one is fixed base and closed source).
2. The code and documentation is still under revising, feel free to ask questions.
3. ...

## Citation
If you found the code usefull, please cite this repository.