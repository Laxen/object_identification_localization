##Overview
System for object identification and localization using point clouds, developed by Alexander Ganslandt and Andreas Svensson as a master's thesis at the Institute of Computer Science at the Faculty of Engineering, Lund University. For details look at our master's thesis report [NEED LINK]. 

This system identifies and localizes arbitrary objects using point clouds from a depth camera. It is intended for applications in robotics where the depth camera is mounted on a robot arm and positional data from the robot is used to merge point clouds. The system suggests where to move the camera to gain as much information as possible about the object, based on Next Best View heuristics. New objects to be identified/localized are easily added as CAD-models.

##Installation
Installation instructions for Linux can be found here [LINK NEEDED]. Please note that this project requires the following special dependencies. 
* Ubuntu 14/16 with kernel newer than version 4.10
	* A kernel patch is needed for other versions, see librealsense installation instructions [LINK NEEDED]
	* Check kernel version using "uname -r" in a terminal
	* Upgrade to latest version using "sudo apt-get dist-upgrade"
	* Restart
* gcc 5.0.0+
* CMake 3.1.0+
	* sudo apt-get install cmake
* Boost 1.40+
	* sudo apt-get install libboost-all-dev
* Eigen 3.0+
	* sudo apt-get install libeigen3-dev
* FLANN 1.7.1+
	* sudo apt-get install libflann-dev
* VTK 5
	* Note that VTK 6 does not work (visualization module in PCL will not be able to run)
	* sudo apt-get install libvtk5-dev
* QHull
	* sudo apt-get install libqhull-dev
* udev
	* sudo apt-get install libudev-dev
* pkg-config
	* sudo apt-get install pkg-config
* GTK+
	* sudo apt-get install libgtk-3
* GLFW
	* sudo apt-get install libglfw3-dev
	* sudo apt-get install glfw3

##License
