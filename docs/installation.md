# Linux Installation
This project requires PCL [LINK NEEDED], librealsense [LINK NEEDED], Java JDK, MATLAB Compiler Runtime. These will be installed in this guide.

**Make sure you have the dependencies listed below before starting the installation process.**

* Ubuntu 14/16 with kernel newer than version 4.10
	* A kernel patch is needed for other versions, see [librealsense installation instructions](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
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

The following hardware requirements are needed in order to run the entire system
* An ABB robot with the Robot Web Services API
* A depth camera (Intel RealSense point cloud capturer is included in this repo)
	* Intel RealSense SR300 and D435 have been tested for this system, but other depth cameras should work as well with a proper point cloud capturing program
* (A USB 3.0 port for the depth camera)
