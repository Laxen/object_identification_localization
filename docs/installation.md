# Linux Installation
This project requires [PCL](https://github.com/PointCloudLibrary/pcl), [librealsense](https://github.com/IntelRealSense/librealsense), Java JDK, MATLAB Compiler Runtime. These will be installed in this guide.

## Dependencies

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

## librealsense
* Follow the installation guide on the [librealsense GitHub page](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
	* Compiling with examples and demos is not necessary
	* Note that if you have a kernel with version later than 4.10, the steps in "Video4Linux backend preparation" are not necessary
	* Regardless of kernel version you need to install the udev rules in the librealsense source folder after librealsense has been installed
		* sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
		* sudo udevadm control --reload-rules && udevadm trigger
* Test the installation by plugging in the camera and running "realsense-viewer" in a terminal
	* This should start a graphical application that streams depth and color data from the camera
	* If there is some error, look at the troubleshooting section in the installation guide, or at the issues posted on the Github page
	
## Point Cloud Library
* Follow instructions on the [PCL homepage](http://www.pointclouds.org/documentation/tutorials/compiling_pcl_posix.php) to compile from source (we have been running 1.8 and 1.8.1, but later versions should work as well)
	* When running cmake also include the build apps flag, this is needed to run parts of the system
		* cmake -DBUILD_apps=ON ..
		
## Java JDK
* sudo apt-get install default-jdk
	* Environment variable JAVA_HOME needs to be set and point to the Java installation directory (this is sometimes done by default)
	
## MATLAB Compiler Runtime (MCR)
* Download and install version 9.2 from the [MATLAB Runtime Compiler page](https://www.mathworks.com/products/compiler/matlab-runtime.html)

# Compiling and running the system
Each part of the system has its own instruction for compiling and running. An overview of all the parts can be found here [LINK NEEDED]. 
