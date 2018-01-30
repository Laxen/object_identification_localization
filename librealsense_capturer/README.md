# librealsense_capturer

Program for saving pointclouds as PCD files using librealsense, tested on SR300 and D435. Color is mapped to depth which results in some points not having any color data outside of the RGB cameras FOV (they become white points). This program has a reset-loop to fix the librealsense stream synchronization issues explained in https://github.com/IntelRealSense/librealsense/issues/840.

## Compile and run
Execute the following command in the system directory to compile the program
```
mkdir build && cd build && cmake .. && make
```
Execute the following command to run the program
```
./librealsense_capturer
```

## Usage
* The program will show a window with the depth/color stream from the depth camera
  * This window supports basic rotation and zoom to allow for easier visualization of the data
* Press Enter to capture the cloud, it will be saved as "0.pcd" at the save_path specified in config.ini in the main folder
* Each time a new cloud is captured, the name of the cloud (index) will be incremented to allow for capturing multiple clouds without closing the program
