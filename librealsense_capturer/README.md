# Librealsense Capturer

Program for saving pointclouds as PCD files using librealsense, tested on SR300 and D435. Color is mapped to depth which results in some points not having any color data outside of the RGB cameras FOV. This program has a reset-loop to fix the librealsense stream synchronization issues explained in https://github.com/IntelRealSense/librealsense/issues/840.

To compile and run create a folder named "build" in the main folder, and execute the following command from that folder.
```
cmake ..; make
./librealsense_capturer
```
