# background_creator
This program is used for creating background clouds that can be used to improve segmentation. When the full system is run and a point cloud of an object is captured, this background cloud can be subtracted to easily remove most of the background. This leaves only the objects of interest left in the cloud, which will improve identification and localization, as well as speed up the process. This is highly recommended to get good results!

## Compile and run
Execute the following command in the system directory to compile the program
```
mkdir build && cd build && cmake .. && make
```
Execute the following command to run the program
```
./background_creator
```

## Usage
* When the program is executed it will wait for clouds in the save path directory specified in the config.ini file
* When a cloud is found the robot hand positional data will be fetched and the cloud can be visualized in the viewer
* When a new cloud is captured, press Q in the viewer to automatically merge and visualize the new cloud
* This can be used to capture and merge multiple clouds of the background 
* When you are done, press Ctrl-C to exit the program, the finished background cloud is found in the build folder
* Put background.pcd and background.csv in the save path directory to allow the system to find it
