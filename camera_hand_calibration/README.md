# camera_hand_calibration
This program is used to find the unknown camera-to-hand transformation matrix (for more details abot this, see our master's thesis report [LINK NEEDED]). The program works by using point picking in multiple clouds (three clouds are recommended) taken from different angles of an environment with lots of varying geometry. By picking a couple of corresponding points in these clouds (6+ recommended) the unknown transformation can be computed. Note that this only has to be done once as long as the camera doesn't move in relation to the robot hand! A good calibration is crucial to get good results when running the system, below is an image showing an example of good calibration clouds.

![Point picking example](https://github.com/Laxen/object_identification_localization/blob/master/docs/images/pp.png)
These three clouds are taken from different angles, and the environment they capture has varying geometry that makes it easy to pick corresponding points. As a rule of thumb the geometry of the environment should be varied enough such that not many points lie in the same plane. 

## Compile and run
Execute the following command in the system directory to compile the program
```
mkdir build && cd build && cmake .. && make
```
Execute the following command to run the program
```
./camera_hand_calibration pcd_paths... [options]
```
pcd_paths are paths to the point clouds that should be used in the calibration (at least 3 clouds recommended).
The only option available is the following
> -l <saved_points> 
Where <save_points> are paths to saved point data files (need to be the same number of files as the number of clouds).

## Usage
* To get a good calibration, an environment with lots of varying geometry is needed. 
* Capture three (or more) point clouds of this environment from different angles. 
* Run the camera_hand_calibration program and pass the path to these point clouds as argument, as explained above. 
	* A window showing the three point clouds side by side will show up. 
* Select corresponding points in the different cloud starting from the left-most cloud and moving to the right. 
	* After each picked point the program will ask if the point is OK, press 'y' or 'n' followed by Enter in the terminal to continue. 
	* Pick 6+ points for best result. 
* When all points have been picked, press Q in the viewer to start the calibration. 
	* Before the calibration starts the program will ask if you want to save the points, type 'y' or 'n' followed by Enter in the terminal to continue. 
	* The program will compute the calibration using a MATLAB script and show the merged clouds in the viewer. 
* Verify that the clouds have been merged correctly and press Q in the viewer to exit.
* The calibration is now done and a calibration file is saved as a file with name "T_CtoH" in data/calibration_results
