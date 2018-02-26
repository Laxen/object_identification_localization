# object_identification_debugger
This program enables the user to debugg the object identification subsystem.

## Compile and run
Execute the following command in the object_identification_debugger directory to compile the program
```
mkdir build && cd build && cmake .. && make
```
Execute the following command to run the program
```
./object_identification_debugger <scene> <cluster>
```
where \<scene\> is the scene index and \<cluster>\ is the cluster index (<cluster> is only optional)

## Usage
*  Run the [system](https://github.com/Laxen/object_identification_localization/tree/master/system) in order to generate identification data. 
*  If you e.g. wish to debugg scene 0, execute the following command:
    ```
    ./object_identification_debugger 0
    ```
   This will iterate through all the clusters in the scene.  
*  If you e.g. wish to debugg scene 0 and cluster 1, execute the following command:
```
./object_identification_debugger 0 1
```
* The first window shows the view-graph with the identified model in the center. The green node indicates the identified viewpoint. The second window shows the point cloud from the scene in the left image and the identified rendered view point cloud in the right image. In the terminal each model will have its 10 best matcing views with corresponding score. A low score (0.05-0.2) indicates a good match. 

