# system
This is the entire system that runs the complete pipeline involving object identification, localization, point cloud merging and hint generation. The results are visualized and saved in the data/results/ folder, as well as optionally (specified in the config.ini file) uploaded to an RDF database. The connection to the RDF database is handled by Java code, which is accessible through C++ code using [JNI](https://docs.oracle.com/javase/8/docs/technotes/guides/jni/). The Java code is stored in a jar file in the "headers" directory. This Java code can be found in [this repository](https://github.com/Andreas93S/rdf4j_project), it can be changed and recompiled into a jar file and then used in this project. 

## Compile and run
Execute the following command in the system directory to compile the program
```
mkdir build && cd build && cmake .. && make
```
Execute the following command to run the program
```
./system
```

## Usage
* Set the point cloud save path in the config file ('save_path' variable in config.ini in main directory)
* When running, the system will use point clouds from this directory
	* If background.pcd and background.csv is found in this folder, it will be used in the segmentation
* For every cloud found the system will fetch robot hand position, run the pipeline, and then wait for the next cloud
* When a new cloud is found it will automatically be merged with the previous cloud and the pipeline will be run again
* This continues until the user stops the program using Ctrl-C
