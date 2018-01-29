# system

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
* For every cloud found the system will fetch robot hand position, run the pipeline, and then wait for the next cloud
* When a new cloud is found it will automatically be merged with the previous cloud and the pipeline will be run again
* This continues until the user stops the program using Ctrl-C