
# feature_utility_display
This program is used to graphically display the feature-utility values

## Compile and run
Execute the following command in the Feature_utility directory to compile the program
```
mkdir build && cd build && cmake .. && make
```
Execute the following command to run the program
```
./feature_utility_display <input>
```

where \<input\> is the name of the model for which to display the feature-utility values

## Usage
* To view the feature-utility values for e.g. the example.obj CAD model located in the CAD_models folder, Execute the following command to run the program 
    ```
    ./feature_utility_display example
    ```
* Note: Do not include the file extension for the CAD model in the input, just include the model name
* The model must be added before this program can be used (see [add_model](https://github.com/Laxen/object_identification_localization/tree/master/add_model)) 
* A green node/pointcloud represents a high feature-utility value and a red node/pointcloud represents a low feature-utility value. 
