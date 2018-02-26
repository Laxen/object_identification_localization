
# distinguish_utility_display
This program is used to graphically display the distinguish-utility values

## Compile and run
Execute the following command in the Distinguish_utility directory to compile the program
```
mkdir build && cd build && cmake .. && make
```
Execute the following command to run the program
```
./distinguish_utility_display <input1> <input2> <options>
```

where \<input1\> is the name of the primary model for which to display the distinguish-utility values and \<input2\> is the secondary (similar) model.

\<options>\ contains one option: "-normalize" which can be used to normalize the distinguish-utility values.   

## Usage
* To view the normalized distinguish-utility values for e.g. example.obj and a similar looking CAD model called similar_model.obj, both located in the CAD_models folder, execute the following command: 
    ```
    ./distinguish_utility_display example similar_model -normalize
    ```
* Note: Do not include the file extensions for the CAD models in the input, just include the model names
* The models must be added before this program can be used (see [add_model](https://github.com/Laxen/object_identification_localization/tree/master/add_model)) 
* A green node/pointcloud represents a high distinguish-utility value and a red node/pointcloud represents a low distinguish-utility value. 
