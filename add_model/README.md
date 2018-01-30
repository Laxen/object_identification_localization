# add_model
Adds a CAD model to the model base. This is needed for the system to be able to recognize and estimate 6DOF poses for the given models.

## Compile and run
Execute the following command in the add_model directory to compile the program
```
mkdir build && cd build && cmake .. && make
```
Execute the following command to run the program
```
./offline_data_generation <input>
```
where \<input\> is an input file of a CAD model. The supported types are OBJ, PLY and STL. 

## Usage
*  Place the CAD models inside the CAD_models folder located in the root directory for this project.
*  If you e.g. wish to add a model called "example.obj", place the example.obj file inside the CAD_models folder and
execute the following command to run the program
    ```
    ./offline_data_generation example.obj
    ```
* Edit the config.ini file in the root directory for this project in order to change the configuration parameters when adding a new model.
* 
