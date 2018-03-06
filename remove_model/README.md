
# remove_model
Removes a model from model_data. Note that only the data in model_data is removed, not the CAD model in CAD_models.

## Compile and run
Execute the following command in the remove_model directory to compile the program
```
mkdir build && cd build && cmake .. && make
```
Execute the following command to run the program
```
./remove_model <input>
```
where \<input\> is the name of the model without file extensions.

## Usage
*  If you e.g. wish to remove a model called "example", execute the following command:
    ```
    ./remove_model example
    ```
