# Exercises Multi-Target Tracking

Here are the instructions for the MTT exercises.

## 1_initialization.py

In this exercise, you will implement a track initialization based on an unassigned measurement. 

### Your Task

In the `__init__()` function of the Track class, please initialize the 6x1 numpy matrix `x` and the 6x6 numpy matrix `P` based on the 
measurement input `meas`. The measurement input is an instance of the `Measurement` class. The `Measurement` class already contains the 
correct transformation matrix including rotation and translation from sensor to vehicle coordinates.

### Desired Result

If you run the script, the left plot contains the test measurement in sensor coordinates. We want to use this measurement to initialize a new 
track. The second plot shows the initialized track. Finally, the last plot shows the measurement converted to vehicle coordinates and the 
initialized track. If you have implemented everything correctly, the track should be initialized where the measurement lies, so the blue and 
red marker should align.

## 2_fov.py

In this exercise, you will implement a sensor visibility check.

### Your Task

In the `Camera` class, please implement the function `in_fov()`. It takes the current state x as an input and should return true if x lies in the camera's field of view, otherwise it should return false. The `Camera` class contains a given field of view `fov` that you should use. Don't forget to transform from vehicle to sensor coordinates with the given transformation matrices.

### Desired Result

If you run the script, you can see the opening angle of the camera in blue, so the visible range is in the area between the blue lines. The script generates random points and calculates whether the points lie inside the field of view. If you have implemented everything correctly, the result should be `False` for all red points outside the field of view, but the visible points in between the blue lines should be green and return `True`. 