# L2 Examples

It's important to note that certain examples in this lesson make use of the outputs of other examples.

For instance, Example C2-3-3 (and Exercise C2-3-2 before it) use the output of Example C2-3-1.

Otherwise, it is often preferred to comment back out previous examples/exercises so you don't have to cycle through all of the related visualizations.

## Use the Escape Key!

To traverse frames, you may need to use the Escape key to progress frames. In limited instances, closing a given frame visualization by clicking on the X of the window may also traverse frames.

## Special Note

In Example C2-4-3, add `True` as an additional argument to `render_bb_over_bev()` for visualization, but remove it when using Example C2-4-4.


# Exercise C1-5-5

## Files to work in:
- `basic_loop.py` 
- `lesson1-lidar-sensor/exercises/starter/l1_exercises.py`

Implement the `vis_intensity_channel()` function.

## Running the Code

Run `python basic_loop.py` from within the `nd013-c2-fusion-exercises` directory when you have implemented your code.

Note that you'll need to make use of the Desktop window (see button in bottom right of the workspace if working directly in the terminal) to view the visualization output, and use the Escape key to progress through frames.


# Exercise C1-5-2

## Files to work in:
- `basic_loop.py` 
- `lesson1-lidar-sensor/exercises/starter/l1_exercises.py`

Implement the `print_pitch_resolution()` function.

## Running the Code

Run `python basic_loop.py` from within the `nd013-c2-fusion-exercises` directory when you have implemented your code.


# Exercise C1-3-1

## Files to work in: 
- `basic_loop.py` 
- `lesson1-lidar-sensor/exercises/starter/l1_exercises.py`

Now that you are familiar with the starter code, please try to solve the following exercise: 
- In `basic_loop.py`, parameterize the code to load sequence 1 and start the loop with frame 1. 
- Pass the current frame to function `print_no_of_vehicles` in file `l1_exercises.py`. 
- Write code to find out how many objects of type "vehicle" have been provided as ground-truth labels for the LiDAR sensors in this frame and print the result. 

## Running the Code

Run `python basic_loop.py` from within the `nd013-c2-fusion-exercises` directory when you have implemented your code.