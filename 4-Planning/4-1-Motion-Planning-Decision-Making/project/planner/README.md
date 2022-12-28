Cubic Spiral Path Planner Integration
========================================

This example creates a C++ server that connects to a python Carla API client.
The output is visualizing the cubic spirals form the motion planner inside Carla

Compile
---------------

'cd project/solution_cubic_spirals_integrated'
'cmake .'
'make'

Run
------------
launch Carla simulator in new window


'cd project/solution_cubic_spirals_integrated'
'./spiral_planner'

Open new window

'cd project'
'python3 simulatorAPI.py'

Camera Control for spectator window from carla server script

key listener is from python script window, then use arrow keys to change camera angle
and use w/s to change camera zoom

IF rpclib is not a populated folder can get the source files with
git clone https://github.com/carla-simulator/rpclib.git
