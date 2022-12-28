#!/bin/bash

./planner/spiral_planner&
sleep 1.0
python3 simulatorAPI.py
