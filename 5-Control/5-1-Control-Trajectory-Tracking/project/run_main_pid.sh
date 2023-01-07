#!/bin/bash

./pid_controller/pid_controller&
sleep 1.0
python3 simulatorAPI.py
