#! /bin/bash
# This script safely launches ros nodes with buffer time to allow param server population
x-terminal-emulator -e roslaunch pr2_pickplace pick_place_demo.launch & sleep 10 &&
x-terminal-emulator -e roslaunch pr2_moveit_newurdf pr2_moveit_newurdf.launch & sleep 20 &&
x-terminal-emulator -e rosrun pr2_pickplace pr2_motion
