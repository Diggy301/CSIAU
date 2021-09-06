# Hackathon
Disruptive Hackathon Competition

In here are the script for the different stages

Stage 1: give a set of waypoints and the robot should reach them (with some tolerance)  
Stage 2: move robot without touching walls  
Stage 3: move robot to gas source  


HOW TO RUN:  
to run sim:  
$ cd ~/catkin_ws/src  
$ rosrun stage_ros stageros stageDisruptive/gas.world  
  
  
to run gas:  
$ cd ~/catkin_ws  
$ rosrun stageControl task03.py  

run main script:  
$ rosrun stageControl csiau_code.py
