#!/usr/bin/env bash

export GAZEBO_RESOURCE_PATH=~/catkin_ws/src/drone_rover/worlds/
gnome-terminal -- roslaunch husky_gazebo husky_empty_world.launch world_name:=robo.world
gnome-terminal -- sim_vehicle.py -v ArduCopter -f gazebo-iris 
sleep 30s
cd ~/catkin_ws/src/drone_rover/dronekit/ && gnome-terminal -- python drone_move.py --connect 127.0.0.1:14550
gnome-terminal -- rqt
