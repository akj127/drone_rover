# Robotics-Project

Firstly install ROS melodic,gazebo, SITL,ardupilot-gazebo link


Now install Husky gazebo(the rover gazebo package) using, (http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky)

# Setup catkin ws
To start working with ros, you need to setup ROS workspace using catkin.
```
open terminal
mkdir -p catkin_ws/src
cd catkin_ws
catkin_make
```

once done, clone this repo inside catkin_ws/src,
`git clone https://github.com/akj127/Robotics-project`

# Initiate animation in gazebo

open new terminal and enter following command,
```
export GAZEBO_RESOURCE_PATH=~/catkin_ws/src/iq_sim/worlds/
roslaunch husky_gazebo husky_empty_world.launch world_name:=robo.world
```
Now gazebo should be launched, and a world consisting of rover, drone should be visible
open new terminal,
```
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```
