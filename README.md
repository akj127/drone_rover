# Robotics-Project

## Dependencies

- ROS-melodic
- Gazebo
- SITL
- Ardupilot-gazebo link
- [The Husky Rover gazebo package](http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky)

Install Ardupilot from [here](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux)

Setup ardupilot-gazebo link from [here](https://github.com/khancyr/ardupilot_gazebo)

## Setup catkin workspace

To start working with ros, you need to setup ROS workspace using catkin. Open up a terminal to setup the base directory.

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

Once done, clone this repo inside catkin_ws/src,

```bash
cd ~/catkin_ws/src
git clone https://github.com/akj127/Robotics-project
```

## Import models

### Add the Open Gazebo Models Database

Use git to get a bunch of open source gazebo models from the Open Source Robotics Foundation (OSRF) 

Also, add the models directory to the `GAZEBO_MODEL_PATH`

```bash
cd ~/
git clone https://github.com/osrf/gazebo_models.git
echo 'export GAZEBO_MODEL_PATH=~/gazebo_models:${GAZEBO_MODEL_PATH}' >> ~/.bashrc
source ~/.bashrc
```

### Import the new Husky model (with the Aruco marker)

The new Husky model can be found in the `models` directory of this repo. Simply replace the original Husky model with this updated model. To do that you would need sudo priveleges

```bash
sudo cp ~/catkin_ws/src/Robotics-project/models/tagged_husky/husky.urdf.xacro $(catkin_find husky_description/urdf)/
```

Model taken from [here](https://github.com/mzahana/mavros_apriltag_tracking)

## Initiate animation in gazebo

Open a new terminal and enter following command,

```bash
export GAZEBO_RESOURCE_PATH=~/catkin_ws/src/drone_rover/worlds/
roslaunch husky_gazebo husky_empty_world.launch world_name:=robo.world
```

Now gazebo should be launched, and a world consisting of rover, drone should be visible
open new terminal,

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
```

## Start Tracking
In a new terminal (python 2.7) \\
The ports are generally either `14550` or `14551`.
```bash
cd ~/catkin_ws/src/drone_rover/dronekit/
python drone_move.py --connect 127.0.0.1:14550
```

## To view the Tracking
In a new terminal 
```bash
rqt
```

## Plotting the Errors
In a new terminal (python 3.5+)
```bash
cd ~/catkin_ws/src/drone_rover/dronekit/
python3 plot_error.py
```
