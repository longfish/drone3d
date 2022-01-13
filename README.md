# Drone3D 

Drone3D is a C++ drone package used to rescue randomly distributed survivors based on Gazebo and ROS-1. This package adopted the drone controller program of [sjtu-drone](https://github.com/tahsinkose/sjtu-drone) and has beed tested in Ubuntu 18.04/20.04. To get rid of all the dependency issues, a docker image is also created to ease the running of drone3d package.

The package first create a world with a drone landed at the central red plate. Then multiple survivors are generated at random positions. The shortest route for traversing all survivors is calculated using RoutePlanner class. Drone would follow this route to visit each survivor and finally return to its initial position.

<img src="map.png" width="600" height="450" />

## Installation
There are two installation options, i.e., docker and local ones. To get rid of dependency issues, Docker is a better choice to run the package. One need to have a Docker engine in the local machine, follow the instruction on [here](https://docs.docker.com/engine/install/ubuntu/).

### Option-1: Docker
Choose an empty folder, download the Dockerfile and build the docker image (name it as rosdrone3d):
```
wget https://raw.githubusercontent.com/longfish/drone3d/main/Dockerfile
sudo xhost +local:root
sudo docker build -t rosdrone3d .
```
Create a container:
```
sudo docker run --name my_rosdrone -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" rosdrone3d bash
```

**Attention please**: If one is using Nvidia gpu and drivers, nvidia-docker2 should be installed from this [site](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker) for properly running Gazebo. Uncomment the following lines in Dockerfile:
```
#ENV NVIDIA_VISIBLE_DEVICES \
#    ${NVIDIA_VISIBLE_DEVICES:-all}
#ENV NVIDIA_DRIVER_CAPABILITIES \
#    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
```
Build the docker image and then create a container using the following command:
```
sudo xhost +local:root
sudo docker build -t rosdrone3d .
sudo docker run --name my_rosdrone -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --runtime=nvidia rosdrone3d bash
```

Following all the above steps, the user will then be in `/home/catkin_ws#` with all dependencies installed.

### Option-2: Local environment (Ubuntu 18.04)
**Note**: use ROS-Melodic and Gazebo-9, otherwise there cause some issues. The current package is not guaranteed to work on older versions of Ubuntu.

* ROS melodic **Desktop** install (not the **Desktop-Full**), follow the instructions from [here](http://wiki.ros.org/melodic/Installation/Ubuntu).
* Environment setup: 
``` 
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
* Remove the previously installed gazebo, otherwise it will interfere with the current environment.
```
$ sudo apt-get remove ros-ROS_DISTRO-gazebo* # such as ros-kinetic-gazebo*
$ sudo apt-get remove libgazebo*
$ sudo apt-get remove gazebo*
$ sudo apt autoclean && sudo apt autoremove
```
* Install Gazebo-9 follow the instructions from [here](http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=9.0)
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo9
sudo apt-get install libgazebo9-dev
```
* Install gazebo-ros packages
```
sudo apt-get install ros-melodic-gazebo-ros ros-melodic-gazebo-ros-pkgs  ros-melodic-gazebo-ros-control
```
* Install catkin
```
sudo apt-get install python3-catkin-tools python3-osrf-pycommon
```
* Create a workspace (anywhere you like, e.g., `~/catkin_ws`, but should keep the same for the current installation):
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
```

## Cloning and building
**Note**: the following should work in both the docker and local environment, we prefer docker.

* Build sjtu-drone in the above workspace (must be the same as the one created in previous section):
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/tahsinkose/sjtu-drone.git
$ cd ~/catkin_ws
$ catkin_make
```
* Build the drone3d package in the workspace:
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/longfish/drone3d.git 
$ cd ~/catkin_ws
$ source devel/setup.bash
$ catkin_make
```

## Running
**Note**: if one is using docker as environment, first create two terminals and following this
```
sudo docker start my_rosdrone
sudo docker exec -it my_rosdrone bash
```

Launch the Gazebo world in the 1st terminal:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch drone3d drone3d.launch
```

Simulate the survivor behavior and drone in the 2nd terminal:
```
$ cd ~/catkin_ws
$ source devel/setup.bash
$ rosrun drone3d simulate
```

## Program structure

This package contains a Gazebo world plugin i.e., `SurvivorsPlugin`, and two C++ classes, i.e., `DroneObject` and `RoutePlanner`. It also contains `launch/drone3d.launch`, `urdf/sjtu_drone.urdf` and `worlds/survivors.world`, which are required components for simulating objects in Gazebo environment. 

In `SurvivorsPlugin`, a transport node is created to subscribe the survivor position message under the `/survivors/pose` topic published by the node in main() in `simulate.cpp`. After this message is published, it would then be subscribed by Gazebo to create the survivors in the simulated world. 

A ROS nodehandle is created in main() to create drones and control their behavior. The `DroneObject` class is created to ease the control of these drones. A shorest route that connect all survivors can be generated by the `RoutePlanner` class. 

## Rubric points

Following the Rubric points described in https://review.udacity.com/#!/rubrics/2533/view. Here only five criteria are listed to satisfy the requirements of Udacity.

* `Loops, Functions, I/O::The project demonstrates an understanding of C++ functions and control structures`: The package involves for... and while... loops and conditional expressions like if... structures. Expressions are also encapsulated into functions like the line-4 in simulate.cpp and the lambda like line-45 in route_planner.cpp.
* `Object Oriented Programming::The project uses Object Oriented Programming techniques`: The package is divided into several classes. 
* `Object Oriented Programming::Classes use appropriate access specifiers for class members`: Proper access specifiers are used in header files like `drone_object.h` and `route_planner.h`.
* `Object Oriented Programming::Class constructors utilize member initialization lists`: line-5 in route_planner.cpp.
* `Memory Management::The project makes use of references in function declarations`: e.g., line-4, line-36 in route_planner.cpp.
