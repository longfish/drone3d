# Drone3D Project

Drone3D is a C++ version of navigation program used for drone. 

<img src="map.png" width="600" height="450" />

## Dependencies 
* Used [sjtu-drone](https://github.com/tahsinkose/sjtu-drone) for controlling, need to install the desktop version of ROS
* Tested in ROS Noetic + Gazebo 11 environment (Ubuntu 20.04)

## Cloning and compiling

```
$ cd <catkin_ws>/src
$ git clone https://github.com/longfish/drone3d.git 
$ cd <catkin_ws>
$ catkin_make
```

## Running

Open the 1st terminal:
```
roscore
```

Open the 2nd terminal:
```
$ cd <catkin_ws>
$ source devel/setup.bash
$ roslaunch drone3d drone3d.launch
```

Open the 3rd terminal:
```
$ cd <catkin_ws>
$ rosrun drone3d simulate
```
