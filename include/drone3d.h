#ifndef DRONE3D_H
#define DRONE3D_H

#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include "drone_object.h"
#include "grid.h"
//#include "route_planner.h"

enum class State
{
    kEmpty,
    kObstacle,
    kClosed,
    kPath,
    kStart,
    kFinish
};

extern float current_coord[3]; // global variable to store initial positions

#endif