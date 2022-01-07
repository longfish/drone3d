#ifndef DRONE3D_H
#define DRONE3D_H

#include <thread>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

#include "drone_object.h"
#include "grid.h"

enum class State
{
    kEmpty,
    kObstacle,
    kClosed,
    kPath,
    kStart,
    kFinish
};

extern float initial_coord[2]; // global variable to store initial node coordinate

#endif