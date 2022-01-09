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
#include <std_msgs/Float64MultiArray.h>

#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/gazebo_client.hh>

#include "drone_object.h"
//#include "route_planner.h"

#define EPS 0.1            // position tolerance
#define H_TAKEOFF 0.5 // drone height after take off
#define H_FLIGHT 5    // drone height during flight
#define H_PAUSE 2.5    // drone height when stop at the sky
#define H_LAND 0.0    // drone height after land

extern float current_coord[3]; // global variable to store initial positions

#endif