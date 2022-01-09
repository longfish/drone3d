#ifndef DRONE_OBJECT_H
#define DRONE_OBJECT_H

#include "drone3d.h"

/**
 * @brief A simple class to send the commands to the drone through
 * the corresponding topics
 * @ref This code was modified from the DroneObjectROS object in sjtu-drone 
 * package: https://github.com/tahsinkose/sjtu-drone
 */

class DroneObject
{
protected:
    DroneObject() {}

public:
    DroneObject(ros::NodeHandle &node) { InitROS(node); }

    bool isFlying;
    bool isPositionCtrl;
    bool isVelocityMode;
    ros::Publisher pub_positionctrl;
    ros::Publisher pub_takeoff;
    ros::Publisher pub_land;
    ros::Publisher pub_cmd;
    ros::Publisher pub_velocitymode;
    geometry_msgs::Twist twist_msg;

    bool TakeOff();
    bool Land();
    bool Hover();
    bool MoveTo(float x, float y, float z);
    bool FlyAlongPath(std::vector<std::vector<float>> routes);
    void PositionCtrl(bool on);
    void VelocityMode(bool on);
    void InitROS(ros::NodeHandle &node);
    void PoseCallBack(const geometry_msgs::Pose::ConstPtr &msg);
    void Wait(const float time_s);


};

#endif