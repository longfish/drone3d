#include "drone_object.h"

float current_coord[3];

void DroneObject::InitROS(ros::NodeHandle &node)
{
    isFlying = false;
    isPositionCtrl = false;
    isVelocityMode = false;
    pub_takeoff = node.advertise<std_msgs::Empty>("/drone/takeoff", 1024);
    pub_land = node.advertise<std_msgs::Empty>("/drone/land", 1024);
    pub_cmd = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1024);
    pub_velocitymode = node.advertise<std_msgs::Bool>("/drone/vel_mode", 1024);
    pub_positionctrl = node.advertise<std_msgs::Bool>("/drone/posctrl", 1024);
}

void DroneObject::PoseCallBack(const geometry_msgs::Pose::ConstPtr &msg)
{
    current_coord[0] = msg->position.x;
    current_coord[1] = msg->position.y;
    current_coord[2] = msg->position.z;
    ros::spinOnce();
}

bool DroneObject::TakeOff()
{
    if (isFlying)
        return false;
    else
        isFlying = true;

    ROS_INFO("Taking Off...");
    while (abs(current_coord[2] - H_TAKEOFF) > EPS)
    {
        pub_takeoff.publish(std_msgs::Empty());
        ros::spinOnce();
        Wait(0.01);
    }

    return true;
}

bool DroneObject::Land()
{
    if (!isFlying)
        return false;
    else
        isFlying = false;

    ROS_INFO("Landing...");
    while (abs(current_coord[2] - H_LAND) > EPS)
    {
        pub_land.publish(std_msgs::Empty());
        ros::spinOnce();
        Wait(0.01);
    }

    return true;
}

bool DroneObject::Hover()
{
    if (!isFlying)
        return false;

    twist_msg.linear.x = 0;
    twist_msg.linear.y = 0;
    twist_msg.linear.z = 0;
    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;

    ROS_INFO("Hovering...");
    pub_cmd.publish(twist_msg);
    ros::spinOnce();

    return true;
}

void DroneObject::PositionCtrl(bool on)
{
    isPositionCtrl = on;
    std_msgs::Bool bool_msg;
    bool_msg.data = on ? 1 : 0;
    pub_positionctrl.publish(bool_msg);
    if (on)
        ROS_INFO("Switching position control on...");
    else
        ROS_INFO("Switching position control off...");
}

void DroneObject::VelocityMode(bool on)
{
    std_msgs::Bool bool_msg;
    bool_msg.data = on ? 1 : 0;
    pub_velocitymode.publish(bool_msg);
    isVelocityMode = on;
    if (isVelocityMode)
        ROS_INFO("Switching velocity mode on...");
    else
        ROS_INFO("Switching velocity mode off...");
}

bool DroneObject::MoveTo(float x, float y, float z)
{
    if (!isFlying || !isPositionCtrl)
        return false;

    twist_msg.linear.x = x;
    twist_msg.linear.y = y;
    twist_msg.linear.z = z;
    twist_msg.angular.x = 0;
    twist_msg.angular.y = 0;
    twist_msg.angular.z = 0;

    ROS_INFO("Moving...");
    while (abs(current_coord[0] - x) > EPS || abs(current_coord[1] - y) > EPS || abs(current_coord[2] - z) > EPS)
    {
        pub_cmd.publish(twist_msg);
        ros::spinOnce();
        Wait(0.01);
    }

    return true;
}

bool DroneObject::FlyAlongPath(std::vector<std::vector<float>> route)
{
    for (auto site : route)
    {
        MoveTo(site[0], site[1], site[2]);
        ROS_INFO("Current position: (%f, %f, %f)", current_coord[0], current_coord[1], current_coord[2]);
    }
    return true;
}

void DroneObject::Wait(const float time_s)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(int(1000 * time_s)));
}