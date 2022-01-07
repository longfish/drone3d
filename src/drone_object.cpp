#include "drone_object.h"

void DroneObject::InitROS(ros::NodeHandle &node)
{
    init_height = 55;
    flight_level_diff = 10;
    isFlying = false;
    isPositionCtrl = false;
    isVelocityMode = false;
    pub_takeoff = node.advertise<std_msgs::Empty>("/drone/takeoff", 1);
    pub_land = node.advertise<std_msgs::Empty>("/drone/land", 1);
    pub_cmd = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1024);
    pub_velocitymode = node.advertise<std_msgs::Bool>("/drone/vel_mode", 1024);
    pub_positionctrl = node.advertise<std_msgs::Bool>("/drone/posctrl", 1024);
}

void DroneObject::PoseCallBack(const geometry_msgs::Pose::ConstPtr &msg)
{
    float current_coord[3];
    current_coord[0] = msg->position.x;
    current_coord[1] = msg->position.y;
    current_coord[2] = msg->position.z;
    ROS_INFO("Current position: %f, %f, %f", current_coord[0], current_coord[1], current_coord[2]);
}

bool DroneObject::TakeOff()
{
    if (isFlying)
        return false;
    else
        isFlying = true;

    pub_takeoff.publish(std_msgs::Empty());
    ROS_INFO("Taking Off...");
    return true;
}

bool DroneObject::Land()
{
    if (!isFlying)
        return false;
    else
        isFlying = false;

    pub_land.publish(std_msgs::Empty());
    ROS_INFO("Landing...");
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

    pub_cmd.publish(twist_msg);
    ROS_INFO("Hovering...");
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

    pub_cmd.publish(twist_msg);
    ROS_INFO("Moving...");
    return true;
}

bool DroneObject::FlyAlongPath(std::vector<std::vector<int>> path_node, int wait_time)
{
    for (auto path : path_node)
    {
        std::vector<float> coord_Gazebo = CoordN2G(path);
        MoveTo(1, 2, 3); // move to a point

        std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));
    }
    return true;
}

// convert the coordinates between nodegrid and Gazebo
std::vector<int> DroneObject::CoordG2N(float xG, float yG)
{
    return std::vector<int>{0, int(std::round((-yG + 97.5) / 5.0)), int(std::round((xG + 97.5) / 5.0))};
}

std::vector<float> DroneObject::CoordN2G(std::vector<int> coordN)
{
    std::vector<float> coordG{};
    coordG.push_back(float(coordN[2]) * 5 - 97.5);                        //x
    coordG.push_back(float(coordN[1]) * (-5) + 97.5);                     //y
    coordG.push_back(float(coordN[0]) * flight_level_diff + init_height); //z
    return coordG;
}

void DroneObject::SpendTime(const int time_s)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 * time_s));
}