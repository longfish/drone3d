#include "drone3d.h"

// TODO: change the gazebo_2Dmap_plugin to allow message passing, this is
// to get the 2D occupied map without restart the world.

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulate_drone");
    ros::NodeHandle nh;
    DroneObject drone(nh);

    // get the initial position of drone
    ros::Subscriber sub = nh.subscribe("/drone/gt_pose", 1024, &DroneObject::PoseCallBack, &drone);

    drone.VelocityMode(true);                                     // switching velocity mode on
    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // sleep for 2000ms

    drone.TakeOff();
    std::this_thread::sleep_for(std::chrono::milliseconds(5000)); // sleep for 5000ms

    // send the drone some routes
    std::vector<std::vector<int>> path_node{};
    path_node.push_back(std::vector<int>{1,2,3});
    drone.PositionCtrl(true);                                     // switching position control on
    drone.FlyAlongPath(path_node, 5000);
    drone.Land(); // landing

    ros::spin();

    return 0;
}