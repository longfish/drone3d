#include "drone3d.h"
#include "route_planner.h"

// TODO: change the gazebo_2Dmap_plugin to allow message passing, this is
// to get the 2D occupied map without restart the world.

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulate_drone");
    ros::NodeHandle nh;
    DroneObject drone(nh);

    // get the real-time position of drone
    ros::Subscriber sub = nh.subscribe("/drone/gt_pose", 1024, &DroneObject::PoseCallBack, &drone);

    drone.VelocityMode(true); // switching velocity mode on
    drone.TakeOff();          // drone take off
    drone.PositionCtrl(true); // switching position control on

    // send to drone some routes
    std::vector<std::vector<int>> routes{};
    routes.push_back(std::vector<int>{1, 2, 3});
    routes.push_back(std::vector<int>{5, 20, 30});
    routes.push_back(std::vector<int>{40, 20, 80});
    routes.push_back(std::vector<int>{40, 20, 60});
    routes.push_back(std::vector<int>{40, 20, 20});
    routes.push_back(std::vector<int>{20, 20, 20});
    routes.push_back(std::vector<int>{10, 20, 5});
    routes.push_back(std::vector<int>{0, 0, 5});
    routes.push_back(std::vector<int>{0, 0, 2});

    drone.FlyAlongPath(routes);
    drone.Land();

    ROS_INFO("Landing position: (%f, %f, %f)", current_coord[0], current_coord[1], current_coord[2]);

    return 0;
}