#include "drone3d.h"
#include "route_planner.h"

std::vector<std::vector<float>> RandPositionGenerator(int n_survivors)
{
    // the world is in the range of [-30, 30]
    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<float> distr(-30, 30);
    std::vector<std::vector<float>> rand_p;
    for (int n = 0; n < n_survivors; n++)
        rand_p.push_back({distr(e2), distr(e2)});

    return rand_p;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulate_drone");
    ros::NodeHandle n;

    ////////////////////////////////////////////////////////////////////////////////
    // randomly generate survivors
    int n_survivors(10);
    std::vector<std::vector<float>> survivors_pos = RandPositionGenerator(n_survivors);

    ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/survivors/pose", 1024);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    ros::spinOnce();

    for (int i = 0; i < n_survivors; i++)
    {
        auto pos = survivors_pos[i];
        std_msgs::Float64MultiArray msg;
        msg.data.push_back(pos[0]);
        msg.data.push_back(pos[1]);

        pub.publish(msg);
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    ROS_INFO("Survivors posed");

    ////////////////////////////////////////////////////////////////////////////////
    // create a drone object
    DroneObject drone(n);

    // subscribe the real-time position of drone
    ros::Subscriber sub = n.subscribe("/drone/gt_pose", 1024, &DroneObject::PoseCallBack, &drone);

    std::vector<float> start_site{current_coord[0], current_coord[1]};
    RoutePlanner rp(survivors_pos, start_site);
    std::vector<std::vector<float>> final_route = rp.FinalRoute();

    drone.VelocityMode(true);        // switching velocity mode on
    drone.TakeOff();                 // drone take off
    drone.PositionCtrl(true);        // switching position control on
    drone.FlyAlongPath(final_route); // send the route to drone
    drone.Land();                    // landing

    ROS_INFO("Landing position: (%f, %f, %f)", current_coord[0], current_coord[1], current_coord[2]);

    return 0;
}