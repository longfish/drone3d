#include "route_planner.h"

RoutePlanner::RoutePlanner(std::vector<State> &grid, float start[2], float end[2])
{
    // Convert 2D Gazebo coordinates to grid:
    // start_x *= 0.01;
    // start_y *= 0.01;
    // end_x *= 0.01;
    // end_y *= 0.01;

    // start_node = &(m_Model.FindClosestNode(start_x, start_y));
    // end_node = &(m_Model.FindClosestNode(end_x, end_y));
}