#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <cmath>
#include "drone3d.h"

class RoutePlanner
{
public:
    RoutePlanner(std::vector<State> &grid, float start[2], float end[2]);

    void AStarSearch();
    void AddNeighbors(std::vector<int> current_node);
    float CalculateHValue(std::vector<int> const current_node);
    std::vector<int> NextNode();
    std::vector<std::vector<int>> routes;

private:
    // directional deltas
    const int delta[6][3]{{-1, 0, 0}, {1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}};

    std::vector<std::vector<int>> open_list;
    std::vector<int> start_node;
    std::vector<int> end_node;
};

#endif