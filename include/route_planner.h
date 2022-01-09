#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <cmath>
#include "drone3d.h"

class RoutePlanner
{
public:
    RoutePlanner(std::vector<std::vector<float>> &route, std::vector<float> &start);
    std::vector<std::vector<float>> FinalRoute();

private:
    std::vector<float> FindClosest();
    float CalculateDist(std::vector<float> const &other_site);
    bool Compare(const std::vector<float> &a, const std::vector<float> &b);

    std::vector<std::vector<float>> unexplored_sites;
    std::vector<std::vector<float>> final_route;
    std::vector<float> current_site;
};

#endif