#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(std::vector<std::vector<float>> &route, std::vector<float> &start)
    : unexplored_sites(route), current_site(start)
{
    final_route.push_back(current_site); // push the start site to the final route
    while (!unexplored_sites.empty())
    {
        current_site = FindClosest();
        final_route.push_back(current_site);
    }
    final_route.push_back(start); // return back to the start site
}

// generate a 3D route that a drone can fly
std::vector<std::vector<float>> RoutePlanner::FinalRoute()
{
    std::vector<std::vector<float>> final_route_3d;
    for (int i = 0; i < final_route.size(); i++)
    {
        auto site = final_route[i];
        final_route_3d.push_back(std::vector<float>{site[0], site[1], H_FLIGHT});

        if (i == final_route.size() - 1)
            final_route_3d.push_back(std::vector<float>{site[0], site[1], H_TAKEOFF});
        else if(i != 0)
        {
            final_route_3d.push_back(std::vector<float>{site[0], site[1], H_PAUSE});
            final_route_3d.push_back(std::vector<float>{site[0], site[1], H_FLIGHT});
        }
    }
    return final_route_3d;
}

bool RoutePlanner::Compare(const std::vector<float> &a, const std::vector<float> &b)
{
    float dis1 = CalculateDist(a);
    float dis2 = CalculateDist(b);
    return dis1 > dis2;
}

std::vector<float> RoutePlanner::FindClosest()
{
    sort(unexplored_sites.begin(), unexplored_sites.end(), [this](const std::vector<float> &a, const std::vector<float> &b)
         { return this->Compare(a, b); }); // sort in descending order
    auto curr = unexplored_sites.back();
    unexplored_sites.pop_back(); // remove the closest site

    return curr;
}

float RoutePlanner::CalculateDist(std::vector<float> const &other_site)
{
    float dx = (current_site[0] - other_site[0]);
    float dy = (current_site[1] - other_site[1]);
    return dx * dx + dy * dy;
}