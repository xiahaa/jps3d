#ifndef INTERFACE_HPP
#define INTERFACE_HPP

#include <vector>

int plan_2d(
    std::vector<float>& origin,
    std::vector<int>& dim,
    std::vector<signed char>& map_data,
    std::vector<float>& start_f,
    std::vector<float>& goal_f,
    float resolution,
    std::vector<std::vector<double>>& path,
    double& time_spent);

#endif  // INTERFACE_HPP