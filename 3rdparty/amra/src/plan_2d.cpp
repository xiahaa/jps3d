// #include <amra/interface.h>
#include <amra/grid2d.hpp>
#include <amra/helpers.hpp>
#include <chrono>
#include "plan_2d.h"

int plan_2d(
    std::vector<float>& origin,
    std::vector<int>& dim,
    std::vector<signed char>& map_data,
    std::vector<float>& start_f,
    std::vector<float>& goal_f,
    float resolution,
    std::vector<std::vector<double>>& path,
    double& time_spent)
{
    auto start_time = std::chrono::high_resolution_clock::now();

    int width = dim[0];
    int height = dim[1];

    // MOVINGAI_DICT = {
    //     '.': 1,
    //     'G': 1,
    //     '@': -1,
    //     'O': -1,
    //     'T': 0,
    //     'S': 1,
    //     'W': 2,    # water is only traversible from water
    //     '(': 1000, # start
    //     '*': 1001, # path
    //     ')': 1002, # goal
    //     'E': 1003, # expanded state
    // }
    // we need to convert the map data to a format that AMRA can use
    // I use 1 for obstacles, 0 for free space
    // moving AI use . for free space and @ for obstacles
    // so we need to convert the map data to 1 for free space and 0 for obstacles
    // also, we need to convert the start and goal positions to the same format
    for (std::size_t i = 0; i < map_data.size(); ++i)
    {
        if (map_data[i] == 0) // free space
            map_data[i] = 1;
        else
            map_data[i] = -1; // obstacle
    }

    AMRA::Grid2D grid(width, height, map_data);
    grid.CreateSearch();

    int start_x = (start_f[0] - origin[0]) / resolution;
    int start_y = (start_f[1] - origin[1]) / resolution;
    int goal_x = (goal_f[0] - origin[0]) / resolution;
    int goal_y = (goal_f[1] - origin[1]) / resolution;

    grid.SetStart(start_x, start_y);
    grid.SetGoal(goal_x, goal_y);

    std::vector<std::vector<int>> ipath;

    bool success = grid.Plan(ipath);

    if (success)
    {
        for (const auto& icoord : ipath)
        {
            std::vector<double> point;
            point.push_back(icoord[0] * resolution + origin[0]);
            point.push_back(icoord[1] * resolution + origin[1]);
            path.push_back(point);
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    // in miliseconds
    time_spent = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    // in seconds
    // time_spent = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();

    return success ? 1 : 0;
}