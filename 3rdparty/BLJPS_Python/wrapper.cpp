#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include "BL_JPS.h"
#include <vector>

namespace py = pybind11;
using namespace pybind11::literals;

class Timer {
    typedef std::chrono::high_resolution_clock high_resolution_clock;
    typedef std::chrono::milliseconds milliseconds;
    public:
    explicit Timer(bool run = false)
    {
        if (run)
        Reset();
    }
    void Reset()
    {
        _start = high_resolution_clock::now();
    }
    milliseconds Elapsed() const
    {
        return std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - _start);
    }
    private:
    high_resolution_clock::time_point _start;
};

int plan_2d(std::vector<float> &origin, std::vector<int> &dim, std::vector<signed char> &map, std::vector<float> &start, std::vector<float> &goal, float resolution, std::vector<std::vector<double> > &path, double &time_spent, bool use_theta)
{
    std::vector<std::vector<int>> map_grid;
    int height = dim[1];
    int width = dim[0];
    map_grid.resize(height);
    for (int i = 0; i < height; ++i)
    {
        map_grid[i].resize(width);
        for (int j = 0; j < width; ++j)
        {
            map_grid[i][j] = map[i * width + j];
        }
    }
    int start_x = static_cast<int>((start[0] - origin[0]) / resolution);
    int start_y = static_cast<int>((start[1] - origin[1]) / resolution);
    int goal_x = static_cast<int>((goal[0] - origin[0]) / resolution);
    int goal_y = static_cast<int>((goal[1] - origin[1]) / resolution);

    if (start_x < 0 || start_x >= width || start_y < 0 || start_y >= height ||
        goal_x < 0 || goal_x >= width || goal_y < 0 || goal_y >= height)
    {
        return -1; // Invalid start or goal position
    }

    Mission mission;
    int cellSize = resolution;
    if (!mission.getMap(start_x, start_y, goal_x, goal_y, cellSize, map_grid))
    {
        return -1; // Failed to get the map
    }

    if (use_theta)
    {
        Timer time_theta(true);
        mission.setDefaultConfig(true); // Set default configuration for Theta*
        mission.createEnvironmentOptions();
        mission.createSearch();
        mission.startSearch();
        double dt_theta = time_theta.Elapsed().count();
        std::vector<std::vector<int>> path_int;
        mission.getPath(path_int); // Get the path from the mission
        path.clear();
        for (const auto &pt : path_int)
        {
            double x = origin[0] + pt[0] * cellSize;
            double y = origin[1] + pt[1] * cellSize;
            path.push_back({x, y});
        }
        time_spent = dt_theta;
        return mission.getPathValid() ? 0 : -1; // Return 0 if the path is valid
    }
    else{
        Timer time_astar(true);
        mission.setDefaultConfig(false); // Set default configuration for A*
        mission.createEnvironmentOptions();
        mission.createSearch();
        mission.startSearch();
        double dt_astar = time_astar.Elapsed().count();
        std::vector<std::vector<int>> path_int;
        mission.getPath(path_int); // Get the path from the mission
        path.clear();
        for (const auto &pt : path_int)
        {
            double x = origin[0] + pt[0] * cellSize;
            double y = origin[1] + pt[1] * cellSize;
            path.push_back({x, y});
        }
        time_spent = dt_astar;
        return mission.getPathValid() ? 0 : -1; // Return 0 if the path is valid
    }
    return 0;
}