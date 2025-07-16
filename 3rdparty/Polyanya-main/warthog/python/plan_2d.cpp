#include "plan_2d.h"

#include "flexible_astar.h"
#include "gridmap.h"
#include "gridmap_expansion_policy.h"
#include "jps_expansion_policy.h"
#include "jps2_expansion_policy.h"
#include "jpsplus_expansion_policy.h"
#include "jps2plus_expansion_policy.h"
#include "octile_heuristic.h"
#include "timer.h"
#include "weighted_gridmap.h"
#include "wgridmap_expansion_policy.h"

#include <iostream>
#include <set>
#include <string>

int
warthog::plan_2d(std::vector<float> &origin, std::vector<int> &dim,
                std::vector<signed char> &map_data, std::vector<float> &start_f,
                std::vector<float> &goal_f, float resolution,
                std::vector<std::vector<double> > &path, double &time_spent,
                std::string &algorithm)
{
    if (dim.size() != 2)
    {
        std::cerr << "plan_2d: dim must have 2 elements" << std::endl;
        return 1;
    }

    // Add more supported algorithms
    const std::set<std::string> supported_algorithms = {"astar", "jps", "jps2", "jps+", "jps2+"};

    if (supported_algorithms.find(algorithm) == supported_algorithms.end())
    {
        std::cerr << "plan_2d: unsupported algorithm: " << algorithm << std::endl;
        return 1;
    }

    int width = dim[0];
    int height = dim[1];

    if (map_data.size() != (size_t)(width * height))
    {
        std::cerr << "plan_2d: map_data size does not match dim" << std::endl;
        return 1;
    }

    warthog::timer mytimer;
    mytimer.start();

    // Select map type
    warthog::gridmap map(height, width);
    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            bool traversable = map_data[y * width + x] == 0;
            map.set_label(map.to_padded_id(x, y), traversable);
        }
    }

    int start_x = (start_f[0] - origin[0]) / resolution;
    int start_y = (start_f[1] - origin[1]) / resolution;
    int goal_x = (goal_f[0] - origin[0]) / resolution;
    int goal_y = (goal_f[1] - origin[1]) / resolution;

    uint32_t start_id = map.to_padded_id(start_x, start_y);
    uint32_t goal_id = map.to_padded_id(goal_x, goal_y);

    warthog::octile_heuristic heuristic(width, height);

    std::stack<uint32_t> path_ids;

    // Algorithm selection
    if (algorithm == "jps")
    {
        warthog::jps_expansion_policy jps_expander(&map);
        warthog::flexible_astar<warthog::octile_heuristic, warthog::jps_expansion_policy> astar(&heuristic, &jps_expander);
        path_ids = astar.get_path(start_id, goal_id);
    }
    else if (algorithm == "jps+")
    {
        warthog::jpsplus_expansion_policy jpsplus_expander(&map);
        warthog::flexible_astar<warthog::octile_heuristic, warthog::jpsplus_expansion_policy> astar(&heuristic, &jpsplus_expander);
        path_ids = astar.get_path(start_id, goal_id);
    }
    else if (algorithm == "jps2+")
    {
        warthog::jps2plus_expansion_policy jps2plus_expander(&map);
        warthog::flexible_astar<warthog::octile_heuristic, warthog::jps2plus_expansion_policy> astar(&heuristic, &jps2plus_expander);
        path_ids = astar.get_path(start_id, goal_id);
    }
    else if (algorithm == "jps2")
    {
        warthog::jps2_expansion_policy jps2_expander(&map);
        warthog::flexible_astar<warthog::octile_heuristic, warthog::jps2_expansion_policy> astar(&heuristic, &jps2_expander);
        path_ids = astar.get_path(start_id, goal_id);
    }
    else if (algorithm == "astar")
    {
        warthog::gridmap_expansion_policy expander(&map);
        warthog::flexible_astar<warthog::octile_heuristic, warthog::gridmap_expansion_policy> astar(&heuristic, &expander);
        path_ids = astar.get_path(start_id, goal_id);
    }

    mytimer.stop();
    time_spent = mytimer.elapsed_time_micro();

    if (path_ids.empty())
    {
        return 1; // No path found
    }

    while (!path_ids.empty())
    {
        uint32_t pid = path_ids.top();
        path_ids.pop();
        uint32_t x, y;

        map.to_unpadded_xy(pid, x, y);

        std::vector<double> point;
        point.push_back(origin[0] + x * resolution);
        point.push_back(origin[1] + y * resolution);
        path.push_back(point);
    }

    return 0;
}