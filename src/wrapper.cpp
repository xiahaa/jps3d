#include <jps_basis/data_utils.h>
#include <jps_basis/data_type.h>
#include <jps_planner/jps_planner/jps_planner.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <chrono>

using namespace JPS;

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

int plan_2d(std::vector<float> &origin, std::vector<int> &dim, std::vector<signed char> &map, std::vector<float> &start, std::vector<float> &goal, float resolution, std::vector<std::vector<double> > &jps_path, std::vector<std::vector<double> > &astar_path)
{
    // store map in map_util
    std::shared_ptr<OccMapUtil> map_util = std::make_shared<OccMapUtil>();

    Vec2f origin_f(origin[0], origin[1]);
    Vec2i dim_i(dim[0], dim[1]);
    map_util->setMap(origin_f, dim_i, map, resolution);

    const Vec2f start_pt(start[0], start[1]);
    const Vec2f goal_pt(goal[0], goal[1]);

    // Declare a planner
    std::unique_ptr<JPSPlanner2D> planner_ptr(new JPSPlanner2D(false)); // false for not using JPS
    planner_ptr->setMapUtil(map_util); // Set collision checking function
    planner_ptr->updateMap();
    Timer time_jps(true);
    bool valid_jps = planner_ptr->plan(start_pt, goal_pt, 1, true); // Plan from start to goal using JPS
    double dt_jps = time_jps.Elapsed().count();
    const auto path_jps = planner_ptr->getRawPath(); // Get the planned raw path from JPS
    printf("JPS Planner takes: %f ms\n", dt_jps);
    printf("JPS Path Distance: %f\n", total_distance2f(path_jps));
    Timer time_astar(true);
    bool valid_astar = planner_ptr->plan(start_pt, goal_pt, 1, false); // Plan from start to goal using A*
    double dt_astar = time_astar.Elapsed().count();
    const auto path_astar = planner_ptr->getRawPath(); // Get the planned raw path from A*
    printf("AStar Planner takes: %f ms\n", dt_astar);
    printf("AStar Path Distance: %f\n", total_distance2f(path_astar));

    // Store the paths in output vectors
    jps_path.clear();
    astar_path.clear();
    for (const auto &pt : path_jps)
        jps_path.push_back({pt(0), pt(1)});
    for (const auto &pt : path_astar)
        astar_path.push_back({pt(0), pt(1)});

    return 0;
}