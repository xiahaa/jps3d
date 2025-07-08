#include "timer.hpp"
#include "read_map.hpp"
#include "wrapper.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <vector>
#include <iostream>
#include <jps_basis/data_utils.h>

using namespace JPS;

int main(int argc, char ** argv){
    if(argc != 2) {
        printf(ANSI_COLOR_RED "Input yaml required!\n" ANSI_COLOR_RESET);
        return -1;
    }

    // Read the map from yaml
    MapReader<Vec2i, Vec2f> reader(argv[1], true); // Map read from a given file
    if(!reader.exist()) {
        printf(ANSI_COLOR_RED "Cannot read input file [%s]!\n" ANSI_COLOR_RESET, argv[1]);
        return -1;
    }

    // Prepare data for wrapper
    std::vector<float> origin = {static_cast<float>(reader.origin()(0)), static_cast<float>(reader.origin()(1))};
    std::vector<int> dim = {reader.dim()(0), reader.dim()(1)};
    std::vector<signed char> map = reader.data();
    std::vector<float> start = {static_cast<float>(reader.start(0)), static_cast<float>(reader.start(1))};
    std::vector<float> goal = {static_cast<float>(reader.goal(0)), static_cast<float>(reader.goal(1))};
    float resolution = static_cast<float>(reader.resolution());
    std::vector<std::vector<double>> jps_path, astar_path;

    // Call the wrapper
    int ret = plan_2d(origin, dim, map, start, goal, resolution, jps_path, astar_path);
    if(ret != 0) {
        std::cerr << "Planning failed!" << std::endl;
        return -1;
    }

    // Plot the result in svg image
    typedef boost::geometry::model::d2::point_xy<double> point_2d;
    std::ofstream svg("output.svg");
    boost::geometry::svg_mapper<point_2d> mapper(svg, 1000, 1000);

    // Draw the canvas
    boost::geometry::model::polygon<point_2d> bound;
    const double origin_x = origin[0];
    const double origin_y = origin[1];
    const double range_x = dim[0] * resolution;
    const double range_y = dim[1] * resolution;
    std::vector<point_2d> points;
    points.push_back(point_2d(origin_x, origin_y));
    points.push_back(point_2d(origin_x, origin_y+range_y));
    points.push_back(point_2d(origin_x+range_x, origin_y+range_y));
    points.push_back(point_2d(origin_x+range_x, origin_y));
    points.push_back(point_2d(origin_x, origin_y));
    boost::geometry::assign_points(bound, points);
    boost::geometry::correct(bound);
    mapper.add(bound);
    mapper.map(bound, "fill-opacity:1.0;fill:rgb(255,255,255);stroke:rgb(0,0,0);stroke-width:2");

    // Draw start and goal
    point_2d start_pt(start[0], start[1]);
    point_2d goal_pt(goal[0], goal[1]);
    mapper.add(start_pt);
    mapper.map(start_pt, "fill-opacity:1.0;fill:rgb(255,0,0);", 10);
    mapper.add(goal_pt);
    mapper.map(goal_pt, "fill-opacity:1.0;fill:rgb(255,0,0);", 10);

    // Draw the obstacles
    for(int x = 0; x < dim[0]; x ++) {
        for(int y = 0; y < dim[1]; y ++) {
            int idx = x + dim[0] * y;
            if(map[idx] != 0) {
                double px = origin_x + x * resolution;
                double py = origin_y + y * resolution;
                point_2d a(px, py);
                mapper.add(a);
                mapper.map(a, "fill-opacity:1.0;fill:rgb(0,0,0);", 1);
            }
        }
    }

    // Draw the path from JPS
    if(!jps_path.empty()) {
        boost::geometry::model::linestring<point_2d> line;
        for(const auto& pt: jps_path)
            line.push_back(point_2d(pt[0], pt[1]));
        mapper.add(line);
        mapper.map(line, "opacity:0.4;fill:none;stroke:rgb(212,0,0);stroke-width:5");
    }
    // Draw the path from A*
    if(!astar_path.empty()) {
        boost::geometry::model::linestring<point_2d> line;
        for(const auto& pt: astar_path)
            line.push_back(point_2d(pt[0], pt[1]));
        mapper.add(line);
        mapper.map(line, "opacity:0.4;fill:none;stroke:rgb(1,212,0);stroke-width:5");
    }

    // Write title at the lower right corner on canvas
    mapper.text(point_2d(origin_x + range_x - 6, origin_y+1.8), "test_planner_2d",
                "fill-opacity:1.0;fill:rgb(10,10,250);");
    mapper.text(point_2d(origin_x + range_x - 8, origin_y+1.2), "Green: ",
                "fill-opacity:1.0;fill:rgb(1,212,0);");
    mapper.text(point_2d(origin_x + range_x - 5.5, origin_y+1.2), "astar path",
                "fill-opacity:1.0;fill:rgb(0,0,0);");
    mapper.text(point_2d(origin_x + range_x - 8, origin_y+0.6), "Red: ",
                "fill-opacity:1.0;fill:rgb(212,0,0);");
    mapper.text(point_2d(origin_x + range_x - 5.5, origin_y+0.6), "jps path",
                "fill-opacity:1.0;fill:rgb(0,0,0);");

    return 0;
}
