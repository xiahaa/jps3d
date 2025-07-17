#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "plan_2d.h"

namespace py = pybind11;

PYBIND11_MODULE(AMRA, m) {
    m.doc() = "Python bindings for the AMRA pathfinding library";

    m.def("plan_2d", [](std::vector<float>& origin, std::vector<int>& dim, std::vector<signed char>& map,
                        std::vector<float>& start, std::vector<float>& goal, float resolution) {
        std::vector<std::vector<double>> path;
        double time_spent = 0;
        int ret = plan_2d(origin, dim, map, start, goal, resolution, path, time_spent);
        return std::make_tuple(ret, path, time_spent);
    });
}