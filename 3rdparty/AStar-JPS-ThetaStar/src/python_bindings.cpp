#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // For automatic conversion of std::vector
#include "wrapper.h"     // Contains the declaration of plan_2d
#include <tuple>         // For std::tuple

namespace py = pybind11;

PYBIND11_MODULE(ThetaStarPlanner, m) {
    m.doc() = R"pbdoc(
        Python module for A* and Theta* path planning
        ---------------------------------------------
        .. currentmodule:: ThetaStarPlanner
        .. autosummary::
           :toctree: _generate
           plan_2d
    )pbdoc";

    m.def("plan_2d",
          [](std::vector<float> &origin, std::vector<int> &dim, std::vector<signed char> &map_data, std::vector<float> &start, std::vector<float> &goal, float resolution, bool use_theta) {
              std::vector<std::vector<double>> path;
              double time_spent = 0.0;
              int status = ::plan_2d(origin, dim, map_data, start, goal, resolution, path, time_spent, use_theta);
              return std::make_tuple(status, path, time_spent);
          },
          py::arg("origin"),
          py::arg("dim"),
          py::arg("map_data"),
          py::arg("start"),
          py::arg("goal"),
          py::arg("resolution"),
          py::arg("use_theta"),
          R"pbdoc(
            Plans a 2D path using A* or Theta* algorithm.

            Args:
                origin (list[float]): Origin of the map [x, y] in meters.
                dim (list[int]): Dimensions of the map [height, width] in grid cells.
                map_data (list[int]): Map data as a flattened list (row-major). 0 for free, 1 for obstacle.
                                     Note: C++ expects signed char, Python will pass integers.
                start (list[float]): Start coordinates [x, y] in meters.
                goal (list[float]): Goal coordinates [x, y] in meters.
                resolution (float): Map resolution in meters/cell.
                use_theta (bool): If true, uses Theta*; otherwise, uses A*.

            Returns:
                tuple[int, list[list[float]], float]: A tuple containing:
                    - status (int): 0 if successful, -1 otherwise.
                    - path (list[list[float]]): The calculated path, a list of [x,y] coordinates.
                    - time_spent (float): The time spent in planning (in milliseconds).
          )pbdoc"
    );

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
