#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // For automatic conversion of STL containers like std::vector
#include <vector> // Make sure std::vector is included

// Forward declaration of the function from wrapper.cpp
// This is crucial now that we are not including wrapper.cpp directly.
// The definition will be linked from the compiled wrapper.o.
int plan_2d(std::vector<float> &origin,
            std::vector<int> &dim,
            std::vector<signed char> &map,
            std::vector<float> &start,
            std::vector<float> &goal,
            float resolution,
            std::vector<std::vector<double>> &path,
            double &time_spent, bool use_jps);

namespace py = pybind11;

// Helper struct to make returning multiple values (the two paths) cleaner.
// Pybind11 can automatically convert this struct to a Python tuple or a custom Python object.
struct PlanResult {
    std::vector<std::vector<double>> path;
    double time_spent; // Time spent in seconds
    // We could also include other results like planning time or validity if needed.
};

// Wrapper function that calls the original plan_2d and returns the paths.
// This is a common pattern when the C++ function modifies arguments by reference to return values.
PlanResult plan_2d_wrapper(const std::vector<float> &origin,
                           const std::vector<int> &dim,
                           const std::vector<signed char> &map_data, // Renamed to avoid conflict if map is a global
                           const std::vector<float> &start,
                           const std::vector<float> &goal,
                           float resolution, bool use_jps = true) {
    // Create non-const copies for the C++ function if it expects non-const references for these inputs
    // Based on the signature `std::vector<float> &origin`, etc. it seems it might modify them or just didn't use const.
    // For safety, let's assume it might (though it's unlikely for origin, dim, map, start, goal).
    // However, the critical output parameters jps_path and astar_path are handled locally.

    std::vector<float> origin_copy = origin;
    std::vector<int> dim_copy = dim;
    std::vector<signed char> map_data_copy = map_data;
    std::vector<float> start_copy = start;
    std::vector<float> goal_copy = goal;

    std::vector<std::vector<double>> path_output;
    double time_spent = 0.0; // Initialize time spent

    // Call the original C++ function
    // The original function from wrapper.cpp is `plan_2d`
    // int plan_2d(std::vector<float> &origin, std::vector<int> &dim, std::vector<signed char> &map, std::vector<float> &start, std::vector<float> &goal, float resolution, std::vector<std::vector<double> > &jps_path, std::vector<std::vector<double> > &astar_path)
    plan_2d(origin_copy, dim_copy, map_data_copy, start_copy, goal_copy, resolution, path_output, time_spent, use_jps);

    return {path_output, time_spent};
}

PYBIND11_MODULE(jps_planner_bindings, m) {
    m.doc() = "Pybind11 bindings for JPS 2D planner";

    // Define the PlanResult struct for Python
    py::class_<PlanResult>(m, "PlanResult")
        .def(py::init<>())
        .def_readwrite("path", &PlanResult::path)
        .def_readwrite("time_spent", &PlanResult::time_spent);

    // Expose the wrapper function to Python
    m.def("plan_2d", &plan_2d_wrapper, "Plans a 2D path using JPS and A*",
          py::arg("origin"),
          py::arg("dim"),
          py::arg("map_data"),
          py::arg("start"),
          py::arg("goal"),
          py::arg("resolution"),
          py::arg("use_jps") = true
    );
}
