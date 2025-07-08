# Python Bindings for JPS Planner

This project provides Python bindings for a C++ JPS (Jump Point Search) and A* path planner using Pybind11.

## Prerequisites

To build and run this project, you will need the following:

1.  **C++ Compiler:** A C++ compiler that supports C++14 (e.g., g++).
    *   On Ubuntu/Debian: `sudo apt-get install build-essential`
2.  **Python:** Python development headers are required. The version should be compatible with Pybind11.
    *   On Ubuntu/Debian: `sudo apt-get install python3-dev` (adjust for your specific Python version if needed, e.g., `python3.X-dev`).
3.  **Pip:** Python package installer.
    *   On Ubuntu/Debian: `sudo apt-get install python3-pip`
4.  **Pybind11:** For generating Python bindings.
    *   Installation: `pip install pybind11`
5.  **Setuptools:** For building the Python package.
    *   Installation: `pip install setuptools`
6.  **Eigen3 Library:** A C++ template library for linear algebra, used by the JPS planner.
    *   On Ubuntu/Debian: `sudo apt-get install libeigen3-dev`
    *   The headers are typically installed to `/usr/include/eigen3/`. If they are elsewhere, you'll need to adjust the `setup.py` script.
7.  **Boost Libraries:** Several Boost libraries are used (e.g., Boost.Geometry, Boost.Heap).
    *   On Ubuntu/Debian: `sudo apt-get install libboost-dev` (this is a meta-package that should pull in most necessary Boost development files).
    *   Headers are typically installed to `/usr/include/boost/` or `/usr/local/include/boost/`. The `setup.py` script attempts to find these. If your Boost installation is in a non-standard location, you might need to adjust `setup.py` or set the `BOOST_INCLUDE_DIR` environment variable.

## Project Structure (Relevant to Python Bindings)

*   `src/`: Contains the core C++ planner code.
    *   `wrapper.cpp`: Contains the `plan_2d` function that orchestrates JPS and A* planning. This is the primary C++ function exposed to Python.
    *   `jps_planner/`: Contains the JPS planner implementation (`jps_planner.cpp`, `graph_search.cpp`). These are compiled as part of the Python module.
    *   `jps_basis/`: (Assumed to be in an `include` directory relative to the project root, e.g., `include/jps_basis/`) Contains JPS data types and utilities (header files). These headers are needed during compilation.
*   `include/`: (Assumed location for JPS headers like `jps_basis/` and `jps_planner/`) The `setup.py` is configured to look for JPS headers here. If your JPS headers are located elsewhere (e.g., system-wide or in a `third_party` directory), you'll need to update the `jps_include_dir` variable in `setup.py` or set the `JPS_INCLUDE_DIR` environment variable.
*   `bindings.cpp`: Contains the Pybind11 wrapper code to expose `plan_2d` to Python.
*   `setup.py`: Script to compile the C++ code into a Python module. This handles finding headers, source files, and compiler settings.
*   `test_planner.py`: A Python script to test the generated bindings by importing the module and calling the `plan_2d` function.

## Compilation Steps

1.  **Ensure all prerequisites are installed.** Refer to the "Prerequisites" section.
2.  **Verify Header Locations:**
    *   **JPS Headers:** The `setup.py` script expects the JPS headers (e.g., `jps_basis/data_utils.h`, `jps_planner/jps_planner.h`) to be findable through an include path specified by `jps_include_dir`. By default, this is set to `'include'` (relative to the project root). If your `jps_basis` and `jps_planner` directories are located in `my_custom_jps_lib/include/`, you would set `jps_include_dir = 'my_custom_jps_lib/include'` in `setup.py`, or set the `JPS_INCLUDE_DIR` environment variable to this path.
    *   **Eigen3 Headers:** Typically found in `/usr/include/eigen3/` after installation with `apt`. `setup.py` includes this path by default.
    *   **Boost Headers:** Typically found in `/usr/include/` or `/usr/local/include/` after installation with `apt`. `setup.py` tries `/usr/local/include` by default for `BOOST_INCLUDE_DIR` (which might fall back to `/usr/include` if the compiler checks standard paths). If your Boost headers are elsewhere, adjust this in `setup.py` or via the environment variable.
3.  **Build the Python Module:**
    Navigate to the root directory of this project in your terminal and run:
    ```bash
    python setup.py build_ext --inplace
    ```
    This command will:
    *   Compile `bindings.cpp`, `src/wrapper.cpp`, `src/jps_planner/jps_planner.cpp`, and `src/jps_planner/graph_search.cpp`.
    *   Link them together into a shared object file (e.g., `jps_planner_bindings.cpython-XYZ.so`) in the current directory. The exact name depends on your Python version and platform.

## Running the Test Script

After successful compilation, you can run the example test script:

```bash
python test_planner.py
```

This script will import the compiled module (`jps_planner_bindings`), set up a sample map and planning request, call the `plan_2d` function, and print the resulting JPS and A* paths.

## Troubleshooting

*   **`fatal error: jps_basis/data_utils.h: No such file or directory` (or similar for JPS headers):**
    The compiler cannot find the JPS library headers. Ensure the `jps_basis` and `jps_planner` directories (containing the `.h` files) are located within the path specified by `jps_include_dir` in `setup.py` (default is an `include/` subdirectory in your project root), or that the `JPS_INCLUDE_DIR` environment variable is correctly set to the parent directory of `jps_basis` and `jps_planner`.
*   **`fatal error: Eigen/Geometry: No such file or directory`:**
    Eigen3 headers are not found. Ensure `libeigen3-dev` is installed. If it's installed in a non-standard location, add that location to `include_dirs` in `setup.py`. `/usr/include/eigen3` is standard for Debian/Ubuntu and is added by default in the current `setup.py`.
*   **`fatal error: boost/heap/d_ary_heap.hpp: No such file or directory` (or similar for Boost headers):**
    Boost development headers are not found or incomplete. Ensure `libboost-dev` is installed. If Boost is in a non-standard location, update `boost_include_dir` in `setup.py` or set the `BOOST_INCLUDE_DIR` environment variable. The system path `/usr/include` (where boost headers are usually installed by `apt`) is typically checked by the compiler by default.
*   **C++ Standard Errors (e.g., related to `std::conditional_t`, `auto` in lambdas):**
    The version of Boost installed (likely Boost 1.83 if `libboost-dev` was installed recently on a modern Ubuntu) requires C++14 or newer. The `setup.py` has been configured to use `-std=c++14`. If you encounter such errors, verify this flag in `setup.py`.
*   **Linker Errors (`undefined symbol: _ZN10JPSPlannerILi2EE4planE...` or similar):**
    This usually means that some C++ source files providing necessary definitions were not compiled and linked.
    *   If the error relates to `JPSPlanner` or similar, ensure all necessary `.cpp` files from the JPS library (e.g., `src/jps_planner/jps_planner.cpp`, `src/jps_planner/graph_search.cpp`) are listed in the `sources` of the `Extension` in `setup.py`. The current `setup.py` includes these.
    *   If the error relates to other libraries (e.g., a specific Boost component that isn't header-only, though most commonly used ones are), you might need to add library linkage flags (`library_dirs`, `libraries`) to `setup.py`.

This `README.md` provides a guide to setting up the environment, compiling the Python module, and running the test.
