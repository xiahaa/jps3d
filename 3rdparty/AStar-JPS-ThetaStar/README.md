# AStar-ThetaStar
Basic algorithms for single-shot grid-based 2D path finding.

Single-shot means that the algorithms are tailored to answering a single pathfinding query (as opposed to other pathfinders that are suited to solving sequences of alike queries, e.g. queries on the same map but with different start and goal locations).

Grid-based means that a (regular square) grid is an essential part of the input. We follow center-based assumption which implies that distinct agent's locations are tied to the centers of the traversable cells (not to the corners).

Description
==========
This project contains implementations of the following algorithms:
- A*
- Theta*

Python Wrapper
==============
This project also includes a Python wrapper allowing you to use the A* and Theta* path planning functionalities from Python.

Prerequisites
-------------
-   **CMake** (version 3.1 or higher recommended)
-   A C++ compiler that supports C++11 (e.g., GCC, Clang, MSVC)
-   **Python** (version 3.6 or higher recommended)
-   **pybind11**: This will be used for creating the Python bindings. You can install it via pip:
    ```bash
    pip install pybind11
    ```
-   **setuptools** (usually comes with Python/pip):
    ```bash
    pip install setuptools
    ```

Building the Python Module
--------------------------
The Python module `ThetaStarPlanner` is built using CMake.

1.  **Navigate to the project root directory** (the one containing this README).
2.  **Create a build directory and navigate into it:**
    ```bash
    mkdir build
    cd build
    ```
3.  **Run CMake to configure the project:**
    *   On Linux/macOS:
        ```bash
        cmake ../src
        ```
    *   On Windows (if using Visual Studio, you might need to specify a generator, e.g., `cmake ../src -G "Visual Studio 16 2019" -A x64`). Simpler for command line (like MinGW):
        ```bash
        cmake ../src
        ```
    This command tells CMake to look for the `CMakeLists.txt` in the `../src` directory.
4.  **Compile the project:**
    *   On Linux/macOS:
        ```bash
        make
        ```
    *   On Windows (with MSVC):
        ```bash
        cmake --build . --config Release
        ```
        (Or open the generated solution file in Visual Studio and build).
        If using MinGW makefiles:
        ```bash
        mingw32-make
        ```

    After a successful build, you should find the Python module file (e.g., `ThetaStarPlanner.cpython-38-x86_64-linux-gnu.so` on Linux, `ThetaStarPlanner.cp38-win_amd64.pyd` on Windows) inside the `build` directory (or a subdirectory like `build/Release` depending on your CMake generator and build type).

Running the Example
-------------------
An example Python script `example.py` is provided in the root directory.

1.  **Ensure the Python module is built** as described above.
2.  **Run the example script from the project root directory:**
    ```bash
    python example.py
    ```
    The `example.py` script attempts to locate the compiled module in common build directory locations (e.g., `build/`, `build/Release/`).

    If you encounter an `ImportError`, ensure that:
    *   The module (e.g., `planner_cpp...so` or `planner_cpp...pyd`) exists in your build directory.
    *   The build directory is in your `PYTHONPATH` environment variable, or you are running the script in a way that Python can find the module (the script tries to add the build directory to `sys.path` automatically, but this might not cover all build configurations). For example, you can copy the `.so`/`.pyd` file from the build directory to the project root directory next to `example.py`.

Using the Wrapper in Your Own Python Code
-----------------------------------------
1.  Make sure the compiled `planner_cpp` module is in your Python path.
2.  Import the module and use the `plan_2d` function:

```python
import ThetaStarPlanner # Or ensure the .so/.pyd file is in your PYTHONPATH

origin = [0.0, 0.0]
dim = [100, 100] # height, width
# Map data: flat list, row-major, 0=free, 1=obstacle
map_data = [0] * (dim[0] * dim[1])
# Populate map_data with obstacles as needed, e.g.:
# map_data[row * dim[1] + col] = 1

start_coords = [0.5, 0.5] # meters
goal_coords = [9.5, 9.5] # meters
map_resolution = 0.1 # meters/cell
use_theta_star = False # False for A*, True for Theta*

status, path, time_ms = planner_cpp.plan_2d(
    origin,
    dim,
    map_data,
    start_coords,
    goal_coords,
    map_resolution,
    use_theta_star
)

if status == 0:
    print(f"Path found: {path}")
    print(f"Time taken: {time_ms} ms")
else:
    print("Failed to find path.")

```
The `map_data` list should contain integers that can be safely converted to `signed char` by the C++ backend (typically values like 0 for free, 1 for obstacle).
The path is returned as a list of `[x, y]` coordinate pairs in meters.
Time spent is returned in milliseconds.
