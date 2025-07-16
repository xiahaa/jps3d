# BLJPS_SubGoal_Exp

This folder contains a C++ implementation and Python binding for 2D path planning using the BLJPS (Bounded-Length Jump Point Search) algorithm with subgoal expansion. The main entry point is the `plan_2d` function, which can be called from both C++ and Python.

代码确实快，但是需要预计算，没有预计算的情况下，会段错误，比较适合静态地图。

## Files

- `plan_2d.cpp`: Implements the `plan_2d` function, which prepares the map, preprocesses if needed, and computes a path between start and goal.
- `wrapper.cpp`: Provides Python bindings for `plan_2d` using pybind11.
- `test.py`: Example Python script to test the `plan_2d` binding.

## Usage

### C++
Call the `plan_2d` function with the following signature:
```cpp
int plan_2d(std::vector<float> &origin, std::vector<int> &dim,
            std::vector<signed char> &map_data, std::vector<float> &start_f,
            std::vector<float> &goal_f, float resolution,
            std::vector<std::vector<double> > &path, double &time_spent);
```

### Python
Build the module using pybind11, then use as follows:
```python
import warthog
origin = [0.0, 0.0]
dim = [5, 5]
map_data = [0] * 25
start = [0.0, 0.0]
goal = [4.0, 4.0]
resolution = 1.0
ret, path, time_spent = warthog.plan_2d(origin, dim, map_data, start, goal, resolution)
```

See `test.py` for a complete example.

## Requirements
- C++ compiler
- [pybind11](https://github.com/pybind/pybind11) (for Python bindings)
- Python 3 (for running `test.py`)

## Notes
- The map is preprocessed and saved as `map.txt` if it does not already exist.
- The planner expects the map to be a 2D grid, with `0` for traversable and non-zero for obstacles in `map_data`.

## License
See the main project for license information.
