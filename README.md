# Repository for Benchmarking Single Agent Path Planning in Large Scale Map

This repository provides a framework and tools for benchmarking single agent path planning algorithms on large-scale maps. It includes C++ implementations, Python bindings, and example scripts for both C++ and Python.

## Folder Structure

- `3rdparty/`: Open-source code for single agent path planning, including the AMRA library and its dependencies.
- `dat/`: Example map files in MovingAI format.
- `test/`: Example C++ test programs.
- `python/`: Python bindings and test scripts.

## Building

### Requirements

- CMake >= 3.5
- Boost (filesystem, program_options, system)
- Eigen3
- pybind11 (for Python bindings)
- Python 3

### Build Steps

```sh
mkdir build
cd build
cmake ..
make
```

This will build the C++ libraries, example executables, and the Python extension module.

## Usage

### C++ Example

Run the C++ test program (after building):

```sh
./test/python/test
```

### Python Example

After building, you can run the Python test script:

```sh
python3 3rdparty/amra/python/test.py
```

## Map Format

The framework uses the MovingAI map format. Example maps are provided in the `dat/` directory.

## License

See the LICENSE file if available.

## Contact

For questions or contributions, please contact the project maintainer.
