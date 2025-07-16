# Warthog Pathfinding Library

## Build Instructions

### Prerequisites

- C++ compiler (GCC or Clang, supporting C++14)
- CMake (>= 3.14)
- Python 3 (for bindings and testing)
- pybind11 (for Python bindings)

### Build C++ Library and Executables

To build the static library and executables:

```sh
make
```

This will build:
- The static library: `lib/libwarthog.a`
- The main executable: `bin/warthog`
- The test executable: `bin/tests`

### Build Python Bindings

To build the Python module:

```sh
make python
```

This will create a Python extension module (e.g., `warthog.cpython-<ver>-<platform>.so`) in the current directory.

Alternatively, you can use CMake directly in the `python` subdirectory:

```sh
cd python
mkdir build
cd build
cmake ..
make
```

### Testing Python Bindings

To test the Python bindings:

```sh
cd python
python3 test.py
```

This will run a simple test using the `warthog` Python module.

## Notes

- If you install the library and headers to a custom location, update the include and library paths in the CMakeLists.txt or Makefile accordingly.
- Supported algorithms include: `astar`, `jps`, `jps2`, `jps+`, `jps2+`, and more.
- For more advanced usage, see the source code and comments in `warthog.cpp` and `python/test.py`.

## Troubleshooting

- If you encounter linker errors about `-fPIC`, ensure all objects and libraries are compiled with `-fPIC`.
- For Python bindings, ensure `pybind11` is installed and available to CMake.
