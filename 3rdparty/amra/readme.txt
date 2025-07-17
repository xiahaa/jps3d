AMRA Path Planning Library
=========================

This directory contains the AMRA path planning library and its Python bindings.

速度较慢，且代码有诸多bug。

Build Instructions
------------------
You can build the library using either CMake or the provided Makefile.

**CMake:**
1. Create a build directory:
   mkdir build && cd build
2. Run CMake and build:
   cmake ..
   make

**Makefile:**
- To build the C++ library and Python module:
  make python

The resulting Python module will be named `amra.so` or similar, depending on your platform.

Python Usage
------------
- The Python bindings expose the `plan_2d` function and the `MOVINGAI_DICT` dictionary.
- Example usage can be found in `python/test.py`.

C++ Usage
---------
- The main C++ interface is provided in `interface.h`.
- Example usage can be found in `python/test.cpp`.

Dependencies
------------
- Boost (filesystem, program_options, system)
- Eigen3
- pybind11 (for Python bindings)
- Python 3

License
-------
See the LICENSE file if available.

Contact
-------
For questions or contributions, please contact the project maintainer.
