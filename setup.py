import os
import sys
from setuptools import setup, Extension
import pybind11

# Get the path to the Pybind11 include directory
pybind11_include_path = pybind11.get_include()
pybind11_half_include_path = pybind11.get_include(True) # For pybind11/half_float.h if needed

# Define compiler arguments. C++11 is required by Pybind11.
# Boost 1.83 (installed) requires C++14 or higher for Boost.Geometry.
# Add any other necessary flags here (e.g., for optimization, warnings)
cpp_args = ['-std=c++14'] # Using C++14

# Define include directories
# Add paths to JPS headers and Boost headers if they are not in standard locations.
# For example: include_dirs = [pybind11_include_path, '/path/to/jps_headers', '/path/to/boost_headers']
# If JPS headers are in a subdirectory like 'jps_planner_lib/include', adjust accordingly.
# Assuming 'src' is in the same directory as setup.py for src/wrapper.cpp
# and JPS headers are structured relative to some base include path.
# For now, we'll assume the JPS headers are in a location the compiler can find,
# or we'll need to add their specific paths.
# Let's add 'src' as an include directory so #include "src/wrapper.cpp" in bindings.cpp can be found if needed,
# though a better approach is to list wrapper.cpp as a source file.
# However, given the current bindings.cpp directly includes src/wrapper.cpp,
# we need 'src' to be in the include path if src/wrapper.cpp itself has includes relative to src.
# The original wrapper.cpp has includes like <jps_basis/data_utils.h>.
# This implies that the directory containing 'jps_basis' should be an include path.

# Let's assume for now that the JPS headers are in a directory structure
# such that an include path like "/usr/local/include" or similar would grant access.
# If they are part of this project, e.g., in a 'third_party' or 'extern' directory,
# that path needs to be added.
# For the purpose of this example, I'll add a placeholder for JPS_INCLUDE_DIR.
# You will LIKELY need to change this or set an environment variable.
# User indicated JPS headers are in 'include/jps_basis', so the root for jps_basis is 'include'.
jps_include_dir = 'include'
boost_include_dir = os.environ.get('BOOST_INCLUDE_DIR', '/usr/include') # Example, adjust as needed. This might also be in 'include' or a subfolder.


ext_modules = [
    Extension(
        'jps_planner_bindings', # Name of the module
        sources=[
            'bindings.cpp',
            'src/wrapper.cpp',
            'src/jps_planner/jps_planner.cpp',
            'src/jps_planner/graph_search.cpp'
            # Potentially also:
            # 'src/distance_map_planner/distance_map_planner.cpp',
            # 'src/distance_map_planner/graph_search.cpp',
            # if they are needed for common utilities or full JPS functionality.
            # Starting with just jps_planner files.
        ],
        include_dirs=[
            pybind11_include_path,
            pybind11_half_include_path,
            'src', # To find src/wrapper.cpp if it were included as <src/wrapper.cpp>
                   # and for headers inside src/wrapper.cpp if they are relative to src/
            jps_include_dir, # Directory containing jps_basis, jps_planner
            boost_include_dir, # Directory containing boost
            '/usr/include/eigen3' # Standard path for Eigen3 headers after apt install
        ],
        language='c++',
        extra_compile_args=cpp_args,
        # Add extra_link_args if needed, e.g., for linking specific libraries
        # extra_link_args = ['-L/path/to/libs', '-ljps_library_name']
    ),
]

setup(
    name='jps_planner_bindings',
    version='0.0.1',
    author='Jules AI',
    description='Python bindings for JPS and A* planner',
    ext_modules=ext_modules,
    # Pybind11 is a build-time dependency
    setup_requires=['pybind11>=2.5'], # Or a more specific version
    # If you have a requirements.txt, you can specify install_requires here
    # install_requires=['numpy'], # For example, if your bindings use numpy
)

print("---")
print("IMPORTANT: Review the `include_dirs` in `setup.py`.")
print(f"JPS headers are expected to be found via: {jps_include_dir}")
print(f"Boost headers are expected to be found via: {boost_include_dir}")
print("If your JPS or Boost headers are in a different location,")
print("you MUST either edit `setup.py` or set the environment variables")
print("JPS_INCLUDE_DIR and BOOST_INCLUDE_DIR before running the build.")
print("For example, if jps_basis is in 'extern/jps/include/jps_basis', then JPS_INCLUDE_DIR should be 'extern/jps/include'.")
print("---")
