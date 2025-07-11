cmake_minimum_required(VERSION 3.1) # pybind11 prefers a slightly newer version, 2.8 might be too old.
project(AStar-ThetaStar)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -Wextra")

# Find pybind11 - more robust method for pip-installed pybind11
# First, ensure Python is found to get its executable.
find_package(PythonInterp REQUIRED) # Ensures PYTHON_EXECUTABLE is set

execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c "import pybind11; print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE pybind11_cmake_dir
    RESULT_VARIABLE pybind11_get_cmake_dir_result
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(NOT pybind11_get_cmake_dir_result EQUAL 0)
    message(FATAL_ERROR "Failed to execute Python to get pybind11 CMake directory. Ensure pybind11 is installed in the Python environment being used by CMake (PYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}).")
endif()

if(NOT EXISTS "${pybind11_cmake_dir}/pybind11Config.cmake" AND NOT EXISTS "${pybind11_cmake_dir}/pybind11-config.cmake")
    message(WARNING "pybind11Config.cmake or pybind11-config.cmake not found in directory reported by pybind11.get_cmake_dir(): ${pybind11_cmake_dir}. Trying find_package directly, but it might fail.")
    find_package(pybind11 REQUIRED) # Fallback to direct find_package
else()
    # Add the found directory to CMAKE_PREFIX_PATH so find_package can use it
    list(APPEND CMAKE_PREFIX_PATH "${pybind11_cmake_dir}")
    message(STATUS "Found pybind11 CMake directory: ${pybind11_cmake_dir}. Added to CMAKE_PREFIX_PATH.")
    find_package(pybind11 REQUIRED)
endif()


set(SOURCE_FILES
        asearch.cpp
        # config.h # Headers usually not listed directly
        config.cpp # ensure config.cpp is also present
        environmentoptions.cpp
        mission.cpp # Added mission.cpp
        # mission.h # Headers usually not listed directly
        # searchresult.h # Headers usually not listed directly
        astar.cpp
        # environmentoptions.h # Headers usually not listed directly
        isearch.cpp
        map.cpp
        # mission.h # Headers usually not listed directly (duplicate)
        theta.cpp
        # astar.h # Headers usually not listed directly
        config.cpp
        # gl_const.h # Headers usually not listed directly
        # isearch.h # Headers usually not listed directly
        # map.h # Headers usually not listed directly
        # node.h # Headers usually not listed directly
        # theta.h # Headers usually not listed directly
        # path_smoothing.h # Headers usually not listed directly
        path_smoothing.cpp
        # tinyxml2.h # Headers usually not listed directly
        tinyxml2.cpp
        wrapper.cpp # Added wrapper.cpp
        python_bindings.cpp # Added python_bindings.cpp
        )

add_library(ThetaStarPlanner SHARED ${SOURCE_FILES})

# Link pybind11
target_link_libraries(ThetaStarPlanner PRIVATE pybind11::module)

# Ensure the library is placed in a location where Python can find it,
# or provide instructions to the user to add the build directory to PYTHONPATH.
# For simplicity, CMake will build it in the build directory.

# Optional: Define a module name that Python expects, if different from target name.
# By default, pybind11 creates a module based on the target name.
# On Linux, it will be ThetaStarPlanner.so, on Windows ThetaStarPlanner.pyd.
# pybind11_add_module(ThetaStarPlanner_py ${SOURCE_FILES} src/python_bindings.cpp) # Example if using pybind11_add_module

# Include directories (important for CMake to find headers)
target_include_directories(ThetaStarPlanner PUBLIC ../include) # Make headers in 'include' available
# pybind11::module should handle its own include directories when linked.

# Get Python's extension suffix (e.g., .cpython-38-x86_64-linux-gnu.so or .pyd on Windows)
execute_process(
    COMMAND "${PYTHON_EXECUTABLE}" -c "import sysconfig; suffix = sysconfig.get_config_var('EXT_SUFFIX'); print(suffix if suffix is not None else sysconfig.get_config_var('SO'))"
    OUTPUT_VARIABLE PYTHON_EXT_SUFFIX
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE PYTHON_EXT_SUFFIX_RESULT
)

if(NOT PYTHON_EXT_SUFFIX_RESULT EQUAL 0 OR NOT PYTHON_EXT_SUFFIX)
    message(WARNING "Could not determine Python extension suffix automatically.")
    if(WIN32)
        set(PYTHON_EXT_SUFFIX ".pyd")
        message(STATUS "Defaulting Python extension suffix to .pyd for Windows.")
    else()
        set(PYTHON_EXT_SUFFIX ".so")
        message(STATUS "Defaulting Python extension suffix to .so for non-Windows.")
    endif()
else()
    message(STATUS "Determined Python extension suffix: ${PYTHON_EXT_SUFFIX}")
endif()

# Set the output name and suffix for the Python module
set_target_properties(ThetaStarPlanner PROPERTIES
    PREFIX ""                          # No 'lib' prefix for Python modules
    LIBRARY_OUTPUT_NAME "ThetaStarPlanner"  # Output name
    SUFFIX "${PYTHON_EXT_SUFFIX}"      # Suffix like .cpython-XYZ.so or .pyd
)