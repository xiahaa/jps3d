# Docker Build for JPS Planners

This directory contains Docker configurations to build shared object (.so) files for both BLJPS_Python and jps3d libraries that can be used in deployment.

## Files

- `Dockerfile` - Main Dockerfile for building both libraries
- `Dockerfile.robust` - More robust version with additional error handling and verification
- `build_docker.sh` - Build script with automated testing
- `Docker_README.md` - This documentation

## Quick Start

### Option 1: Using the build script (Recommended)
```bash
./build_docker.sh
```

### Option 2: Manual Docker build
```bash
# Build the Docker image
docker build -t jps-planners:latest .

# Test the built libraries
docker run --rm jps-planners:latest python3 -c "
import sys
sys.path.append('/usr/local/lib/python3.10/site-packages')
import BL_JPS
import jps_planner_bindings
print('Both libraries imported successfully!')
"
```

### Option 3: Using the robust Dockerfile
```bash
docker build -f Dockerfile.robust -t jps-planners:robust .
```

## Built Libraries

The Docker image contains two compiled shared libraries:

### 1. BL_JPS (BLJPS_Python)
- **Location**: `/usr/local/lib/python3.10/site-packages/BL_JPS.*.so`
- **Usage**:
  ```python
  import BL_JPS

  map_data = [0, 0, 0, 1]
  bljps = BL_JPS.BL_JPS()
  bljps.preProcessGrid(map_data, width=2, height=2)
  path = bljps.findSolution(sX=0, sY=0, eX=1, eY=0)
  ```

### 2. jps_planner_bindings (jps3d)
- **Location**: `/usr/local/lib/python3.10/site-packages/jps_planner_bindings.*.so`
- **Usage**:
  ```python
  import jps_planner_bindings

  result = jps_planner_bindings.plan_2d(
      origin=[0.0, 0.0],
      dim=[10, 10],
      map_data=[0] * 100,  # 10x10 map
      start=[0.0, 0.0],
      goal=[9.0, 9.0],
      resolution=1.0,
      use_jps=True
  )
  print(f"Path: {result.path}")
  print(f"Time: {result.time_spent}")
  ```

## Dependencies

The Docker image includes all necessary dependencies:

### System Dependencies
- build-essential (gcc, g++, make)
- cmake (>= 3.5)
- libeigen3-dev
- libyaml-cpp-dev
- libboost-all-dev
- python3-dev

### Python Dependencies
- pybind11
- numpy
- setuptools
- wheel

## Build Process

The Dockerfile uses a multi-stage build that pulls source code directly from GitHub:

1. **Source Retrieval**: Clones the repository from https://github.com/xiahaa/jps3d.git
2. **Builder Stage**: Compiles both libraries with all build dependencies, ensuring clean builds
3. **Production Stage**: Creates a minimal runtime image with only the compiled libraries and runtime dependencies

This approach ensures:
- No local build cache conflicts
- Consistent builds across different environments
- Always uses the latest code from the repository

### BLJPS_Python Build
- Uses CMake with pybind11
- Requires C++17 support
- Builds against Python 3.10

### jps3d Build
- First builds C++ libraries with CMake
- Then builds Python bindings with setuptools
- Requires Boost, Eigen3, and yaml-cpp

## Troubleshooting

### Common Issues

1. **Import errors**: Make sure PYTHONPATH includes `/usr/local/lib/python3.10/site-packages`
2. **Missing dependencies**: Use `Dockerfile.robust` for more comprehensive dependency handling
3. **Build failures**: Check that all git submodules are properly initialized

### Debugging

To debug build issues, you can run the builder stage interactively:
```bash
docker build --target builder -t jps-debug .
docker run -it jps-debug /bin/bash
```

### Verification

The robust Dockerfile includes verification steps that check:
- Shared libraries are built successfully
- Libraries can be imported in Python
- No missing runtime dependencies

## Deployment

To use these libraries in your deployment:

1. Copy the built .so files from the Docker image:
   ```bash
   docker create --name temp-container jps-planners:latest
   docker cp temp-container:/usr/local/lib/python3.10/site-packages/ ./libs/
   docker rm temp-container
   ```

2. Or use the Docker image directly as a base for your application:
   ```dockerfile
   FROM jps-planners:latest
   COPY your_app.py /app/
   CMD ["python3", "your_app.py"]
   ```

## Base Image

Uses `idea-laser.tencentcloudcr.com/public/python:3.10.6` as specified in requirements.
