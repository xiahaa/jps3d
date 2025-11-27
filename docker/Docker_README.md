# Docker Build for JPS Planners

This directory contains Docker configurations to build shared object (.so) files for both BLJPS_Python and jps3d libraries that can be used in deployment.

## Files

- `Dockerfile` - Main Dockerfile for building both libraries
- `Dockerfile.robust` - Enhanced version with additional debugging and error handling
- `build_docker.sh` - Build script with automated testing and fallback options
- `Docker_README.md` - This documentation

## Quick Start

### Option 1: Using the build script (Recommended)
```bash
chmod +x build_docker.sh
./build_docker.sh
```

The build script will:
1. Try the standard Dockerfile first
2. Fall back to the robust version if the standard build fails
3. Test both libraries after successful build

### Option 2: Manual Docker build
```bash
# Build the Docker image
docker build -t jps-planners:latest .

# Or use the robust version if you encounter issues
docker build -f Dockerfile.robust -t jps-planners:robust .
```

## Common Build Issues and Solutions

### Issue 1: Python Header Detection
**Error**: `Can't find python.h in /usr/local/include/python3.10`

**Solution**: The Dockerfile now uses `sysconfig.get_path('include')` for proper Python header detection.

### Issue 2: Missing pybind11 Submodule
**Error**: `add_subdirectory given source "pybind11" which is not an existing directory`

**Solutions**:
1. The robust Dockerfile manually clones pybind11 if the submodule is missing
2. Uses `git clone --recursive` to ensure all submodules are initialized

### Issue 3: CMake Cache Conflicts
**Error**: `CMakeCache.txt directory is different than the directory where CMakeCache.txt was created`

**Solution**: All Dockerfiles now pull fresh code from GitHub and clean build directories.

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

## Build Process

The Dockerfile uses a multi-stage build that pulls source code directly from GitHub:

1. **Source Retrieval**: Clones the repository from https://github.com/xiahaa/jps3d.git
2. **Dependency Resolution**: Ensures all git submodules (especially pybind11) are properly initialized
3. **Builder Stage**: Compiles both libraries with all build dependencies, ensuring clean builds
4. **Production Stage**: Creates a minimal runtime image with only the compiled libraries and runtime dependencies

### Key Improvements:

- **Robust Python Detection**: Uses `sysconfig` for accurate Python header and library paths
- **Submodule Handling**: Automatically handles missing pybind11 submodule
- **Clean Builds**: Removes any existing build artifacts before compilation
- **Debug Information**: Robust version includes detailed debugging output
- **Fallback Strategy**: Build script tries multiple approaches

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

## Troubleshooting

### Debug Build Issues

To debug build issues, you can run the builder stage interactively:
```bash
docker build --target builder -t jps-debug .
docker run -it jps-debug /bin/bash
```

### Check Python Configuration
The robust Dockerfile includes debug output showing:
- Python executable path
- Python version
- Include directory
- Library paths
- Available python.h locations

### Manual Submodule Initialization
If submodule issues persist:
```bash
docker run -it --entrypoint /bin/bash jps-debug
cd /workspace/3rdparty/BLJPS_Python
git submodule update --init --recursive
```

## Deployment

To use these libraries in your deployment:

1. **Copy libraries from Docker image**:
   ```bash
   docker create --name temp-container jps-planners:latest
   docker cp temp-container:/usr/local/lib/python3.10/site-packages/ ./libs/
   docker rm temp-container
   ```

2. **Use as base image**:
   ```dockerfile
   FROM jps-planners:latest
   COPY your_app.py /app/
   CMD ["python3", "your_app.py"]
   ```

3. **Multi-stage deployment**:
   ```dockerfile
   FROM jps-planners:latest as libs
   FROM your-base-image:latest
   COPY --from=libs /usr/local/lib/python3.10/site-packages/ /usr/local/lib/python3.10/site-packages/
   ```

## Base Image

Uses `idea-laser.tencentcloudcr.com/public/python:3.10.6` as specified in requirements.

## Testing

The build process includes automatic testing that verifies:
- Both libraries can be imported successfully
- Basic functionality works (object creation, simple planning calls)
- No missing runtime dependencies
