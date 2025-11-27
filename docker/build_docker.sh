#!/bin/bash

# Build script for BLJPS_Python and jps3d Docker image
# This script builds the Docker image by pulling source code from GitHub
set -e

echo "Building Docker image for BLJPS_Python and jps3d..."
echo "Source code will be pulled from: https://github.com/xiahaa/jps3d.git"

# Build the Docker image
echo "Building with standard Dockerfile..."
docker build -t jps-planners:latest . || {
    echo "Standard build failed, trying robust version..."
    docker build -f Dockerfile.robust -t jps-planners:robust .
    echo "Tagging robust build as latest..."
    docker tag jps-planners:robust jps-planners:latest
}

echo "Build completed successfully!"

# Test the built libraries
echo "Testing the built libraries..."
docker run --rm jps-planners:latest python3 -c "
import sys
sys.path.append('/usr/local/lib/python3.10/site-packages')

try:
    import BL_JPS
    print('[OK] BL_JPS imported successfully')

    # Test basic functionality
    bljps = BL_JPS.BL_JPS()
    print('[OK] BL_JPS object created successfully')
except ImportError as e:
    print('[ERROR] Failed to import BL_JPS:', e)
except Exception as e:
    print('[ERROR] BL_JPS test failed:', e)

try:
    import jps_planner_bindings
    print('[OK] jps_planner_bindings imported successfully')

    # Test basic functionality
    result = jps_planner_bindings.plan_2d(
        [0.0, 0.0], [5, 5], [0]*25, [0.0, 0.0], [4.0, 4.0], 1.0, True
    )
    print('[OK] jps_planner_bindings basic test passed')
except ImportError as e:
    print('[ERROR] Failed to import jps_planner_bindings:', e)
except Exception as e:
    print('[ERROR] jps_planner_bindings test failed:', e)

print('Library test completed.')
"

echo "All tests completed!"
