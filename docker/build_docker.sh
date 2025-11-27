#!/bin/bash

# Build script for BLJPS_Python and jps3d Docker image
# This script builds the Docker image by pulling source code from GitHub
set -e

echo "Building Docker image for BLJPS_Python and jps3d..."
echo "Source code will be pulled from: https://github.com/xiahaa/jps3d.git"

# Build the Docker image
docker build -t jps-planners:latest .

echo "Build completed successfully!"

# Test the built libraries
echo "Testing the built libraries..."
docker run --rm jps-planners:latest python3 -c "
import sys
sys.path.append('/usr/local/lib/python3.10/site-packages')

try:
    import BL_JPS
    print('✓ BL_JPS imported successfully')
except ImportError as e:
    print('✗ Failed to import BL_JPS:', e)

try:
    import jps_planner_bindings
    print('✓ jps_planner_bindings imported successfully')
except ImportError as e:
    print('✗ Failed to import jps_planner_bindings:', e)

print('Library test completed.')
"

echo "All tests completed!"
