#!/usr/bin/env python3
"""
Simple benchmark script that loads image1.png, picks random start/end points,
and tests path planning runtime.
"""

import sys
import time
import random
from pathlib import Path
import numpy as np
import cv2

SILAS_OS_PATH = Path('.').resolve() / '3rdparty' / 'silas-os-python'
sys.path.append(str(SILAS_OS_PATH))
try:
    from silas.os.kernel.tspatial_processing.planning.single_agent_planning.single_agent_planning_2d import single_agent_planning_2d
    print('SUCCESS: silas package imported successfully')
    SILAS_AVAILABLE = True
except ImportError as e:
    print(f'IMPORT ERROR: {e}')
except Exception as e:
    print(f'OTHER ERROR: {e}')
    SILAS_AVAILABLE = False

def find_random_free_points(image, num_points=2, max_attempts=1000):
    """Find random points in free space (black areas)."""
    height, width = image.shape
    free_points = []

    # Create mask of free space (black pixels, value < 128)
    free_mask = image < 128

    attempts = 0
    while len(free_points) < num_points and attempts < max_attempts:
        row = random.randint(0, height - 1)
        col = random.randint(0, width - 1)

        if free_mask[row, col]:
            free_points.append((col, row))  # Return as (x, y) = (col, row)

        attempts += 1

    if len(free_points) < num_points:
        raise ValueError(f"Could not find {num_points} free points after {max_attempts} attempts")

    return free_points


def placeholder_planner(grid, start, goal, scale_factor=0.1):
    """Placeholder planner for when silas is not available."""
    time.sleep(0.05)  # Simulate some processing time

    # Simple straight line path
    start_row, start_col = start
    goal_row, goal_col = goal

    num_steps = max(abs(goal_row - start_row), abs(goal_col - start_col))
    if num_steps == 0:
        return [start]

    path = []
    for i in range(num_steps + 1):
        t = i / num_steps
        row = int(start_row + t * (goal_row - start_row))
        col = int(start_col + t * (goal_col - start_col))
        path.append((row, col))

    return path


def run_simple_benchmark():
    """Run a simple benchmark test."""
    print("=== Simple Path Planning Benchmark ===")

    # Load the image
    image_path = Path('.').resolve() / "image.png"
    if not image_path.exists():
        print(f"Error: {image_path} not found")
        return False

    print(f"Loading image: {image_path}")
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        print("Error: Could not load image")
        return False

    print(f"Image shape: {image.shape}")
    print(f"Image dtype: {image.dtype}")
    print(f"Pixel value range: {image.min()} - {image.max()}")

    # Find free space statistics
    free_pixels = np.sum(image < 128)
    total_pixels = image.shape[0] * image.shape[1]
    free_percentage = (free_pixels / total_pixels) * 100
    print(f"Free space: {free_pixels}/{total_pixels} pixels ({free_percentage:.1f}%)")

    # Find random start and end points
    print("\nFinding random start and end points...")
    try:
        points = find_random_free_points(image, num_points=2)
        start_point = points[0]
        end_point = points[1]

        print(f"Start point (x, y): {start_point}")
        print(f"End point (x, y): {end_point}")

        # Calculate distance
        distance = np.sqrt((end_point[0] - start_point[0])**2 + (end_point[1] - start_point[1])**2)
        print(f"Euclidean distance: {distance:.1f} pixels")

    except ValueError as e:
        print(f"Error: {e}")
        return False

    # Run path planning
    print(f"\nRunning path planning...")
    print(f"Using {'silas plan2d' if SILAS_AVAILABLE else 'placeholder'} planner")

    # Convert to format expected by planner (row, col)
    start_rc = (start_point[1], start_point[0])  # (y, x) -> (row, col)
    goal_rc = (end_point[1], end_point[0])      # (y, x) -> (row, col)

    # Convert image to boolean grid (True = obstacle)
    grid = image >= 128

    # Run planning with timing
    start_time = time.perf_counter()

    if SILAS_AVAILABLE:
        try:
            path = single_agent_planning_2d(grid, start_rc, goal_rc, scale_factor=0.1)
            success = path is not None and len(path) > 0
        except Exception as e:
            print(f"Planning failed: {e}")
            path = None
            success = False
    else:
        path = placeholder_planner(grid, start_rc, goal_rc)
        success = True

    end_time = time.perf_counter()
    runtime_ms = (end_time - start_time) * 1000

    # Report results
    print(f"\n=== Results ===")
    print(f"Success: {'✓' if success else '✗'}")
    print(f"Runtime: {runtime_ms:.2f} ms")

    if success and path:
        path_length = len(path)
        print(f"Path length: {path_length} waypoints")

        # Calculate path distance
        if len(path) > 1:
            path_distance = 0
            for i in range(1, len(path)):
                prev_row, prev_col = path[i-1]
                curr_row, curr_col = path[i]
                segment_dist = np.sqrt((curr_col - prev_col)**2 + (curr_row - prev_row)**2)
                path_distance += segment_dist
            print(f"Path distance: {path_distance:.1f} pixels")
            print(f"Path efficiency: {distance/path_distance:.2f} (1.0 = straight line)")

        # Show first and last few waypoints
        if path_length > 0:
            print(f"First waypoint: {path[0]}")
            print(f"Last waypoint: {path[-1]}")
            if path_length > 4:
                print(f"Path preview: {path[0]} -> {path[1]} -> ... -> {path[-2]} -> {path[-1]}")

    # Save a simple visualization
    try:
        # Create visualization image
        vis_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

        # Mark start point (green)
        cv2.circle(vis_image, start_point, 5, (0, 255, 0), -1)
        cv2.putText(vis_image, "START", (start_point[0] + 10, start_point[1]),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Mark end point (red)
        cv2.circle(vis_image, end_point, 5, (0, 0, 255), -1)
        cv2.putText(vis_image, "END", (end_point[0] + 10, end_point[1]),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # Draw path if available
        if success and path and len(path) > 1:
            path_points = [(point[1], point[0]) for point in path]  # Convert (row,col) to (x,y)
            path_array = np.array(path_points, dtype=np.int32)
            cv2.polylines(vis_image, [path_array], False, (255, 0, 0), 2)

        # Save visualization
        output_path = Path('.').resolve() / "simple_benchmark_result.png"
        cv2.imwrite(str(output_path), vis_image)
        print(f"\nVisualization saved to: {output_path}")

    except Exception as e:
        print(f"Warning: Could not save visualization: {e}")

    return success


if __name__ == "__main__":
    success = run_simple_benchmark()
    print(f"\nBenchmark {'completed successfully' if success else 'failed'}")
    sys.exit(0 if success else 1)
