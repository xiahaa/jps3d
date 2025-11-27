#!/usr/bin/env python3
"""
Visual test script to verify all planners work correctly before running the full benchmark.
This script runs each planner once and draws the planned paths on the image.
"""

import sys
import time
import numpy as np
import cv2
import matplotlib.pyplot as plt
from pathlib import Path

# Import planners
try:
    import jps_planner_bindings
    JPS_AVAILABLE = True
    print("✓ jps_planner_bindings imported successfully")
except ImportError as e:
    JPS_AVAILABLE = False
    print(f"✗ jps_planner_bindings import failed: {e}")

try:
    import BL_JPS
    BLJPS_AVAILABLE = True
    print("✓ BL_JPS imported successfully")
except ImportError as e:
    BLJPS_AVAILABLE = False
    print(f"✗ BL_JPS import failed: {e}")

# Import silas
SILAS_OS_PATH = Path('.').resolve() / '3rdparty' / 'silas-os-python'
sys.path.append(str(SILAS_OS_PATH))
try:
    from silas.os.kernel.tspatial_processing.planning.astar.astar_2d import AStar
    SILAS_AVAILABLE = True
    print("✓ silas AStar imported successfully")
except ImportError as e:
    SILAS_AVAILABLE = False
    print(f"✗ silas import failed: {e}")

def uncompress_bljps_path(path):
    """Uncompress BL_JPS path format."""
    if not path:
        return []

    uncompressed_path = []
    for p_id in range(len(path) - 1):
        current_p = [path[p_id][0], path[p_id][1]]
        uncompressed_path.append(tuple(current_p))

        while current_p[0] != path[p_id + 1][0] or current_p[1] != path[p_id + 1][1]:
            if path[p_id + 1][0] != current_p[0]:
                if path[p_id + 1][0] > current_p[0]:
                    current_p[0] += 1
                else:
                    current_p[0] -= 1
                uncompressed_path.append(tuple(current_p))
            if path[p_id + 1][1] != current_p[1]:
                if path[p_id + 1][1] > current_p[1]:
                    current_p[1] += 1
                else:
                    current_p[1] -= 1
                uncompressed_path.append(tuple(current_p))

    return uncompressed_path

def test_jps_planner(grid, start, goal):
    """Test JPS planner."""
    if not JPS_AVAILABLE:
        return None, 0.0, "JPS not available"

    try:
        height, width = grid.shape
        origin = [0, 0]
        dim = [width, height]
        resolution = 1.0

        # Convert grid to format expected by JPS
        map_data = []
        for y in range(height):
            for x in range(width):
                if grid[y, x]:  # True = obstacle
                    map_data.append(1)
                else:
                    map_data.append(0)

        # Convert start/goal from (row, col) to world coordinates
        start_world = [float(start[1]), float(start[0])]  # (row, col) -> (x, y)
        goal_world = [float(goal[1]), float(goal[0])]

        result = jps_planner_bindings.plan_2d(
            origin, dim, map_data, start_world, goal_world, resolution, True
        )

        runtime_ms = result.time_spent

        if result.path and len(result.path) > 0:
            # Convert path back to (row, col) format
            path = [(int(p[1]), int(p[0])) for p in result.path]  # (x, y) -> (row, col)
            return path, runtime_ms, "Success"
        else:
            return None, runtime_ms, "No path found"

    except Exception as e:
        return None, 0.0, f"Error: {e}"

def test_bljps_planner(grid, start, goal):
    """Test BL_JPS planner."""
    if not BLJPS_AVAILABLE:
        return None, 0.0, "BL_JPS not available"

    try:
        height, width = grid.shape
        origin = (0, 0)

        # Convert grid to format expected by BL_JPS
        map_data = grid.astype(np.int32).flatten().tolist()

        # BL_JPS expects (x, y) coordinates
        start_x, start_y = start[1], start[0]  # (row, col) -> (x, y)
        goal_x, goal_y = goal[1], goal[0]

        bljps = BL_JPS.BL_JPS()
        result = bljps.plan_2d(
            map_data, width=width, height=height,
            startX=start_x, startY=start_y,
            endX=goal_x, endY=goal_y,
            originX=origin[0], originY=origin[1],
            resolution=1
        )

        runtime_ms = result.time_spent

        if result.path and len(result.path) > 0:
            # Uncompress and convert path to (row, col) format
            path = uncompress_bljps_path(result.path)
            path = [(p[1], p[0]) for p in path]  # (x, y) -> (row, col)
            return path, runtime_ms, "Success"
        else:
            return None, runtime_ms, "No path found"

    except Exception as e:
        return None, 0.0, f"Error: {e}"

def test_silas_planner(grid, start, goal):
    """Test Silas AStar planner."""
    if not SILAS_AVAILABLE:
        return None, 0.0, "Silas not available"

    try:
        # Convert grid to format expected by AStar (True = free space, False = obstacle)
        free_space_grid = np.invert(grid.astype(bool))

        # Create AStar instance
        astar = AStar(free_space_grid)

        start_time = time.perf_counter()
        path = astar.search(start, goal)
        end_time = time.perf_counter()

        runtime_ms = (end_time - start_time) * 1000

        if path and len(path) > 0:
            return path, runtime_ms, "Success"
        else:
            return None, runtime_ms, "No path found"

    except Exception as e:
        return None, 0.0, f"Error: {e}"

def main():
    """Main function to test all planners visually."""
    # Configuration
    image_path = Path("data/image.png")

    # Check if image exists
    if not image_path.exists():
        print(f"Error: Image not found at {image_path}")
        return False

    # Load and process image
    print(f"Loading image: {image_path}")
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        print(f"Could not load image: {image_path}")
        return False
    # resize image to 1024x1024
    # image = cv2.resize(image, (2048, 2048))
    print(f"Image shape: {image.shape}")

    # Create boolean grid (True = obstacle, False = free)
    grid = image >= 128

    # Define test points (you can modify these)
    height, width = image.shape
    start = (50, 50)  # (row, col)
    goal = (height - 50, width - 50)  # (row, col)

    # Check if start and goal are in free space
    if grid[start[0], start[1]] or grid[goal[0], goal[1]]:
        print("Warning: Start or goal is in obstacle space, finding free points...")
        # Find free points
        free_points = []
        for _ in range(1000):  # Try up to 1000 times
            r = np.random.randint(0, height)
            c = np.random.randint(0, width)
            if not grid[r, c]:
                free_points.append((r, c))
                if len(free_points) >= 2:
                    break

        if len(free_points) >= 2:
            start, goal = free_points[0], free_points[1]
        else:
            print("Could not find free points!")
            return False

    print(f"Testing with start: {start}, goal: {goal}")

    # Test all planners
    planners = [
        ("JPS", test_jps_planner),
        ("BL_JPS", test_bljps_planner),
        ("Silas", test_silas_planner)
    ]

    results = {}

    for planner_name, planner_func in planners:
        print(f"\nTesting {planner_name}...")
        path, runtime_ms, status = planner_func(grid, start, goal)
        results[planner_name] = {
            'path': path,
            'runtime_ms': runtime_ms,
            'status': status
        }
        print(f"  {planner_name}: {status}, Runtime: {runtime_ms:.2f} ms")
        if path:
            print(f"  Path length: {len(path)} points")

    # Create visualization
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle('Path Planning Test Results', fontsize=16)

    # Original image
    axes[0, 0].imshow(image, cmap='gray')
    axes[0, 0].plot(start[1], start[0], 'go', markersize=10, label='Start')
    axes[0, 0].plot(goal[1], goal[0], 'ro', markersize=10, label='Goal')
    axes[0, 0].set_title('Original Image with Start/Goal')
    axes[0, 0].legend()
    axes[0, 0].axis('off')

    # Plot results for each planner
    plot_positions = [(0, 1), (1, 0), (1, 1)]
    colors = ['red', 'blue', 'green']

    for i, (planner_name, planner_func) in enumerate(planners):
        if i >= len(plot_positions):
            break

        row, col = plot_positions[i]
        ax = axes[row, col]

        # Show image
        ax.imshow(image, cmap='gray')

        # Plot start and goal
        ax.plot(start[1], start[0], 'go', markersize=8, label='Start')
        ax.plot(goal[1], goal[0], 'ro', markersize=8, label='Goal')

        # Plot path if available
        result = results[planner_name]
        if result['path']:
            path = result['path']
            path_x = [p[1] for p in path]  # col coordinates
            path_y = [p[0] for p in path]  # row coordinates
            ax.plot(path_x, path_y, colors[i], linewidth=2, alpha=0.7, label='Path')

        title = f"{planner_name}\n{result['status']}\nTime: {result['runtime_ms']:.2f} ms"
        if result['path']:
            title += f"\nPath length: {len(result['path'])}"

        ax.set_title(title)
        ax.legend()
        ax.axis('off')

    plt.tight_layout()

    # Save the visualization
    output_path = Path("planner_test_results.png")
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.show()

    print(f"\nVisualization saved to: {output_path}")

    # Summary
    print(f"\n=== Test Summary ===")
    successful_planners = []
    for planner_name, result in results.items():
        if result['path']:
            successful_planners.append(planner_name)
            print(f"✓ {planner_name}: SUCCESS - {len(result['path'])} points, {result['runtime_ms']:.2f} ms")
        else:
            print(f"✗ {planner_name}: FAILED - {result['status']}")

    if successful_planners:
        print(f"\n{len(successful_planners)} planner(s) working correctly: {', '.join(successful_planners)}")
        print("Ready to run full benchmark!")
        return True
    else:
        print("\nNo planners working correctly. Please check the setup.")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
