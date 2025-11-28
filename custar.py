import numpy as np
from numba import cuda
import math
import time
import argparse

# --- Configuration ---
# Threads per block (standard GPU optimization)
TPB = 16

# --- GPU Kernels ---

@cuda.jit
def init_grid_kernel(dist_grid, start_x, start_y):
    """
    Initializes the distance grid.
    Sets the start node to 0 and all others to Infinity.
    """
    x, y = cuda.grid(2)
    rows, cols = dist_grid.shape

    if x < rows and y < cols:
        # 1e9 is effectively Infinity for our grid
        dist_grid[x, y] = 1000000000.0

        if x == start_x and y == start_y:
            dist_grid[x, y] = 0.0

@cuda.jit
def relaxation_kernel(grid, dist_in, dist_out, changed_flag):
    """
    The Core GPU Kernel (The "Pull" Method).
    Each thread handles one cell (x, y).
    It looks at its neighbors (up, down, left, right).
    If a neighbor offers a shorter path to this cell, we update our cost.
    """
    x, y = cuda.grid(2)
    rows, cols = grid.shape

    if x >= rows or y >= cols:
        return

    # If this cell is an obstacle (0), distance remains Infinity
    if grid[x, y] == 0:
        dist_out[x, y] = 1000000000.0
        return

    current_dist = dist_in[x, y]
    min_dist = current_dist

    # Directions: Up, Down, Left, Right
    # dx, dy pairs
    dirs_x = (0, 0, -1, 1)
    dirs_y = (-1, 1, 0, 0)

    # Check all 4 neighbors
    for i in range(4):
        nx = x + dirs_x[i]
        ny = y + dirs_y[i]

        # Bounds check
        if 0 <= nx < rows and 0 <= ny < cols:
            # If neighbor is not an obstacle
            if grid[nx, ny] == 1:
                # Calculate cost to arrive here from neighbor
                # Standard grid movement cost is 1.0
                # Diagonal could be added here (cost 1.414)
                new_val = dist_in[nx, ny] + 1.0

                if new_val < min_dist:
                    min_dist = new_val

    # Write the new minimum distance to the output buffer
    dist_out[x, y] = min_dist

    # If we found a shorter path, set the changed flag to True
    # We use a tolerance of 1e-6 for float comparison
    if min_dist < current_dist - 1e-6:
        changed_flag[0] = 1 # True

def solve_gpu_pathfinding(grid, start, end):
    """
    Manages the GPU memory and control loop.
    """
    rows, cols = grid.shape
    start_x, start_y = start
    end_x, end_y = end

    # 1. Allocate Device Memory
    d_grid = cuda.to_device(grid)
    # Double buffering for stable parallel updates
    d_dist_1 = cuda.device_array((rows, cols), dtype=np.float32)
    d_dist_2 = cuda.device_array((rows, cols), dtype=np.float32)
    # Flag to check if we need another iteration
    d_changed = cuda.device_array(1, dtype=np.int32)

    # 2. Configure Grid/Block dimensions
    threadsperblock = (TPB, TPB)
    blockspergrid_x = int(math.ceil(rows / threadsperblock[0]))
    blockspergrid_y = int(math.ceil(cols / threadsperblock[1]))
    blockspergrid = (blockspergrid_x, blockspergrid_y)

    # 3. Initialize Distances
    init_grid_kernel[blockspergrid, threadsperblock](d_dist_1, start_x, start_y)

    # 4. Main Relaxation Loop (The "Wavefront")
    iteration = 0
    max_iter = rows * cols # Safety breaker

    start_time = time.time()

    while iteration < max_iter:
        # Reset changed flag
        d_changed[0] = 0

        # Swap buffers: Read from A, Write to B
        input_dist = d_dist_1 if iteration % 2 == 0 else d_dist_2
        output_dist = d_dist_2 if iteration % 2 == 0 else d_dist_1

        # Launch Kernel
        relaxation_kernel[blockspergrid, threadsperblock](d_grid, input_dist, output_dist, d_changed)
        cuda.synchronize() # Wait for GPU

        # Check if converged
        if d_changed[0] == 0:
            final_dist = output_dist
            break

        iteration += 1

    end_time = time.time()
    print(f"GPU Convergence in {iteration} iterations.")
    print(f"GPU Compute Time: {(end_time - start_time)*1000:.2f} ms")

    # 5. Copy result back to Host
    result_cost_grid = final_dist.copy_to_host()
    return result_cost_grid

def reconstruct_path_cpu(cost_grid, start, end):
    """
    Backtracks from End to Start using the gradient of the cost grid.
    This is fast enough to do on CPU for a single path.
    """
    path = []
    current = end
    rows, cols = cost_grid.shape

    if cost_grid[end] >= 1000000000.0:
        return None # No path found

    while current != start:
        path.append(current)
        x, y = current

        # Look at neighbors to find the one with strictly lower cost
        best_n = None
        min_val = cost_grid[x, y]

        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]: # 4-connectivity
            nx, ny = x + dx, y + dy
            if 0 <= nx < rows and 0 <= ny < cols:
                if cost_grid[nx, ny] < min_val:
                    min_val = cost_grid[nx, ny]
                    best_n = (nx, ny)

        if best_n is None:
            print("Error: Stuck in local minima during reconstruction.")
            break

        current = best_n

    path.append(start)
    return path[::-1] # Reverse list

# --- Visualization Utilities ---
def print_grid_path(grid, path):
    """Prints a text-based representation of the path."""
    visual_grid = [[' ' for _ in range(grid.shape[1])] for _ in range(grid.shape[0])]

    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            if grid[r, c] == 0:
                visual_grid[r][c] = '#' # Wall
            else:
                visual_grid[r][c] = '.' # Empty

    if path:
        for x, y in path:
            visual_grid[x][y] = '*' # Path

    # Print a small section if grid is huge
    max_r = min(grid.shape[0], 20)
    max_c = min(grid.shape[1], 40)

    print("\n--- Grid Visualization (Top-Left Corner) ---")
    for r in range(max_r):
        print("".join(visual_grid[r][:max_c]))
    print("--------------------------------------------")

def main():
    parser = argparse.ArgumentParser(description="GPU Pathfinding with Numba")
    parser.add_argument('--size', type=int, default=1000, help="Grid size (NxN)")
    args = parser.parse_args()

    N = args.size
    print(f"Initializing {N}x{N} Grid...")

    # 1. Create a Random Grid (1 = Walkable, 0 = Obstacle)
    # We make it mostly walkable (80%)
    grid = np.random.choice([0, 1], size=(N, N), p=[0.2, 0.8]).astype(np.float32)

    # Ensure Start and End are walkable
    start = (0, 0)
    end = (N-1, N-1)
    grid[start] = 1.0
    grid[end] = 1.0

    print("Sending to GPU...")
    try:
        # Run Solver
        cost_grid = solve_gpu_pathfinding(grid, start, end)

        # Reconstruct Path
        print("Reconstructing path on CPU...")
        path = reconstruct_path_cpu(cost_grid, start, end)

        if path:
            print(f"Path found! Length: {len(path)} steps")
            print_grid_path(grid, path)
        else:
            print("No path found (Target unreachable).")

    except cuda.CudaSupportError:
        print("Error: No CUDA GPU found. This script requires an NVIDIA GPU.")

if __name__ == "__main__":
    main()