import jps_planner_bindings

def run_test():
    print("Testing jps_planner_bindings.plan_2d...")

    # Define map properties
    origin = [-1.0, -1.0]  # x, y of the map origin
    dim = [10, 10]         # width, height in cells
    resolution = 0.1       # meters per cell

    # Create a simple map: 10x10 grid
    # 0 = free, 1 = occupied (this is an assumption, might need adjustment based on JPS library)
    # Let's make a border of obstacles and a clear path
    map_data = []
    for r in range(dim[1]): # rows (y)
        row_data = []
        for c in range(dim[0]): # cols (x)
            if r == 0 or r == dim[1]-1 or c == 0 or c == dim[0]-1: # Border
                row_data.append(1) # Occupied
            elif r == 5 and c > 1 and c < 8: # Horizontal obstacle
                 row_data.append(1)
            else:
                row_data.append(0) # Free
        map_data.extend(row_data)

    # Ensure map_data is a flat list of signed chars (bytes/integers in Python)
    # Pybind11 should handle list of int to std::vector<signed char>
    # Forcing to signed char range if necessary, though Python ints will likely cast.
    # map_data = [int(x) for x in map_data] # Already list of ints

    # Define start and goal points (in world coordinates)
    # Start at (0.2, 0.2) which is cell ( (0.2 - (-1.0))/0.1, (0.2 - (-1.0))/0.1 ) = (12,12) -> too far
    # Map is 10x10 cells, resolution 0.1. Origin -1.0, -1.0
    # World coordinates:
    # Min X: -1.0 (cell 0)
    # Max X: -1.0 + 10 * 0.1 = 0.0 (cell 9)
    # Min Y: -1.0 (cell 0)
    # Max Y: -1.0 + 10 * 0.1 = 0.0 (cell 9)

    # Start at cell (1,1) -> world (-0.9, -0.9)
    # Goal at cell (8,8) -> world (-0.2, -0.2)
    start_w = [-0.85, -0.85] # cell (1.5, 1.5) -> (1,1)
    goal_w = [-0.15, -0.15]   # cell (8.5, 8.5) -> (8,8)

    print(f"Map Origin: {origin}")
    print(f"Map Dimensions (cells): {dim}")
    print(f"Map Resolution: {resolution}")
    print(f"Start (world): {start_w}")
    print(f"Goal (world): {goal_w}")
    # print(f"Map data (first 20 elements): {map_data[:20]}")
    # print(f"Map data length: {len(map_data)} (Expected: {dim[0]*dim[1]})")

    try:
        # Call the plan_2d function
        # plan_2d(origin, dim, map_data, start_w, goal_w, resolution, jps_path_out, astar_path_out)
        # The wrapper returns a result object
        result = jps_planner_bindings.plan_2d(
            origin,
            dim,
            map_data,
            start_w,
            goal_w,
            resolution,
            use_jps=True  # Set to True to use JPS, False for A* only
        )

        jps_path = result.path
        time_spent = result.time_spent

        print("\nJPS Path:")
        if jps_path:
            for p in jps_path:
                print(f"  ({p[0]:.2f}, {p[1]:.2f})")
        else:
            print("  No JPS path found.")

        print(f"\nTime spent in planning: {time_spent:.4f} seconds")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    run_test()
    print("\nTest script finished.")
