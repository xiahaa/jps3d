
# import sys
# import os
import jps_planner_bindings
import ThetaStarPlanner
import BL_JPS
from datetime import datetime
import warthog
import AMRA

# os.path.insert(0, os.path.abspath("."))

def uncompress_bljps_path(path):
    if not path:
        return []
    uncompressed_path = []
    for p_id in range(len(path)-1):
        current_p = [path[p_id][0],path[p_id][1]]
        uncompressed_path.append(tuple(current_p))

        while current_p[0] != path[p_id+1][0] or current_p[1] != path[p_id+1][1]:
            if path[p_id+1][0] != current_p[0]:
                if path[p_id+1][0] > current_p[0]:
                    current_p[0] += 1
                else:
                    current_p[0] -= 1
                uncompressed_path.append(tuple(current_p))
            if path[p_id+1][1] != current_p[1]:
                if path[p_id+1][1] > current_p[1]:
                    current_p[1] += 1
                else:
                    current_p[1] -= 1
                uncompressed_path.append(tuple(current_p))
    # print (path, uncompressed_path)
    return uncompressed_path

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
        # result = jps_planner_bindings.plan_2d(
        #     origin,
        #     dim,
        #     map_data,
        #     start_w,
        #     goal_w,
        #     resolution,
        #     use_jps=True  # Set to True to use JPS, False for A* only
        # # )
        # result = ThetaStarPlanner.plan_2d(
        #     origin,
        #     dim,
        #     map_data,
        #     start_w,
        #     goal_w,
        #     resolution,
        #     True  # Use JPS
        # )
        # print(result)
        # jps_path = result.path
        # time_spent = result.time_spent

        # status, jps_path, time_spent = result[0], result[1], result[2]

        # use BL_JPS for testing
        # start = datetime.now()
        # bljps = BL_JPS.BL_JPS()
        # bljps.preProcessGrid(map_data, width=dim[1], height=dim[0])
        # sX = int((start_w[0] - origin[0]) // resolution)
        # sY = int((start_w[1] - origin[1]) // resolution)
        # gX = int((goal_w[0] - origin[0]) // resolution)
        # gY = int((goal_w[1] - origin[1]) // resolution)
        # try:
        #     jps_path = bljps.findSolution(sX, sY, gX, gY)
        #     time_spent = (datetime.now()-start).total_seconds() * 1000  # Convert to milliseconds
        # except Exception as e:
        #     print(f"BL_JPS error: {e}")
        #     jps_path = None
        # print("\nJPS Path:")
        # if jps_path:
        #     for p in jps_path:
        #         print(f"  ({p[0]:.2f}, {p[1]:.2f})")
        # else:
        #     print("  No JPS path found.")

        jps_path = []
        time_spent = 0.0
        result, jps_path, time_spent = warthog.plan_2d(origin, dim, map_data, start, goal, resolution, "jps+")

        print(f"\nTime spent in planning: {time_spent:.4f} seconds")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
        import traceback
        traceback.print_exc()

def run_test_image():
    import cv2
    import numpy as np
    filename = "./data/image.png"
    print(f"Loading image from {filename}...")
    img = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)  # Load as grayscale

    # resize
    img = cv2.resize(img, (400, 300))  # Resize to 1000x1000 pixels

    origin = [0.0,0.0]  # x, y of the map origin
    dim = [img.shape[1], img.shape[0]]  # width, height in pixels
    resolution = 1  # meters per pixel (assumed)

    map_data = []
    # transform image to map data, for each pixel, if the value is over 200, it is marked as 100 occipied, otherwise 0 free
    for r in range(dim[1]):  # rows (y)
        row_data = []
        for c in range(dim[0]):  # cols (x)
            pixel_value = img[r, c]
            if isinstance(pixel_value, np.ndarray):
                # If pixel is multi-channel (e.g., RGB), take the first channel
                pixel_value = pixel_value[0]
            if pixel_value > 200:  # Assuming a threshold for occupied
                row_data.append(1)  # Occupied
            else:
                row_data.append(0)
        map_data.extend(row_data)

    # random pick start and goal points that are free and at least 1000 pixels apart
    free_indices = [i for i, v in enumerate(map_data) if v == 0]
    if len(free_indices) < 2:
        print("Not enough free cells to pick start and goal points.")
        return
    import random
    start_index = random.choice(free_indices)
    goal_index = random.choice(free_indices)
    while abs(start_index - goal_index) < 1000:  # Ensure at least 1000 pixels apart
        goal_index = random.choice(free_indices)
    start_w = [start_index % dim[0] * resolution, start_index // dim[0] * resolution]
    goal_w = [goal_index % dim[0] * resolution, goal_index // dim[0] * resolution]

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
        # result = jps_planner_bindings.plan_2d(
        #     origin,
        #     dim,
        #     map_data,
        #     start_w,
        #     goal_w,
        #     resolution,
        #     use_jps=True  # Set to True to use JPS, False for A* only
        # )
        # jps_path = result.path
        # time_spent = result.time_spent
        # result = ThetaStarPlanner.plan_2d(
        #     origin,
        #     dim,
        #     map_data,
        #     start_w,
        #     goal_w,
        #     resolution,
        #     True
        # )

        # status, jps_path, time_spent = result[0], result[1], result[2]

        # use BL_JPS for testing
        # start = datetime.now()
        # bljps = BL_JPS.BL_JPS()
        # bljps.preProcessGrid(map_data, width=dim[0], height=dim[1])
        # sX = int((start_w[0] - origin[0]) // resolution)
        # sY = int((start_w[1] - origin[1]) // resolution)
        # gX = int((goal_w[0] - origin[0]) // resolution)
        # gY = int((goal_w[1] - origin[1]) // resolution)
        # try:
            # jps_path = bljps.findSolution(sX, sY, gX, gY)
            # time_spent = (datetime.now()-start).total_seconds() * 1000  # Convert to milliseconds
        # except Exception as e:
            # print(f"BL_JPS error: {e}")
            # jps_path = None
        # bljps = BL_JPS.BL_JPS()
        # result = bljps.plan_2d(map_data, width=dim[0], height=dim[1], startX=start_w[0], startY=start_w[1], endX=goal_w[0], endY=goal_w[1], originX=origin[0], originY=origin[1], resolution=resolution)
        # plan_time = result.time_spent
        # jps_path = uncompress_bljps_path(result.path)
        # time_spent = result.time_spent

        jps_path = []
        time_spent = 0.0
        result, jps_path, time_spent = AMRA.plan_2d(origin, dim, map_data, start_w, goal_w, resolution)
        print(jps_path)

        # show planning result using matplotlib
        import matplotlib.pyplot as plt
        plt.imshow(np.array(map_data).reshape(dim[1], dim[0]), cmap='gray', origin='lower')
        plt.colorbar(label='Occupancy (0=free, 100=occupied)')
        if jps_path:
            jps_path_np = np.array(jps_path)
            plt.plot(jps_path_np[:, 0] / resolution, jps_path_np[:, 1] / resolution, 'r-', label='JPS Path')
        plt.scatter(start_w[0] / resolution, start_w[1] / resolution, c='green', label='Start', marker='o')
        plt.scatter(goal_w[0] / resolution, goal_w[1] / resolution, c='blue', label='Goal', marker='x')
        plt.legend()
        plt.title('JPS Path Planning Result')
        plt.xlabel('X (cells)')
        plt.ylabel('Y (cells)')
        plt.show()


        print(f"\nTime spent in planning: {time_spent:.4f} ms")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    run_test_image()
    # run_test()
    print("\nTest script finished.")
