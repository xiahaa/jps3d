import warthog
import numpy as np

origin = np.array([0.0, 0.0], dtype=np.float32)
dim = np.array([10, 10], dtype=np.int32)
map_data = np.zeros(100, dtype=np.int8)

# Add an obstacle
map_data[5 * 10 + 5] = 100

start = np.array([1.0, 1.0], dtype=np.float32)
goal = np.array([8.0, 8.0], dtype=np.float32)
resolution = 1.0
path = []
time_spent = 0.0

result, path, time_spent = warthog.plan_2d(origin.tolist(), dim.tolist(), map_data.tolist(), start.tolist(), goal.tolist(), resolution, "astar")
# jps jps2 jps+ jps2+ astar

if result == 0:
    print(f"Path found in {time_spent} microseconds.")
    for p in path:
        print(p)
else:
    print("Failed to find a path.")