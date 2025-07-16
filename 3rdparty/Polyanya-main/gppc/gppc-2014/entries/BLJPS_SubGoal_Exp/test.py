import bljps_sub
import numpy as np

def test_plan_2d():
    # Simple 5x5 empty map
    origin = [0.0, 0.0]
    dim = [5, 5]
    map_data = [0] * 25  # All cells traversable
    start = [0.0, 0.0]
    goal = [4.0, 4.0]
    resolution = 1.0
    # add a small obstacle
    map_data[12] = 1  # Make cell (2, 2)

    ret, path, time_spent = bljps_sub.plan_2d(origin, dim, map_data, start, goal, resolution)
    print(f"Return code: {ret}")
    print(f"Path: {path}")
    print(f"Time spent (ms): {time_spent}")
    assert ret == 0, "Path not found!"
    assert len(path) > 0, "Path is empty!"
    assert np.allclose(path[0], start, atol=1e-3), f"Path does not start at {start}"
    assert np.allclose(path[-1], goal, atol=1e-3), f"Path does not end at {goal}"

if __name__ == "__main__":
    test_plan_2d()
