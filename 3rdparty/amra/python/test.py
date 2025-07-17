import sys

# Assume Python bindings for amra exist
from AMRA import plan_2d


MOVINGAI_DICT = {
    '.': 1,
    'G': 1,
    '@': -1,
    'O': -1,
    'T': 0,
    'S': 1,
    'W': 2,    # water is only traversible from water
    '(': 1000, # start
    '*': 1001, # path
    ')': 1002, # goal
    'E': 1003, # expanded state
}

def read_map(mapfile):
    with open(mapfile, 'r') as f:
        f.readline()  # type octile
        height = int(f.readline().split()[1])
        width = int(f.readline().split()[1])
        f.readline()  # map
        map_data = []
        for _ in range(height):
            line = f.readline().strip()
            for c in line:
                if c in MOVINGAI_DICT:
                    map_data.append(MOVINGAI_DICT[c])
                else:
                    val = ord(c) - ord('a') + 1
                    map_data.append(val * 10)
        return width, height, map_data

def main():
    mapfile = sys.argv[1] if len(sys.argv) > 1 else "../dat/Boston_0_1024.map"
    width, height, map_data = read_map(mapfile)
    origin = [0.0, 0.0]
    dim = [width, height]
    resolution = 1.0
    start_f = [28.0, 12.0]
    goal_f = [5.0, 45.0]
    path = []
    time_spent = 0.0
    result, path, time_spent = plan_2d(origin, dim, map_data, start_f, goal_f, resolution)
    if result == 1:
        print(f"Path found in {time_spent} seconds.")
        # for p in path:
        #     print(f"{p[0]}, {p[1]}")
    else:
        print("Failed to find a path.")

if __name__ == "__main__":
    main()