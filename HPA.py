import numpy as np
import heapq
from numba import jit
from colorama import init, Fore, Back, Style
import time

# Initialize colorama
init(autoreset=True)

# --- CONFIGURATION ---
# 0 = Free, 1 = Obstacle (Matches typical JPS3D/OccupancyGrid standards)
FREE = 0
OBSTACLE = 1

@jit(nopython=True)
def is_passable(grid, x, y):
    h, w = grid.shape
    if 0 <= x < w and 0 <= y < h:
        return grid[y, x] == FREE
    return False

@jit(nopython=True)
def jump(grid, cx, cy, dx, dy, end_x, end_y):
    """
    The core JPS 'Jump' function. Scans the grid in direction (dx, dy)
    until a jump point (forced neighbor) or obstacle is found.
    Returns: (jx, jy) if jump point found, else (-1, -1).
    """
    x, y = cx + dx, cy + dy

    # 1. Check bounds and obstacle
    if not is_passable(grid, x, y):
        return -1, -1

    # 2. Check if we reached the end
    if x == end_x and y == end_y:
        return x, y

    # 3. Check for forced neighbors
    # Diagonal Move
    if dx != 0 and dy != 0:
        # Check adjacent blocks for forced neighbors
        # Look for open nodes behind obstacles
        if (is_passable(grid, x - dx, y + dy) and not is_passable(grid, x - dx, y)) or \
           (is_passable(grid, x + dx, y - dy) and not is_passable(grid, x, y - dy)):
            return x, y

        # Recursive check on horizontal/vertical axes
        # If either horizontal or vertical jump finds something, this diagonal point is a jump point
        jx_h, jy_h = jump(grid, x, y, dx, 0, end_x, end_y)
        if jx_h != -1:
            return x, y

        jx_v, jy_v = jump(grid, x, y, 0, dy, end_x, end_y)
        if jx_v != -1:
            return x, y

    # Horizontal/Vertical Move
    else:
        if dx != 0: # Horizontal
            if (is_passable(grid, x + dx, y + 1) and not is_passable(grid, x, y + 1)) or \
               (is_passable(grid, x + dx, y - 1) and not is_passable(grid, x, y - 1)):
                return x, y
        else: # Vertical
            if (is_passable(grid, x + 1, y + dy) and not is_passable(grid, x + 1, y)) or \
               (is_passable(grid, x - 1, y + dy) and not is_passable(grid, x - 1, y)):
                return x, y

    # 4. Continue jumping recursively (or iteratively in Numba logic)
    return jump(grid, x, y, dx, dy, end_x, end_y)

@jit(nopython=True)
def get_jps_successors(grid, cx, cy, parent_x, parent_y, end_x, end_y):
    """
    Identifies valid directions to jump based on parent direction (pruning).
    Then calls jump() for each valid direction.
    """
    # Calculate direction from parent
    # If no parent (start node), dx=0, dy=0
    if parent_x == -1:
        # Check all 8 neighbors
        dirs = [
            (0, -1), (0, 1), (-1, 0), (1, 0),
            (-1, -1), (-1, 1), (1, -1), (1, 1)
        ]
    else:
        dx = np.sign(cx - parent_x)
        dy = np.sign(cy - parent_y)
        dirs = []

        if dx != 0 and dy != 0: # Diagonal
            # Natural neighbors
            dirs.append((dx, dy))
            dirs.append((dx, 0))
            dirs.append((0, dy))
            # Forced neighbors
            if not is_passable(grid, cx - dx, cy):
                dirs.append((-dx, dy))
            if not is_passable(grid, cx, cy - dy):
                dirs.append((dx, -dy))

        else: # Straight
            if dx != 0: # Horizontal
                dirs.append((dx, 0))
                if not is_passable(grid, cx, cy - 1):
                    dirs.append((dx, -1))
                if not is_passable(grid, cx, cy + 1):
                    dirs.append((dx, 1))
            else: # Vertical
                dirs.append((0, dy))
                if not is_passable(grid, cx - 1, cy):
                    dirs.append((-1, dy))
                if not is_passable(grid, cx + 1, cy):
                    dirs.append((1, dy))

    # Try jumping in all valid directions
    # We use a fixed size array for return to keep Numba happy,
    # but here we just return a list of tuples since this function is called from Python loop
    # NOTE: To keep Numba fast, usually we'd avoid lists, but for logic clarity in successors:
    out_x = []
    out_y = []
    out_dist = []

    for d in dirs:
        dx, dy = d
        jx, jy = jump(grid, cx, cy, dx, dy, end_x, end_y)
        if jx != -1:
            out_x.append(jx)
            out_y.append(jy)
            # Euclidean distance for cost
            dist = np.sqrt((jx - cx)**2 + (jy - cy)**2)
            out_dist.append(dist)

    return out_x, out_y, out_dist

class JPSPlanner:
    def __init__(self, width, height, obstacle_val=1):
        self.width = width
        self.height = height
        self.obstacle_val = obstacle_val
        # Default empty grid
        self.grid = np.zeros((height, width), dtype=np.int8)

    def update_map(self, new_grid):
        """
        Instant map update. No graph repair needed.
        """
        if new_grid.shape != self.grid.shape:
            print("Shape mismatch")
            return
        # Just reference or copy. O(1) or O(N) copy.
        self.grid = new_grid.copy()
        print(f"{Fore.GREEN}Map Updated (Instant){Style.RESET_ALL}")

    def find_path(self, start, end):
        """
        Standard A* loop using JPS successors.
        """
        sx, sy = start
        ex, ey = end

        # Check start/end
        if not is_passable(self.grid, sx, sy) or not is_passable(self.grid, ex, ey):
            print(f"{Fore.RED}Start or End blocked{Style.RESET_ALL}")
            return None

        # Priority Queue: (f_score, x, y, parent_x, parent_y)
        # We store parents in the queue to handle the "direction of arrival" logic
        open_set = []
        heapq.heappush(open_set, (0.0, sx, sy, -1, -1))

        g_score = {(sx, sy): 0.0}
        came_from = {}

        closed_set = set()

        nodes_expanded = 0

        while open_set:
            f, cx, cy, px, py = heapq.heappop(open_set)

            if (cx, cy) in closed_set:
                continue
            closed_set.add((cx, cy))

            # Record path
            if px != -1:
                came_from[(cx, cy)] = (px, py)

            if cx == ex and cy == ey:
                print(f"JPS found path. Nodes expanded: {nodes_expanded}")
                return self._reconstruct_path(came_from, start, end)

            nodes_expanded += 1

            # Get JPS Successors (Numba compiled)
            jx_list, jy_list, costs = get_jps_successors(self.grid, cx, cy, px, py, ex, ey)

            for i in range(len(jx_list)):
                jx, jy = jx_list[i], jy_list[i]
                cost = costs[i]

                if (jx, jy) in closed_set:
                    continue

                new_g = g_score[(cx, cy)] + cost

                if (jx, jy) not in g_score or new_g < g_score[(jx, jy)]:
                    g_score[(jx, jy)] = new_g
                    h = np.sqrt((ex - jx)**2 + (ey - jy)**2) # Euclidean Heuristic
                    priority = new_g + h
                    heapq.heappush(open_set, (priority, jx, jy, cx, cy))

        return None

    def _reconstruct_path(self, came_from, start, end):
        current = end
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        return path

def print_map_jps(planner, path=None):
    print("\n" + "="*40)
    path_set = set(path) if path else set()
    h, w = planner.grid.shape

    # Print simplified view (first 40x40)
    view_h = min(h, 40)
    view_w = min(w, 40)

    for y in range(view_h):
        line = ""
        for x in range(view_w):
            if (x, y) in path_set:
                char = f"{Back.CYAN} *{Style.RESET_ALL}"
            elif planner.grid[y, x] == planner.obstacle_val:
                char = f"{Back.RED}  {Style.RESET_ALL}"
            elif (x, y) == (0,0): # Origin marker
                char = "S "
            else:
                char = f"{Fore.BLACK}. {Style.RESET_ALL}"
            line += char
        print(line)
    print("="*40 + "\n")

if __name__ == "__main__":
    # --- DEMO ---
    W, H = 50, 50
    # JPSPlanner expects 1 = Obstacle by default
    planner = JPSPlanner(W, H, obstacle_val=1)

    # 1. Setup Map with a Wall
    grid = np.zeros((H, W), dtype=np.int8)
    grid[10:40, 25] = 1 # Vertical Wall
    planner.update_map(grid)

    start = (5, 25)
    end = (45, 25)

    print(f"\n{Fore.MAGENTA}--- JPS Search 1 (Wall) ---{Style.RESET_ALL}")
    t0 = time.time()
    path = planner.find_path(start, end)
    print(f"Time: {(time.time()-t0)*1000:.4f} ms")
    if path: print_map_jps(planner, path)

    # 2. Dynamic Update (Gap)
    print(f"\n{Fore.MAGENTA}--- Dynamic Update (Gap) ---{Style.RESET_ALL}")
    grid[20:25, 25] = 0 # Open gap
    planner.update_map(grid) # Instant

    print(f"\n{Fore.MAGENTA}--- JPS Search 2 (Gap) ---{Style.RESET_ALL}")
    t0 = time.time()
    path = planner.find_path(start, end)
    print(f"Time: {(time.time()-t0)*1000:.4f} ms")
    if path: print_map_jps(planner, path)