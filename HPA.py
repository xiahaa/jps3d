import numpy as np
import networkx as nx
from colorama import init, Fore, Back, Style
import heapq

# Initialize colorama
init(autoreset=True)

class HPAStar:
    def __init__(self, width, height, cluster_size=10):
        self.width = width
        self.height = height
        self.cluster_size = cluster_size

        # The raw grid (0 = Obstacle, 1 = Walkable)
        self.grid = np.ones((height, width), dtype=np.int8)

        # The Abstract Graph (High-Level)
        self.G = nx.Graph()

        # Cache for paths inside clusters: {(node_id_a, node_id_b): [path_steps]}
        self.intra_cluster_cache = {}

        # Mapping from Cluster Coordinate (cx, cy) to list of Abstract Node IDs
        self.cluster_nodes = {}

        # Build the initial graph
        self.build_graph()

    def get_cluster_id(self, x, y):
        """Returns the cluster coordinates (cx, cy) for a given grid pixel."""
        return x // self.cluster_size, y // self.cluster_size

    def build_graph(self):
        """Initial full build of the abstract graph."""
        print(f"{Fore.CYAN}Building HPA* Graph...{Style.RESET_ALL}")
        self.G.clear()
        self.intra_cluster_cache.clear()
        self.cluster_nodes.clear()

        # 1. Create Inter-Cluster Edges (Transitions between clusters)
        # Vertical boundaries
        for cx in range(self.width // self.cluster_size - 1):
            for cy in range(self.height // self.cluster_size):
                self._build_inter_cluster_edges_vertical(cx, cy)

        # Horizontal boundaries
        for cx in range(self.width // self.cluster_size):
            for cy in range(self.height // self.cluster_size - 1):
                self._build_inter_cluster_edges_horizontal(cx, cy)

        # 2. Create Intra-Cluster Edges (Paths inside a cluster)
        self._build_all_intra_edges()

    def _build_inter_cluster_edges_vertical(self, cx, cy):
        """Finds entrances between cluster(cx, cy) and cluster(cx+1, cy)."""
        x = (cx + 1) * self.cluster_size - 1 # Right edge of left cluster
        next_x = x + 1                       # Left edge of right cluster

        start_y = cy * self.cluster_size
        end_y = min((cy + 1) * self.cluster_size, self.height)

        self._scan_boundary(x, next_x, start_y, end_y, is_vertical=True)

    def _build_inter_cluster_edges_horizontal(self, cx, cy):
        """Finds entrances between cluster(cx, cy) and cluster(cx, cy+1)."""
        y = (cy + 1) * self.cluster_size - 1 # Bottom edge of top cluster
        next_y = y + 1                       # Top edge of bottom cluster

        start_x = cx * self.cluster_size
        end_x = min((cx + 1) * self.cluster_size, self.width)

        self._scan_boundary(y, next_y, start_x, end_x, is_vertical=False)

    def _scan_boundary(self, c1_coord, c2_coord, start, end, is_vertical):
        """
        Scans a boundary line for walkable segments and creates abstract nodes.
        Logic: Place a node in the middle of every contiguous open segment.
        """
        open_start = -1

        # If we are re-scanning, we should check if nodes already exist to avoid duplicates
        # But for simplicity in this repair logic, we relies on set checks later

        for k in range(start, end):
            # Check if both sides of the boundary are walkable
            if is_vertical:
                walkable = self.grid[k, c1_coord] == 1 and self.grid[k, c2_coord] == 1
            else:
                walkable = self.grid[c1_coord, k] == 1 and self.grid[c2_coord, k] == 1

            if walkable:
                if open_start == -1:
                    open_start = k
            else:
                if open_start != -1:
                    self._create_transition(open_start, k - 1, c1_coord, c2_coord, is_vertical)
                    open_start = -1

        # Handle segment ending at boundary edge
        if open_start != -1:
            self._create_transition(open_start, end - 1, c1_coord, c2_coord, is_vertical)

    def _create_transition(self, seg_start, seg_end, c1_coord, c2_coord, is_vertical):
        """Creates abstract nodes at the midpoint of the transition."""
        mid = (seg_start + seg_end) // 2

        if is_vertical:
            n1 = (c1_coord, mid)
            n2 = (c2_coord, mid)
        else:
            n1 = (mid, c1_coord)
            n2 = (mid, c2_coord)

        # Add nodes to graph if they don't exist
        if not self.G.has_node(n1):
            self.G.add_node(n1)
            self._register_node_to_cluster(n1)

        if not self.G.has_node(n2):
            self.G.add_node(n2)
            self._register_node_to_cluster(n2)

        # Add simple edge between them (cost 1)
        if not self.G.has_edge(n1, n2):
            self.G.add_edge(n1, n2, weight=1.0)

    def _register_node_to_cluster(self, node):
        cx, cy = self.get_cluster_id(node[0], node[1])
        if (cx, cy) not in self.cluster_nodes:
            self.cluster_nodes[(cx, cy)] = []
        if node not in self.cluster_nodes[(cx, cy)]:
            self.cluster_nodes[(cx, cy)].append(node)

    def _build_all_intra_edges(self):
        """Connects abstract nodes INSIDE each cluster."""
        for (cx, cy), nodes in self.cluster_nodes.items():
            self._connect_cluster_nodes(cx, cy, nodes)

    def _connect_cluster_nodes(self, cx, cy, nodes):
        """
        Runs local A* between all pairs of abstract nodes within a single cluster.
        This allows the agent to traverse the cluster.
        """
        # Filter out nodes that might have been deleted or are invalid
        valid_nodes = [n for n in nodes if self.G.has_node(n)]

        # Simple All-Pairs logic for the cluster nodes
        for i in range(len(valid_nodes)):
            for j in range(i + 1, len(valid_nodes)):
                n1 = valid_nodes[i]
                n2 = valid_nodes[j]

                # Check bounds for safety
                path = self._local_astar(n1, n2, bounds=(cx, cy))
                if path:
                    cost = len(path) - 1
                    self.G.add_edge(n1, n2, weight=cost)
                    self.intra_cluster_cache[(n1, n2)] = path
                    self.intra_cluster_cache[(n2, n1)] = list(reversed(path))

    def _local_astar(self, start, end, bounds=None):
        """
        Standard A* restricted to a specific cluster (bounds).
        """
        # Safety Check: If start or end is now blocked, return None
        if self.grid[start[1], start[0]] == 0: return None
        if self.grid[end[1], end[0]] == 0: return None

        # Heuristic: Manhattan
        def h(a, b): return abs(a[0] - b[0]) + abs(a[1] - b[1])

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}

        cx, cy = bounds
        min_x, max_x = cx * self.cluster_size, (cx + 1) * self.cluster_size
        min_y, max_y = cy * self.cluster_size, (cy + 1) * self.cluster_size

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == end:
                # Reconstruct
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return list(reversed(path))

            x, y = current
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                nx, ny = x + dx, y + dy

                # Check Bounds (Cluster restrictions)
                if not (min_x <= nx < max_x and min_y <= ny < max_y):
                    continue
                if not (0 <= nx < self.width and 0 <= ny < self.height):
                    continue
                if self.grid[ny, nx] == 0: # Check Grid Obstacle (Note: numpy is y,x)
                    continue

                temp_g = g_score[current] + 1
                if nx == end[0] and ny == end[1]: # End might be on boundary
                    pass

                if (nx, ny) not in g_score or temp_g < g_score[(nx, ny)]:
                    came_from[(nx, ny)] = current
                    g_score[(nx, ny)] = temp_g
                    f = temp_g + h((nx, ny), end)
                    heapq.heappush(open_set, (f, (nx, ny)))

        return None

    # --- DYNAMIC UPDATES ---

    def update_map(self, new_grid):
        """
        Batch update: Takes a full new grid (numpy array), finds differences,
        and efficiently repairs only the affected clusters.
        """
        if new_grid.shape != self.grid.shape:
            print(f"{Fore.RED}Error: New grid shape {new_grid.shape} mismatch.{Style.RESET_ALL}")
            return

        # 1. Find differences efficiently using NumPy
        diff = self.grid != new_grid

        if not np.any(diff):
            return # No changes

        ys, xs = np.where(diff)

        # 2. Update the internal grid state
        self.grid = new_grid.copy()

        # 3. Identify unique affected clusters
        # We also need to mark neighbors because boundary changes affect them
        affected_clusters = set()
        for i in range(len(xs)):
            cx, cy = self.get_cluster_id(xs[i], ys[i])
            affected_clusters.add((cx, cy))
            # Neighbors
            for dcx, dcy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                 ncx, ncy = cx + dcx, cy + dcy
                 if 0 <= ncx < (self.width // self.cluster_size) and 0 <= ncy < (self.height // self.cluster_size):
                     affected_clusters.add((ncx, ncy))

        print(f"{Fore.YELLOW}Batch Update: Repairing {len(affected_clusters)} clusters (including neighbors)...{Style.RESET_ALL}")

        # 4. Repair each affected cluster
        for (cx, cy) in affected_clusters:
            self._repair_cluster_fully(cx, cy)

    def update_obstacle(self, x, y, blocked=True):
        """
        Single-pixel update.
        """
        self.grid[y, x] = 0 if blocked else 1
        cx, cy = self.get_cluster_id(x, y)
        print(f"{Fore.YELLOW}Single Update at ({x}, {y}) -> Repairing Cluster ({cx}, {cy}){Style.RESET_ALL}")
        self._repair_cluster_fully(cx, cy)
        # Also repair neighbors just in case we hit a boundary
        for dcx, dcy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
             ncx, ncy = cx + dcx, cy + dcy
             if 0 <= ncx < (self.width // self.cluster_size) and 0 <= ncy < (self.height // self.cluster_size):
                 self._repair_cluster_fully(ncx, ncy)

    def _repair_cluster_fully(self, cx, cy):
        """
        Robust repair:
        1. Validate existing nodes (remove if blocked).
        2. Re-scan boundaries (add if opened).
        3. Re-connect internal paths.
        """
        if (cx, cy) not in self.cluster_nodes:
            self.cluster_nodes[(cx, cy)] = []

        nodes = self.cluster_nodes[(cx, cy)]

        # --- Step 1: Remove Invalid Nodes & Edges ---
        nodes_to_remove = []
        for n in nodes:
            # If node is now an obstacle
            if self.grid[n[1], n[0]] == 0:
                nodes_to_remove.append(n)

        for n in nodes_to_remove:
            if self.G.has_node(n):
                self.G.remove_node(n)
            nodes.remove(n)

        # Clear existing internal cache/edges for this cluster
        edges_to_remove = []
        # We have to check all known nodes to find edges to clear
        # But efficiently: we can just check self.intra_cluster_cache
        # If path is inside bounds, remove it
        keys_to_del = []
        for (u, v), path in self.intra_cluster_cache.items():
            # Check if this path belongs to this cluster
            # Simple check: is midpoint of path in cluster?
            if path:
                mid_pt = path[len(path)//2]
                mcx, mcy = self.get_cluster_id(mid_pt[0], mid_pt[1])
                if mcx == cx and mcy == cy:
                    keys_to_del.append((u, v))
                    if self.G.has_edge(u, v):
                         edges_to_remove.append((u, v))

        for u, v in edges_to_remove:
            self.G.remove_edge(u, v)
        for k in keys_to_del:
            del self.intra_cluster_cache[k]

        # --- Step 2: Re-Scan Boundaries for New Nodes ---
        # Right Boundary (Vertical)
        if cx < (self.width // self.cluster_size) - 1:
            self._build_inter_cluster_edges_vertical(cx, cy)
        # Bottom Boundary (Horizontal)
        if cy < (self.height // self.cluster_size) - 1:
            self._build_inter_cluster_edges_horizontal(cx, cy)
        # Left Boundary (Vertical - actually belongs to cx-1, but we might need to ensure consistency)
        # Top Boundary (Horizontal - belongs to cy-1)
        # In a perfect world, the neighbor update handles Left/Top.
        # But _build functions add nodes to *both* clusters, so calling it for (cx, cy) updates Right/Bottom.
        # Calling it for (cx-1, cy) updates Left. Since we update neighbors in update_map, this is covered!

        # --- Step 3: Re-Connect Internal Paths ---
        # Refresh node list in case scan added new ones
        if (cx, cy) in self.cluster_nodes:
            self._connect_cluster_nodes(cx, cy, self.cluster_nodes[(cx, cy)])

    def find_path(self, start, end):
        """
        The Hierarchical Search Query.
        1. Connect Start/End to Abstract Graph.
        2. Search High-Level Graph.
        3. Refine path.
        """
        # 0. Basic Checks
        if self.grid[start[1], start[0]] == 0 or self.grid[end[1], end[0]] == 0:
            print(f"{Fore.RED}Start or End is blocked.{Style.RESET_ALL}")
            return None

        # 1. Temporarily add Start and End to the Graph
        temp_nodes = [start, end]
        for pt in temp_nodes:
            self.G.add_node(pt)
            cx, cy = self.get_cluster_id(pt[0], pt[1])

            # Connect to existing entrances in this cluster
            if (cx, cy) in self.cluster_nodes:
                for entrance in self.cluster_nodes[(cx, cy)]:
                    # Ensure entrance is valid
                    if self.G.has_node(entrance):
                        path = self._local_astar(pt, entrance, bounds=(cx, cy))
                        if path:
                            cost = len(path) - 1
                            self.G.add_edge(pt, entrance, weight=cost)
                            self.intra_cluster_cache[(pt, entrance)] = path
                            self.intra_cluster_cache[(entrance, pt)] = list(reversed(path))

        # 2. High-Level Search
        try:
            abstract_path = nx.shortest_path(self.G, start, end, weight='weight')
            print(f"{Fore.GREEN}Abstract Path Found: {len(abstract_path)} nodes{Style.RESET_ALL}")
        except nx.NetworkXNoPath:
            print(f"{Fore.RED}No Abstract Path Found.{Style.RESET_ALL}")
            self._cleanup_temp_nodes(start, end)
            return None

        # 3. Refine (Stitch low-level paths)
        full_path = []
        for i in range(len(abstract_path) - 1):
            u = abstract_path[i]
            v = abstract_path[i+1]

            # Is it a direct jump (Inter-cluster) or a path (Intra-cluster)?
            if (u, v) in self.intra_cluster_cache:
                segment = self.intra_cluster_cache[(u, v)]
                # Avoid duplicating the join point
                full_path.extend(segment[:-1] if i < len(abstract_path)-2 else segment)
            else:
                # Inter-cluster edge (adjacent)
                full_path.append(u)

        # Include the very last node if not added
        if full_path[-1] != end:
            full_path.append(end)

        # 4. Cleanup Temp Nodes
        self._cleanup_temp_nodes(start, end)

        return full_path

    def _cleanup_temp_nodes(self, start, end):
        if self.G.has_node(start):
            self.G.remove_node(start)
        if self.G.has_node(end):
            self.G.remove_node(end)

def print_map(hpa, path=None):
    """Visualization using Colorama."""
    print("\n" + "="*40)
    path_set = set(path) if path else set()

    # Show cluster boundaries roughly
    for y in range(hpa.height):
        line = ""
        for x in range(hpa.width):

            char = "  "

            # Check cluster boundary
            is_bound_x = (x + 1) % hpa.cluster_size == 0
            is_bound_y = (y + 1) % hpa.cluster_size == 0

            if (x, y) in path_set:
                char = f"{Back.CYAN} *{Style.RESET_ALL}"
            elif hpa.grid[y, x] == 0:
                char = f"{Back.RED}  {Style.RESET_ALL}"
            elif (x, y) in hpa.G.nodes:
                char = f"{Back.YELLOW} O{Style.RESET_ALL}"
            elif is_bound_x or is_bound_y:
                 char = f"{Fore.BLACK}. {Style.RESET_ALL}"

            line += char
        print(line)
    print("="*40 + "\n")

if __name__ == "__main__":
    # --- DEMO ---
    W, H = 40, 40
    CLUSTER = 10

    hpa = HPAStar(W, H, CLUSTER)

    # 1. Create a Wall using Batch Update (Simulating a new map frame)
    print(f"\n{Fore.BLUE}--- Simulating Sensor Input (New Map) ---{Style.RESET_ALL}")

    # Create a copy of current grid
    new_map = hpa.grid.copy()

    # Draw a wall on the new map (numpy array operation)
    new_map[5:35, 20] = 0 # Block column 20 from y=5 to 35

    # Batch update
    hpa.update_map(new_map)

    start = (2, 2)
    end = (38, 38)

    # 2. First Search
    print(f"\n{Fore.MAGENTA}--- Search with Wall ---{Style.RESET_ALL}")
    path = hpa.find_path(start, end)
    print_map(hpa, path)

    # 3. Batch Update: Open a hole in the wall
    print(f"\n{Fore.BLUE}--- Simulating Sensor Update (Hole appeared) ---{Style.RESET_ALL}")
    new_map_2 = hpa.grid.copy()
    new_map_2[15:18, 20] = 1 # Open gap

    # Update map
    hpa.update_map(new_map_2)

    # 4. Search Again
    print(f"\n{Fore.MAGENTA}--- Second Search (Through Hole) ---{Style.RESET_ALL}")
    path = hpa.find_path(start, end)
    print_map(hpa, path)